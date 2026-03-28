import { useCallback, useEffect, useRef, useState } from 'react'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'
import { api } from '../services/api'

const MAX_BUF = 500_000

export function usePiSetup(droneId, connected) {
  const [status, setStatus] = useState({ state: 'unknown', running: false })
  const [actionLoading, setActionLoading] = useState('')
  const [actionResult, setActionResult] = useState(null)
  const [streamConnected, setStreamConnected] = useState(false)

  const terminalHostRef = useRef(null)
  const terminalRef = useRef(null)
  const fitAddonRef = useRef(null)
  const bufRef = useRef('')
  const streamRef = useRef(null)
  const reconnectTimerRef = useRef(null)
  const logCursorRef = useRef({ offset: 0, inode: 0 })
  const statusInFlightRef = useRef(false)

  const resetTerminal = useCallback((text = '') => {
    const trimmed = text.length > MAX_BUF ? text.slice(text.length - MAX_BUF) : text
    bufRef.current = trimmed
    const t = terminalRef.current
    if (!t) return
    t.reset()
    if (trimmed) t.write(trimmed)
  }, [])

  const appendTerminal = useCallback((text) => {
    if (!text) return
    const next = bufRef.current + text
    if (next.length <= MAX_BUF) {
      bufRef.current = next
      terminalRef.current?.write(text)
      return
    }
    const trimmed = next.slice(next.length - MAX_BUF)
    bufRef.current = trimmed
    terminalRef.current?.reset()
    terminalRef.current?.write(trimmed)
  }, [])

  // Mount xterm
  useEffect(() => {
    const host = terminalHostRef.current
    if (!host) return
    const term = new Terminal({
      disableStdin: true,
      convertEol: true,
      cursorBlink: false,
      fontSize: 12,
      fontFamily: 'SF Mono, Fira Code, ui-monospace, monospace',
      scrollback: 100000,
      theme: { background: '#0c0c12', foreground: '#d6d6e2' },
    })
    const fit = new FitAddon()
    term.loadAddon(fit)
    term.open(host)
    fit.fit()
    terminalRef.current = term
    fitAddonRef.current = fit
    if (bufRef.current) term.write(bufRef.current)
    const onResize = () => fitAddonRef.current?.fit()
    window.addEventListener('resize', onResize)
    return () => {
      window.removeEventListener('resize', onResize)
      fitAddonRef.current = null
      terminalRef.current = null
      term.dispose()
    }
  }, [])

  const closeStream = useCallback(() => {
    if (reconnectTimerRef.current) {
      clearTimeout(reconnectTimerRef.current)
      reconnectTimerRef.current = null
    }
    const ws = streamRef.current
    if (!ws) return
    ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null
    try { ws.close() } catch { /* ignore */ }
    streamRef.current = null
    setStreamConnected(false)
  }, [])

  const openStream = useCallback(() => {
    if (streamRef.current || !connected) return
    const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
    const cursor = logCursorRef.current
    const url = `${proto}://${window.location.host}/ws/pi-connections/${droneId}/terminal?offset=${cursor.offset}&inode=${cursor.inode}`
    const ws = new WebSocket(url)
    streamRef.current = ws

    ws.onopen = () => setStreamConnected(true)

    ws.onmessage = e => {
      if (typeof e.data !== 'string') return
      let parsed = null
      try { parsed = JSON.parse(e.data) } catch { appendTerminal(e.data); return }
      if (parsed?.type === 'chunk') {
        const nextOffset = Number(parsed.next_offset ?? logCursorRef.current.offset)
        const nextInode = Number(parsed.inode ?? logCursorRef.current.inode)
        const data = typeof parsed.data === 'string' ? parsed.data : ''
        if (parsed.reset) resetTerminal(data)
        else appendTerminal(data)
        logCursorRef.current = {
          offset: Number.isFinite(nextOffset) ? nextOffset : logCursorRef.current.offset,
          inode: Number.isFinite(nextInode) ? nextInode : logCursorRef.current.inode,
        }
      }
    }

    ws.onerror = () => setStreamConnected(false)

    ws.onclose = () => {
      if (streamRef.current === ws) streamRef.current = null
      setStreamConnected(false)
      // Always reconnect while connected — so we see logs even after process stops
      if (connected) {
        if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current)
        reconnectTimerRef.current = setTimeout(() => {
          reconnectTimerRef.current = null
          openStream()
        }, 1500)
      }
    }
  }, [appendTerminal, connected, droneId, resetTerminal])

  const loadLogs = useCallback(async () => {
    if (!connected) return
    const data = await api(`/api/pi-connections/${droneId}/logs?offset=0&inode=0`)
    if (data.success) {
      const text = data.logs || ''
      resetTerminal(text)
      logCursorRef.current = {
        offset: Number(data.next_offset || 0),
        inode: Number(data.inode || 0),
      }
    }
  }, [connected, droneId, resetTerminal])

  const refreshStatus = useCallback(async () => {
    if (!connected || statusInFlightRef.current) return
    statusInFlightRef.current = true
    try {
      const data = await api(`/api/pi-connections/${droneId}/status`)
      if (data.success !== false) {
        setStatus({ state: data.state || 'unknown', running: !!data.running, pid: data.pid })
      }
    } finally {
      statusInFlightRef.current = false
    }
  }, [connected, droneId])

  const launch = useCallback(async () => {
    if (!connected) return
    setActionLoading('launch')
    setActionResult(null)
    resetTerminal('')
    logCursorRef.current = { offset: 0, inode: 0 }
    // Close stream so it reconnects from offset 0 after launch
    closeStream()
    const data = await api(`/api/pi-connections/${droneId}/launch`, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshStatus()
    openStream()
  }, [closeStream, connected, droneId, openStream, refreshStatus, resetTerminal])

  const stop = useCallback(async () => {
    if (!connected) return
    setActionLoading('stop')
    setActionResult(null)
    const data = await api(`/api/pi-connections/${droneId}/stop`, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshStatus()
  }, [connected, droneId, refreshStatus])

  // On mount / when connected: load existing logs, open stream, start polling status
  useEffect(() => {
    if (!connected) {
      closeStream()
      setStatus({ state: 'offline', running: false })
      setActionResult(null)
      setStreamConnected(false)
      resetTerminal('Pi offline — connect to launch.\r\n')
      return
    }

    loadLogs().then(() => openStream())
    refreshStatus()

    const interval = setInterval(refreshStatus, 2000)
    return () => {
      clearInterval(interval)
      closeStream()
    }
  }, [closeStream, connected, droneId, loadLogs, openStream, refreshStatus, resetTerminal])

  return {
    terminalHostRef,
    status,
    streamConnected,
    actionLoading,
    actionResult,
    launch,
    stop,
  }
}
