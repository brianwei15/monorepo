import { useCallback, useEffect, useRef, useState } from 'react'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'
import { api } from '../services/api'

const MAX_BUF = 500_000

export function usePiSetup(droneId, connected, launchConfig) {
  const [status, setStatus] = useState({ state: 'unknown', running: false })
  const [buildStatus, setBuildStatus] = useState({ state: 'unknown', running: false })
  const [actionLoading, setActionLoading] = useState('')
  const [actionResult, setActionResult] = useState(null)
  const [streamConnected, setStreamConnected] = useState(false)
  const [logType, setLogType] = useState('launch') // 'build' | 'launch'

  const terminalHostRef = useRef(null)
  const terminalRef = useRef(null)
  const fitAddonRef = useRef(null)
  const bufRef = useRef('')
  const streamRef = useRef(null)
  const reconnectTimerRef = useRef(null)
  const logCursorRef = useRef({ offset: 0, inode: 0 })
  const statusInFlightRef = useRef(false)
  const buildStatusInFlightRef = useRef(false)
  const logTypeRef = useRef('launch') // mirrors logType state but stable for callbacks

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

  // openStream reads logTypeRef so it doesn't need logType in deps
  const openStream = useCallback(() => {
    if (streamRef.current || !connected) return
    const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
    const cursor = logCursorRef.current
    const url = `${proto}://${window.location.host}/ws/pi-connections/${droneId}/terminal?offset=${cursor.offset}&inode=${cursor.inode}&log_type=${logTypeRef.current}`
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
      if (connected) {
        if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current)
        reconnectTimerRef.current = setTimeout(() => {
          reconnectTimerRef.current = null
          openStream()
        }, 1500)
      }
    }
  }, [appendTerminal, connected, droneId, resetTerminal])

  // loadLogs reads logTypeRef so it doesn't need logType in deps
  const loadLogs = useCallback(async () => {
    if (!connected) return
    const data = await api(`/api/pi-connections/${droneId}/logs?offset=0&inode=0&log_type=${logTypeRef.current}`)
    if (data.success) {
      const text = data.logs || ''
      resetTerminal(text)
      logCursorRef.current = {
        offset: Number(data.next_offset || 0),
        inode: Number(data.inode || 0),
      }
    }
  }, [connected, droneId, resetTerminal])

  // Switch which log the terminal is streaming. Closes and reopens the WS.
  const switchLogType = useCallback((type) => {
    if (logTypeRef.current === type) return
    logTypeRef.current = type
    setLogType(type)
    closeStream()
    logCursorRef.current = { offset: 0, inode: 0 }
    resetTerminal('')
    if (connected) {
      loadLogs().then(() => openStream())
    }
  }, [closeStream, connected, loadLogs, openStream, resetTerminal])

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

  const refreshBuildStatus = useCallback(async () => {
    if (!connected || buildStatusInFlightRef.current) return
    buildStatusInFlightRef.current = true
    try {
      const data = await api(`/api/pi-connections/${droneId}/build-status`)
      if (data.success !== false) {
        setBuildStatus({ state: data.state || 'unknown', running: !!data.running, pid: data.pid })
      }
    } finally {
      buildStatusInFlightRef.current = false
    }
  }, [connected, droneId])

  const build = useCallback(async () => {
    if (!connected) return
    setActionLoading('build')
    setActionResult(null)
    // Switch terminal to build log
    logTypeRef.current = 'build'
    setLogType('build')
    closeStream()
    logCursorRef.current = { offset: 0, inode: 0 }
    resetTerminal('')
    const data = await api(`/api/pi-connections/${droneId}/build`, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshBuildStatus()
    openStream()
  }, [closeStream, connected, droneId, openStream, refreshBuildStatus, resetTerminal])

  const launch = useCallback(async () => {
    if (!connected) return
    setActionLoading('launch')
    setActionResult(null)
    // Switch terminal to launch log
    logTypeRef.current = 'launch'
    setLogType('launch')
    closeStream()
    logCursorRef.current = { offset: 0, inode: 0 }
    resetTerminal('')
    const data = await api(`/api/pi-connections/${droneId}/launch`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(launchConfig ?? {}),
    })
    setActionResult(data)
    setActionLoading('')
    await refreshStatus()
    openStream()
  }, [closeStream, connected, droneId, launchConfig, openStream, refreshStatus, resetTerminal])

  const stop = useCallback(async () => {
    if (!connected) return
    setActionLoading('stop')
    setActionResult(null)
    const data = await api(`/api/pi-connections/${droneId}/stop`, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshStatus()
  }, [connected, droneId, refreshStatus])

  const stopBuild = useCallback(async () => {
    if (!connected) return
    setActionLoading('stop-build')
    setActionResult(null)
    const data = await api(`/api/pi-connections/${droneId}/stop-build`, { method: 'POST' })
    setActionResult(data)
    setActionLoading('')
    await refreshBuildStatus()
  }, [connected, droneId, refreshBuildStatus])

  // On mount / when connected: load logs, open stream, start polling both statuses
  useEffect(() => {
    if (!connected) {
      closeStream()
      setStatus({ state: 'offline', running: false })
      setBuildStatus({ state: 'offline', running: false })
      setActionResult(null)
      setStreamConnected(false)
      resetTerminal('Pi offline — connect to launch.\r\n')
      return
    }

    loadLogs().then(() => openStream())
    refreshStatus()
    refreshBuildStatus()

    const interval = setInterval(() => {
      refreshStatus()
      refreshBuildStatus()
    }, 2000)
    return () => {
      clearInterval(interval)
      closeStream()
    }
  }, [closeStream, connected, droneId, loadLogs, openStream, refreshBuildStatus, refreshStatus, resetTerminal])

  return {
    terminalHostRef,
    status,
    buildStatus,
    streamConnected,
    actionLoading,
    actionResult,
    logType,
    switchLogType,
    build,
    launch,
    stop,
    stopBuild,
  }
}
