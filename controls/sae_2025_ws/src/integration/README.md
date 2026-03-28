# PennAiR Auton Deploy

React + FastAPI dashboard for Pi connectivity, WiFi control, build deploy/rollback, mission control, and live ROS camera streaming.

## Structure

```
integration/
├── app.py              # FastAPI entrypoint
├── launch.sh           # Dev launcher (backend + frontend)
├── backend/            # Backend modules
│   ├── app_factory.py
│   ├── config.py
│   ├── context.py
│   ├── models.py
│   ├── ssh.py
│   ├── state.py
│   ├── routers/
│   │   ├── config.py
│   │   ├── connection.py
│   │   ├── deploy.py
│   │   ├── mission.py
│   │   ├── stream.py       # MJPEG video stream endpoints
│   │   ├── terminal_ws.py
│   │   └── wifi.py
│   └── services/
│       ├── deploy.py
│       ├── mission.py
│       ├── ros_stream.py   # ROS topic subscription + frame conversion
│       └── wifi.py
├── frontend/           # React frontend
│   ├── src/
│   │   ├── App.jsx     # Main UI composition
│   │   ├── hooks/      # API/WebSocket hooks
│   │   ├── services/   # API client helpers
│   │   ├── App.css     # Styles
│   │   ├── main.jsx    # Entry point
│   │   └── index.css   # Global styles
│   ├── index.html
│   ├── vite.config.js
│   └── package.json
```

## Setup & Run

### Basic launch (no camera streams)

`./launch.sh` auto-checks all dependencies at startup:
- Frontend deps via `npm install`
- Backend Python deps in conda mode (`fastapi[standard]`, `python-multipart`, `httpx`)
- `sshpass` via package manager (`brew`, `apt`, or `dnf`)

```bash
cd src/integration
./launch.sh
```

- Backend API: `http://localhost:8080`
- Frontend: `http://localhost:3000`

Stop both with `Ctrl+C`.

### With live ROS camera streams

The video stream backend subscribes to local ROS 2 topics, so the backend must run inside a sourced workspace.

**1. Source the workspace:**

```bash
cd /home/yuzhiliu8/code/monorepo/controls/sae_2025_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**2. Install OpenCV (if not already present):**

```bash
pip install opencv-python numpy
# or with conda:
conda install -c conda-forge opencv
```

**3. Run the launcher:**

```bash
cd src/integration
./launch.sh
```

Open `http://localhost:3000`. The **Video & Debug Streams** panel at the bottom of Mission Control will show live feeds from:

| Stream | Topic | Type |
|--------|-------|------|
| px4_1 camera (compressed) | `/px4_1/camera/compressed` | `CompressedImage` |
| px4_2 camera (compressed) | `/px4_2/camera/compressed` | `CompressedImage` |
| px4_1 camera (raw) | `/px4_1/camera` | `Image` |

These topics match the remaps in `src/uav/launch/pi-setup.launch.py` (default `px4_id=1`).

You can add or remove streams at runtime using the **+ Add stream** button — any ROS topic that publishes `sensor_msgs/Image` or `sensor_msgs/CompressedImage` works.

#### How streaming works

- Topics ending in `/compressed` are treated as `CompressedImage` (JPEG passthrough).
- All other topics are treated as raw `Image` and converted to JPEG via `cv2`.
- The backend serves each stream as MJPEG (`multipart/x-mixed-replace`) at `GET /api/stream/video?topic=<topic>`.
- The frontend uses a plain `<img>` tag — the browser keeps the connection alive and updates the frame automatically.
- If a topic is not yet publishing, the image box stays blank. Hit the refresh button (↻) to reconnect.
- If ROS is not available (workspace not sourced), the stream boxes show **No stream** and `GET /api/stream/status` returns `{"available": false}`.

### Conda workflow (optional)

```bash
conda activate <your-env>
cd src/integration
./launch.sh
```

When a conda env is active, `launch.sh` uses `python app.py` and installs missing backend deps automatically. Without conda it uses `uv run app.py`.

## Building for Production

```bash
cd frontend
npm run build
```

Creates a `dist/` folder served directly by the FastAPI backend at `/`.
