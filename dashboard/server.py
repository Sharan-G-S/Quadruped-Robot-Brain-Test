"""
server.py — FastAPI Dashboard Server
======================================
Real-time WebSocket for telemetry & camera streaming,
REST endpoints for commands, and static file serving.
"""

import asyncio
import json
import logging
import time
import os
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logger = logging.getLogger(__name__)

# The robot instance will be set by main.py
_robot = None


def set_robot(robot):
    global _robot
    _robot = robot


app = FastAPI(title="QuadBot-AI Dashboard", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve static files
static_dir = os.path.join(os.path.dirname(__file__), "static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")


# ── Pages ───────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def index():
    return FileResponse(os.path.join(static_dir, "index.html"))


# ── REST API ────────────────────────────────────────────────────

@app.get("/api/status")
async def get_status():
    if _robot:
        return _robot.get_telemetry()
    return {"status": "robot not initialized"}


@app.post("/api/command")
async def send_command(payload: dict):
    """Send a command to the robot. Body: {"command": "walk_forward", ...}"""
    if not _robot:
        return {"error": "robot not initialized"}

    command = payload.get("command", "")
    if command:
        _robot.process_command(command)
        return {"status": "ok", "command": command}
    return {"error": "no command provided"}


@app.post("/api/action")
async def send_action(payload: dict):
    """Send a direct action. Body: {"action": "walk", "speed": 0.5}"""
    if not _robot:
        return {"error": "robot not initialized"}

    _robot.execute_action(payload)
    return {"status": "ok", "action": payload}


@app.post("/api/emergency_stop")
async def emergency_stop():
    if _robot:
        _robot.emergency_stop()
        return {"status": "emergency_stop_triggered"}
    return {"error": "robot not initialized"}


# ── WebSocket: Telemetry ────────────────────────────────────────

@app.websocket("/ws/telemetry")
async def telemetry_ws(websocket: WebSocket):
    """Stream real-time telemetry data."""
    await websocket.accept()
    logger.info("Telemetry WebSocket connected")

    try:
        while True:
            if _robot:
                data = _robot.get_telemetry()
                await websocket.send_json(data)
            else:
                await websocket.send_json({"status": "waiting"})
            await asyncio.sleep(0.1)  # 10 Hz
    except WebSocketDisconnect:
        logger.info("Telemetry WebSocket disconnected")
    except Exception as e:
        logger.error(f"Telemetry WS error: {e}")


# ── WebSocket: Camera Stream ───────────────────────────────────

@app.websocket("/ws/camera")
async def camera_ws(websocket: WebSocket):
    """Stream live camera frames as JPEG bytes."""
    await websocket.accept()
    logger.info("Camera WebSocket connected")

    try:
        while True:
            if _robot and hasattr(_robot, "camera"):
                frame = _robot.camera.capture_frame()
                jpeg = _robot.camera.frame_to_jpeg_bytes(frame)
                if jpeg:
                    await websocket.send_bytes(jpeg)
            await asyncio.sleep(1.0 / 15)  # 15 FPS
    except WebSocketDisconnect:
        logger.info("Camera WebSocket disconnected")
    except Exception as e:
        logger.error(f"Camera WS error: {e}")


# ── WebSocket: Commands (bidirectional) ─────────────────────────

@app.websocket("/ws/command")
async def command_ws(websocket: WebSocket):
    """Accept text commands and return responses."""
    await websocket.accept()
    logger.info("Command WebSocket connected")

    try:
        while True:
            data = await websocket.receive_text()
            response = {"status": "received", "command": data}

            if _robot:
                _robot.process_command(data)
                response["robot_state"] = _robot.state_machine.state.value

            await websocket.send_json(response)
    except WebSocketDisconnect:
        logger.info("Command WebSocket disconnected")
    except Exception as e:
        logger.error(f"Command WS error: {e}")


def run_dashboard(host: str = "0.0.0.0", port: int = 8080):
    """Start the dashboard server."""
    logger.info(f"Dashboard: http://{host}:{port}")
    uvicorn.run(app, host=host, port=port, log_level="warning")
