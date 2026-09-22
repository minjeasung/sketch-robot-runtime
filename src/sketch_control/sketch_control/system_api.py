"""FastAPI supervisor for the sketch robot, following SNUCEM runtime conventions."""
import argparse
from contextlib import asynccontextmanager
import fcntl
import ipaddress
import os
from pathlib import Path
import secrets

from fastapi import APIRouter, Depends, FastAPI, HTTPException, Query, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, StrictBool

from .process_supervisor import Supervisor, SupervisorError
from .outpost_camera import get_json


class ConfigurationRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    profile: str = "dry_run"
    robot_ip: str = "10.0.2.7"
    model_id: str = "rb10_1300e_u"
    launch_rviz: StrictBool | None = None
    launch_zed_driver: StrictBool | None = None
    launch_d405_driver: StrictBool | None = None
    camera_backend: str = 'outpost'
    outpost_http: str = 'http://127.0.0.1:8100'
    outpost_zed_hw_id: str = ''
    outpost_zed_serial: str = ''
    outpost_d405_hw_id: str = ''
    outpost_d405_serial: str = ''


def is_loopback(host):
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def create_app(supervisor, *, api_token=None, manage_monitor=False, local_only=True):
    @asynccontextmanager
    async def lifespan(_app):
        await supervisor.open()
        try:
            yield
        finally:
            try:
                await supervisor.close()
            finally:
                if manage_monitor:
                    supervisor.monitor.close()

    app = FastAPI(title="Sketch Robot Supervisor", version="1.0.0", lifespan=lifespan,
                  description="Process management API. Robot motion stays in the guarded sketch executor.")
    app.state.supervisor = supervisor
    bearer = HTTPBearer(auto_error=False)

    async def authenticate(credentials: HTTPAuthorizationCredentials | None = Depends(bearer)):
        if api_token and (credentials is None or not secrets.compare_digest(credentials.credentials, api_token)):
            raise HTTPException(401, "missing or invalid bearer token", headers={"WWW-Authenticate": "Bearer"})

    @app.middleware("http")
    async def origin_guard(request: Request, call_next):
        host = request.url.hostname or ""
        if local_only and not is_loopback(host):
            return JSONResponse(status_code=403, content={"detail": "Invalid local Host"})
        origin = request.headers.get("origin")
        expected = f"{request.url.scheme}://{request.headers.get('host', '')}"
        if origin is not None and origin != expected:
            return JSONResponse(status_code=403, content={"detail": "Cross-origin browser access is disabled"})
        return await call_next(request)

    @app.exception_handler(SupervisorError)
    async def handle_error(_request, exc):
        return JSONResponse(status_code=exc.status, content={"detail": str(exc)})

    @app.get("/healthz")
    async def health():
        return {"status": "live"}

    router = APIRouter(dependencies=[Depends(authenticate)])

    @router.get("/status")
    async def status():
        return supervisor.status()

    @router.get("/events")
    async def events():
        return {"events": list(supervisor.event_list)}

    @router.get("/configuration")
    async def configuration():
        return supervisor.options

    @router.get('/outpost/cameras')
    async def outpost_cameras():
        import asyncio
        try:
            cameras = await asyncio.to_thread(get_json, supervisor.options['outpost_http'], '/cameras')
            return {'cameras': cameras}
        except (OSError, ValueError) as exc:
            raise HTTPException(503, 'Outpost unavailable: ' + str(exc)) from None

    @router.post("/configuration")
    async def configure(body: ConfigurationRequest):
        return await supervisor.configure(body.model_dump(exclude_none=True))

    @router.get("/processes/{name}")
    async def process_status(name: str):
        return supervisor.snapshot(name)

    @router.get("/processes/{name}/logs")
    async def logs(name: str, lines: int = Query(100, ge=1, le=500)):
        return supervisor.logs(name, lines)

    @router.post("/processes/{name}/start")
    async def start(name: str):
        return await supervisor.start(name)

    @router.post("/processes/{name}/stop")
    async def stop(name: str, cascade: bool = False):
        return await supervisor.stop(name, cascade)

    @router.post("/processes/{name}/restart")
    async def restart(name: str, cascade: bool = False):
        return await supervisor.restart(name, cascade)

    @router.post("/prepare-system")
    async def prepare():
        return await supervisor.prepare()

    @router.post("/shutdown-system")
    async def shutdown():
        return await supervisor.shutdown()

    app.include_router(router)
    web = supervisor.workspace / "web"

    @app.get("/", include_in_schema=False)
    async def home():
        return FileResponse(web / "system.html", headers={"Cache-Control": "no-store"})

    @app.get("/sketch", include_in_schema=False)
    async def sketch_redirect():
        return RedirectResponse("/sketch/")

    app.mount("/sketch", StaticFiles(directory=web, html=True), name="sketch")
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--host", default=os.environ.get("SKETCH_SUPERVISOR_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("SKETCH_SUPERVISOR_PORT", "8081")))
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    token = os.environ.get("SKETCH_SUPERVISOR_API_TOKEN") or None
    if not is_loopback(args.host) and not token:
        parser.error("SKETCH_SUPERVISOR_API_TOKEN is required for remote access")
    workspace = args.workspace.resolve()
    runtime = workspace / "logs/system_api"
    runtime.mkdir(parents=True, exist_ok=True)
    options = {"profile": os.environ.get("SKETCH_PROFILE", "dry_run"),
               "robot_ip": os.environ.get("SKETCH_ROBOT_IP", "10.0.2.7"),
               "model_id": os.environ.get("SKETCH_MODEL_ID", "rb10_1300e_u")}
    for key in ('camera_backend', 'outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial',
                'outpost_d405_hw_id', 'outpost_d405_serial'):
        value = os.environ.get('SKETCH_' + key.upper())
        if value is not None:
            options[key] = value
    for key in ("launch_rviz", "launch_zed_driver", "launch_d405_driver"):
        value = os.environ.get("SKETCH_" + key.upper())
        if value is not None:
            if value.lower() not in ("true", "false"):
                parser.error(f"SKETCH_{key.upper()} must be true or false")
            options[key] = value.lower() == "true"
    with (runtime / "server.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another supervisor owns this workspace")
        from .system_ros_monitor import RosMonitor
        import uvicorn
        monitor = RosMonitor()
        try:
            supervisor = Supervisor(workspace, monitor, options)
            app = create_app(supervisor, api_token=token, local_only=is_loopback(args.host))
            # ROS Jazzy may provide an older websockets package. This service is
            # HTTP-only; the existing rosbridge owns WebSocket connections.
            uvicorn.run(app, host=args.host, port=args.port, workers=1, ws="none")
        finally:
            monitor.close()


if __name__ == "__main__":
    main()
