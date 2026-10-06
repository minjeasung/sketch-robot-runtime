"""Management API; path execution remains an explicit action in the ROS UI."""
import hmac
import json
from pathlib import Path
from fastapi import FastAPI, Header, HTTPException, Request
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles


def create_app(config, runtime, web_root):
    web = Path(web_root)
    app = FastAPI(title='SNUCEM Humble Spray Add-on')

    def authorized(authorization):
        expected = 'Bearer '+config.api_token
        if config.api_token and not hmac.compare_digest(authorization or '', expected):
            raise HTTPException(401, 'API token required')

    @app.middleware('http')
    async def reject_cross_origin_writes(request: Request, call_next):
        if request.method not in ('GET', 'HEAD', 'OPTIONS'):
            origin = request.headers.get('origin')
            if origin and origin.rstrip('/') != str(request.base_url).rstrip('/'):
                return Response('Cross-origin control request rejected', status_code=403)
        return await call_next(request)

    @app.get('/health')
    def health():
        return {'ok': True}

    @app.get('/status')
    def status():
        return dict(runtime.status(), configuration=dict(config.public(), camera_backend='outpost'))

    @app.get('/addon/capabilities')
    def capabilities():
        return dict(process_modes=['spray'], upstream='SNUCEM_Robot_22.04', ros_distro='humble',
                    profile=config.profile, rosbridge_url=config.rosbridge_url, image_topic=config.image_topic)

    @app.post('/prepare')
    def prepare(authorization: str | None = Header(default=None)):
        authorized(authorization)
        try:
            return runtime.prepare()
        except (ValueError, RuntimeError, OSError) as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post('/shutdown')
    def shutdown(authorization: str | None = Header(default=None)):
        authorized(authorization)
        return runtime.shutdown()

    @app.get('/', response_class=HTMLResponse)
    def manager():
        return (web/'addon.html').read_text(encoding='utf-8')

    @app.get('/sketch/', response_class=HTMLResponse)
    def sketch():
        html = (web/'index.html').read_text(encoding='utf-8')
        settings = json.dumps(dict(rosbridge_url=config.rosbridge_url, image_topic=config.image_topic,
                                   addon=True), ensure_ascii=True).replace('<', '\\u003c')
        html = html.replace('<script src="js/roslib.min.js">', '<script>window.SKETCH_RUNTIME='+settings+';</script><script src="js/roslib.min.js">')
        html = html.replace('<option value="paint">롤러 도장</option>', '')
        html = html.replace('<a href="setup.html">F/T 센서 설정 ↗</a>', '<span>SNUCEM · ROS 2 Humble · Spray</span>')
        return html

    app.mount('/sketch', StaticFiles(directory=web), name='sketch-assets')
    return app
