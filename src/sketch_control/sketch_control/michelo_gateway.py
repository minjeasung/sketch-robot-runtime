"""Serve the installed Outpost console with one external Sketch button.

No upstream files are edited. HTTP and WebSockets stay on this gateway origin;
the browser opens the independent sketch application in a new tab.
"""
import argparse
import asyncio
import base64
from contextlib import asynccontextmanager
import json
import os
import secrets
from urllib.parse import urlsplit

import httpx
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
import websockets

from .outpost_camera import validate_origin
from .system_api import is_loopback


HOP_HEADERS = {'connection', 'keep-alive', 'proxy-authenticate', 'proxy-authorization',
               'te', 'trailer', 'transfer-encoding', 'upgrade', 'content-length',
               'content-encoding', 'host', 'authorization', 'cookie'}


def launcher_script(sketch_url, sketch_port):
    if sketch_url:
        url = urlsplit(sketch_url)
        if url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password:
            raise ValueError('Sketch URL must be HTTP(S) without credentials')
    if not 1 <= sketch_port <= 65535:
        raise ValueError('Sketch port must be 1..65535')
    return '''(() => {
  if (document.getElementById('sketch-launcher')) return;
  const url = new URL(%s || location.href);
  if (!%s) { url.port = %s; url.pathname = '/sketch/'; url.search = ''; url.hash = ''; }
  const link = document.createElement('a');
  link.id = 'sketch-launcher'; link.href = url.href;
  link.target = '_blank'; link.rel = 'noopener noreferrer';
  link.textContent = '스케치 작업 ↗';
  link.style.cssText = 'display:inline-block;margin:8px;padding:9px 16px;border-radius:8px;background:#2468c9;color:white;text-decoration:none;font-weight:600';
  (document.querySelector('header') || document.body).appendChild(link);
})();''' % (json.dumps(sketch_url), json.dumps(sketch_url), json.dumps(str(sketch_port)))


def create_gateway(outpost='http://127.0.0.1:8100', *, sketch_url='', sketch_port=8081,
                   token=None, local_only=True, transport=None):
    origin = validate_origin(outpost)
    script = launcher_script(sketch_url, sketch_port)

    @asynccontextmanager
    async def lifespan(app):
        async with httpx.AsyncClient(timeout=30, follow_redirects=False, transport=transport, trust_env=False) as client:
            app.state.client = client
            yield

    app = FastAPI(title='Michelo Sketch launcher', lifespan=lifespan, docs_url=None,
                  redoc_url=None, openapi_url=None)

    def authorized(headers):
        if not token:
            return True
        try:
            scheme, value = headers.get('authorization', '').split(' ', 1)
            user, password = base64.b64decode(value, validate=True).decode().split(':', 1)
            return scheme.lower() == 'basic' and user == 'sketch' and secrets.compare_digest(password, token)
        except (ValueError, UnicodeError):
            return False

    def allowed(request):
        host = request.url.hostname or ''
        expected = ('https' if request.url.scheme in ('https', 'wss') else 'http') + '://' + request.headers.get('host', '')
        return ((not local_only or is_loopback(host))
                and request.headers.get('origin', expected) == expected)

    @app.websocket('/{path:path}')
    async def socket_proxy(websocket: WebSocket, path: str):
        if not allowed(websocket) or not authorized(websocket.headers) or path not in ('ws/events', 'ws/stream'):
            await websocket.close(code=1008)
            return
        upstream = 'ws' + origin[4:] + '/' + path
        if websocket.url.query:
            upstream += '?' + websocket.url.query
        tasks = []
        try:
            async with websockets.connect(upstream, max_size=128*1024*1024, open_timeout=5) as peer:
                await websocket.accept()

                async def upstream_to_browser():
                    async for data in peer:
                        if isinstance(data, bytes):
                            await websocket.send_bytes(data)
                        else:
                            await websocket.send_text(data)

                async def browser_to_upstream():
                    while True:
                        event = await websocket.receive()
                        if event['type'] == 'websocket.disconnect':
                            return
                        await peer.send(event.get('bytes') if event.get('bytes') is not None else event['text'])

                tasks = [asyncio.create_task(upstream_to_browser()), asyncio.create_task(browser_to_upstream())]
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        except (OSError, WebSocketDisconnect, websockets.exceptions.WebSocketException):
            pass
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            try:
                await websocket.close()
            except RuntimeError:
                pass

    @app.api_route('/{path:path}', methods=['GET', 'HEAD', 'POST', 'PUT', 'PATCH', 'DELETE', 'OPTIONS'])
    async def proxy(request: Request, path: str):
        if not allowed(request):
            return Response('Invalid Host or Origin', status_code=403)
        if not authorized(request.headers):
            return Response('Authentication required', status_code=401,
                            headers={'WWW-Authenticate': 'Basic realm="Michelo Sketch"'})
        if path == '__sketch/launcher.js':
            return Response(script, media_type='application/javascript', headers={'Cache-Control': 'no-store'})
        target = origin + '/' + path
        if request.url.query:
            target += '?' + request.url.query
        headers = {k:v for k,v in request.headers.items() if k.lower() not in HOP_HEADERS | {'origin'}}
        headers['accept-encoding'] = 'identity'
        try:
            response = await app.state.client.request(request.method, target, headers=headers, content=await request.body())
        except httpx.HTTPError:
            return Response('Outpost is unavailable. Start the existing camera daemon.', status_code=502)
        headers = {k:v for k,v in response.headers.items() if k.lower() not in HOP_HEADERS}
        location = headers.get('location', '')
        if location.startswith(origin + '/'):
            headers['location'] = location[len(origin):]
        data = response.content
        if 'text/html' in headers.get('content-type', '') and response.status_code == 200:
            marker = b'<script src="/__sketch/launcher.js"></script>'
            data = data.replace(b'</body>', marker+b'</body>') if b'</body>' in data else data + marker
            headers.pop('etag', None)
            headers['cache-control'] = 'no-store'
        return Response(data, status_code=response.status_code, headers=headers)

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=os.environ.get('SKETCH_MICHELO_HOST', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('SKETCH_MICHELO_PORT', '8101')))
    parser.add_argument('--outpost', default=os.environ.get('SKETCH_OUTPOST_HTTP', 'http://127.0.0.1:8100'))
    parser.add_argument('--sketch-url', default=os.environ.get('SKETCH_PUBLIC_URL', ''))
    parser.add_argument('--sketch-port', type=int, default=int(os.environ.get('SKETCH_SUPERVISOR_PORT', '8081')))
    args = parser.parse_args()
    token = os.environ.get('SKETCH_MICHELO_TOKEN')
    if not is_loopback(args.host) and not token:
        parser.error('SKETCH_MICHELO_TOKEN is required for LAN access (Basic username: sketch)')
    if not 1 <= args.port <= 65535:
        parser.error('Invalid port')
    app = create_gateway(args.outpost, sketch_url=args.sketch_url, sketch_port=args.sketch_port,
                         token=token, local_only=is_loopback(args.host))
    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, ws='wsproto')


if __name__ == '__main__':
    main()
