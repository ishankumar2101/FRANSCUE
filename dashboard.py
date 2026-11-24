"""Local research dashboard. Run python dashboard.py and open printed URL."""
import json, argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from live_engine import Arena,Session,PROFILES,POLICIES,LAYOUTS

class Handler(BaseHTTPRequestHandler):
    def reply(self,status,data,ctype='application/json'):
        if ctype=='application/json':data=json.dumps(data,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path=='/':self.reply(200,Path(__file__).with_name('live_dashboard.html').read_bytes(),'text/html; charset=utf-8')
        else:self.reply(404,{'error':'Not found'})
    def do_POST(self):
        try:
            # Browser sends JSON; bind to localhost only.
            n=int(self.headers.get('Content-Length',0))
            if n>4096:raise ValueError('Request too large')
            data=json.loads(self.rfile.read(n) or b'{}')
            if self.path=='/api/new':
                layout=data.get('layout','original');seed=int(data.get('seed',8100));profile=data.get('profile','nominal');method=data.get('method','robust')
                if layout not in LAYOUTS and layout not in ('generated-301','generated-302','generated-303') or profile not in PROFILES or method not in POLICIES or not 0<=seed<=999999:raise ValueError('Invalid experiment configuration')
                if getattr(self.server,'session',None):
                    old=self.server.session.world;old.route.cache_clear();old.shadowed.cache_clear();old.shadow_row.cache_clear()
                self.server.session=Session(Arena(layout),seed,profile,method)
            elif self.path=='/api/step':
                if getattr(self.server,'session',None) is None:raise ValueError('Start a new experiment first')
                self.server.session.step()
            else:return self.reply(404,{'error':'Not found'})
            self.reply(200,self.server.session.state())
        except (ValueError,TypeError,KeyError,AttributeError) as e:self.reply(400,{'error':str(e)})

def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8765);a=p.parse_args()
    with HTTPServer(('127.0.0.1',a.port),Handler) as s:
        print(f'FRANSCUE live simulation: http://127.0.0.1:{a.port}',flush=True);s.serve_forever()
if __name__=='__main__':main()
