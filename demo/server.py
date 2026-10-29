"""Run the repository's PurePursuit controller in a small kinematic sandbox."""
import json
import math
import sys
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'sim_ws/src/amr_control/amr_control'))
from pure_pursuit import PurePursuit


def simulate(lookahead=.3):
    path = [(i*.04, .55*math.sin(i*.04*1.6)) for i in range(101)]
    controller = PurePursuit(.05, lookahead_distance=lookahead, simulation=True)
    controller.path = path
    x,y,theta = 0.,-.45,0.
    frames=[]
    reached=False
    for step in range(2200):
        v,w=controller.compute_commands(x,y,theta)
        x += float(v)*math.cos(theta)*.05
        y += float(v)*math.sin(theta)*.05
        theta += float(w)*.05
        distance=math.hypot(x-path[-1][0],y-path[-1][1])
        if step%3==0: frames.append({'x':x,'y':y,'theta':theta,'v':float(v),'w':float(w),'t':step*.05})
        if distance<.08:
            reached=True
            break
    return {'path':path,'frames':frames,'reached':reached,'lookahead':lookahead,'duration':step*.05,'final_distance':distance,'mode':'original-controller-kinematic-simulation'}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed=urlparse(self.path)
        if parsed.path=='/':
            data=Path(__file__).with_name('index.html').read_bytes(); mime='text/html; charset=utf-8'
        elif parsed.path=='/simulate':
            try:
                lookahead=float(parse_qs(parsed.query).get('lookahead',['.3'])[0])
                if not math.isfinite(lookahead) or not .15<=lookahead<=.8: raise ValueError()
            except ValueError:
                self.send_error(400); return
            data=json.dumps(simulate(lookahead)).encode();mime='application/json'
        else:
            self.send_error(404);return
        self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def log_message(self,*_): pass


if __name__=='__main__':
    if '--verify' in sys.argv:
        for distance in [.15,.3,.5,.8]:
            result=simulate(distance)
            assert result['reached'], f'Goal not reached: {distance}'
            assert all(math.isfinite(row[k]) for row in result['frames'] for k in ['x','y','theta','v','w'])
            print(f"Lookahead {distance}: goal reached in {result['duration']:.2f} simulated seconds")
    else:
        print('Robot demo: http://127.0.0.1:8766',flush=True)
        ThreadingHTTPServer(('127.0.0.1',8766),Handler).serve_forever()
