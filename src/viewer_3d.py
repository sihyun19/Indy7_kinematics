import http.server
import socketserver
import webbrowser
import os
import sys
import json
import urllib.parse
import numpy as np

# 프로젝트 루트 경로를 sys.path에 추가하고 작업 디렉토리 설정
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
os.chdir(PROJECT_ROOT)

from src.Indy7_FK import fk, rot_rpy

PORT = 8000

class KinematicsRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == '/api/fk':
            params = urllib.parse.parse_qs(parsed.query)
            q_str = params.get('q', ['0,0,0,0,0,0'])[0]
            try:
                q = np.array([float(val) for val in q_str.split(',')])
            except ValueError:
                q = np.zeros(6)
            
            T, orig, zax, link_transforms = fk(q)
            
            # Three.js Column-major 규격에 맞춰 전치(.T) 후 1차원 평탄화 (16개 원소)
            matrices_col_major = [mat.T.flatten().tolist() for mat in link_transforms]
            tcp_pos = T[:3, 3].tolist()
            tcp_rpy = np.degrees(rot_rpy(T[:3, :3])).tolist()
            tcp_matrix = T.T.flatten().tolist()
            
            payload = json.dumps({
                "matrices": matrices_col_major,
                "tcp": tcp_pos,
                "tcp_rpy": tcp_rpy,
                "tcp_matrix": tcp_matrix
            }).encode('utf-8')
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
        else:
            # HTML, JS, STL 등 정적 파일 기본 서빙
            super().do_GET()

def run():
    http.server.ThreadingHTTPServer.allow_reuse_address = True
    with http.server.ThreadingHTTPServer(("", PORT), KinematicsRequestHandler) as httpd:
        print(f"Server started at http://localhost:{PORT}/web/index.html")
        webbrowser.open(f"http://localhost:{PORT}/web/index.html")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    run()