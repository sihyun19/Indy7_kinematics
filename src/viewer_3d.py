import http.server
import socketserver
import webbrowser
import os
import json
import urllib.parse
import numpy as np

# 현재 파일의 차상위 디렉토리(프로젝트 루트)를 프로세스 작업 기준 디렉토리로 강제 이동
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.Indy7_FK import fk

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
            
            payload = json.dumps({
                "matrices": matrices_col_major,
                "tcp": tcp_pos
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
    # 새로 정의한 KinematicsRequestHandler를 핸들러로 전달
    with socketserver.TCPServer(("", PORT), KinematicsRequestHandler) as httpd:
        print(f"Server started at http://localhost:{PORT}/web/index.html")
        webbrowser.open(f"http://localhost:{PORT}/web/index.html")
        httpd.serve_forever()

if __name__ == "__main__":
    run()