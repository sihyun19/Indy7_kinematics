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
            
            T_tcp, p_joints, z_axes, p_com, T_links = fk(q)
            
            # Three.js Column-major 규격에 맞춰 전치(.T) 후 1차원 평탄화 (16개 원소)
            matrices_col_major = [mat.T.flatten().tolist() for mat in T_links]
            tcp_pos = T_tcp[:3, 3].tolist()
            tcp_rpy = np.degrees(rot_rpy(T_tcp[:3, :3])).tolist()
            tcp_matrix = T_tcp.T.flatten().tolist()
            
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
        elif parsed.path == '/api/trajectories':
            traj_dir = os.path.join(PROJECT_ROOT, 'data', 'trajectories')
            files = []
            if os.path.exists(traj_dir):
                # .npz 확장자 파일만 정렬하여 추출
                files = sorted([f for f in os.listdir(traj_dir) if f.endswith('.npz')])
            
            payload = json.dumps(files).encode('utf-8')
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        elif parsed.path == '/api/trajectory':
            params = urllib.parse.parse_qs(parsed.query)
            filename = params.get('file', [''])[0]
            # 상위 디렉터리 접근 방지
            safe_name = os.path.basename(filename)
            file_path = os.path.join(PROJECT_ROOT, 'data', 'trajectories', safe_name)
            
            if not safe_name or not os.path.exists(file_path):
                self.send_error(404, "Trajectory file not found")
                return

            try:
                data = np.load(file_path)
                times = data['time'].tolist() if 'time' in data else []
                q_arr = data['q'] # shape: (N, 3) 또는 (N, 6)
                N = len(q_arr)

                all_matrices = []
                all_tcp = []

                for i in range(N):
                    # 2D 3자유도 궤적일 경우 6D로 확장 [0, q0, q1, 0, q2, 0]
                    if q_arr.shape[1] == 3:
                        q6 = np.array([0.0, q_arr[i, 0], q_arr[i, 1], 0.0, q_arr[i, 2], 0.0])
                    else:
                        q6 = q_arr[i]
                    
                    T_tcp, _, _, _, T_links = fk(q6)
                    # Three.js column-major 1차원 평탄화
                    col_mats = [mat.T.flatten().tolist() for mat in T_links]
                    all_matrices.append(col_mats)
                    all_tcp.append(T_tcp[:3, 3].tolist())

                payload = json.dumps({
                    "times": times,
                    "q": q_arr.tolist(),
                    "matrices": all_matrices,
                    "tcp": all_tcp
                }).encode('utf-8')

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            except Exception as e:
                self.send_error(500, f"Error processing trajectory: {str(e)}")

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