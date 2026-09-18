import http.server
import socketserver
import webbrowser
import os

PORT = 8000

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) 
#현재 파일의 차상위 디렉토리를 프로세스 작업 기준 디렉토리로 강제 이동
#이렇게 별도로 지정해주지 않으면 http.server는 현재 파일이 위치한 디렉토리를 최상위루트로 인식함

Handler = http.server.SimpleHTTPRequestHandler
#작업 디렉토리 내 파일들을 읽어서 전달하는 기본 핸들러 클래스. 확장자에 맞는 MIME타입 헤더를 전달

def run():
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"Server started at http://localhost:{PORT}/web/index.html")
        webbrowser.open(f"http://localhost:{PORT}/web/index.html")
        httpd.serve_forever()

if __name__ == "__main__":
    run()
