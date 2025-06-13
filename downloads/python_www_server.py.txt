import http.server
import socketserver
import threading
import webbrowser
import time
import os
import sys
import subprocess

PORT = 8000
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

def open_edge(url):
    # Try common Edge paths on Windows
    edge_paths = [
        r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"
    ]
    for path in edge_paths:
        if os.path.exists(path):
            subprocess.Popen([path, url])
            return True
    # Fallback: try using webbrowser module (may open in default browser)
    try:
        webbrowser.get('windows-default').open(url)
    except:
        webbrowser.open(url)
    return False

def run_server():
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"Serving at http://localhost:{PORT}")
        httpd.serve_forever()

if __name__ == "__main__":
    # Start server in a thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # Wait a moment for the server to start
    time.sleep(1)

    # Open Edge to the local index.html
    url = f"http://localhost:{PORT}/streaming_viewer/index4.html"
    print(f"Opening {url} in Edge...")
    open_edge(url)

    # Keep main thread alive to keep server running
    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        print("Shutting down.")
        sys.exit(0)