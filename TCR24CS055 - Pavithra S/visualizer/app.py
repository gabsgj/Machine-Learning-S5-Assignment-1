import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8080

def run_server():
    # Change working directory to visualizer folder
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    Handler = http.server.SimpleHTTPRequestHandler

    print(f"\n=======================================================")
    print(f"  Safe Semantic Planner - Interactive Visualizer")
    print(f"  Serving at: http://localhost:{PORT}")
    print(f"  Press Ctrl+C to stop the server.")
    print(f"=======================================================\n")

    webbrowser.open(f"http://localhost:{PORT}")

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    run_server()
