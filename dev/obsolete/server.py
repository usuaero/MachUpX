from http.server import HTTPServer, BaseHTTPRequestHandler
from io import BytesIO
import subprocess as sp
import json

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        self.send_response(200)
        self.end_headers()
        print(json.loads(body))
        response = BytesIO()
        response.write(body)
        self.wfile.write(response.getvalue())

httpd = HTTPServer(('localhost', 8000), SimpleHTTPRequestHandler)
httpd.serve_forever()