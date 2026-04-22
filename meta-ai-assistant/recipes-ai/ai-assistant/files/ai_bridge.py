#!/usr/bin/env python3
"""
AI Bridge - handles Gemini API calls with vision
Listens on port 5000 for requests from the C++ server
"""

import http.server
import json
import base64
import urllib.request
import urllib.error

# --- PASTE YOUR GEMINI API KEY HERE ---
GEMINI_API_KEY = "AIzaSyBe9gMj2QE1uok32nXygR8oB3nh4Part9g"
# --------------------------------------

GEMINI_URL = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key={GEMINI_API_KEY}"
CAMERA_STREAM_URL = "http://127.0.0.1:8080"

def capture_frame():
    """Capture a single JPEG frame from the MJPEG stream"""
    try:
        req = urllib.request.Request(CAMERA_STREAM_URL)
        with urllib.request.urlopen(req, timeout=3) as resp:
            data = b""
            while True:
                chunk = resp.read(1024)
                if not chunk:
                    break
                data += chunk
                start = data.find(b'\xff\xd8')
                end = data.find(b'\xff\xd9', start)
                if start != -1 and end != -1:
                    return data[start:end+2]
    except Exception as e:
        print(f"Frame capture error: {e}")
        return None

def ask_gemini(question, image_data):
    """Send question + image to Gemini API"""
    image_b64 = base64.standard_b64encode(image_data).decode("utf-8")

    payload = {
        "contents": [
            {
                "parts": [
                    {
                        "inline_data": {
                            "mime_type": "image/jpeg",
                            "data": image_b64
                        }
                    },
                    {
                        "text": question
                    }
                ]
            }
        ]
    }

    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        GEMINI_URL,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )

    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            result = json.loads(resp.read().decode("utf-8"))
            return result["candidates"][0]["content"]["parts"][0]["text"]
    except urllib.error.HTTPError as e:
        error_body = e.read().decode("utf-8")
        return f"API error {e.code}: {error_body}"
    except Exception as e:
        return f"Error: {str(e)}"

class AIHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_POST(self):
        if self.path == "/ask":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            question = body.get("question", "What do you see?")

            print(f"Question: {question}")
            image = capture_frame()
            if not image:
                response = {"error": "Could not capture frame"}
            else:
                answer = ask_gemini(question, image)
                print(f"Answer: {answer[:100]}...")
                response = {"answer": answer}

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                self.wfile.write(json.dumps(response).encode("utf-8"))
            except BrokenPipeError:
                pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

if __name__ == "__main__":
    server = http.server.HTTPServer(("0.0.0.0", 5000), AIHandler)
    print("AI bridge running on port 5000")
    server.serve_forever()
