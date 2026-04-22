#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <signal.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

#define PORT          8081
#define AI_BRIDGE_PORT 5000
#define STREAM_PORT   8080

// CAN IDs
#define CAN_ID_DRIVE_CMD 0x1F0
#define CAN_ID_ESTOP     0x103

// Commands
#define CMD_FORWARD  0x01
#define CMD_BACKWARD 0x02
#define CMD_LEFT     0x03
#define CMD_RIGHT    0x04
#define CMD_STOP     0x05

static int can_sock = -1;
static uint8_t seq = 0;
static std::mutex can_mutex;

// ── CAN init ──────────────────────────────────────────────────
int can_init(const char *ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock < 0) return -1;

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(can_sock, SIOCGIFINDEX, &ifr) < 0) return -1;

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) return -1;

    return 0;
}

int can_send_drive(uint8_t cmd, uint8_t speed) {
    if (can_sock < 0) return -1;
    std::lock_guard<std::mutex> lock(can_mutex);

    struct can_frame frame = {0};
    // Convert direction to left/right percentages for ESP32
    int8_t left = 0, right = 0;
    int8_t pct = (int8_t)((speed * 100) / 255);
    switch(cmd) {
        case 0x01: left =  pct; right =  pct; break; // FORWARD
        case 0x02: left =  pct; right =  pct; break; // BACKWARD - same as forward for now
        case 0x03: left = -pct; right =  pct; break;
        case 0x04: left =  pct; right = -pct; break;
        default:   left =  0;   right =  0;   break;
    }
    frame.can_id  = CAN_ID_DRIVE_CMD;
    frame.can_dlc = 8;
    frame.data[0] = (cmd == 0x05) ? 0x00 : 0x01;
    frame.data[1] = 0x01;
    frame.data[2] = (uint8_t)left;
    frame.data[3] = (uint8_t)right;
    frame.data[4] = seq++;
    frame.data[5] = (cmd == 0x05) ? 0x00 : 0x01;
    frame.data[6] = 0;
    frame.data[7] = 0;

    return write(can_sock, &frame, sizeof(frame));
}

// ── AI bridge call ────────────────────────────────────────────
std::string call_ai_bridge(const std::string& question) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(AI_BRIDGE_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(sock);
        return "{\"error\":\"AI bridge not available\"}";
    }

    std::string escaped = question;
    size_t pos = 0;
    while ((pos = escaped.find('"', pos)) != std::string::npos) {
        escaped.replace(pos, 1, "\\\"");
        pos += 2;
    }

    std::string body = "{\"question\":\"" + escaped + "\"}";
    std::string request =
        "POST /ask HTTP/1.1\r\n"
        "Host: localhost\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: " + std::to_string(body.size()) + "\r\n"
        "Connection: close\r\n\r\n" + body;

    send(sock, request.c_str(), request.size(), 0);

    std::string response;
    char buf[4096];
    int n;
    while ((n = recv(sock, buf, sizeof(buf)-1, 0)) > 0) {
        buf[n] = 0;
        response += buf;
    }
    close(sock);

    size_t json_start = response.find("\r\n\r\n");
    if (json_start != std::string::npos)
        return response.substr(json_start + 4);
    return "{\"error\":\"invalid response\"}";
}

// ── Camera stream proxy ───────────────────────────────────────
void proxy_stream(int client_fd) {
    int cam_sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(STREAM_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(cam_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        const char* err = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n";
        send(client_fd, err, strlen(err), 0);
        close(cam_sock);
        return;
    }

    const char* req = "GET / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n";
    send(cam_sock, req, strlen(req), 0);

    char buf[65536];
    int n;
    while ((n = recv(cam_sock, buf, sizeof(buf), 0)) > 0) {
        if (send(client_fd, buf, n, 0) < 0) break;
    }
    close(cam_sock);
}

// ── HTML Page ─────────────────────────────────────────────────
const std::string HTML_PAGE = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>AI Rover</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: Arial, sans-serif; background: #0d1117; color: #e6edf3; height: 100vh; display: flex; flex-direction: column; }
        header { background: #161b22; padding: 10px 20px; border-bottom: 1px solid #30363d; display: flex; align-items: center; gap: 16px; }
        header h1 { font-size: 18px; color: #58a6ff; flex: 1; }
        .mode-btn { padding: 8px 20px; border: none; border-radius: 6px; font-size: 14px; font-weight: bold; cursor: pointer; }
        .mode-btn.manual { background: #238636; color: white; }
        .mode-btn.auto   { background: #1f6feb; color: white; }
        .mode-label { font-size: 13px; color: #8b949e; }
        .main { display: flex; flex: 1; overflow: hidden; }
        .camera-panel { flex: 1; background: #000; border-right: 1px solid #30363d; display: flex; align-items: center; justify-content: center; }
        .camera-panel img { width: 100%; height: 100%; object-fit: contain; }
        .right-panel { width: 380px; display: flex; flex-direction: column; background: #161b22; }

        /* Manual controls */
        .manual-controls { padding: 16px; border-bottom: 1px solid #30363d; }
        .manual-controls h3 { font-size: 13px; color: #8b949e; margin-bottom: 12px; }
        .dpad { display: grid; grid-template-columns: repeat(3, 60px); grid-template-rows: repeat(3, 60px); gap: 6px; justify-content: center; }
        .dpad-btn { width: 60px; height: 60px; border: none; border-radius: 8px; background: #21262d; color: #e6edf3; font-size: 20px; cursor: pointer; border: 1px solid #30363d; }
        .dpad-btn:active { background: #1f6feb; }
        .dpad-btn.stop { background: #da3633; }
        .dpad-btn.stop:active { background: #f85149; }
        .dpad-empty { width: 60px; height: 60px; }
        .speed-row { display: flex; align-items: center; gap: 10px; margin-top: 12px; }
        .speed-row label { font-size: 13px; color: #8b949e; }
        .speed-row input { flex: 1; }
        .speed-row span { font-size: 13px; min-width: 30px; }

        /* Chat */
        .chat-messages { flex: 1; overflow-y: auto; padding: 12px; display: flex; flex-direction: column; gap: 10px; }
        .msg { padding: 8px 12px; border-radius: 8px; max-width: 90%; line-height: 1.5; font-size: 13px; white-space: pre-wrap; }
        .msg.user   { background: #1f6feb; align-self: flex-end; }
        .msg.ai     { background: #21262d; border: 1px solid #30363d; align-self: flex-start; }
        .msg.system { background: #1a2a1a; border: 1px solid #238636; align-self: center; font-size: 11px; color: #3fb950; }
        .msg.cmd    { background: #2d1f3d; border: 1px solid #8957e5; align-self: center; font-size: 11px; color: #d2a8ff; }
        .chat-input { padding: 10px; border-top: 1px solid #30363d; display: flex; gap: 6px; }
        .chat-input input { flex: 1; padding: 8px 12px; background: #0d1117; border: 1px solid #30363d; border-radius: 6px; color: #e6edf3; font-size: 13px; outline: none; }
        .chat-input input:focus { border-color: #58a6ff; }
        .chat-input button { padding: 8px 14px; background: #1f6feb; border: none; border-radius: 6px; color: white; font-size: 13px; cursor: pointer; }
        .chat-input button:disabled { background: #30363d; cursor: not-allowed; }
        .thinking { display: none; padding: 6px 12px; color: #8b949e; font-size: 12px; font-style: italic; }
    </style>
</head>
<body>
<header>
    <h1>🤖 AI Rover</h1>
    <span class="mode-label" id="modeLabel">Mode: MANUAL</span>
    <button class="mode-btn manual" id="modeBtn" onclick="toggleMode()">Switch to AUTO</button>
</header>
<div class="main">
    <div class="camera-panel">
        <img src="/stream" alt="Camera Feed" />
    </div>
    <div class="right-panel">
        <!-- Manual controls -->
        <div class="manual-controls" id="manualControls">
            <h3>MANUAL CONTROL</h3>
            <div class="dpad">
                <div class="dpad-empty"></div>
                <button class="dpad-btn" onmousedown="sendCmd('FORWARD')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('FORWARD')" ontouchend="sendCmd('STOP')">▲</button>
                <div class="dpad-empty"></div>
                <button class="dpad-btn" onmousedown="sendCmd('LEFT')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('LEFT')" ontouchend="sendCmd('STOP')">◀</button>
                <button class="dpad-btn stop" onclick="sendCmd('STOP')">■</button>
                <button class="dpad-btn" onmousedown="sendCmd('RIGHT')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('RIGHT')" ontouchend="sendCmd('STOP')">▶</button>
                <div class="dpad-empty"></div>
                <button class="dpad-btn" onmousedown="sendCmd('BACKWARD')" onmouseup="sendCmd('STOP')" ontouchstart="sendCmd('BACKWARD')" ontouchend="sendCmd('STOP')">▼</button>
                <div class="dpad-empty"></div>
            </div>
            <div class="speed-row">
                <label>Speed</label>
                <input type="range" min="50" max="255" value="150" id="speedSlider" oninput="document.getElementById('speedVal').textContent=this.value">
                <span id="speedVal">150</span>
            </div>
        </div>

        <!-- Chat -->
        <div class="chat-messages" id="messages">
            <div class="msg system">Rover ready. Manual mode active.</div>
        </div>
        <div class="thinking" id="thinking">Gemini is thinking...</div>
        <div class="chat-input">
            <input type="text" id="input" placeholder="Ask about what you see..."
                   onkeypress="if(event.key==='Enter') sendQuestion()"/>
            <button id="askBtn" onclick="sendQuestion()">Ask</button>
        </div>
    </div>
</div>
<script>
let autoMode = false;
let autoInterval = null;

function toggleMode() {
    autoMode = !autoMode;
    const btn = document.getElementById('modeBtn');
    const label = document.getElementById('modeLabel');
    const controls = document.getElementById('manualControls');

    if (autoMode) {
        btn.textContent = 'Switch to MANUAL';
        btn.className = 'mode-btn auto';
        label.textContent = 'Mode: AUTO';
        controls.style.opacity = '0.4';
        controls.style.pointerEvents = 'none';
        addMessage('AUTO mode activated - Gemini is navigating', 'system');
        startAutoMode();
    } else {
        btn.textContent = 'Switch to AUTO';
        btn.className = 'mode-btn manual';
        label.textContent = 'Mode: MANUAL';
        controls.style.opacity = '1';
        controls.style.pointerEvents = 'auto';
        stopAutoMode();
        sendCmd('STOP');
        addMessage('MANUAL mode activated', 'system');
    }
}

function startAutoMode() {
    runAutoStep();
    autoInterval = setInterval(runAutoStep, 10000);
}

function stopAutoMode() {
    if (autoInterval) { clearInterval(autoInterval); autoInterval = null; }
}

function runAutoStep() {
    fetch('/auto', { method: 'POST' })
    .then(r => r.json())
    .then(data => {
        if (data.cmd) addMessage('AUTO: ' + data.cmd + (data.answer ? ' — ' + data.answer.substring(0,80) : ''), 'cmd');
    })
    .catch(e => console.error('Auto error:', e));
}

function sendCmd(direction) {
    if (autoMode) return;
    const speed = document.getElementById('speedSlider').value;
    fetch('/cmd', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({direction: direction, speed: parseInt(speed)})
    });
}

function sendQuestion() {
    const input = document.getElementById('input');
    const question = input.value.trim();
    if (!question) return;
    addMessage(question, 'user');
    input.value = '';
    document.getElementById('askBtn').disabled = true;
    document.getElementById('thinking').style.display = 'block';
    fetch('/ask', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({question: question})
    })
    .then(r => r.json())
    .then(data => { addMessage(data.answer || data.error, 'ai'); })
    .catch(e => { addMessage('Error: ' + e.message, 'ai'); })
    .finally(() => {
        document.getElementById('askBtn').disabled = false;
        document.getElementById('thinking').style.display = 'none';
    });
}

function addMessage(text, type) {
    const msgs = document.getElementById('messages');
    const div = document.createElement('div');
    div.className = 'msg ' + type;
    div.textContent = text;
    msgs.appendChild(div);
    msgs.scrollTop = msgs.scrollHeight;
}
</script>
</body>
</html>
)HTML";

// ── handler HTTP ────────────────────────────────�
void handle_client(int client_fd) {
    char buf[4096] = {};
    recv(client_fd, buf, sizeof(buf)-1, 0);
    std::string request(buf);

    std::string method, path;
    std::istringstream ss(request);
    ss >> method >> path;

    if (method == "GET" && path == "/") {
        std::string response =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: " + std::to_string(HTML_PAGE.size()) + "\r\n\r\n" + HTML_PAGE;
        send(client_fd, response.c_str(), response.size(), 0);

    } else if (method == "GET" && path == "/stream") {
        proxy_stream(client_fd);

    } else if (method == "POST" && path == "/cmd") {
        // Manual drive command
        size_t body_start = request.find("\r\n\r\n");
        std::string body = (body_start != std::string::npos) ? request.substr(body_start + 4) : "";

        uint8_t cmd = CMD_STOP;
        uint8_t speed = 150;

        // Parse direction
        if (body.find("FORWARD")  != std::string::npos) cmd = CMD_FORWARD;
        else if (body.find("BACKWARD") != std::string::npos) cmd = CMD_BACKWARD;
        else if (body.find("LEFT")     != std::string::npos) cmd = CMD_LEFT;
        else if (body.find("RIGHT")    != std::string::npos) cmd = CMD_RIGHT;
        else cmd = CMD_STOP;

        // Parse speed
        size_t sp = body.find("\"speed\":");
        if (sp != std::string::npos) speed = (uint8_t)std::stoi(body.substr(sp + 8));

        can_send_drive(cmd, speed);

        const char* ok = "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nOK";
        send(client_fd, ok, strlen(ok), 0);

    } else if (method == "POST" && path == "/auto") {
        // Auto mode — ask AI and send command
        std::string ai_response = call_ai_bridge(
            "You are controlling a rover. Based on the camera feed, "
            "decide the next movement. Reply with ONLY one word: "
            "FORWARD, BACKWARD, LEFT, RIGHT, or STOP."
        );

        uint8_t cmd = CMD_STOP;
        if (ai_response.find("FORWARD")  != std::string::npos) cmd = CMD_FORWARD;
        else if (ai_response.find("BACKWARD") != std::string::npos) cmd = CMD_BACKWARD;
        else if (ai_response.find("LEFT")     != std::string::npos) cmd = CMD_LEFT;
        else if (ai_response.find("RIGHT")    != std::string::npos) cmd = CMD_RIGHT;

        // Remove manual mode flag to resume rover_hybrid
        system("rm -f /tmp/rover_manual");

        can_send_drive(cmd, 150);

        const char *cmd_names[] = {"","FORWARD","BACKWARD","LEFT","RIGHT","STOP"};
        std::string result = "{\"cmd\":\"" + std::string(cmd_names[cmd < 6 ? cmd : 0]) +
                             "\",\"answer\":" + ai_response + "}";
        std::string response =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: " + std::to_string(result.size()) + "\r\n\r\n" + result;
        send(client_fd, response.c_str(), response.size(), 0);

    } else if (method == "POST" && path == "/ask") {
        size_t body_start = request.find("\r\n\r\n");
        std::string body = (body_start != std::string::npos) ? request.substr(body_start + 4) : "";

        size_t q_start = body.find("\"question\":\"");
        std::string question = "What do you see?";
        if (q_start != std::string::npos) {
            q_start += 12;
            size_t q_end = body.find("\"", q_start);
            if (q_end != std::string::npos)
                question = body.substr(q_start, q_end - q_start);
        }

        std::string ai_response = call_ai_bridge(question);
        std::string response =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: " + std::to_string(ai_response.size()) + "\r\n\r\n" + ai_response;
        send(client_fd, response.c_str(), response.size(), 0);

    } else {
        const char* not_found = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
        send(client_fd, not_found, strlen(not_found), 0);
    }

    close(client_fd);
}


static uint8_t hb_counter = 0;

void *heartbeat_thread(void *arg) {
    while (1) {
        if (can_sock >= 0) {
            struct can_frame frame = {0};
            frame.can_id  = 0x110;
            frame.can_dlc = 1;
            frame.data[0] = hb_counter++;
            std::lock_guard<std::mutex> lock(can_mutex);
            write(can_sock, &frame, sizeof(frame));
        }
        sleep(1);
    }
    return NULL;
}
int main() {
    signal(SIGPIPE, SIG_IGN);

    // Init CAN
    if (can_init("can0") == 0)
        printf("[UI] CAN interface initialized\n");
    else
        printf("[UI] CAN not available - running without motor control\n");

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    bind(server_fd, (struct sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 10);
    printf("[UI] Server running on port %d\n", PORT);
    pthread_t hb_t;
    pthread_create(&hb_t, NULL, heartbeat_thread, NULL);

    while (true) {
        int client_fd = accept(server_fd, nullptr, nullptr);
        if (client_fd >= 0)
            std::thread(handle_client, client_fd).detach();
    }
    return 0;
}
