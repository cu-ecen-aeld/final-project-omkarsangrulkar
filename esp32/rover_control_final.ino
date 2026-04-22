#include <SPI.h>
#include <mcp_can.h>

// ============================================================
// USER WIRING
// ============================================================

// MCP2515 -> ESP32
static const int CAN_CS_PIN   = 5;
static const int CAN_MISO_PIN = 12;
static const int CAN_MOSI_PIN = 13;
static const int CAN_SCK_PIN  = 14;
static const int CAN_INT_PIN  = 4;

// Ultrasonic -> ESP32
static const int US_TRIG_PIN  = 15;
static const int US_ECHO_PIN  = 2;   

// Left L298N -> ESP32
static const int L_ENA = 21;
static const int L_IN1 = 16;
static const int L_IN2 = 17;
static const int L_IN3 = 18;
static const int L_IN4 = 19;
static const int L_ENB = 22;

// Right L298N -> ESP32
static const int R_ENA = 32;
static const int R_IN1 = 23;
static const int R_IN2 = 25;
static const int R_IN3 = 26;
static const int R_IN4 = 27;
static const int R_ENB = 33;

// ============================================================
// CAN CONFIG
// ============================================================

static const uint32_t CAN_RX_ID_DRIVE       = 0x1F0;
static const uint32_t CAN_RX_ID_PI_HB       = 0x110;

static const uint32_t CAN_TX_ID_ESP_HB      = 0x120;
static const uint32_t CAN_TX_ID_ESP_STATUS  = 0x121;
static const uint32_t CAN_TX_ID_ESP_TLM     = 0x122;
static const uint32_t CAN_TX_ID_TEST        = 0x123;
static const uint32_t CAN_TX_ID_USONIC      = 0x124;

static const byte CAN_SPEED = CAN_500KBPS;
static const byte CAN_CLOCK = MCP_8MHZ;

MCP_CAN CAN0(CAN_CS_PIN);

// ============================================================
// MOTOR PWM
// ============================================================

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;
static const int PWM_MAX  = 255;

// ============================================================
// COMMANDS / FLAGS
// ============================================================

static const uint8_t CMD_STOP  = 0x00;
static const uint8_t CMD_DRIVE = 0x01;
static const uint8_t CMD_ESTOP = 0x02;

static const uint8_t MODE_OPEN_LOOP = 0x01;

static const uint8_t FLAG_ENABLE = 0x01;
static const uint8_t FLAG_ESTOP  = 0x02;

// ============================================================
// TIMING
// ============================================================

static const unsigned long HB_TX_PERIOD_MS      = 1000;
static const unsigned long STATUS_TX_PERIOD_MS  = 1000;
static const unsigned long TLM_TX_PERIOD_MS     = 500;
static const unsigned long PI_TIMEOUT_MS        = 1500;

static const unsigned long US_SAMPLE_PERIOD_MS  = 100;
static const unsigned long US_TX_PERIOD_MS      = 200;
static const unsigned long US_TIMEOUT_US        = 30000;

// ============================================================
// ULTRASONIC CONFIG
// ============================================================

static const float US_STOP_DISTANCE_CM = 20.0f;
static const uint8_t US_SAMPLE_COUNT   = 5;

// ============================================================
// STATE
// ============================================================

bool canReady = false;
bool manualMode = true;
bool estopActive = false;
bool piAlive = false;
bool debugMode = false;

int manualSpeed = 180;

int currentLeftCmd = 0;
int currentRightCmd = 0;

uint8_t lastCmd   = CMD_STOP;
uint8_t lastMode  = MODE_OPEN_LOOP;
int8_t  lastLeftPercent  = 0;
int8_t  lastRightPercent = 0;
uint8_t lastSeq   = 0;
uint8_t lastFlags = 0;
uint8_t lastRamp  = 0;
uint8_t lastDebug = 0;

float ultrasonicCm = -1.0f;
bool ultrasonicValid = false;
bool obstacleBlocked = false;

unsigned long lastPiSeenMs    = 0;
unsigned long lastHbTxMs      = 0;
unsigned long lastStatusTxMs  = 0;
unsigned long lastTlmTxMs     = 0;
unsigned long lastUsSampleMs  = 0;
unsigned long lastUsTxMs      = 0;

String serialLine;

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void stopAllMotors();

// ============================================================
// DEBUG HELPERS
// ============================================================

void dbg(const char *msg) {
  if (debugMode) {
    Serial.println(msg);
  }
}

template<typename... Args>
void dbgf(const char *fmt, Args... args) {
  if (debugMode) {
    Serial.printf(fmt, args...);
  }
}

// ============================================================
// HELPERS
// ============================================================

int clampValue(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int percentToPwm(int8_t percent) {
  int p = clampValue((int)percent, -100, 100);
  return (p * PWM_MAX) / 100;
}

void printFrame(const char *prefix, unsigned long id, byte len, const byte *buf) {
  if (!debugMode) return;

  Serial.printf("%s ID=0x%03lX LEN=%u DATA=", prefix, id, len);
  for (byte i = 0; i < len; i++) {
    Serial.printf("%02X", buf[i]);
    if (i < len - 1) Serial.print(" ");
  }
  Serial.println();
}

bool isForwardMotion(int leftCmd, int rightCmd) {
  return (leftCmd > 0 && rightCmd > 0);
}

bool shouldBlockForward(int leftCmd, int rightCmd) {
  if (!ultrasonicValid) return false;
  if (!isForwardMotion(leftCmd, rightCmd)) return false;
  return ultrasonicCm > 0.0f && ultrasonicCm <= US_STOP_DISTANCE_CM;
}

// ============================================================
// MODE CONTROL
// ============================================================

void enterManualMode(const char *reason) {
  bool changed = !manualMode;
  manualMode = true;
  piAlive = false;
  stopAllMotors();

  if (changed || debugMode) {
    dbgf("[MODE] MANUAL: %s\n", reason);
  }
}

void enterCanMode(const char *reason) {
  bool changed = manualMode;
  manualMode = false;

  if (changed || debugMode) {
    dbgf("[MODE] CAN: %s\n", reason);
  }
}

// ============================================================
// ULTRASONIC
// ============================================================

void initUltrasonic() {
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);
}

float readUltrasonicOnceCm() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(US_ECHO_PIN, HIGH, US_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUsSampleMs < US_SAMPLE_PERIOD_MS) {
    return;
  }
  lastUsSampleMs = now;

  float sum = 0.0f;
  uint8_t good = 0;

  for (uint8_t i = 0; i < US_SAMPLE_COUNT; i++) {
    float d = readUltrasonicOnceCm();
    if (d >= 2.0f && d <= 400.0f) {
      sum += d;
      good++;
    }
    delay(2);
  }

  if (good > 0) {
    ultrasonicCm = sum / good;
    ultrasonicValid = true;
  } else {
    ultrasonicCm = -1.0f;
    ultrasonicValid = false;
  }

  obstacleBlocked = ultrasonicValid && (ultrasonicCm <= US_STOP_DISTANCE_CM);

  if (debugMode) {
    if (ultrasonicValid) {
      Serial.printf("[US] %.1f cm blocked=%s\n",
                    ultrasonicCm,
                    obstacleBlocked ? "true" : "false");
    } else {
      Serial.println("[US] timeout / invalid");
    }
  }
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void motorOne(int in1, int in2, int pwmPin, int cmd) {
  int pwm = abs(cmd);
  pwm = clampValue(pwm, 0, PWM_MAX);

  if (cmd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (cmd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmPin, pwm);
}

void applyMotors(int leftCmd, int rightCmd) {
  leftCmd  = clampValue(leftCmd, -PWM_MAX, PWM_MAX);
  rightCmd = clampValue(rightCmd, -PWM_MAX, PWM_MAX);

  currentLeftCmd = leftCmd;
  currentRightCmd = rightCmd;

  motorOne(L_IN1, L_IN2, L_ENA, leftCmd);
  motorOne(L_IN3, L_IN4, L_ENB, leftCmd);

  motorOne(R_IN1, R_IN2, R_ENA, rightCmd);
  motorOne(R_IN3, R_IN4, R_ENB, rightCmd);
}

void stopAllMotors() {
  applyMotors(0, 0);
}

void setDrive(int leftCmd, int rightCmd, const char *reason) {
  if (estopActive) {
    stopAllMotors();
    dbgf("[MOTOR] Ignored, ESTOP active: %s\n", reason);
    return;
  }

  if (shouldBlockForward(leftCmd, rightCmd)) {
    stopAllMotors();
    dbgf("[SAFETY] Forward blocked by ultrasonic: %.1f cm (%s)\n", ultrasonicCm, reason);
    return;
  }

  applyMotors(leftCmd, rightCmd);
  dbgf("[MOTOR] %s -> left=%d right=%d\n", reason, currentLeftCmd, currentRightCmd);
}

void activateEstop(const char *reason) {
  estopActive = true;
  stopAllMotors();
  dbgf("[SAFETY] ESTOP ACTIVE: %s\n", reason);
}

void clearEstop(const char *reason) {
  estopActive = false;
  dbgf("[SAFETY] ESTOP CLEARED: %s\n", reason);
}

// ============================================================
// UI
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("=========== SERIAL COMMANDS ===========");
  Serial.println("help        : show help");
  Serial.println("debug       : toggle debug mode");
  Serial.println("state       : print state");
  Serial.println("manual      : force manual mode");
  Serial.println("can         : force can mode");
  Serial.println("forward     : move forward");
  Serial.println("reverse     : move reverse");
  Serial.println("left        : turn left");
  Serial.println("right       : turn right");
  Serial.println("stop        : stop motors");
  Serial.println("test        : self test");
  Serial.println("speed <n>   : set speed 0..255");
  Serial.println("us          : print ultrasonic reading");
  Serial.println("sendhb      : send heartbeat");
  Serial.println("sendstatus  : send status");
  Serial.println("sendtlm     : send telemetry");
  Serial.println("sendus      : send ultrasonic");
  Serial.println("sendall     : send all four");
  Serial.println("sendtest    : send test frame 0x123");
  Serial.println("=======================================");
  Serial.println();
}

void printState() {
  Serial.println();
  Serial.println("============= STATE =============");
  Serial.printf("canReady          : %s\n", canReady ? "true" : "false");
  Serial.printf("manualMode        : %s\n", manualMode ? "true" : "false");
  Serial.printf("debugMode         : %s\n", debugMode ? "true" : "false");
  Serial.printf("estopActive       : %s\n", estopActive ? "true" : "false");
  Serial.printf("piAlive           : %s\n", piAlive ? "true" : "false");
  Serial.printf("manualSpeed       : %d\n", manualSpeed);
  Serial.printf("currentLeftCmd    : %d\n", currentLeftCmd);
  Serial.printf("currentRightCmd   : %d\n", currentRightCmd);
  Serial.printf("lastCmd           : 0x%02X\n", lastCmd);
  Serial.printf("lastMode          : 0x%02X\n", lastMode);
  Serial.printf("lastLeftPercent   : %d\n", lastLeftPercent);
  Serial.printf("lastRightPercent  : %d\n", lastRightPercent);
  Serial.printf("lastSeq           : 0x%02X\n", lastSeq);
  Serial.printf("lastFlags         : 0x%02X\n", lastFlags);
  Serial.printf("lastRamp          : 0x%02X\n", lastRamp);
  Serial.printf("lastDebug         : 0x%02X\n", lastDebug);
  Serial.printf("ultrasonicValid   : %s\n", ultrasonicValid ? "true" : "false");
  Serial.printf("ultrasonicCm      : %.1f\n", ultrasonicCm);
  Serial.printf("obstacleBlocked   : %s\n", obstacleBlocked ? "true" : "false");
  Serial.println("================================");
  Serial.println();
}

// ============================================================
// CAN TX
// ============================================================

bool sendCanRaw(uint32_t id, byte len, byte *buf, const char *name) {
  if (!canReady) {
    dbgf("[CAN] %s not sent, CAN not ready\n", name);
    return false;
  }

  byte rc = CAN0.sendMsgBuf(id, 0, len, buf);
  if (rc == CAN_OK) {
    printFrame("[CAN TX]", id, len, buf);
    return true;
  }

  dbgf("[CAN] %s TX failed, rc=%d\n", name, rc);
  return false;
}

void sendHeartbeat() {
  byte data[8];
  data[0] = 0xEE;
  data[1] = manualMode ? 1 : 0;
  data[2] = estopActive ? 1 : 0;
  data[3] = piAlive ? 1 : 0;
  data[4] = lastSeq;
  data[5] = (byte)((int8_t)clampValue(currentLeftCmd / 2, -128, 127));
  data[6] = (byte)((int8_t)clampValue(currentRightCmd / 2, -128, 127));
  data[7] = obstacleBlocked ? 1 : 0;

  sendCanRaw(CAN_TX_ID_ESP_HB, 8, data, "HEARTBEAT");
}

void sendStatus() {
  byte data[8];
  data[0] = manualMode ? 1 : 0;
  data[1] = estopActive ? 1 : 0;
  data[2] = piAlive ? 1 : 0;
  data[3] = lastSeq;
  data[4] = lastFlags;
  data[5] = lastCmd;
  data[6] = lastMode;
  data[7] = obstacleBlocked ? 1 : 0;

  sendCanRaw(CAN_TX_ID_ESP_STATUS, 8, data, "STATUS");
}

void sendTelemetry() {
  byte data[8];
  data[0] = (byte)((int8_t)clampValue(currentLeftCmd / 2, -128, 127));
  data[1] = (byte)((int8_t)clampValue(currentRightCmd / 2, -128, 127));
  data[2] = (byte)manualSpeed;
  data[3] = lastCmd;
  data[4] = (byte)lastLeftPercent;
  data[5] = (byte)lastRightPercent;
  data[6] = lastRamp;
  data[7] = ultrasonicValid ? 1 : 0;

  sendCanRaw(CAN_TX_ID_ESP_TLM, 8, data, "TELEMETRY");
}

void sendUltrasonic() {
  byte data[8] = {0};

  uint16_t distanceMm = 0;
  if (ultrasonicValid && ultrasonicCm > 0.0f) {
    distanceMm = (uint16_t)(ultrasonicCm * 10.0f);
  }

  data[0] = (byte)(distanceMm & 0xFF);
  data[1] = (byte)((distanceMm >> 8) & 0xFF);
  data[2] = ultrasonicValid ? 1 : 0;
  data[3] = obstacleBlocked ? 1 : 0;
  data[4] = lastSeq;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x5A;

  sendCanRaw(CAN_TX_ID_USONIC, 8, data, "ULTRASONIC");
}

void sendTestCan() {
  byte data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44};
  bool ok = sendCanRaw(CAN_TX_ID_TEST, 8, data, "TEST");
  Serial.printf("[SERIAL] TEST CAN SEND = %s\n", ok ? "OK" : "FAIL");
}

// ============================================================
// CAN RX
// ============================================================

void handleDriveFrame(byte len, byte *buf) {
  if (len < 8) {
    dbg("[CAN] Drive frame too short");
    return;
  }

  lastCmd          = buf[0];
  lastMode         = buf[1];
  lastLeftPercent  = (int8_t)buf[2];
  lastRightPercent = (int8_t)buf[3];
  lastSeq          = buf[4];
  lastFlags        = buf[5];
  lastRamp         = buf[6];
  lastDebug        = buf[7];

  dbgf("[CAN] DRIVE RX -> cmd=0x%02X mode=0x%02X left=%d right=%d seq=0x%02X flags=0x%02X ramp=0x%02X dbg=0x%02X\n",
       lastCmd, lastMode, lastLeftPercent, lastRightPercent,
       lastSeq, lastFlags, lastRamp, lastDebug);

  lastPiSeenMs = millis();
  piAlive = true;
  enterCanMode("Pi drive frame detected");

  if ((lastFlags & FLAG_ESTOP) || lastCmd == CMD_ESTOP) {
    activateEstop("CAN ESTOP");
    return;
  }

  if (estopActive && lastCmd == CMD_STOP) {
    clearEstop("STOP received after ESTOP");
  }

  if ((lastFlags & FLAG_ENABLE) == 0) {
    dbg("[CAN] ENABLE flag not set -> stop");
    stopAllMotors();
    return;
  }

  if (lastCmd == CMD_STOP) {
    setDrive(0, 0, "CAN STOP");
    return;
  }

  if (lastCmd == CMD_DRIVE) {
    int leftPwm  = percentToPwm(lastLeftPercent);
    int rightPwm = percentToPwm(lastRightPercent);
    setDrive(leftPwm, rightPwm, "CAN DRIVE");
    return;
  }

  dbgf("[CAN] Unknown drive cmd 0x%02X\n", lastCmd);
}

void pollCan() {
  if (!canReady) return;

  if (digitalRead(CAN_INT_PIN) == HIGH) {
    return;
  }

  unsigned long rxId = 0;
  byte len = 0;
  byte buf[8] = {0};

  if (CAN0.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
    dbg("[CAN] Failed to read frame");
    return;
  }

  printFrame("[CAN RX]", rxId, len, buf);

  if (rxId == CAN_RX_ID_PI_HB) {
    lastPiSeenMs = millis();
    piAlive = true;
    enterCanMode("Pi heartbeat detected");
  } else if (rxId == CAN_RX_ID_DRIVE) {
    handleDriveFrame(len, buf);
  } else {
    dbgf("[CAN] Unhandled ID 0x%03lX\n", rxId);
  }
}

void updateConnectionState() {
  if (!canReady) return;

  if (!manualMode && (millis() - lastPiSeenMs) > PI_TIMEOUT_MS) {
    if (piAlive) {
      dbg("[LINK] Pi timeout detected -> stopping motors");
    }
    enterManualMode("Pi timeout");
  }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================

void runSelfTest() {
  Serial.println("[SELFTEST] Forward");
  setDrive(170, 170, "SELFTEST FORWARD");
  delay(1200);
  stopAllMotors();
  delay(400);

  Serial.println("[SELFTEST] Reverse");
  setDrive(-170, -170, "SELFTEST REVERSE");
  delay(1200);
  stopAllMotors();
  delay(400);

  Serial.println("[SELFTEST] Left");
  setDrive(-170, 170, "SELFTEST LEFT");
  delay(1000);
  stopAllMotors();
  delay(400);

  Serial.println("[SELFTEST] Right");
  setDrive(170, -170, "SELFTEST RIGHT");
  delay(1000);
  stopAllMotors();
  delay(400);

  Serial.println("[SELFTEST] Done");
}

void handleSerialLine(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "debug") {
    debugMode = !debugMode;
    Serial.printf("[SERIAL] debugMode = %s\n", debugMode ? "true" : "false");
  }
  else if (cmd == "help" || cmd == "h") {
    printHelp();
  }
  else if (cmd == "state" || cmd == "p") {
    printState();
  }
  else if (cmd == "manual") {
    enterManualMode("Serial command");
    Serial.println("[SERIAL] manualMode = true");
  }
  else if (cmd == "can") {
    if (!canReady) {
      Serial.println("[SERIAL] CAN not ready");
    } else {
      manualMode = false;
      lastPiSeenMs = millis();
      Serial.println("[SERIAL] manualMode = false");
    }
  }
  else if (cmd == "forward" || cmd == "w") {
    if (manualMode) {
      setDrive(manualSpeed, manualSpeed, "SERIAL FORWARD");
      Serial.printf("[SERIAL] FORWARD speed=%d\n", manualSpeed);
    } else {
      Serial.println("[SERIAL] Not in manual mode");
    }
  }
  else if (cmd == "reverse" || cmd == "s") {
    if (manualMode) {
      setDrive(-manualSpeed, -manualSpeed, "SERIAL REVERSE");
      Serial.printf("[SERIAL] REVERSE speed=%d\n", manualSpeed);
    } else {
      Serial.println("[SERIAL] Not in manual mode");
    }
  }
  else if (cmd == "left" || cmd == "a") {
    if (manualMode) {
      setDrive(-manualSpeed, manualSpeed, "SERIAL LEFT");
      Serial.printf("[SERIAL] LEFT speed=%d\n", manualSpeed);
    } else {
      Serial.println("[SERIAL] Not in manual mode");
    }
  }
  else if (cmd == "right" || cmd == "d") {
    if (manualMode) {
      setDrive(manualSpeed, -manualSpeed, "SERIAL RIGHT");
      Serial.printf("[SERIAL] RIGHT speed=%d\n", manualSpeed);
    } else {
      Serial.println("[SERIAL] Not in manual mode");
    }
  }
  else if (cmd == "stop" || cmd == "x") {
    stopAllMotors();
    Serial.println("[SERIAL] STOP");
  }
  else if (cmd == "test" || cmd == "t") {
    if (manualMode) {
      runSelfTest();
    } else {
      Serial.println("[SERIAL] Self-test allowed only in manual mode");
    }
  }
  else if (cmd == "us") {
    if (ultrasonicValid) {
      Serial.printf("[SERIAL] Ultrasonic = %.1f cm blocked=%s\n",
                    ultrasonicCm,
                    obstacleBlocked ? "true" : "false");
    } else {
      Serial.println("[SERIAL] Ultrasonic invalid / timeout");
    }
  }
  else if (cmd == "sendhb") {
    sendHeartbeat();
    Serial.println("[SERIAL] Heartbeat sent");
  }
  else if (cmd == "sendstatus") {
    sendStatus();
    Serial.println("[SERIAL] Status sent");
  }
  else if (cmd == "sendtlm") {
    sendTelemetry();
    Serial.println("[SERIAL] Telemetry sent");
  }
  else if (cmd == "sendus") {
    sendUltrasonic();
    Serial.println("[SERIAL] Ultrasonic sent");
  }
  else if (cmd == "sendall") {
    sendHeartbeat();
    sendStatus();
    sendTelemetry();
    sendUltrasonic();
    Serial.println("[SERIAL] Heartbeat + Status + Telemetry + Ultrasonic sent");
  }
  else if (cmd == "sendtest") {
    sendTestCan();
  }
  else if (cmd.startsWith("speed ")) {
    int v = cmd.substring(6).toInt();
    manualSpeed = constrain(v, 0, 255);
    Serial.printf("[SERIAL] manualSpeed = %d\n", manualSpeed);
  }
  else {
    Serial.printf("[SERIAL] Unknown command: %s\n", cmd.c_str());
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      handleSerialLine(serialLine);
      serialLine = "";
    } else {
      serialLine += c;
    }
  }
}

// ============================================================
// INIT
// ============================================================

void initMotors() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_IN3, OUTPUT);
  pinMode(L_IN4, OUTPUT);

  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  ledcAttach(L_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(L_ENB, PWM_FREQ, PWM_RES);
  ledcAttach(R_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(R_ENB, PWM_FREQ, PWM_RES);

  stopAllMotors();
}

bool initCan() {
  dbg("[CAN] Starting SPI");
  SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN, CAN_CS_PIN);

  pinMode(CAN_INT_PIN, INPUT);

  dbg("[CAN] Initializing MCP2515");
  byte rc = CAN0.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK);
  if (rc != CAN_OK) {
    dbgf("[CAN] MCP2515 init failed, rc=%d\n", rc);
    return false;
  }

  CAN0.setMode(MCP_NORMAL);
  dbg("[CAN] MCP2515 set to NORMAL mode");
  return true;
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  initMotors();
  initUltrasonic();

  canReady = initCan();

  lastPiSeenMs = millis();
  lastHbTxMs = millis();
  lastStatusTxMs = millis();
  lastTlmTxMs = millis();
  lastUsSampleMs = 0;
  lastUsTxMs = millis();

  manualMode = true;
  piAlive = false;

  Serial.println();
  Serial.println("==================================================");
  Serial.println("ESP32 MOTOR + MCP2515 + ULTRASONIC READY");
  Serial.println("Starts in manual mode. Type 'help' for commands.");
  Serial.println("Type 'debug' to enable debug prints.");
  Serial.println("==================================================");

  Serial.printf("canReady          : %s\n", canReady ? "true" : "false");
}

void loop() {
  handleSerial();
  updateUltrasonic();
  pollCan();
  updateConnectionState();

  unsigned long now = millis();

  if (obstacleBlocked && currentLeftCmd > 0 && currentRightCmd > 0) {
    stopAllMotors();
  }

  if (canReady && piAlive) {
    if (now - lastHbTxMs >= HB_TX_PERIOD_MS) {
      lastHbTxMs = now;
      sendHeartbeat();
    }

    if (now - lastStatusTxMs >= STATUS_TX_PERIOD_MS) {
      lastStatusTxMs = now;
      sendStatus();
    }

    if (now - lastTlmTxMs >= TLM_TX_PERIOD_MS) {
      lastTlmTxMs = now;
      sendTelemetry();
    }

    if (now - lastUsTxMs >= US_TX_PERIOD_MS) {
      lastUsTxMs = now;
      sendUltrasonic();
    }
  }
}
