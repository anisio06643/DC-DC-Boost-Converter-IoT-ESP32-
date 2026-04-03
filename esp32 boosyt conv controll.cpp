#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- OLED Display Configuration ---
// Using I2C: SCL=Pin 8, SDA=Pin 9
// Try different constructors if one doesn't work:
// Option 1: Hardware I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 8, /* data=*/ 9);
// Option 2: If above doesn't work, try software I2C:
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 8, /* data=*/ 9, /* reset=*/ U8X8_PIN_NONE);

// --- Hardware Configuration ---
namespace Hardware {
    const int pwmPin = 6;       
    const int enablePin = 5;    
    const int pwmChannel = 0;   
    const int resolution = 10;  
    const int rpmSensorPin = 3;      // FC-15 IR Sensor (Digital/Analog)
    const int currentSensePin = 1;   // Current sensing via MCP6002
    const int vOutSensePin = 4;      // Output Voltage sensing

    // VOUT Voltage Divider Constants (Pin 4)
    const float VOUT_R1 = 57500.0;  // 57.5k ohm
    const float VOUT_R2 = 10000.0;  // 10k ohm
    const float V_REF = 3.3;        // ESP32 ADC Reference
    const float VOUT_VOLT_DIV_RATIO = (VOUT_R1 + VOUT_R2) / VOUT_R2; // 6.75
    
    // Current Sense Amplifier Configuration (MCP6002)
    // Gain = 1 + (R2/R1) = 1 + (57k/10k) = 6.7
    const float CURRENT_AMP_R1 = 10000.0;   // 10k ohm
    const float CURRENT_AMP_R2 = 57000.0;   // 57k ohm
    const float CURRENT_AMP_GAIN = 1.0 + (CURRENT_AMP_R2 / CURRENT_AMP_R1); // 6.7
    const float SHUNT_RESISTOR = 0.1;       // 0.1 Ohm shunt resistor
    
    const int ABSOLUTE_MAX_DUTY = 972; // 95% of 1023
    
    // RPM Sensing Configuration
    const unsigned long RPM_DEBOUNCE_US = 1000;  // 1ms debounce time
    const unsigned long RPM_TIMEOUT_MS = 2000;   // No pulse for 2s = 0 RPM
    
    // Noise filtering thresholds
    const float MIN_CURRENT_THRESHOLD = 0.05;  // Ignore readings below 50mA
    const float MIN_VOLTAGE_THRESHOLD = 0.5;   // Ignore voltages below 0.5V
}

// --- System State ---
namespace State {
    const int currentFreq = 35000;   // Locked to 35kHz
    int manualDutyRaw = 0;      // 0-1023 PWM value
    int targetDutyRaw = 0;      
    int currentDutyRaw = 0;     
    bool powerEnabled = false;  
    float lastVout = 0.0;
    float lastCurrent = 0.0;
    volatile unsigned long lastRpmPulse = 0;
    volatile unsigned long rpmPulseInterval = 0;
    volatile bool newRpmPulse = false;
    int currentRPM = 0;
    
    // Calibration offsets (saved to flash)
    float voutOffset = 0.0;
    float currentOffset = 0.0;
}

// --- Network Configuration ---
const bool ConnectionType = true;  // false = AP Mode, true = WiFi Station Mode
const char* ssid = ConnectionType ? "Idoom 4G_E1BFE" : "BCDCControl";
const char* password = ConnectionType ? "90526052" : "";

// AP Mode Configuration (used when ConnectionType = false)
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// --- RPM Interrupt Handler ---
void IRAM_ATTR rpmPulseISR() {
    unsigned long currentTime = micros();
    unsigned long timeSinceLastPulse = currentTime - State::lastRpmPulse;
    
    // Debouncing: ignore pulses that come too quickly
    if (timeSinceLastPulse > Hardware::RPM_DEBOUNCE_US) {
        State::rpmPulseInterval = timeSinceLastPulse;
        State::lastRpmPulse = currentTime;
        State::newRpmPulse = true;
    }
}

// --- OLED Display Update Function - Clean Organized Layout ---
void updateOLED() {
    u8g2.clearBuffer();
    
    // === ROW 1: Title + Power Status ===
    u8g2.setFont(u8g2_font_helvB12_tr);  // Bold 12pt
    u8g2.drawStr(5, 12, "BCDC");
    
    // Power indicator (top right)
    u8g2.setFont(u8g2_font_helvB10_tr);
    if (State::powerEnabled) {
        u8g2.drawStr(105, 12, "[ON]");
    } else {
        u8g2.drawStr(102, 12, "[OFF]");
    }
    
    u8g2.drawLine(0, 15, 128, 15);  // Separator
    
    // === ROW 2: Voltage and Current - Large Display ===
    u8g2.setFont(u8g2_font_helvB14_tr);  // Large 14pt for values
    
    // VOUT (Left)
    char voutStr[10];
    dtostrf(State::lastVout, 4, 1, voutStr);
    u8g2.drawStr(2, 36, voutStr);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(42, 33, "V");
    u8g2.drawStr(2, 42, "VOUT");
    
    // IOUT (Right)
    u8g2.setFont(u8g2_font_helvB14_tr);
    char currentStr[10];
    dtostrf(State::lastCurrent, 4, 1, currentStr);
    u8g2.drawStr(75, 36, currentStr);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(115, 33, "A");
    u8g2.drawStr(75, 42, "IOUT");
    
    u8g2.drawLine(0, 45, 128, 45);  // Separator
    
    // === ROW 3: RPM and Duty Cycle ===
    u8g2.setFont(u8g2_font_helvB10_tr);
    
    // RPM (Left)
    char rpmStr[8];
    sprintf(rpmStr, "%d", State::currentRPM);
    u8g2.drawStr(2, 57, rpmStr);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(2, 64, "RPM");
    
    // Duty Cycle (Center-Right)
    u8g2.setFont(u8g2_font_helvB10_tr);
    int dutyPercent = (int)(State::currentDutyRaw / 10.23);
    char dutyStr[8];
    sprintf(dutyStr, "%d%%", dutyPercent);
    u8g2.drawStr(60, 57, dutyStr);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(60, 64, "DUTY");
    
    // Mini PWM bar (Right) - small visual indicator
    int barWidth = 20;
    int barHeight = 8;
    int fillWidth = (dutyPercent * barWidth) / 100;
    u8g2.drawFrame(102, 50, barWidth, barHeight);  // Outline
    if (fillWidth > 0) {
        u8g2.drawBox(102, 50, fillWidth, barHeight);  // Filled portion
    }
    
    u8g2.sendBuffer();
}

// --- Clean Simplified UI ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>BCDC Controller</title>
  <style>
    * { 
      margin: 0; 
      padding: 0; 
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }
    body { 
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; 
      background: #0a0e27;
      color: #e8edf4; 
      min-height: 100vh;
      padding: 15px;
      overflow-x: hidden;
      touch-action: pan-y;
    }
    .header {
      text-align: center;
      margin-bottom: 20px;
    }
    .header h1 {
      font-size: 1.8rem;
      font-weight: 700;
      color: #00d9ff;
      margin-bottom: 5px;
    }
    .container {
      max-width: 600px;
      margin: 0 auto;
    }
    .card { 
      background: #151b3d;
      padding: 20px;
      border-radius: 12px;
      border: 1px solid rgba(255,255,255,0.1);
      margin-bottom: 15px;
    }
    .card.active {
      border-color: #2ed573;
      box-shadow: 0 0 20px rgba(46, 213, 115, 0.3);
    }
    
    /* PWM Graph */
    canvas {
      width: 100%;
      height: 120px;
      border-radius: 8px;
      background: rgba(0,0,0,0.4);
      display: block;
      touch-action: none;
    }
    
    /* Measurements */
    .measurements {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin: 15px 0;
    }
    .measure {
      text-align: center;
      padding: 15px 10px;
      background: rgba(0,0,0,0.3);
      border-radius: 8px;
      border: 2px solid rgba(255,255,255,0.1);
    }
    .measure-label {
      font-size: 0.7rem;
      text-transform: uppercase;
      color: #8b92b0;
      margin-bottom: 5px;
      font-weight: 600;
    }
    .measure-value {
      font-size: 1.8rem;
      font-weight: 700;
      color: #00d9ff;
    }
    .measure:nth-child(2) .measure-value {
      color: #ffa502;
    }
    .measure:nth-child(3) .measure-value {
      color: #2ed573;
    }
    .measure-unit {
      font-size: 1rem;
      opacity: 0.7;
      margin-left: 2px;
    }
    
    /* Power Switch */
    .power-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 15px;
      background: rgba(0,0,0,0.3);
      border-radius: 10px;
      margin-bottom: 15px;
    }
    .power-label {
      font-weight: 600;
      font-size: 1rem;
      color: #ff4757;
      text-transform: uppercase;
    }
    .toggle {
      position: relative;
      width: 60px;
      height: 32px;
    }
    .toggle input {
      opacity: 0;
      width: 0;
      height: 0;
    }
    .slider {
      position: absolute;
      cursor: pointer;
      top: 0; left: 0; right: 0; bottom: 0;
      background-color: #34495e;
      transition: .3s;
      border-radius: 32px;
    }
    .slider:before {
      position: absolute;
      content: "";
      height: 24px;
      width: 24px;
      left: 4px;
      bottom: 4px;
      background-color: white;
      transition: .3s;
      border-radius: 50%;
    }
    input:checked + .slider {
      background-color: #2ed573;
    }
    input:checked + .slider:before {
      transform: translateX(28px);
    }
    
    /* Duty Cycle Control */
    .control-group {
      margin-bottom: 15px;
    }
    .control-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 10px;
    }
    .control-label {
      font-size: 0.85rem;
      text-transform: uppercase;
      color: #8b92b0;
      font-weight: 600;
    }
    .control-value {
      font-size: 1.2rem;
      font-weight: 700;
      color: #00d9ff;
    }
    input[type=range] {
      -webkit-appearance: none;
      width: 100%;
      height: 8px;
      border-radius: 4px;
      background: rgba(255,255,255,0.1);
      outline: none;
      touch-action: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: #00d9ff;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0,217,255,0.5);
    }
    input[type=range]::-moz-range-thumb {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: #00d9ff;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0,217,255,0.5);
      border: none;
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>⚡ BCDC Controller</h1>
  </div>

  <div class="container">
    <div class="card" id="mainCard">
      <!-- PWM Graph on Top -->
      <canvas id="pwmCanvas"></canvas>
      
      <!-- Measurements: VOUT, IOUT, RPM -->
      <div class="measurements">
        <div class="measure">
          <div class="measure-label">VOUT</div>
          <div class="measure-value">
            <span id="vOutDisp">0.00</span><span class="measure-unit">V</span>
          </div>
        </div>
        <div class="measure">
          <div class="measure-label">IOUT</div>
          <div class="measure-value">
            <span id="iOutDisp">0.00</span><span class="measure-unit">A</span>
          </div>
        </div>
        <div class="measure">
          <div class="measure-label">RPM</div>
          <div class="measure-value">
            <span id="rpmDisp">0</span>
          </div>
        </div>
      </div>
      
      <!-- Power Switch -->
      <div class="power-row">
        <span class="power-label">Power</span>
        <label class="toggle">
          <input type="checkbox" id="pwrToggle" onchange="togglePower()">
          <span class="slider"></span>
        </label>
      </div>
      
      <!-- Duty Cycle Control -->
      <div class="control-group">
        <div class="control-header">
          <span class="control-label">Duty Cycle</span>
          <span class="control-value" id="dPct">0%</span>
        </div>
        <input type="range" id="dSlide" min="0" max="95" value="0" oninput="updateDuty(this.value)">
      </div>
    </div>
  </div>

<script>
  let isPwr = false, currentDuty = 0;
  const canvas = document.getElementById('pwmCanvas');
  const ctx = canvas.getContext('2d');
  
  // Set canvas size
  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * window.devicePixelRatio;
    canvas.height = 120 * window.devicePixelRatio;
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
  }
  resizeCanvas();

  function draw() {
    const w = canvas.offsetWidth;
    const h = 120;
    ctx.clearRect(0, 0, w, h);
    
    const color = isPwr ? '#00d9ff' : '#34495e';
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.shadowBlur = isPwr ? 10 : 0;
    ctx.shadowColor = color;
    
    const pulses = 8;
    const pulseWidth = w / pulses;
    const highWidth = (currentDuty / 100) * pulseWidth;
    
    ctx.beginPath();
    for (let i = 0; i < pulses; i++) {
      const x = i * pulseWidth;
      ctx.moveTo(x, h - 20);
      ctx.lineTo(x, 20);
      ctx.lineTo(x + highWidth, 20);
      ctx.lineTo(x + highWidth, h - 20);
      ctx.lineTo(x + pulseWidth, h - 20);
    }
    ctx.stroke();
    
    requestAnimationFrame(draw);
  }

  function togglePower() {
    isPwr = document.getElementById('pwrToggle').checked;
    document.getElementById('mainCard').classList.toggle('active', isPwr);
    fetch(`/power?en=${isPwr ? 1 : 0}`);
  }

  function updateDuty(v) {
    const raw = Math.round(v * 10.23);
    fetch(`/set?d=${raw}`);
  }

  setInterval(() => {
    fetch('/status').then(r => r.json()).then(d => {
      document.getElementById('vOutDisp').innerText = d.vout.toFixed(2);
      document.getElementById('iOutDisp').innerText = d.current.toFixed(2);
      document.getElementById('rpmDisp').innerText = d.rpm;
      document.getElementById('dPct').innerText = d.d + '%';
      document.getElementById('pwrToggle').checked = d.pwr == 1;
      
      isPwr = (d.pwr == 1);
      currentDuty = d.d;
      
      document.getElementById('mainCard').classList.toggle('active', isPwr);
    });
  }, 250);

  draw();
</script>
</body>
</html>
)rawliteral";

// --- Read Output Voltage from Pin 4 with Offset and Noise Filter ---
float readVoltageOut() {
    long rawSum = 0;
    for (int i = 0; i < 50; i++) {  // Increased averaging
        rawSum += analogRead(Hardware::vOutSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 50.0;
    float pinVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    float realVolts = pinVolts * Hardware::VOUT_VOLT_DIV_RATIO;
    
    // Apply calibration offset
    float calibratedVolts = realVolts + State::voutOffset;
    
    // Noise filter - set to 0 if below threshold
    if (calibratedVolts < Hardware::MIN_VOLTAGE_THRESHOLD) {
        return 0.0;
    }
    
    return calibratedVolts;
}

// --- Read Current from Pin 1 with Improved Noise Filtering ---
float readCurrent() {
    long rawSum = 0;
    for (int i = 0; i < 50; i++) {  // Increased averaging
        rawSum += analogRead(Hardware::currentSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 50.0;
    float ampOutVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    
    // Calculate shunt voltage: Vshunt = Vout / Gain
    float shuntVolts = ampOutVolts / Hardware::CURRENT_AMP_GAIN;
    
    // Calculate current: I = Vshunt / Rshunt
    float current = shuntVolts / Hardware::SHUNT_RESISTOR;
    
    // Apply calibration offset
    float calibratedCurrent = current + State::currentOffset;
    
    // Noise filter - set to 0 if below threshold
    if (abs(calibratedCurrent) < Hardware::MIN_CURRENT_THRESHOLD) {
        return 0.0;
    }
    
    // Clamp negative values to 0 (shouldn't happen with proper wiring)
    if (calibratedCurrent < 0) {
        return 0.0;
    }
    
    return calibratedCurrent;
}

// --- Calculate RPM from pulse interval ---
void updateRPM() {
    // Check for timeout (no pulses)
    if (millis() - (State::lastRpmPulse / 1000) > Hardware::RPM_TIMEOUT_MS) {
        State::currentRPM = 0;
        return;
    }
    
    // Calculate RPM if we have a valid pulse interval
    if (State::newRpmPulse && State::rpmPulseInterval > 0) {
        // RPM = 60,000,000 / interval_in_microseconds (for 1 pulse per revolution)
        // Adjust multiplier based on your sensor's pulses per revolution
        State::currentRPM = 60000000UL / State::rpmPulseInterval;
        State::newRpmPulse = false;
    }
}

// --- HTTP Handlers ---
void handleStatus() {
    State::lastVout = readVoltageOut();
    State::lastCurrent = readCurrent();
    updateRPM();
    
    if (!State::powerEnabled) {
        State::targetDutyRaw = 0;
    } else {
        State::targetDutyRaw = State::manualDutyRaw;
    }
    
    String json = "{\"vout\":" + String(State::lastVout, 2) + 
                  ",\"current\":" + String(State::lastCurrent, 2) + 
                  ",\"rpm\":" + String(State::currentRPM) +
                  ",\"d\":" + String((int)(State::currentDutyRaw / 10.23)) + 
                  ",\"raw\":" + String(State::currentDutyRaw) +
                  ",\"pwr\":" + String(State::powerEnabled ? 1 : 0) + "}";
    server.send(200, "application/json", json);
}

void handleCalibrate() {
    if (server.hasArg("vout")) {
        State::voutOffset = server.arg("vout").toFloat();
        preferences.putFloat("voutOffset", State::voutOffset);
    }
    if (server.hasArg("current")) {
        State::currentOffset = server.arg("current").toFloat();
        preferences.putFloat("currentOffset", State::currentOffset);
    }
    
    Serial.printf("Calibration saved: VOUT offset=%.2fV, Current offset=%.2fA\n", 
                  State::voutOffset, State::currentOffset);
    
    server.send(200, "text/plain", "OK");
}

void handleGetOffsets() {
    String json = "{\"vout\":" + String(State::voutOffset, 2) + 
                  ",\"current\":" + String(State::currentOffset, 2) + "}";
    server.send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    
    // Initialize OLED display
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(5, 10, "BCDC");
    u8g2.drawStr(5, 20, "CONTROLLER");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(15, 35, "Connecting...");
    u8g2.sendBuffer();
    delay(1500);
    
    // Load calibration from flash
    preferences.begin("bcdc", false);
    State::voutOffset = preferences.getFloat("voutOffset", 0.0);
    State::currentOffset = preferences.getFloat("currentOffset", 0.0);
    
    Serial.printf("Loaded calibration: VOUT offset=%.2fV, Current offset=%.2fA\n", 
                  State::voutOffset, State::currentOffset);
    
    // Setup RPM sensor interrupt on Pin 3
    pinMode(Hardware::rpmSensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Hardware::rpmSensorPin), rpmPulseISR, FALLING);
    
    pinMode(Hardware::enablePin, OUTPUT);
    digitalWrite(Hardware::enablePin, LOW);

    ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
    ledcAttachPin(Hardware::pwmPin, Hardware::pwmChannel);
    ledcWrite(Hardware::pwmChannel, 0);

    // WiFi Configuration
    if (ConnectionType) {
        // Station Mode - Connect to Router
        Serial.println("WiFi Mode: STATION (Connecting to Router)");
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n✓ Connected to Router!");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
        } else {
            Serial.println("\n✗ Failed to connect to router. Check credentials.");
        }
    } else {
        // AP Mode - Create Access Point
        Serial.println("WiFi Mode: ACCESS POINT (AP Mode)");
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(local_IP, gateway, subnet);
        WiFi.softAP(ssid);
        dnsServer.start(53, "*", local_IP);
        Serial.print("AP SSID: ");
        Serial.println(ssid);
        Serial.print("AP IP: ");
        Serial.println(local_IP);
    }

    server.on("/", []() { 
        server.send(200, "text/html", index_html); 
    });
    
    server.on("/status", handleStatus);
    server.on("/calibrate", handleCalibrate);
    server.on("/get_offsets", handleGetOffsets);
    
    server.on("/power", []() {
        if (server.hasArg("en")) {
            State::powerEnabled = (server.arg("en") == "1");
            digitalWrite(Hardware::enablePin, State::powerEnabled ? HIGH : LOW);
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/set", []() {
        if (server.hasArg("d")) {
            State::manualDutyRaw = constrain(server.arg("d").toInt(), 0, Hardware::ABSOLUTE_MAX_DUTY);
        }
        server.send(200, "text/plain", "OK");
    });

    server.onNotFound([]() {
        server.sendHeader("Location", ConnectionType ? "/" : "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
    
    Serial.println("=== BCDC Controller Started ===");
    Serial.println("Frequency: LOCKED at 35kHz");
    Serial.println("VOUT (Pin 4): 10k / (10k + 57.5k) - Multiplier: 6.75");
    Serial.println("Current Sense (Pin 1): MCP6002 Gain 6.7, Rshunt 0.1Ω");
    Serial.println("RPM Sensor (Pin 3): FC-15 IR with interrupt");
    Serial.println("OLED Display: SCL=Pin 8, SDA=Pin 9");
    Serial.println("Calibration offsets loaded from flash");
}

void loop() {
    dnsServer.processNextRequest();
    server.handleClient();

    // Update OLED display every 150ms (smoother updates)
    static unsigned long lastOLEDUpdate = 0;
    if (millis() - lastOLEDUpdate > 150) {
        State::lastVout = readVoltageOut();
        State::lastCurrent = readCurrent();
        updateRPM();
        updateOLED();
        lastOLEDUpdate = millis();
    }

    // Debug output every 1 second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.printf("VOUT: %.2fV | Current: %.2fA | RPM: %d | Duty: %d%% | Power: %.2fW\n", 
                      State::lastVout, State::lastCurrent, State::currentRPM, 
                      (int)(State::currentDutyRaw / 10.23),
                      State::lastVout * State::lastCurrent);
        lastDebug = millis();
    }

    // Smooth PWM ramping
    static unsigned long lastRamp = 0;
    if (millis() - lastRamp > 2) {
        if (State::currentDutyRaw < State::targetDutyRaw) {
            State::currentDutyRaw++;
        } else if (State::currentDutyRaw > State::targetDutyRaw) {
            State::currentDutyRaw--;
        }
        
        ledcWrite(Hardware::pwmChannel, State::currentDutyRaw);
        lastRamp = millis();
    }
}
