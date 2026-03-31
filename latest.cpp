#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// --- Hardware Configuration ---
namespace Hardware {
    const int pwmPin = 6;       
    const int enablePin = 5;    
    const int pwmChannel = 0;   
    const int resolution = 10;  
    const int vInSensePin = 3;   // Input Voltage sensing on Pin 3
    const int vOutSensePin = 1;  // Output Voltage sensing on Pin 1

    // VIN Voltage Divider Constants (Pin 3)
    const float VIN_R1 = 24800.0;   // 24.8k ohm
    const float VIN_R2 = 10000.0;   // 10k ohm
    const float V_REF = 3.3;        // ESP32 ADC Reference
    const float VIN_VOLT_DIV_RATIO = (VIN_R1 + VIN_R2) / VIN_R2; // 3.48
    
    // VOUT Voltage Divider Constants (Pin 1)
    const float VOUT_R1 = 57500.0;  // 57.5k ohm
    const float VOUT_R2 = 10000.0;  // 10k ohm
    const float VOUT_VOLT_DIV_RATIO = (VOUT_R1 + VOUT_R2) / VOUT_R2; // 6.75
    
    const int ABSOLUTE_MAX_DUTY = 972; // 95% of 1023
}

// --- System State ---
namespace State {
    int currentFreq = 100000;   // 100kHz default
    int manualDutyRaw = 0;      // 0-1023 PWM value
    int targetDutyRaw = 0;      
    int currentDutyRaw = 0;     
    bool powerEnabled = false;  
    float lastVin = 0.0;
    float lastVout = 0.0;
}

// --- Network Configuration ---
const char* ssid = "BCDCControl";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
DNSServer dnsServer;

// --- Modern Minimal UI ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>BCDC Controller</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    :root { 
      --primary: #00d9ff; 
      --bg: #0a0e27; 
      --card: #151b3d; 
      --text: #e8edf4; 
      --accent: #7c3aed;
      --danger: #ff4757;
      --success: #2ed573;
      --warning: #ffa502;
    }
    body { 
      font-family: 'Inter', 'Segoe UI', sans-serif; 
      background: linear-gradient(135deg, var(--bg) 0%, #1a1f42 100%);
      color: var(--text); 
      min-height: 100vh;
      display: flex; 
      flex-direction: column; 
      align-items: center; 
      padding: 20px;
    }
    .header {
      text-align: center;
      margin-bottom: 30px;
      padding: 20px;
    }
    .header h1 {
      font-size: 2rem;
      font-weight: 700;
      background: linear-gradient(135deg, var(--primary), var(--accent));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
      letter-spacing: 1px;
      margin-bottom: 5px;
    }
    .header p {
      color: #8b92b0;
      font-size: 0.9rem;
    }
    .container {
      width: 100%;
      max-width: 500px;
    }
    .card { 
      background: var(--card);
      padding: 25px;
      border-radius: 16px;
      border: 1px solid rgba(255,255,255,0.1);
      margin-bottom: 20px;
      box-shadow: 0 8px 32px rgba(0,0,0,0.3);
      transition: all 0.3s ease;
    }
    .card.active {
      border-color: var(--success);
      box-shadow: 0 8px 32px rgba(46, 213, 115, 0.2);
    }
    .card.disabled {
      opacity: 0.5;
    }
    .power-switch {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 15px;
      background: rgba(0,0,0,0.3);
      border-radius: 12px;
      margin-bottom: 20px;
    }
    .power-label {
      font-weight: 600;
      font-size: 0.95rem;
      color: var(--danger);
      text-transform: uppercase;
      letter-spacing: 1px;
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
      background-color: var(--success);
    }
    input:checked + .slider:before {
      transform: translateX(28px);
    }
    .voltage-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-bottom: 25px;
    }
    .voltage-display {
      text-align: center;
      padding: 25px 15px;
      background: linear-gradient(135deg, rgba(0,217,255,0.1), rgba(124,58,237,0.1));
      border-radius: 12px;
      border: 2px solid rgba(0,217,255,0.3);
    }
    .voltage-display.output {
      background: linear-gradient(135deg, rgba(255,165,2,0.1), rgba(255,71,87,0.1));
      border-color: rgba(255,165,2,0.3);
    }
    .voltage-label {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 2px;
      color: #8b92b0;
      margin-bottom: 8px;
      font-weight: 600;
    }
    .voltage-value {
      font-size: 2.2rem;
      font-weight: 700;
      background: linear-gradient(135deg, var(--primary), var(--accent));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
      line-height: 1;
    }
    .voltage-display.output .voltage-value {
      background: linear-gradient(135deg, var(--warning), var(--danger));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }
    .voltage-unit {
      font-size: 1rem;
      opacity: 0.7;
    }
    .efficiency-display {
      text-align: center;
      padding: 15px;
      background: rgba(46, 213, 115, 0.1);
      border-radius: 8px;
      margin-bottom: 20px;
      border: 1px solid rgba(46, 213, 115, 0.3);
    }
    .efficiency-label {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 2px;
      color: #8b92b0;
      margin-bottom: 5px;
    }
    .efficiency-value {
      font-size: 1.5rem;
      font-weight: 700;
      color: var(--success);
    }
    canvas {
      width: 100%;
      height: 100px;
      border-radius: 8px;
      background: rgba(0,0,0,0.3);
      margin: 20px 0;
    }
    .control-group {
      margin-bottom: 25px;
    }
    .control-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
    }
    .control-label {
      font-size: 0.85rem;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: #8b92b0;
      font-weight: 600;
    }
    .control-value {
      font-size: 1.1rem;
      font-weight: 700;
      color: var(--primary);
    }
    input[type=range] {
      -webkit-appearance: none;
      width: 100%;
      height: 6px;
      border-radius: 3px;
      background: rgba(255,255,255,0.1);
      outline: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--primary);
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0,217,255,0.5);
    }
    input[type=range]::-moz-range-thumb {
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--primary);
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0,217,255,0.5);
      border: none;
    }
    .stats {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 15px;
      margin-top: 20px;
    }
    .stat-box {
      padding: 15px;
      background: rgba(0,0,0,0.3);
      border-radius: 8px;
      text-align: center;
    }
    .stat-label {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: #8b92b0;
      margin-bottom: 5px;
    }
    .stat-value {
      font-size: 1.3rem;
      font-weight: 700;
      color: var(--text);
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>⚡ BCDC Controller</h1>
    <p>ESP32 PWM Power Control System</p>
  </div>

  <div class="container">
    <div class="card" id="mainCard">
      <div class="power-switch">
        <span class="power-label">System Power</span>
        <label class="toggle">
          <input type="checkbox" id="pwrToggle" onchange="togglePower()">
          <span class="slider"></span>
        </label>
      </div>

      <div class="voltage-grid">
        <div class="voltage-display">
          <div class="voltage-label">Input (VIN)</div>
          <div class="voltage-value">
            <span id="vInDisp">0.00</span><span class="voltage-unit">V</span>
          </div>
        </div>
        <div class="voltage-display output">
          <div class="voltage-label">Output (VOUT)</div>
          <div class="voltage-value">
            <span id="vOutDisp">0.00</span><span class="voltage-unit">V</span>
          </div>
        </div>
      </div>

      <div class="efficiency-display">
        <div class="efficiency-label">Conversion Efficiency</div>
        <div class="efficiency-value" id="effDisp">--</div>
      </div>

      <canvas id="pwmCanvas"></canvas>

      <div class="control-group">
        <div class="control-header">
          <span class="control-label">Frequency</span>
          <span class="control-value" id="fVal">100.0 kHz</span>
        </div>
        <input type="range" id="fSlide" min="5000" max="200000" step="1000" value="100000" oninput="updateFreq(this.value)">
      </div>

      <div class="control-group">
        <div class="control-header">
          <span class="control-label">Duty Cycle</span>
          <span class="control-value" id="dPct">0%</span>
        </div>
        <input type="range" id="dSlide" min="0" max="95" value="0" oninput="updateDuty(this.value)">
      </div>

      <div class="stats">
        <div class="stat-box">
          <div class="stat-label">Status</div>
          <div class="stat-value" id="statusText">Offline</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">PWM Output</div>
          <div class="stat-value" id="pwmRaw">0</div>
        </div>
      </div>
    </div>
  </div>

<script>
  let isPwr = false, currentDuty = 0;
  const canvas = document.getElementById('pwmCanvas');
  const ctx = canvas.getContext('2d');
  canvas.width = canvas.offsetWidth * window.devicePixelRatio;
  canvas.height = 100 * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);

  function draw() {
    const w = canvas.offsetWidth;
    const h = 100;
    ctx.clearRect(0, 0, w, h);
    
    // Draw waveform
    const color = isPwr ? '#00d9ff' : '#34495e';
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.shadowBlur = isPwr ? 10 : 0;
    ctx.shadowColor = color;
    
    const pulses = 6;
    const pulseWidth = w / pulses;
    const highWidth = (currentDuty / 100) * pulseWidth;
    
    ctx.beginPath();
    for (let i = 0; i < pulses; i++) {
      const x = i * pulseWidth;
      ctx.moveTo(x, h - 15);
      ctx.lineTo(x, 15);
      ctx.lineTo(x + highWidth, 15);
      ctx.lineTo(x + highWidth, h - 15);
      ctx.lineTo(x + pulseWidth, h - 15);
    }
    ctx.stroke();
    
    requestAnimationFrame(draw);
  }

  function togglePower() {
    isPwr = document.getElementById('pwrToggle').checked;
    document.getElementById('mainCard').classList.toggle('active', isPwr);
    document.getElementById('mainCard').classList.toggle('disabled', !isPwr);
    document.getElementById('statusText').innerText = isPwr ? 'Active' : 'Offline';
    fetch(`/power?en=${isPwr ? 1 : 0}`);
  }

  function updateDuty(v) {
    const raw = Math.round(v * 10.23);
    fetch(`/set?d=${raw}`);
  }

  function updateFreq(v) {
    document.getElementById('fVal').innerText = (v / 1000).toFixed(1) + ' kHz';
    fetch(`/set?f=${v}`);
  }

  setInterval(() => {
    fetch('/status').then(r => r.json()).then(d => {
      document.getElementById('vInDisp').innerText = d.vin.toFixed(2);
      document.getElementById('vOutDisp').innerText = d.vout.toFixed(2);
      document.getElementById('dPct').innerText = d.d + '%';
      document.getElementById('pwmRaw').innerText = d.raw;
      document.getElementById('pwrToggle').checked = d.pwr == 1;
      document.getElementById('statusText').innerText = d.pwr == 1 ? 'Active' : 'Offline';
      
      // Calculate efficiency
      if (d.vin > 0.5 && d.vout > 0.1) {
        const eff = ((d.vout / d.vin) * 100).toFixed(1);
        document.getElementById('effDisp').innerText = eff + '%';
      } else {
        document.getElementById('effDisp').innerText = '--';
      }
      
      isPwr = (d.pwr == 1);
      currentDuty = d.d;
      
      // Update card state
      document.getElementById('mainCard').classList.toggle('active', isPwr);
      document.getElementById('mainCard').classList.toggle('disabled', !isPwr);
    });
  }, 250);

  draw();
</script>
</body>
</html>
)rawliteral";

// --- Read Input Voltage from Pin 3 ---
float readVoltageIn() {
    // Take multiple samples and average
    long rawSum = 0;
    for (int i = 0; i < 20; i++) {
        rawSum += analogRead(Hardware::vInSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 20.0;
    
    // Convert ADC reading to voltage at ESP32 pin
    float pinVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    
    // Apply voltage divider ratio
    float realVolts = pinVolts * Hardware::VIN_VOLT_DIV_RATIO;
    
    return realVolts;
}

// --- Read Output Voltage from Pin 1 ---
float readVoltageOut() {
    // Take multiple samples and average
    long rawSum = 0;
    for (int i = 0; i < 20; i++) {
        rawSum += analogRead(Hardware::vOutSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 20.0;
    
    // Convert ADC reading to voltage at ESP32 pin
    float pinVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    
    // Apply voltage divider ratio: 57.5k + 10k / 10k = 6.75
    float realVolts = pinVolts * Hardware::VOUT_VOLT_DIV_RATIO;
    
    return realVolts;
}

// --- HTTP Handlers ---
void handleStatus() {
    State::lastVin = readVoltageIn();
    State::lastVout = readVoltageOut();
    
    if (!State::powerEnabled) {
        State::targetDutyRaw = 0;
    } else {
        State::targetDutyRaw = State::manualDutyRaw;
    }
    
    String json = "{\"vin\":" + String(State::lastVin, 2) + 
                  ",\"vout\":" + String(State::lastVout, 2) + 
                  ",\"d\":" + String((int)(State::currentDutyRaw / 10.23)) + 
                  ",\"raw\":" + String(State::currentDutyRaw) +
                  ",\"f\":" + String(State::currentFreq) + 
                  ",\"pwr\":" + String(State::powerEnabled ? 1 : 0) + "}";
    server.send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    
    pinMode(Hardware::enablePin, OUTPUT);
    digitalWrite(Hardware::enablePin, LOW);

    ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
    ledcAttachPin(Hardware::pwmPin, Hardware::pwmChannel);
    ledcWrite(Hardware::pwmChannel, 0);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid);
    dnsServer.start(53, "*", local_IP);

    server.on("/", []() { 
        server.send(200, "text/html", index_html); 
    });
    
    server.on("/status", handleStatus);
    
    server.on("/power", []() {
        if (server.hasArg("en")) {
            State::powerEnabled = (server.arg("en") == "1");
            digitalWrite(Hardware::enablePin, State::powerEnabled ? HIGH : LOW);
        }
        server.send(200, "text/plain", "OK");
    });

    server.on("/set", []() {
        if (server.hasArg("f")) {
            State::currentFreq = server.arg("f").toInt();
            ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
        }
        if (server.hasArg("d")) {
            State::manualDutyRaw = constrain(server.arg("d").toInt(), 0, Hardware::ABSOLUTE_MAX_DUTY);
        }
        server.send(200, "text/plain", "OK");
    });

    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
    
    Serial.println("BCDC Controller Started - Dual Voltage Sensing");
    Serial.println("VIN (Pin 3): 10k / (10k + 24.8k) - Multiplier: 3.48");
    Serial.println("VOUT (Pin 1): 10k / (10k + 57.5k) - Multiplier: 6.75");
}

void loop() {
    dnsServer.processNextRequest();
    server.handleClient();

    // Debug output every 1 second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        // VIN Debug
        int rawIn = analogRead(Hardware::vInSensePin);
        float pinVin = (rawIn / 4095.0) * Hardware::V_REF;
        float realVin = pinVin * Hardware::VIN_VOLT_DIV_RATIO;
        
        // VOUT Debug
        int rawOut = analogRead(Hardware::vOutSensePin);
        float pinVout = (rawOut / 4095.0) * Hardware::V_REF;
        float realVout = pinVout * Hardware::VOUT_VOLT_DIV_RATIO;
        
        Serial.printf("VIN: ADC=%d Pin=%.3fV Real=%.2fV | VOUT: ADC=%d Pin=%.3fV Real=%.2fV\n", 
                      rawIn, pinVin, realVin, rawOut, pinVout, realVout);
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
