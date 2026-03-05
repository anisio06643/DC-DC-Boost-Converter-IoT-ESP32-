#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// --- Configuration Namespaces ---
namespace Hardware {
    const int pwmPin = 1;       
    const int pwmChannel = 0;   
    const int resolution = 10;  // 10-bit (0-1023)
    const int vSensePin = 2;    
    const int ntcPin = 3;       

    const float SERIES_RESISTOR = 90000.0; 
    const float NOMINAL_RESISTANCE = 470000.0; 
    const float NOMINAL_TEMPERATURE = 25.0;
    const float BETA_COEFF = 4750.0; 
    const int ABSOLUTE_MAX_DUTY = 972; // Hard 95% Limit
}

namespace State {
    int currentFreq = 50000;
    int manualDutyRaw = 0;   
    int targetDutyRaw = 0;   // What the logic wants
    int currentDutyRaw = 0;  // What is actually set (for soft-start)
    bool ntcEnabled = false;  
    int targetTemp = 50;      
    int maxAutoDutyPct = 90;  
    float lastTemp = 0.0;
    float lastVolts = 0.0;
}

// --- WiFi Credentials ---
const char* ssid = "YOUR-NETWORK-SSID";
const char* password = "YOUR-NETWORK-PASSWORD";

WebServer server(80);

// --- Cyberpunk Modern UI ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root { --neon: #00e676; --bg: #050505; --card: #111; --text: #eee; --danger: #17e0ff; }
    body { font-family: 'Segoe UI', sans-serif; background: var(--bg); color: var(--text); display: flex; flex-direction: column; align-items: center; padding: 10px; margin: 0; }
    .card { background: var(--card); padding: 20px; border-radius: 12px; border: 1px solid #222; width: 90%; max-width: 400px; margin-top: 20px; box-shadow: 0 0 20px rgba(0,0,0,0.5); }
    .header { color: var(--neon); text-align: center; text-transform: uppercase; letter-spacing: 2px; font-weight: bold; margin-bottom: 15px; }
    .val-large { font-size: 3rem; text-align: center; color: var(--neon); font-weight: bold; }
    canvas { background: #000; width: 100%; height: 80px; border-radius: 4px; border: 1px solid #333; margin: 15px 0; }
    input[type=range] { width: 100%; accent-color: var(--neon); }
    .auto-active { border-color: var(--danger) !important; box-shadow: 0 0 15px rgba(6, 68, 79, 0.97); }
    .label-row { display: flex; justify-content: space-between; font-size: 0.8rem; color: #888; margin-top: 10px; }
    input[type=number] { background: #000; color: var(--neon); border: 1px solid #444; padding: 4px; width: 60px; text-align: center; }
  </style>
</head>
<body>
  <div class="card" id="mainCard">
    <div class="header" id="modeTitle">Manual Mode</div>
    <div class="val-large" id="vDisp">0.00V</div>
    <canvas id="pwmCanvas"></canvas>
    <div class="label-row"><span>FREQUENCY</span><span id="fVal">50kHz</span></div>
    <input type="range" id="fSlide" min="5000" max="100000" step="500" value="50000" oninput="updateFreq(this.value)">
    <div class="label-row"><span>DUTY CYCLE</span><span id="dPct">0%</span></div>
    <input type="range" id="dSlide" min="0" max="95" value="0" oninput="updateDuty(this.value)">
  </div>
  <div class="card" id="thermalCard">
    <div style="display:flex; justify-content:space-between; align-items:center;">
        <span>THERMAL GOVERNOR</span>
        <input type="checkbox" id="ntcToggle" style="transform:scale(1.5)" onchange="toggleAuto()">
    </div>
    <div id="tDisp" style="font-size: 2.5rem; text-align:center; margin: 15px 0; color: var(--danger);">--°C</div>
    <div style="display:flex; justify-content:space-around;">
        <div class="label-row" style="flex-direction:column">LIMIT °C<input type="number" id="tLim" value="50" onchange="sendNtc()"></div>
        <div class="label-row" style="flex-direction:column">MAX %<input type="number" id="tSafe" value="90" onchange="sendNtc()"></div>
    </div>
  </div>
<script>
  let isAuto = false, lastFreq = 50000, currentDuty = 0;
  const canvas = document.getElementById('pwmCanvas'), ctx = canvas.getContext('2d');

  function draw() {
    ctx.clearRect(0,0,canvas.width, canvas.height);
    ctx.strokeStyle = isAuto ? '#03ffee' : '#00e676';
    ctx.lineWidth = 3;
    ctx.shadowBlur = 8; ctx.shadowColor = ctx.strokeStyle;
    ctx.beginPath();
    let pulses = 6, step = canvas.width / pulses, w = (currentDuty/100) * step;
    for(let i=0; i<pulses; i++){
        let x = i * step;
        ctx.moveTo(x, 70); ctx.lineTo(x, 10); ctx.lineTo(x+w, 10); ctx.lineTo(x+w, 70); ctx.lineTo(x+step, 70);
    }
    ctx.stroke();
    requestAnimationFrame(draw);
  }

  function toggleAuto() {
    isAuto = document.getElementById('ntcToggle').checked;
    document.getElementById('dSlide').disabled = isAuto;
    document.getElementById('mainCard').className = isAuto ? "card auto-active" : "card";
    document.getElementById('modeTitle').innerText = isAuto ? "Thermal Active" : "Manual Mode";
    sendNtc();
  }

  function updateDuty(v) { if(!isAuto) fetch(`/set?d=${Math.round(v * 10.23)}`); }
  function updateFreq(v) { document.getElementById('fVal').innerText = (v/1000).toFixed(1) + "kHz"; fetch(`/set?f=${v}`); }
  function sendNtc() {
    fetch(`/ntc?en=${document.getElementById('ntcToggle').checked?1:0}&t=${document.getElementById('tLim').value}&s=${document.getElementById('tSafe').value}`);
  }

  setInterval(() => {
    fetch('/status').then(r => r.json()).then(d => {
        document.getElementById('vDisp').innerText = d.v.toFixed(2) + 'V';
        document.getElementById('tDisp').innerText = d.t.toFixed(1) + '°C';
        document.getElementById('dPct').innerText = d.ad + '%';
        currentDuty = d.ad; lastFreq = d.f;
    });
  }, 250);
  draw();
</script>
</body></html>
)rawliteral";

// --- Hardware Logic ---

float readTemp() {
    int raw = analogRead(Hardware::ntcPin);
    if (raw >= 4095) raw = 4094; if (raw <= 0) raw = 1;
    float res = Hardware::SERIES_RESISTOR * ((float)raw / (4095.0 - (float)raw));
    float sh = log(res / Hardware::NOMINAL_RESISTANCE) / Hardware::BETA_COEFF;
    sh += 1.0 / (Hardware::NOMINAL_TEMPERATURE + 273.15);
    return (1.0 / sh) - 273.15;
}

void handleStatus() {
    State::lastTemp = readTemp();
    State::lastVolts = (analogRead(Hardware::vSensePin) / 4095.0) * 3.3 * 24.0; 

    if (State::ntcEnabled) {
        float base = (float)State::manualDutyRaw / 10.23;
        float targetPct = base;

        if (State::lastTemp >= (float)State::targetTemp) {
            targetPct = (float)State::maxAutoDutyPct;
        } else if (State::lastTemp > 20.0) {
            float range = (float)State::targetTemp - 20.0;
            targetPct = base + ((State::lastTemp - 20.0) / range) * ((float)State::maxAutoDutyPct - base);
        }
        
        State::targetDutyRaw = (int)(constrain(targetPct, base, 95.0) * 10.23);
    } else {
        State::targetDutyRaw = State::manualDutyRaw;
    }

    String json = "{\"v\":" + String(State::lastVolts) + ",\"t\":" + String(State::lastTemp) + 
                  ",\"ad\":" + String((int)(State::currentDutyRaw / 10.23)) + ",\"f\":" + String(State::currentFreq) + "}";
    server.send(200, "application/json", json);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    
    // PWM Setup
    ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
    ledcAttachPin(Hardware::pwmPin, Hardware::pwmChannel);
    ledcWrite(Hardware::pwmChannel, 0);

    // --- Connect to WiFi ---
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Type this IP into your browser

    // --- Routes ---
    server.on("/", []() { server.send(200, "text/html", index_html); });
    server.on("/status", handleStatus);
    server.on("/set", []() {
        if(server.hasArg("f")) {
            State::currentFreq = server.arg("f").toInt();
            ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
        }
        if(server.hasArg("d")) State::manualDutyRaw = constrain(server.arg("d").toInt(), 0, Hardware::ABSOLUTE_MAX_DUTY);
        server.send(200, "text/plain", "OK");
    });
    server.on("/ntc", []() {
        if (server.hasArg("en")) State::ntcEnabled = (server.arg("en") == "1");
        if (server.hasArg("t")) State::targetTemp = server.arg("t").toInt();
        if (server.hasArg("s")) State::maxAutoDutyPct = server.arg("s").toInt();
        server.send(200, "text/plain", "OK");
    });

    server.begin();
}

void loop() {
    server.handleClient();

    // --- Non-Blocking Soft Start / Ramp ---
    static unsigned long lastRamp = 0;
    if (millis() - lastRamp > 2) { 
        if (State::currentDutyRaw < State::targetDutyRaw) State::currentDutyRaw++;
        else if (State::currentDutyRaw > State::targetDutyRaw) State::currentDutyRaw--;
        
        ledcWrite(Hardware::pwmChannel, State::currentDutyRaw);
        lastRamp = millis();
    }
}
