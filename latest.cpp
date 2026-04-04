#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- OLED Display Configuration ---
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 8, /* data=*/ 9);
// Change SSD1306 to SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 8, /* data=*/ 9);
// --- Hardware Configuration ---
namespace Hardware {
    const int pwmPin = 6;       
    const int enablePin = 5;    
    const int pwmChannel = 0;   
    const int resolution = 10;  
    const int rpmSensorPin = 3;
    const int currentSensePin = 1;
    const int vOutSensePin = 4;

    const float VOUT_R1 = 57500.0;
    const float VOUT_R2 = 10000.0;
    const float V_REF = 3.3;
    const float VOUT_VOLT_DIV_RATIO = (VOUT_R1 + VOUT_R2) / VOUT_R2;
    
    const float CURRENT_AMP_R1 = 10000.0;
    const float CURRENT_AMP_R2 = 57000.0;
    const float CURRENT_AMP_GAIN = 1.0 + (CURRENT_AMP_R2 / CURRENT_AMP_R1);
    const float SHUNT_RESISTOR = 0.1;
    
    const int ABSOLUTE_MAX_DUTY = 972;
    
    const unsigned long RPM_DEBOUNCE_US = 1000;
    const unsigned long RPM_TIMEOUT_MS = 2000;
    
    const float MIN_CURRENT_THRESHOLD = 0.05;
    const float MIN_VOLTAGE_THRESHOLD = 0.5;
}

// --- Display State Management ---
enum DisplayMode {
    DISPLAY_LOADING,
    DISPLAY_MAIN,
    DISPLAY_DUTY_FULLSCREEN,
    DISPLAY_POWER_FULLSCREEN
};

namespace DisplayState {
    DisplayMode currentMode = DISPLAY_LOADING;
    unsigned long modeChangeTime = 0;
    unsigned long lastDutyChange = 0;
    unsigned long lastPowerChange = 0;
    int lastDisplayedDuty = 0;
    bool lastDisplayedPower = false;
    int loadingProgress = 0;
    
    const unsigned long LOADING_DURATION = 3000;  // 3 seconds
    const unsigned long FULLSCREEN_TIMEOUT = 3000;  // Return to main after 3s
}

// --- System State ---
namespace State {
    const int currentFreq = 35000;
    int manualDutyRaw = 0;
    int targetDutyRaw = 0;
    int currentDutyRaw = 0;
    bool powerEnabled = false;
    float lastVout = 0.0;
    float lastCurrent = 0.0;
    volatile unsigned long lastRpmPulse = 0;
    volatile unsigned long rpmPulseInterval = 0;
    volatile bool newRpmPulse = false;
    int currentRPM = 0;
    
    float voutOffset = 0.0;
    float currentOffset = 0.0;
}

// --- Network Configuration ---
const bool ConnectionType = true;
const char* ssid = ConnectionType ? "Idoom 4G_E1BFE" : "BCDCControl";
const char* password = ConnectionType ? "90526052" : "";

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
    
    if (timeSinceLastPulse > Hardware::RPM_DEBOUNCE_US) {
        State::rpmPulseInterval = timeSinceLastPulse;
        State::lastRpmPulse = currentTime;
        State::newRpmPulse = true;
    }
}

// --- Calculate derived values ---
float calculateOutputPower() {
    return State::lastVout * State::lastCurrent;
}

float calculateInputCurrent() {
    int dutyPercent = (int)(State::currentDutyRaw / 10.23);
    if (dutyPercent >= 100) return 0.0;  // Avoid division by zero
    
    float dutyCycleFraction = dutyPercent / 100.0;
    if (dutyCycleFraction >= 0.99) return 0.0;  // Safety check
    
    // Iin = Iout / (1 - D)
    return State::lastCurrent / (1.0 - dutyCycleFraction);
}

// --- OLED Display Functions ---

void drawLoadingScreen() {
    u8g2.clearBuffer();
    
    // Calculate progress (0-100% over LOADING_DURATION)
    unsigned long elapsed = millis() - DisplayState::modeChangeTime;
    int progress = (elapsed * 100) / DisplayState::LOADING_DURATION;
    if (progress > 100) progress = 100;
    
    // Title
    u8g2.setFont(u8g2_font_helvB18_tr);
    u8g2.drawStr(25, 25, "BCDC");
    
    // Subtitle
    u8g2.setFont(u8g2_font_helvB10_tr);
    u8g2.drawStr(18, 40, "CONTROLLER");
    
    // Progress bar
    int barWidth = 100;
    int barHeight = 10;
    int barX = (128 - barWidth) / 2;
    int barY = 48;
    
    u8g2.drawFrame(barX, barY, barWidth, barHeight);
    int fillWidth = (progress * (barWidth - 4)) / 100;
    if (fillWidth > 0) {
        u8g2.drawBox(barX + 2, barY + 2, fillWidth, barHeight - 4);
    }
    
    // Percentage
    u8g2.setFont(u8g2_font_6x10_tf);
    char progressStr[8];
    sprintf(progressStr, "%d%%", progress);
    int textWidth = u8g2.getStrWidth(progressStr);
    u8g2.drawStr((128 - textWidth) / 2, 63, progressStr);
    
    u8g2.sendBuffer();
}

void drawPWMWaveform(int x, int y, int width, int height, int dutyPercent) {
    // Draw PWM waveform
    int pulses = 4;
    int pulseWidth = width / pulses;
    int highWidth = (dutyPercent * pulseWidth) / 100;
    
    for (int i = 0; i < pulses; i++) {
        int px = x + (i * pulseWidth);
        
        // Low to High
        u8g2.drawLine(px, y + height - 1, px, y);
        // High period
        u8g2.drawLine(px, y, px + highWidth, y);
        // High to Low
        u8g2.drawLine(px + highWidth, y, px + highWidth, y + height - 1);
        // Low period
        u8g2.drawLine(px + highWidth, y + height - 1, px + pulseWidth, y + height - 1);
    }
}

void drawDutyFullscreen() {
    u8g2.clearBuffer();
    
    int dutyPercent = (int)(State::currentDutyRaw / 10.23);
    
    // Large Duty Cycle Display
    u8g2.setFont(u8g2_font_helvB24_tr);
    char dutyStr[8];
    sprintf(dutyStr, "%d%%", dutyPercent);
    int textWidth = u8g2.getStrWidth(dutyStr);
    u8g2.drawStr((128 - textWidth) / 2, 28, dutyStr);
    
    // Label
    u8g2.setFont(u8g2_font_helvB08_tr);
    const char* label = "DUTY CYCLE";
    textWidth = u8g2.getStrWidth(label);
    u8g2.drawStr((128 - textWidth) / 2, 38, label);
    
    // PWM Waveform Visualization
    u8g2.drawLine(0, 42, 128, 42);  // Separator
    drawPWMWaveform(4, 46, 120, 16, dutyPercent);
    
    u8g2.sendBuffer();
}

void drawPowerFullscreen() {
    u8g2.clearBuffer();
    
    // Large Power Status
    u8g2.setFont(u8g2_font_helvB24_tr);
    const char* statusText = State::powerEnabled ? "ON" : "OFF";
    int textWidth = u8g2.getStrWidth(statusText);
    u8g2.drawStr((128 - textWidth) / 2, 28, statusText);
    
    // Label
    u8g2.setFont(u8g2_font_helvB08_tr);
    const char* label = "POWER STATUS";
    textWidth = u8g2.getStrWidth(label);
    u8g2.drawStr((128 - textWidth) / 2, 38, label);
    
    u8g2.drawLine(0, 42, 128, 42);
    
    if (State::powerEnabled) {
        // Show animated power symbol or waveform
        int dutyPercent = (int)(State::currentDutyRaw / 10.23);
        drawPWMWaveform(4, 46, 120, 16, dutyPercent);
    } else {
        // Show "DISABLED" message
        u8g2.setFont(u8g2_font_helvB10_tr);
        const char* msg = "DISABLED";
        textWidth = u8g2.getStrWidth(msg);
        u8g2.drawStr((128 - textWidth) / 2, 56, msg);
    }
    
    u8g2.sendBuffer();
}

void drawMainMenu() {
    u8g2.setDrawColor(1); // FIX 1: Ensure we are drawing in WHITE at the start
    u8g2.clearBuffer();
    
    char buf[12];
    int labelsX = 0;
    int valuesX = 30; // Slightly adjusted for spacing
    int barsX = 85;
    int barWidth = 40;

    // --- Row 1: VOUT (Max 20V) ---
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(labelsX, 10, "VOUT");
    u8g2.setFont(u8g2_font_7x14B_tr);
    dtostrf(State::lastVout, 5, 1, buf);
    u8g2.drawStr(valuesX, 12, buf);
    
    // FIX 2: Explicitly cast to float to ensure the ratio isn't 0
    int voutBar = (int)((float)State::lastVout / 20.0f * (barWidth - 2));
    voutBar = constrain(voutBar, 0, barWidth - 2);
    u8g2.drawFrame(barsX, 4, barWidth, 7);
    if (voutBar > 0) u8g2.drawBox(barsX + 1, 5, voutBar, 5);

    // --- Row 2: IOUT (Max 10A) ---
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(labelsX, 24, "IOUT");
    u8g2.setFont(u8g2_font_7x14B_tr);
    dtostrf(State::lastCurrent, 5, 2, buf);
    u8g2.drawStr(valuesX, 26, buf);
    
    int ioutBar = (int)((float)State::lastCurrent / 10.0f * (barWidth - 2));
    ioutBar = constrain(ioutBar, 0, barWidth - 2);
    u8g2.drawFrame(barsX, 18, barWidth, 7);
    if (ioutBar > 0) u8g2.drawBox(barsX + 1, 19, ioutBar, 5);

    // --- Row 3: POUT (Max 200W) ---
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(labelsX, 38, "POUT");
    u8g2.setFont(u8g2_font_7x14B_tr);
    float power = calculateOutputPower();
    dtostrf(power, 4, 0, buf);
    u8g2.drawStr(valuesX + 5, 40, buf); // Shifted slightly for 3 digits
    
    int poutBar = (int)(power / 200.0f * (barWidth - 2));
    poutBar = constrain(poutBar, 0, barWidth - 2);
    u8g2.drawFrame(barsX, 32, barWidth, 7);
    if (poutBar > 0) u8g2.drawBox(barsX + 1, 33, poutBar, 5);

    // --- Row 4: RPM (Max 5000) ---
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(labelsX, 52, "RPM");
    u8g2.setFont(u8g2_font_7x14B_tr);
    sprintf(buf, "%4d", State::currentRPM);
    u8g2.drawStr(valuesX, 54, buf);
    
    int rpmBar = (int)((float)State::currentRPM / 5000.0f * (barWidth - 2));
    rpmBar = constrain(rpmBar, 0, barWidth - 2);
    u8g2.drawFrame(barsX, 46, barWidth, 7);
    if (rpmBar > 0) u8g2.drawBox(barsX + 1, 47, rpmBar, 5);

    // --- Footer: Status Bar ---
    u8g2.setDrawColor(1); // Ensure box is white
    u8g2.drawBox(0, 56, 128, 8); 
    u8g2.setDrawColor(0); // Text is black (cutout)
    u8g2.setFont(u8g2_font_5x8_tr);
    
    int dutyPercent = (int)(State::currentDutyRaw / 10.23);
    sprintf(buf, "DUTY:%d%%", dutyPercent);
    u8g2.drawStr(2, 63, buf);
    
    if (State::powerEnabled) {
        u8g2.drawStr(82, 63, "STATUS: RUN");
    } else {
        u8g2.drawStr(82, 63, "STATUS: OFF");
    }
    
    u8g2.setDrawColor(1); // Reset for next loop
    u8g2.sendBuffer();
}

void updateOLEDDisplay() {
    unsigned long currentTime = millis();
    
    // Check for mode transitions
    switch (DisplayState::currentMode) {
        case DISPLAY_LOADING:
            if (currentTime - DisplayState::modeChangeTime >= DisplayState::LOADING_DURATION) {
                DisplayState::currentMode = DISPLAY_MAIN;
                DisplayState::modeChangeTime = currentTime;
            }
            drawLoadingScreen();
            break;
            
        case DISPLAY_MAIN:
            // Check for duty cycle change
            {
                int currentDutyPercent = (int)(State::currentDutyRaw / 10.23);
                if (currentDutyPercent != DisplayState::lastDisplayedDuty) {
                    DisplayState::lastDutyChange = currentTime;
                    DisplayState::lastDisplayedDuty = currentDutyPercent;
                    DisplayState::currentMode = DISPLAY_DUTY_FULLSCREEN;
                    DisplayState::modeChangeTime = currentTime;
                }
                // Check for power change
                else if (State::powerEnabled != DisplayState::lastDisplayedPower) {
                    DisplayState::lastPowerChange = currentTime;
                    DisplayState::lastDisplayedPower = State::powerEnabled;
                    DisplayState::currentMode = DISPLAY_POWER_FULLSCREEN;
                    DisplayState::modeChangeTime = currentTime;
                }
                else {
                    drawMainMenu();
                }
            }
            break;
            
        case DISPLAY_DUTY_FULLSCREEN:
            // Check if duty is still changing
            {
                int currentDutyPercent = (int)(State::currentDutyRaw / 10.23);
                if (currentDutyPercent != DisplayState::lastDisplayedDuty) {
                    DisplayState::lastDutyChange = currentTime;
                    DisplayState::lastDisplayedDuty = currentDutyPercent;
                }
                
                // Return to main menu after timeout
                if (currentTime - DisplayState::lastDutyChange >= DisplayState::FULLSCREEN_TIMEOUT) {
                    DisplayState::currentMode = DISPLAY_MAIN;
                    DisplayState::modeChangeTime = currentTime;
                }
                
                drawDutyFullscreen();
            }
            break;
            
        case DISPLAY_POWER_FULLSCREEN:
            // Check if power is still changing
            if (State::powerEnabled != DisplayState::lastDisplayedPower) {
                DisplayState::lastPowerChange = currentTime;
                DisplayState::lastDisplayedPower = State::powerEnabled;
            }
            
            // Return to main menu after timeout
            if (currentTime - DisplayState::lastPowerChange >= DisplayState::FULLSCREEN_TIMEOUT) {
                DisplayState::currentMode = DISPLAY_MAIN;
                DisplayState::modeChangeTime = currentTime;
            }
            
            drawPowerFullscreen();
            break;
    }
}

// --- HTML Interface (keeping original) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Controller</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
    body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: #0a0e27; color: #e8edf4; min-height: 100vh; padding: 15px; overflow-x: hidden; touch-action: pan-y; }
    .header { text-align: center; margin-bottom: 20px; }
    .header h1 { font-size: 1.8rem; font-weight: 700; color: #00d9ff; margin-bottom: 5px; }
    .container { max-width: 600px; margin: 0 auto; }
    .card { background: #151b3d; padding: 20px; border-radius: 12px; border: 1px solid rgba(255,255,255,0.1); margin-bottom: 15px; }
    .card.active { border-color: #2ed573; box-shadow: 0 0 20px rgba(46, 213, 115, 0.3); }
    
    canvas { width: 100%; height: 100px; border-radius: 8px; background: rgba(0,0,0,0.4); display: block; touch-action: none; margin-bottom: 15px; }
    
    /* Grid updated for 7 items: Vout, Iout, RPM | Pout, Iin, Pin | Efficiency */
    .measurements { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 15px 0; }
    .measure { text-align: center; padding: 12px 5px; background: rgba(0,0,0,0.3); border-radius: 8px; border: 1px solid rgba(255,255,255,0.1); }
    .full-width { grid-column: span 3; border-color: rgba(46, 213, 115, 0.4); }
    
    .measure-label { font-size: 0.65rem; text-transform: uppercase; color: #8b92b0; margin-bottom: 5px; font-weight: 600; }
    .measure-value { font-size: 1.2rem; font-weight: 700; color: #00d9ff; }
    .measure-unit { font-size: 0.7rem; opacity: 0.7; margin-left: 2px; }
    
    /* Specific Colors */
    .val-vout { color: #2096ab; }
    .val-iout { color: #ffa502; }
    .val-pout { color: #ff4757; }
    .val-iin { color: #2ed573; }
    .val-pin { color: #a29bfe; }
    .val-eff { color: #2ed573; }
    .val-rpm { color: #ffffff; }

    .power-row { display: flex; justify-content: space-between; align-items: center; padding: 15px; background: rgba(0,0,0,0.3); border-radius: 10px; margin-bottom: 15px; }
    .power-label { font-weight: 600; font-size: 1rem; color: #e8edf4; text-transform: uppercase; }
    .toggle { position: relative; width: 60px; height: 32px; }
    .toggle input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #34495e; transition: .3s; border-radius: 32px; }
    .slider:before { position: absolute; content: ""; height: 24px; width: 24px; left: 4px; bottom: 4px; background-color: white; transition: .3s; border-radius: 50%; }
    input:checked + .slider { background-color: #2ed573; }
    input:checked + .slider:before { transform: translateX(28px); }
    
    .control-group { margin-bottom: 15px; }
    .control-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
    .control-label { font-size: 0.85rem; text-transform: uppercase; color: #8b92b0; font-weight: 600; }
    .control-value { font-size: 1.2rem; font-weight: 700; color: #00d9ff; }
    input[type=range] { -webkit-appearance: none; width: 100%; height: 8px; border-radius: 4px; background: rgba(255,255,255,0.1); outline: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 24px; height: 24px; border-radius: 50%; background: #00d9ff; cursor: pointer; }
  </style>
</head>
<body>
  <div class="header">
    <h1>IoT DC Controller D-Board</h1>
  </div>

  <div class="container">
    <div class="card" id="mainCard">
      <canvas id="pwmCanvas"></canvas>
      
      <div class="measurements">
        <div class="measure">
          <div class="measure-label">VOUT</div>
          <div class="measure-value val-vout"><span id="vOutDisp">0.00</span><span class="measure-unit">V</span></div>
        </div>
        <div class="measure">
          <div class="measure-label">IOUT</div>
          <div class="measure-value val-iout"><span id="iOutDisp">0.00</span><span class="measure-unit">A</span></div>
        </div>
        <div class="measure">
          <div class="measure-label">RPM</div>
          <div class="measure-value val-rpm"><span id="rpmDisp">0</span></div>
        </div>

        <div class="measure">
          <div class="measure-label">POUT</div>
          <div class="measure-value val-pout"><span id="pOutDisp">0.0</span><span class="measure-unit">W</span></div>
        </div>
        <div class="measure">
          <div class="measure-label">I-INPUT</div>
          <div class="measure-value val-iin"><span id="iInDisp">0.00</span><span class="measure-unit">A</span></div>
        </div>
        <div class="measure">
          <div class="measure-label">P-INPUT</div>
          <div class="measure-value val-pin"><span id="pInDisp">0.0</span><span class="measure-unit">W</span></div>
        </div>

        <div class="measure full-width">
          <div class="measure-label">System Efficiency</div>
          <div class="measure-value val-eff"><span id="effDisp">0.0</span><span class="measure-unit">%</span></div>
        </div>
      </div>
      
      <div class="power-row">
        <span class="power-label">System Power</span>
        <label class="toggle">
          <input type="checkbox" id="pwrToggle" onchange="togglePower()">
          <span class="slider"></span>
        </label>
      </div>
      
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
  
  function resizeCanvas() {
    canvas.width = canvas.offsetWidth;
    canvas.height = 100;
  }
  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  function draw() {
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const color = isPwr ? '#00d9ff' : '#34495e';
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    const pulses = 8;
    const pulseWidth = w / pulses;
    const highWidth = (currentDuty / 100) * pulseWidth;
    ctx.beginPath();
    for (let i = 0; i < pulses; i++) {
      const x = i * pulseWidth;
      ctx.moveTo(x, h - 10);
      ctx.lineTo(x, 10);
      ctx.lineTo(x + highWidth, 10);
      ctx.lineTo(x + highWidth, h - 10);
      ctx.lineTo(x + pulseWidth, h - 10);
    }
    ctx.stroke();
    requestAnimationFrame(draw);
  }

  function togglePower() {
    isPwr = document.getElementById('pwrToggle').checked;
    fetch(`/power?en=${isPwr ? 1 : 0}`);
  }

  function updateDuty(v) {
    const raw = Math.round(v * 10.23);
    fetch(`/set?d=${raw}`);
  }

  setInterval(() => {
    fetch('/status').then(r => r.json()).then(d => {
      // 1. Update basic display from JSON
      document.getElementById('vOutDisp').innerText = d.vout.toFixed(2);
      document.getElementById('iOutDisp').innerText = d.current.toFixed(2);
      document.getElementById('rpmDisp').innerText = d.rpm;
      document.getElementById('dPct').innerText = d.d + '%';
      document.getElementById('pwrToggle').checked = d.pwr == 1;
      
      // 2. Calculations
      // P_out = V_out * I_out
      let pOut = d.vout * d.current;
      
      // I_in (Calculated based on your duty cycle logic)
      let dutyDecimal = d.d / 100;
      let iIn = (dutyDecimal < 1.0) ? (d.current / (1.0 - dutyDecimal)) : 0;

      // P_in = 3.7 * I_in
      let pIn = 3.7 * iIn;

      // Efficiency = P_out / P_in (converted to percentage)
      let efficiency = 0;
      if (pIn > 0) {
          efficiency = (pOut / pIn) * 100;
          if(efficiency > 100) efficiency = 100; // Cap at 100 for display
      }

      // 3. Update Display
      document.getElementById('pOutDisp').innerText = pOut.toFixed(1);
      document.getElementById('iInDisp').innerText = iIn.toFixed(2);
      document.getElementById('pInDisp').innerText = pIn.toFixed(1);
      document.getElementById('effDisp').innerText = efficiency.toFixed(1);

      isPwr = (d.pwr == 1);
      currentDuty = d.d;
      document.getElementById('mainCard').classList.toggle('active', isPwr);
    }).catch(err => console.log("Fetch error: ", err));
  }, 400);

  draw();
</script>
</body>
</html>
)rawliteral";

// --- Sensor Reading Functions ---
float readVoltageOut() {
    long rawSum = 0;
    for (int i = 0; i < 50; i++) {
        rawSum += analogRead(Hardware::vOutSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 50.0;
    float pinVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    float realVolts = pinVolts * Hardware::VOUT_VOLT_DIV_RATIO;
    float calibratedVolts = realVolts + State::voutOffset;
    
    if (calibratedVolts < Hardware::MIN_VOLTAGE_THRESHOLD) {
        return 0.0;
    }
    return calibratedVolts;
}

float readCurrent() {
    long rawSum = 0;
    for (int i = 0; i < 50; i++) {
        rawSum += analogRead(Hardware::currentSensePin);
        delayMicroseconds(100);
    }
    float avgRaw = rawSum / 50.0;
    float ampOutVolts = (avgRaw / 4095.0) * Hardware::V_REF;
    float shuntVolts = ampOutVolts / Hardware::CURRENT_AMP_GAIN;
    float current = shuntVolts / Hardware::SHUNT_RESISTOR;
    float calibratedCurrent = current + State::currentOffset;
    
    if (abs(calibratedCurrent) < Hardware::MIN_CURRENT_THRESHOLD) {
        return 0.0;
    }
    if (calibratedCurrent < 0) {
        return 0.0;
    }
    return calibratedCurrent;
}

void updateRPM() {
    if (millis() - (State::lastRpmPulse / 1000) > Hardware::RPM_TIMEOUT_MS) {
        State::currentRPM = 0;
        return;
    }
    
    if (State::newRpmPulse && State::rpmPulseInterval > 0) {
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
    DisplayState::currentMode = DISPLAY_LOADING;
    DisplayState::modeChangeTime = millis();
    
    // Load calibration from flash
    preferences.begin("bcdc", false);
    State::voutOffset = preferences.getFloat("voutOffset", 0.0);
    State::currentOffset = preferences.getFloat("currentOffset", 0.0);
    
    Serial.printf("Loaded calibration: VOUT offset=%.2fV, Current offset=%.2fA\n", 
                  State::voutOffset, State::currentOffset);
    
    // Setup RPM sensor interrupt
    pinMode(Hardware::rpmSensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(Hardware::rpmSensorPin), rpmPulseISR, FALLING);
    
    pinMode(Hardware::enablePin, OUTPUT);
    digitalWrite(Hardware::enablePin, LOW);

    ledcSetup(Hardware::pwmChannel, State::currentFreq, Hardware::resolution);
    ledcAttachPin(Hardware::pwmPin, Hardware::pwmChannel);
    ledcWrite(Hardware::pwmChannel, 0);

    // WiFi Configuration
    if (ConnectionType) {
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

    // Update OLED display every 100ms
    static unsigned long lastOLEDUpdate = 0;
    if (millis() - lastOLEDUpdate > 100) {
        State::lastVout = readVoltageOut();
        State::lastCurrent = readCurrent();
        updateRPM();
        updateOLEDDisplay();
        lastOLEDUpdate = millis();
    }

    // Debug output every 1 second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        float power = calculateOutputPower();
        float inputCurrent = calculateInputCurrent();
        Serial.printf("VOUT: %.2fV | IOUT: %.2fA | IIN: %.2fA | RPM: %d | Duty: %d%% | POUT: %.2fW\n", 
                      State::lastVout, State::lastCurrent, inputCurrent, State::currentRPM, 
                      (int)(State::currentDutyRaw / 10.23), power);
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
