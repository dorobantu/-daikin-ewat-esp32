#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>

WebServer server(80);

const char* OTA_HOSTNAME = "daikin-simulator";

const int DAC_EVAP = 25;
const int DAC_COND = 26;
const int INPUT_COMPRESSOR1 = 32;
const int INPUT_COMPRESSOR2 = 33;

const float DAC_FULL_SCALE = 3.30;
const float DAC_MIN_V = 0.50;
const float DAC_MAX_V = 3.10;

const float SENSOR_MIN_V = 0.50;
const float SENSOR_MAX_V = 4.50;

const float EVAP_MIN_KPA = 0.0;
const float EVAP_MAX_KPA = 3000.0;
const float COND_MIN_KPA = 0.0;
const float COND_MAX_KPA = 5000.0;

const float STOP_EVAP_KPA = 1500.0;
const float STOP_COND_KPA = 1500.0;
const float ONE_EVAP_KPA = 800.0;
const float ONE_COND_KPA = 1500.0;
const float TWO_EVAP_KPA = 900.0;
const float TWO_COND_KPA = 1200.0;

const float RUN_RAMP_KPA_PER_SEC = 500.0;
const float STOP_RAMP_KPA_PER_SEC = 250.0;

float currentEvapKpa = STOP_EVAP_KPA;
float currentCondKpa = STOP_COND_KPA;
float targetEvapKpa = STOP_EVAP_KPA;
float targetCondKpa = STOP_COND_KPA;
float evapVoltage = 0.0;
float condVoltage = 0.0;

bool compressor1State = false;
bool compressor2State = false;
bool automaticMode = true;
unsigned long lastRampUpdate = 0;

enum ChillerState {
  STATE_STOP,
  STATE_ONE_COMPRESSOR,
  STATE_TWO_COMPRESSORS
};

ChillerState chillerState = STATE_STOP;

float pressureToVoltage(float kpa, float pmin, float pmax) {
  kpa = constrain(kpa, pmin, pmax);
  float ratio = (kpa - pmin) / (pmax - pmin);
  return SENSOR_MIN_V + ratio * (SENSOR_MAX_V - SENSOR_MIN_V);
}

uint8_t voltageToDAC(float voltage) {
  voltage = constrain(voltage, DAC_MIN_V, DAC_MAX_V);
  int value = round(voltage / DAC_FULL_SCALE * 255.0);
  return (uint8_t)constrain(value, 0, 255);
}

void updateDAC() {
  evapVoltage = pressureToVoltage(currentEvapKpa, EVAP_MIN_KPA, EVAP_MAX_KPA);
  condVoltage = pressureToVoltage(currentCondKpa, COND_MIN_KPA, COND_MAX_KPA);
  evapVoltage = constrain(evapVoltage, DAC_MIN_V, DAC_MAX_V);
  condVoltage = constrain(condVoltage, DAC_MIN_V, DAC_MAX_V);
  dacWrite(DAC_EVAP, voltageToDAC(evapVoltage));
  dacWrite(DAC_COND, voltageToDAC(condVoltage));
}

float moveTowards(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void setTargetsForState(ChillerState s) {
  switch (s) {
    case STATE_STOP:
      targetEvapKpa = STOP_EVAP_KPA;
      targetCondKpa = STOP_COND_KPA;
      break;
    case STATE_ONE_COMPRESSOR:
      targetEvapKpa = ONE_EVAP_KPA;
      targetCondKpa = ONE_COND_KPA;
      break;
    case STATE_TWO_COMPRESSORS:
      targetEvapKpa = TWO_EVAP_KPA;
      targetCondKpa = TWO_COND_KPA;
      break;
  }
}

void updateChillerState() {
  compressor1State = digitalRead(INPUT_COMPRESSOR1);
  compressor2State = digitalRead(INPUT_COMPRESSOR2);
  if (!automaticMode) return;

  ChillerState newState;
  if (!compressor1State && !compressor2State) newState = STATE_STOP;
  else if (compressor1State != compressor2State) newState = STATE_ONE_COMPRESSOR;
  else newState = STATE_TWO_COMPRESSORS;

  if (newState != chillerState) {
    chillerState = newState;
    setTargetsForState(chillerState);
  }
}

void updatePressureSimulation() {
  unsigned long now = millis();
  if (lastRampUpdate == 0) {
    lastRampUpdate = now;
    return;
  }

  float dt = (now - lastRampUpdate) / 1000.0f;
  if (dt < 0.02f) return;

  lastRampUpdate = now;

  float speed = (chillerState == STATE_STOP)
      ? STOP_RAMP_KPA_PER_SEC
      : RUN_RAMP_KPA_PER_SEC;

  float step = speed * dt;

  currentEvapKpa = moveTowards(currentEvapKpa, targetEvapKpa, step);
  currentCondKpa = moveTowards(currentCondKpa, targetCondKpa, step);

  updateDAC();
}

String stateName() {
  if (!automaticMode) return "MANUAL";

  switch (chillerState) {
    case STATE_STOP: return "STOP";
    case STATE_ONE_COMPRESSOR: return "1 COMPRESSOR";
    case STATE_TWO_COMPRESSORS: return "2 COMPRESOARE";
  }

  return "UNKNOWN";
}

const char webpage[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="ro">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="Daikin EWAT">
<meta name="theme-color" content="#0d1117">
<link rel="manifest" href="/manifest.json">
<title>Daikin EWAT</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#0d1117;color:#fff;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif}
.wrap{max-width:720px;margin:auto;padding:18px}.title{font-size:28px;font-weight:700;margin:10px 0 18px}.card{background:#161b22;border:1px solid #30363d;border-radius:18px;padding:18px;margin-bottom:16px}
.state{font-size:28px;font-weight:700;margin-bottom:16px}.row{display:flex;gap:12px}.box{flex:1;background:#21262d;border-radius:14px;padding:16px;text-align:center}
.label{font-size:13px;color:#8b949e}.status{font-size:24px;font-weight:700;margin-top:8px}.on{color:#3fb950}.off{color:#8b949e}
.press{display:flex;gap:12px}.pbox{flex:1;text-align:center}.value{font-size:32px;font-weight:700;margin:10px 0}.small{font-size:13px;color:#8b949e;line-height:1.7}
input{width:100%;padding:12px;font-size:18px;border-radius:10px;border:1px solid #30363d;background:#0d1117;color:#fff}
button{width:100%;padding:14px;margin-top:12px;border:0;border-radius:12px;font-size:16px;font-weight:600;color:#fff;background:#1f6feb}.auto{background:#238636}.fw{background:#30363d}
</style>
</head>
<body>
<div class="wrap">
<div class="title">Daikin EWAT Simulator</div>

<div class="card">
<div id="state" class="state">...</div>
<div class="row">
<div class="box"><div class="label">Compressor 1</div><div id="c1" class="status">...</div></div>
<div class="box"><div class="label">Compressor 2</div><div id="c2" class="status">...</div></div>
</div>
</div>

<div class="card press">
<div class="pbox"><div class="label">Evaporator</div><div class="value"><span id="evap">0</span> kPa</div><div class="small">Target <span id="et">0</span> kPa<br>DAC <span id="ev">0</span> V</div></div>
<div class="pbox"><div class="label">Condenser</div><div class="value"><span id="cond">0</span> kPa</div><div class="small">Target <span id="ct">0</span> kPa<br>DAC <span id="cv">0</span> V</div></div>
</div>

<div class="card"><b>Logica automata</b><div class="small" style="margin-top:10px">
STOP: 1500 / 1500 kPa<br>
1 compresor: 800 / 1500 kPa<br>
2 compresoare: 900 / 1200 kPa
</div></div>

<div class="card"><b>Manual Override</b>
<p class="small">Evaporator kPa</p><input id="me" type="number" value="1500">
<p class="small">Condenser kPa</p><input id="mc" type="number" value="1500">
<button onclick="manualMode()">APPLY MANUAL</button>
<button class="auto" onclick="autoMode()">AUTOMATIC</button>
</div>

<div class="card"><b>System</b><div class="small" style="margin-top:10px">IP: <span id="ip">...</span></div>
<button class="fw" onclick="location.href='/update'">FIRMWARE UPDATE</button></div>
</div>

<script>
function manualMode(){fetch('/manual?evap='+me.value+'&cond='+mc.value)}
function autoMode(){fetch('/auto')}
function refresh(){
 fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{
  state.textContent=d.state;
  c1.textContent=d.comp1?'ON':'OFF'; c1.className='status '+(d.comp1?'on':'off');
  c2.textContent=d.comp2?'ON':'OFF'; c2.className='status '+(d.comp2?'on':'off');
  evap.textContent=d.evap.toFixed(0); cond.textContent=d.cond.toFixed(0);
  et.textContent=d.evapTarget.toFixed(0); ct.textContent=d.condTarget.toFixed(0);
  ev.textContent=d.evapVolt.toFixed(2); cv.textContent=d.condVolt.toFixed(2);
  ip.textContent=d.ip;
 }).catch(()=>state.textContent='CONNECTION LOST');
}
setInterval(refresh,1000);
refresh();
</script>
</body>
</html>
)rawliteral";

void handleManifest() {
  server.send(200, "application/manifest+json",
    "{\"name\":\"Daikin EWAT Simulator\",\"short_name\":\"Daikin EWAT\",\"start_url\":\"/\",\"display\":\"standalone\",\"background_color\":\"#0d1117\",\"theme_color\":\"#0d1117\"}");
}

void handleStatus() {
  String ip = (WiFi.status() == WL_CONNECTED)
      ? WiFi.localIP().toString()
      : WiFi.softAPIP().toString();

  String json = "{";
  json += "\"state\":\"" + stateName() + "\"";
  json += ",\"comp1\":" + String(compressor1State ? "true" : "false");
  json += ",\"comp2\":" + String(compressor2State ? "true" : "false");
  json += ",\"evap\":" + String(currentEvapKpa, 1);
  json += ",\"cond\":" + String(currentCondKpa, 1);
  json += ",\"evapTarget\":" + String(targetEvapKpa, 1);
  json += ",\"condTarget\":" + String(targetCondKpa, 1);
  json += ",\"evapVolt\":" + String(evapVoltage, 3);
  json += ",\"condVolt\":" + String(condVoltage, 3);
  json += ",\"ip\":\"" + ip + "\"}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleManual() {
  if (server.hasArg("evap")) targetEvapKpa = server.arg("evap").toFloat();
  if (server.hasArg("cond")) targetCondKpa = server.arg("cond").toFloat();

  targetEvapKpa = constrain(targetEvapKpa, EVAP_MIN_KPA, EVAP_MAX_KPA);
  targetCondKpa = constrain(targetCondKpa, COND_MIN_KPA, COND_MAX_KPA);

  automaticMode = false;
  server.send(200, "text/plain", "MANUAL");
}

void handleAutomatic() {
  automaticMode = true;

  compressor1State = digitalRead(INPUT_COMPRESSOR1);
  compressor2State = digitalRead(INPUT_COMPRESSOR2);

  if (!compressor1State && !compressor2State)
    chillerState = STATE_STOP;
  else if (compressor1State != compressor2State)
    chillerState = STATE_ONE_COMPRESSOR;
  else
    chillerState = STATE_TWO_COMPRESSORS;

  setTargetsForState(chillerState);

  server.send(200, "text/plain", "AUTO");
}

void handleUpdatePage() {
  server.send(200, "text/html",
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:-apple-system,Arial;background:#eee;padding:30px}.c{max-width:520px;margin:auto;background:white;padding:28px;border-radius:18px}"
    "button,input{font-size:18px;margin-top:16px}button{width:100%;padding:15px;background:#0a84ff;color:white;border:0;border-radius:12px}</style></head>"
    "<body><div class='c'><h1>ESP32 Firmware Update</h1><p>Selecteaza fisierul firmware .bin.</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin' required><button type='submit'>UPLOAD FIRMWARE</button>"
    "</form></div></body></html>");
}

void setupWebUpdate() {
  server.on("/update", HTTP_GET, handleUpdatePage);

  server.on("/update", HTTP_POST,
    []() {
      bool success = !Update.hasError();
      server.sendHeader("Connection", "close");

      if (success) {
        server.send(200, "text/html",
          "<h2>Update OK</h2><p>ESP32 se restarteaza...</p>");
        delay(1000);
        ESP.restart();
      } else {
        server.send(500, "text/plain", "Update failed");
      }
    },
    []() {
      HTTPUpload& upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          Update.printError(Serial);
      }
      else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          Update.printError(Serial);
      }
      else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true))
          Update.printError(Serial);
      }
    }
  );
}

void setupArduinoOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([](){ Serial.println("OTA START"); });
  ArduinoOTA.onEnd([](){ Serial.println("OTA COMPLETE"); });
  ArduinoOTA.begin();
}

void startWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  WiFi.softAP("Daikin-Simulator");

  // Refoloseste credentialele Wi-Fi salvate deja in NVS de firmware-ul anterior.
  WiFi.begin();

  unsigned long start = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < 20000UL
  ) {
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin(OTA_HOSTNAME);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(INPUT_COMPRESSOR1, INPUT_PULLDOWN);
  pinMode(INPUT_COMPRESSOR2, INPUT_PULLDOWN);

  updateDAC();

  startWiFi();
  setupArduinoOTA();

  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html; charset=utf-8", webpage);
  });

  server.on("/manifest.json", HTTP_GET, handleManifest);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/manual", HTTP_GET, handleManual);
  server.on("/auto", HTTP_GET, handleAutomatic);

  setupWebUpdate();

  server.begin();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  updateChillerState();
  updatePressureSimulation();
  delay(2);
}
