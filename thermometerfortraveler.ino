#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <vector>
#include <Preferences.h>

// --- LCD 16x2 ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- DS18B20 ---
#include <OneWire.h>
#include <DallasTemperature.h>

// --- DHT11 ---
#include <DHT.h>

// -------------------------------------------------------------------
// 1. HARDWARE & NTC CONFIGURATION
// -------------------------------------------------------------------
const int NTC_PIN    = 34;
const float R_REF    = 100000;
const int ADC_MAX_VALUE    = 4095;
const int ADC_MIN_THRESHOLD = 5;

// --- DS18B20 ---
#define DS18B20_PIN 4           // GPIO untuk DS18B20 (ubah sesuai wiring)
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
float tempDS18B20 = -99.0;

unsigned long lastDSRequest = 0;
bool dsWaiting = false;

// --- DHT11 ---
#define DHT_PIN  5              // GPIO untuk DHT11 (ubah sesuai wiring)
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
float tempDHT11   = -99.0;
float humidDHT11  = -99.0;

// --- LCD ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define BUTTON_PIN 18

int currentPage = 0;
bool lastButtonState = HIGH;

unsigned long startupTime = 0;
bool showStartupScreen = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 40;

// -------------------------------------------------------------------
// 2. WEB SERVER & LOGGING CONFIGURATION
// -------------------------------------------------------------------
const char *ssid     = "ESP32_NTC_LOGGER";
const char *password = "123456789";
WebServer server(80);
Preferences preferences;

const unsigned long LOG_INTERVAL_MS = 30000;

bool isLogging = false;
float tempC    = -99.0;
std::vector<String> logData;

unsigned long logStartTime = 0;
unsigned long lastLogTime  = 0;

// Steinhart-Hart Constants
float current_A = 0.00120;
float current_B = 0.00020;
float current_C = 0.0000001;
float last_R_NTC_measured = 0.0;

// Three-Point Calibration Storage
float CAL_T1_C = 0.0, CAL_R1 = 0.0;
float CAL_T2_C = 0.0, CAL_R2 = 0.0;
float CAL_T3_C = 0.0, CAL_R3 = 0.0;

// -------------------------------------------------------------------
// 3. CORE FUNCTIONS
// -------------------------------------------------------------------

void updateLCD()
{
    if(showStartupScreen)
    {
        if(millis() - startupTime < 2000)
            return;

        showStartupScreen = false;
        lcd.clear();
    }

    static int lastPage = -1;

    if(lastPage != currentPage)
    {
        lcd.clear();
        lastPage = currentPage;
    }

    switch(currentPage)
    {
        case 0: // DHT11
        {
            lcd.setCursor(0,0);
            lcd.print("T:");
            lcd.print(tempDHT11,1);
            lcd.print((char)223);
            lcd.print("C");

            lcd.setCursor(0,1);
            lcd.print("H:");
            lcd.print(humidDHT11,0);
            lcd.print("%");
            break;
        }

        case 1: // DS18B20
        {
            lcd.setCursor(0,0);
            lcd.print("DS18B20 Temp");

            lcd.setCursor(0,1);
            lcd.print(tempDS18B20,1);
            lcd.print((char)223);
            lcd.print("C");
            break;
        }

        case 2: // NTC
        {
            lcd.setCursor(0,0);
            lcd.print("NTC Temp");

            lcd.setCursor(0,1);
            lcd.print(tempC,1);
            lcd.print((char)223);
            lcd.print("C");
            break;
        }

        case 3: // Logging
        {
            lcd.setCursor(0,0);

            if(isLogging)
                lcd.print("Logging: ON ");
            else
                lcd.print("Logging:OFF");

            lcd.setCursor(0,1);
            lcd.print("Data:");
            lcd.print(logData.size());
            break;
        }
    }
}

bool solveMatrix3x3(float matrix[3][3], float vector[3], float result[3]) {
    float detA = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) -
                 matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[2][0] * matrix[1][2]) +
                 matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1]);

    if (fabs(detA) < 1e-9) return false;

    float detA_A = vector[0] * (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) -
                   matrix[0][1] * (vector[1] * matrix[2][2] - vector[2] * matrix[1][2]) +
                   matrix[0][2] * (vector[1] * matrix[2][1] - vector[2] * matrix[1][1]);
    result[0] = detA_A / detA;

    float detA_B = matrix[0][0] * (vector[1] * matrix[2][2] - vector[2] * matrix[1][2]) -
                   vector[0] * (matrix[1][0] * matrix[2][2] - matrix[2][0] * matrix[1][2]) +
                   matrix[0][2] * (matrix[1][0] * vector[2] - matrix[2][0] * vector[1]);
    result[1] = detA_B / detA;

    float detA_C = matrix[0][0] * (matrix[1][1] * vector[2] - matrix[2][1] * vector[1]) -
                   matrix[0][1] * (matrix[1][0] * vector[2] - matrix[2][0] * vector[1]) +
                   vector[0] * (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1]);
    result[2] = detA_C / detA;

    return true;
}

float readNtcResistance() {
    long rawAdcSum = 0;
    for (int i = 0; i < 100; i++) {
        rawAdcSum += analogRead(NTC_PIN);
        delay(1);
    }
    float adcValue = (float)rawAdcSum / 100.0;
    if (adcValue > (ADC_MAX_VALUE - ADC_MIN_THRESHOLD) || adcValue < ADC_MIN_THRESHOLD) return 0.0;

    float R_NTC = R_REF * adcValue / ((float)ADC_MAX_VALUE - adcValue);
    last_R_NTC_measured = R_NTC;
    return R_NTC;
}

void readTemperature() {
    // --- NTC ---
    float R_NTC = readNtcResistance();
    if (R_NTC < 1.0) {
        tempC = -99.0;
    } else {
        float lnR  = log(R_NTC);
        float lnR3 = lnR * lnR * lnR;
        float inv_T = current_A + (current_B * lnR) + (current_C * lnR3);
        if (inv_T == 0.0) {
            tempC = -99.0;
        } else {
            tempC = (1.0 / inv_T) - 273.15;
        }
    }

    // --- DS18B20 ---
    if(!dsWaiting)
    {
        ds18b20.requestTemperatures();
        lastDSRequest = millis();
        dsWaiting = true;
    }
    else if(millis() - lastDSRequest >= 750)
    {
        float t = ds18b20.getTempCByIndex(0);
        tempDS18B20 = (t == DEVICE_DISCONNECTED_C) ? -99.0 : t;
        dsWaiting = false;
    }

    // --- DHT11 ---
    float h = dht.readHumidity();
    float t2 = dht.readTemperature();  // Celsius
    if (isnan(h) || isnan(t2)) {
        tempDHT11  = -99.0;
        humidDHT11 = -99.0;
    } else {
        tempDHT11  = t2;
        humidDHT11 = h;
    }

    // --- LOGGING ---
    if (isLogging) {
        unsigned long currentTime = millis();
        if ((currentTime - lastLogTime) >= LOG_INTERVAL_MS) {
            float relSec = (float)(currentTime - logStartTime) / 1000.0;
            // Format: time, NTC, DS18B20, DHT11_temp, DHT11_humidity
            String logEntry = String(relSec, 2) + "," +
                              String(tempC, 2) + "," +
                              String(tempDS18B20, 2) + "," +
                              String(tempDHT11, 2) + "," +
                              String(humidDHT11, 2);
            logData.push_back(logEntry);
            lastLogTime = currentTime;
        }
    }

    static unsigned long lastLCD = 0;

    if(millis() - lastLCD > 1000)
    {
        updateLCD();
        lastLCD = millis();
    }
}

// -------------------------------------------------------------------
// 4. WEB SERVER HANDLERS
// -------------------------------------------------------------------

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NTC + DS18B20 + DHT11 Logger</title>
<style>
  body{text-align:center;font-family:Arial,sans-serif;background:#f0f2f5;}
  h1,h2,h3{color:#333;}
  .card{display:inline-block;padding:20px 30px;background:white;border-radius:10px;
        box-shadow:0 4px 8px rgba(0,0,0,.1);margin:10px;}
  .sensor-grid{display:flex;flex-wrap:wrap;justify-content:center;margin-bottom:20px;}
  .big-val{font-size:3.5em;font-weight:bold;color:#007bff;}
  .sensor-label{font-size:1em;color:#6c757d;margin-bottom:5px;}
  .unit{font-size:.5em;font-weight:normal;color:#6c757d;}
  .badge-ok{color:green;} .badge-err{color:red;}
  .btn{padding:12px 25px;font-size:1.2em;margin:10px;cursor:pointer;border:none;
       border-radius:5px;transition:background-color .3s;}
  #logBtn{background:#28a745;color:white;}
  #logBtn:hover{background:#218838;}
  #status{color:#6c757d;font-size:1.1em;}
  #calibration-form{padding:15px;background:#e9ecef;border-radius:5px;
                    margin:20px auto;width:90%;max-width:600px;text-align:left;}
  #calibration-form input[type=number]{width:90%;padding:5px;}
  .btn-calib{width:100%;margin-top:10px;background:#ffc107;color:black;font-size:1.1em;
             padding:10px;border:none;border-radius:5px;cursor:pointer;}
  .calib-point{padding:10px;border-bottom:1px solid #ccc;}
  .point-captured{color:green;font-weight:bold;}
</style>
</head>
<body>
<h1>ESP32 Multi-Sensor Monitor</h1>

<div class="sensor-grid">

  <!-- NTC -->
  <div class="card">
    <div class="sensor-label">🌡️ NTC Thermistor</div>
    <div class="big-val"><span id="temp">--.-</span><span class="unit"> °C</span></div>
    <div style="font-size:.85em;color:#888;">R<sub>NTC</sub>: <span id="liveResistance">--.-</span> Ω</div>
  </div>

  <!-- DS18B20 -->
  <div class="card">
    <div class="sensor-label">🌡️ DS18B20 (Digital)</div>
    <div class="big-val" style="color:#fd7e14;"><span id="tempDS">--.-</span><span class="unit"> °C</span></div>
    <div id="ds_status" style="font-size:.85em;color:#888;">–</div>
  </div>

  <!-- DHT11 -->
  <div class="card">
    <div class="sensor-label">🌡️💧 DHT11</div>
    <div class="big-val" style="color:#20c997;"><span id="tempDHT">--.-</span><span class="unit"> °C</span></div>
    <div style="font-size:1.2em;color:#6f42c1;">Humidity: <span id="humDHT">--.-</span> %</div>
    <div id="dht_status" style="font-size:.85em;color:#888;">–</div>
  </div>

</div>

<p id="status">Status: Waiting...</p>
<p>Log Entries: <span id="logCount">0</span> (every 30 s, CSV: time, NTC, DS18B20, DHT_temp, DHT_hum)</p>
<button class="btn" onclick="toggleLogging()" id="logBtn">Start Logging</button>

<hr>

<h2>⚙️ 3-Point Steinhart-Hart Calibration (NTC)</h2>
<div id="calibration-form">
  <p style="text-align:center;">Live R<sub>NTC</sub>: <span id="liveResistance2">--.-</span> Ω</p>
  <p style="text-align:center;font-weight:bold;">
    A=<span id="aValue"></span> &nbsp; B=<span id="bValue"></span> &nbsp; C=<span id="cValue"></span>
  </p>

  <div class="calib-point">
    <h3>Step 1 – Cold <span id="p1_status" class="point-captured"></span></h3>
    <form id="form1" onsubmit="capturePoint(1);return false;">
      T1 (°C): <input type="number" step="0.1" id="t1_input" placeholder="e.g. 0.0" required>
      <input type="submit" class="btn-calib" value="Capture Point 1">
    </form>
  </div>

  <div class="calib-point">
    <h3>Step 2 – Lukewarm <span id="p2_status" class="point-captured"></span></h3>
    <form id="form2" onsubmit="capturePoint(2);return false;">
      T2 (°C): <input type="number" step="0.1" id="t2_input" placeholder="e.g. 25.0" required>
      <input type="submit" class="btn-calib" value="Capture Point 2">
    </form>
  </div>

  <div class="calib-point">
    <h3>Step 3 – Hot <span id="p3_status" class="point-captured"></span></h3>
    <form id="form3" onsubmit="capturePoint(3);return false;">
      T3 (°C): <input type="number" step="0.1" id="t3_input" placeholder="e.g. 60.0" required>
      <input type="submit" class="btn-calib" style="background:#007bff;color:white;"
             value="Capture Point 3 & Calculate A,B,C">
    </form>
  </div>

  <p style="text-align:center;font-size:.8em;margin-top:20px;">
    *Gunakan tiga titik suhu yang berjauhan untuk hasil terbaik.
  </p>
</div>

<script>
let isLogging = false;

function fetchData(){
  var x = new XMLHttpRequest();
  x.onreadystatechange = function(){
    if(this.readyState == 4 && this.status == 200){
      var d = this.responseText.split(',');
      // d[0]=NTC, d[1]=logCount, d[2]=A, d[3]=B, d[4]=C, d[5]=R_NTC, d[6]=DS18B20, d[7]=DHT_temp, d[8]=DHT_hum
      document.getElementById('temp').innerHTML = parseFloat(d[0]).toFixed(2);
      document.getElementById('logCount').innerHTML = d[1];
      document.getElementById('aValue').innerHTML = parseFloat(d[2]).toExponential(5);
      document.getElementById('bValue').innerHTML = parseFloat(d[3]).toExponential(5);
      document.getElementById('cValue').innerHTML = parseFloat(d[4]).toExponential(5);
      document.getElementById('liveResistance').innerHTML = parseFloat(d[5]).toFixed(2);
      document.getElementById('liveResistance2').innerHTML = parseFloat(d[5]).toFixed(2);

      var ds = parseFloat(d[6]);
      document.getElementById('tempDS').innerHTML = ds <= -98 ? 'ERR' : ds.toFixed(2);
      document.getElementById('ds_status').innerHTML = ds <= -98
        ? '<span class="badge-err">Sensor not found / disconnected</span>'
        : '<span class="badge-ok">OK</span>';

      var dt = parseFloat(d[7]);
      var dh = parseFloat(d[8]);
      document.getElementById('tempDHT').innerHTML = dt <= -98 ? 'ERR' : dt.toFixed(1);
      document.getElementById('humDHT').innerHTML  = dh <= -98 ? 'ERR' : dh.toFixed(1);
      document.getElementById('dht_status').innerHTML = dt <= -98
        ? '<span class="badge-err">Sensor not found / no data</span>'
        : '<span class="badge-ok">OK</span>';

      document.getElementById('status').innerHTML = isLogging
        ? 'Status: <span style="color:green;">LOGGING</span>'
        : 'Status: <span style="color:red;">Stopped</span>';
    }
  };
  x.open('GET','/data',true); x.send();
}

function capturePoint(n){
  var tv = document.getElementById('t'+n+'_input').value;
  if(tv===""){alert("Masukkan suhu untuk Point "+n);return;}
  var ep = ['/capture_point_1','/capture_point_2','/capture_point_3_and_solve'][n-1];
  var x = new XMLHttpRequest();
  x.onreadystatechange = function(){
    if(this.readyState==4 && this.status==200){
      if(this.responseText.startsWith("ERROR")){
        alert("Calibration Error: "+this.responseText);
      } else {
        document.getElementById('p'+n+'_status').innerHTML='(CAPTURED)';
        document.getElementById('form'+n).style.display='none';
        if(n===3){alert("Kalibrasi selesai! Koefisien A, B, C disimpan.");fetchData();}
        else{alert("Point "+n+" berhasil diambil!");}
      }
    }
  };
  x.open('GET',ep+'?temp='+tv,true); x.send();
}

function toggleLogging(){
  if(isLogging){
    window.location.href='/logstop';
    document.getElementById('logBtn').innerHTML='Start Logging';
    isLogging=false;
    document.getElementById('status').innerHTML='Status: Download Initiated.';
  } else {
    var x=new XMLHttpRequest(); x.open('GET','/logstart',true); x.send();
    document.getElementById('logBtn').innerHTML='Stop & Download Log';
    isLogging=true;
    document.getElementById('status').innerHTML='Status: LOGGING...';
  }
}

setInterval(fetchData, 2000);
fetchData();
</script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleData() {
    // NTC, logCount, A, B, C, R_NTC, DS18B20_temp, DHT_temp, DHT_hum
    String response = String(tempC, 2)              + "," +
                      String(logData.size() > 1 ? logData.size() - 1 : 0) + "," +
                      String(current_A, 8)          + "," +
                      String(current_B, 8)          + "," +
                      String(current_C, 8)          + "," +
                      String(last_R_NTC_measured, 2) + "," +
                      String(tempDS18B20, 2)         + "," +
                      String(tempDHT11, 2)           + "," +
                      String(humidDHT11, 2);
    server.send(200, "text/plain", response);
}

void handleLogStart() {
    isLogging = true;
    logData.clear();
    logData.push_back("Time(s),NTC_Temp(C),DS18B20_Temp(C),DHT11_Temp(C),DHT11_Humidity(%)");
    logStartTime = millis();
    lastLogTime  = logStartTime;
    server.send(200, "text/plain", "Logging Started");
}

void handleLogStop() {
    isLogging = false;
    String csvData = "";
    for (const String& entry : logData) csvData += entry + "\n";

    if (logData.size() > 1) {
        server.setContentLength(csvData.length());
        server.sendHeader("Content-Disposition", "attachment; filename=\"temperature_log.csv\"");
        server.sendHeader("Cache-Control", "no-cache");
        server.send(200, "text/csv", csvData);
    } else {
        server.send(200, "text/plain", "Logging Stopped. No data recorded.");
    }
    logData.clear();
}

void handleCapturePoint1() {
    if (!server.hasArg("temp")) { server.send(400,"text/plain","ERROR: Missing temp."); return; }
    CAL_T1_C = server.arg("temp").toFloat();
    CAL_R1   = readNtcResistance();
    if (CAL_R1 < 1.0) { server.send(500,"text/plain","ERROR: Invalid R1."); return; }
    Serial.printf("Point 1: T=%.2fC, R=%.2f\n", CAL_T1_C, CAL_R1);
    server.send(200,"text/plain","Point 1 Captured R="+String(CAL_R1,2));
}

void handleCapturePoint2() {
    if (!server.hasArg("temp")) { server.send(400,"text/plain","ERROR: Missing temp."); return; }
    if (CAL_R1 < 1.0) { server.send(500,"text/plain","ERROR: Capture Point 1 first."); return; }
    CAL_T2_C = server.arg("temp").toFloat();
    CAL_R2   = readNtcResistance();
    if (CAL_R2 < 1.0) { server.send(500,"text/plain","ERROR: Invalid R2."); return; }
    Serial.printf("Point 2: T=%.2fC, R=%.2f\n", CAL_T2_C, CAL_R2);
    server.send(200,"text/plain","Point 2 Captured R="+String(CAL_R2,2));
}

void handleCapturePoint3AndSolve() {
    if (!server.hasArg("temp")) { server.send(400,"text/plain","ERROR: Missing temp."); return; }
    if (CAL_R1 < 1.0 || CAL_R2 < 1.0) { server.send(500,"text/plain","ERROR: Capture Points 1 and 2 first."); return; }

    CAL_T3_C = server.arg("temp").toFloat();
    CAL_R3   = readNtcResistance();
    if (CAL_R3 < 1.0) { server.send(500,"text/plain","ERROR: Invalid R3."); return; }
    Serial.printf("Point 3: T=%.2fC, R=%.2f\n", CAL_T3_C, CAL_R3);

    float T1K = CAL_T1_C + 273.15, T2K = CAL_T2_C + 273.15, T3K = CAL_T3_C + 273.15;
    float lnR1 = log(CAL_R1), lnR2 = log(CAL_R2), lnR3 = log(CAL_R3);

    float M[3][3] = {
        {1.0, lnR1, lnR1*lnR1*lnR1},
        {1.0, lnR2, lnR2*lnR2*lnR2},
        {1.0, lnR3, lnR3*lnR3*lnR3}
    };
    float V[3] = {1.0f/T1K, 1.0f/T2K, 1.0f/T3K};
    float R[3];

    if (!solveMatrix3x3(M, V, R)) {
        server.send(500,"text/plain","ERROR: Calculation failed. Suhu terlalu berdekatan.");
        return;
    }

    current_A = R[0]; current_B = R[1]; current_C = R[2];
    preferences.putFloat("sh_A", current_A);
    preferences.putFloat("sh_B", current_B);
    preferences.putFloat("sh_C", current_C);

    Serial.printf("--- KALIBRASI BERHASIL ---\nA: %.8f\nB: %.8f\nC: %.8f\n", current_A, current_B, current_C);
    server.send(200,"text/plain","Calculation Complete and Saved.");
}

void handleNotFound() { server.send(404,"text/plain","Not Found"); }

// -------------------------------------------------------------------
// 5. SETUP AND LOOP
// -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(100);

  // Load saved Steinhart-Hart constants
  preferences.begin("ntc-calib", false);
  current_A = preferences.getFloat("sh_A", current_A);
  current_B = preferences.getFloat("sh_B", current_B);
  current_C = preferences.getFloat("sh_C", current_C);
  Serial.printf("Loaded: A=%.8f, B=%.8f, C=%.8f\n", current_A, current_B, current_C);

  // ADC setup
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // DS18B20 init
  ds18b20.begin();
  delay(200);
  Serial.printf("DS18B20: %d device(s) found\n", ds18b20.getDeviceCount());

  // DHT11 init
  dht.begin();
  Serial.println("DHT11 initialized");

  // WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // Web server routes
  server.on("/",                           HTTP_GET, handleRoot);
  server.on("/data",                       HTTP_GET, handleData);
  server.on("/logstart",                   HTTP_GET, handleLogStart);
  server.on("/logstop",                    HTTP_GET, handleLogStop);
  server.on("/capture_point_1",            HTTP_GET, handleCapturePoint1);
  server.on("/capture_point_2",            HTTP_GET, handleCapturePoint2);
  server.on("/capture_point_3_and_solve",  HTTP_GET, handleCapturePoint3AndSolve);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Thermometer");
  lcd.setCursor(0,1);
  lcd.print("For Traveler");

  startupTime = millis();
  showStartupScreen = true;
}

void loop() {
  readTemperature();      // baca semua sensor (NTC + DS18B20 + DHT11)
  server.handleClient();

  static bool buttonPressed = false;

bool buttonState = digitalRead(BUTTON_PIN);

if(buttonState == LOW && !buttonPressed)
{
    buttonPressed = true;

    currentPage++;

    if(currentPage > 3)
        currentPage = 0;

    updateLCD();

    Serial.print("Page: ");
    Serial.println(currentPage);
}

if(buttonState == HIGH)
{
    buttonPressed = false;
}
  yield();
}