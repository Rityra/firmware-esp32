#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include <WiFiClientSecure.h>
#include <ArduinoMqttClient.h>

#include <BluetoothSerial.h>

// ==================== CHECK BLUETOOTH ====================

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// ==================== GLOBAL CONFIG ======================

// AP provisioning
static const char *AP_SSID = "smartThing";
static const char *AP_PASS = "12345678";

// Timeout and WiFi fallback
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint8_t WIFI_CONNECT_RETRIES = 2;

// Cooldown
uint32_t lastReconnectAttemptMs = 0;
static const uint32_t RECONNECT_COOLDOWN_MS = 2000;

// ==================== CONFIG MQTT / AWS IoT ====================

// AWS IoT endpoint
static const char *AWS_IOT_ENDPOINT = "a2eqw4se8i5px2-ats.iot.eu-west-1.amazonaws.com";
static const int AWS_IOT_PORT = 8883;

// Topic demo
static const char *TOPIC_LED_SET = "demo/led/set";
static const char *TOPIC_LED_STATE = "demo/led/state";
static const char *TOPIC_BUTTONS = "demo/buttons";
static const char *TOPIC_HEARTBEAT = "demo/heartbeat";

// ClientID MQTT -- currently allows smartThing-*
static const char *MQTT_CLIENT_ID = "smartThing-esp32";

// ==================== HW (LED + buttons) ======================
static const int LED_PIN = 2;
static const int BUTTON1_PIN = 12;
static const int BUTTON2_PIN = 14;
static const uint32_t DEBOUNCE_MS = 40;

struct ButtonState
{
  int pin;
  bool stableLevel;
  bool lastReadLevel;
  uint32_t lastChangeMs;
};

ButtonState b1{BUTTON1_PIN, true, true, 0};
ButtonState b2{BUTTON2_PIN, true, true, 0};

// ===================== CERTIFICATES ==========================
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char DEVICE_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUWAq7ryczMLO4Nv3Gq70iwR+OH0IwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDEwNTE2MDUw
NVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAOVuOrF0uVQ3lU6NnTCz
D2pKhNq8nweemqJ1pSJMSnS9laKLpoTqwHhm2MEYhh5/zCDel8eoM4hKRscLNf5I
psnHPxFuG9QGrUHqRuIJp7+D8quZ+5/ypFecTbIQ1weSxnaFzJnUvJqDLQgQhTqt
HpRb2bOfFQCqi58OIvVQ2XkscIkg5DvaHfxl9va3wtr/JCwVcvBVQKeGQGZ6R4os
wl9RLkRV9SJ2t9qA6J0BLqtkAT4NaG9FPxcprQbgY2I0uLYeOOkOZY+30SegV1Cs
ba5fBYlKvzHgddMa/Ia3mh2dhPjKU7QHvQC0pD0dyWqUvYYaMCzd1p3TrpBZvh/G
5MsCAwEAAaNgMF4wHwYDVR0jBBgwFoAUlaapnmT1iujyRPD3wil+q3mmAtQwHQYD
VR0OBBYEFPr4yN8mHTyJyIrf7r02nolPfgQDMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQB911VHSxDTfrNItFAs0XGPwoe/
FsgzThqHSTxGL8M43zlfDEliXsy3d0FTu+JDJKFJbgFSLa0hXFsIuVm64HkTVEQI
aWb3ZHsFl67Fa/OgRCahwRejkswyEn2m7eFPKNRNIaEwQk+Rvs7UYTzJwGzF4hbZ
pG8sCDImedRE6P1qXgAVVt+HmQnactFUOYeBe7yMVwkWorRLT1ta2ovt6Zdj9cyP
jMoSCHnN3hNLsGzaH0EGrYe22+M+80HCgO0kJnKPlq6VzV3ChrmyF0l1rfWGrSDq
3OlvM+hYvwUCeDCPQ9FIyD8F7Gd1RF6tVBi8Oel2FFUkb4t2GHVPd0/ZBMiE
-----END CERTIFICATE-----
)EOF";

static const char DEVICE_PRIVATE_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEA5W46sXS5VDeVTo2dMLMPakqE2ryfB56aonWlIkxKdL2Vooum
hOrAeGbYwRiGHn/MIN6Xx6gziEpGxws1/kimycc/EW4b1AatQepG4gmnv4Pyq5n7
n/KkV5xNshDXB5LGdoXMmdS8moMtCBCFOq0elFvZs58VAKqLnw4i9VDZeSxwiSDk
O9od/GX29rfC2v8kLBVy8FVAp4ZAZnpHiizCX1EuRFX1Ina32oDonQEuq2QBPg1o
b0U/FymtBuBjYjS4th446Q5lj7fRJ6BXUKxtrl8FiUq/MeB10xr8hreaHZ2E+MpT
tAe9ALSkPR3JapS9hhowLN3WndOukFm+H8bkywIDAQABAoIBAGvT14/pTOTQzsJB
ejwWG8uXxELNXWyGVSS/Z+Hrux6Mr5PaUGOttNGhfIOcFASniXd1Rh5W5glK52LK
2RBucrzogLKfbkLC9JDDG5dGdGllCRsvXnw43JY0ZqumkHkFuXKgxZ0Sx6VSu0pM
AWGnVXTiijmKLaPHg6E4UqO64Mq6fRLoY1YbSIVn6+1gYEJemTfjySQtx1NUoEFh
Wc9+ugzT57m6efpi8ITOQ34gNmr5wPe78+tHuPxFFr4K8DZHHFXmJgx0lMWeF+rb
nhieYP7sxBuaH4+oL2AhLT1sXUgJkfWOdijM9IC3QNCUxAbu47A6Au/pVYC5lWiP
p6A8kBECgYEA/OI29UijMVj/Bga7M80D/f3P9oL9iZunFOwUqTE9NImIlqqq5k0k
rGeNq84WEB1ZVDrKjMutwfAZpzLkYALOI6DKqqZx851TU8kogG+cJDtBwVbqII1M
Re3VEZblVMNMWQSHUmDL4q0gdHLhUbt7VHgk/GfJehzUqeBwPlsQU8kCgYEA6EIG
pbbLL8Ftdn28O0CwsMlFYtQH55NqLodCC0lUrK62QsspOCrHeB942XNJmVrAIZWq
3rQ500aLrhETdSM3DXUeqYqkFNc3Hx4xBKXup1/d92NOCLTr1Kq27u36wh7XgDj6
H+zz2TcqBUDm3Pgsh596BG/cPTxgWis8hTJY9fMCgYABuY1EaQVr7b53jQ4z0T8o
uVV0yfnuuVgPZXjSmppfRFzhIjLeNUJ9xosJ0/MEkx6r22bMRQtfWkLiiA9UtFAx
oAQh29bFIZqS5LlY/uniuWC+b9hS9oPBqlicsRnJVTetF8HQhwZ8+k9RXAM7anrx
3YvrCM4v/ghZTEWXt0lKOQKBgDMyCqEQj+0w594MjSdKdSgi1YaZUjy7g9ZJvkca
nO2LfWVvvuBAncNevJbCEOEa3esJQKbidzSZAkUWQoG5h3MR8/ZPNyfY4K+dShTK
NrbyLM1ROGw25Rz1nGlCT9nmTn5XENtkscOZo0uRKNJgagBjKNQ9jAB7hImbqjV6
CZ39AoGAEQ/ALcRqi0b2ok+92Vb5I/dtHHAGTbqxNMFOohgTe7XQ/84am/wfBV4M
RVGHVX4AiTeI5yHBD4G1q2syrMlKoWDr5QSIuJrrEzTWNXfjHkYPjPE2IJWhgVIf
LDRVBquGud1K3lDouofwQSajhMwMU7mPujy2NFIfkeslqSbUj0Y=
-----END RSA PRIVATE KEY-----
)EOF";

// ===================== GLOBALS ================================
WebServer server(80);
BluetoothSerial SerialBT;
Preferences prefs;

String savedSsid;
String savedPwd;

enum class NetMode
{
  STA_OK,
  AP_MODE,
  BT_CLS
};
NetMode mode = NetMode::AP_MODE;

// MQTT client
WiFiClientSecure tlsClient;
MqttClient mqttClient(tlsClient);
bool mqttReady = false;

// actions to be performed after HTTP connection
// avoid closing connection while engaging with POST endpoint
volatile bool pendingStopAp = false;
volatile bool pendingMqtt = false;
volatile bool pendingRestartAp = false;
volatile bool pendingReboot = false;

// ==================== UTILITIES ==============================

const char *wifiStatusToStr(wl_status_t st)
{
  switch (st)
  {
  case WL_IDLE_STATUS:
    return "IDLE";
  case WL_NO_SSID_AVAIL:
    return "NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED:
    return "SCAN_COMPLETED";
  case WL_CONNECTED:
    return "CONNECTED";
  case WL_CONNECT_FAILED:
    return "CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "CONNECTION_LOST";
  case WL_DISCONNECTED:
    return "DISCONNECTED";
  default:
    return "UNKNOWN";
  }
}

void sendJson(int code, const JsonDocument &doc)
{
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

// ==================== EVENTI WIFI (reason code) ====================
void attachWiFiEvents()
{
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
               {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.println(info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.println(WiFi.localIP());
    } else if (event == ARDUINO_EVENT_WIFI_AP_START) {
    } else if (event == ARDUINO_EVENT_WIFI_AP_STOP) {
    } });
}

// ==================== NVS: load/save/clear ====================
void loadCreds()
{
  prefs.begin("wifi", true);
  savedSsid = prefs.getString("ssid", "");
  savedPwd = prefs.getString("pwd", "");
  prefs.end();

  if (savedSsid.length() > 0)
  {
    Serial.println(savedSsid);
  }
  else
  {
  }
}

void saveCreds(const String &ssid, const String &pwd)
{
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pwd", pwd);
  prefs.end();
}

void clearCreds()
{
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pwd");
  prefs.end();
}

// ==================== WIFI CONNECT (persistent) ==========================
bool connectSTA(const char *ssid, const char *pwd, uint32_t timeoutMs)
{
  Serial.println(ssid);
  Serial.println(timeoutMs);

  WiFi.persistent(false);
  // IMPORTANt: when AP provisioning, DO NOT switch to WIFI_STA
  // (terminates AP and access to POST)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Avoid disconnect(true) (it might reset and terminate AP and/or HTTP connection)
  // disconnect(false,false) terminated STA only
  WiFi.disconnect(false, false);
  delay(200);

  WiFi.begin(ssid, pwd);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs)
  {
    delay(200);
  }

  wl_status_t st = WiFi.status();
  Serial.println(wifiStatusToStr(st));

  if (st == WL_CONNECTED)
  {
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.RSSI());
    return true;
  }
  return false;
}

// ==================== AP start/stop ==================================
void stopAPKeepSTA()
{

  WiFi.mode(WIFI_AP_STA);
  delay(100);

  bool r = WiFi.softAPdisconnect(true);

  delay(100);
  WiFi.mode(WIFI_STA);
}

// ==================== MQTT ====================

void publishText(const char *topic, const String &payload)
{
  if (!mqttClient.connected())
    return;
  mqttClient.beginMessage(topic);
  mqttClient.print(payload);
  mqttClient.endMessage();
}

void publishJson(const char *topic, const JsonDocument &doc)
{
  if (!mqttClient.connected())
    return;
  String out;
  serializeJson(doc, out);
  publishText(topic, out);
}

void setLed(bool on)
{
  digitalWrite(LED_PIN, on ? HIGH : LOW);

  StaticJsonDocument<128> doc;
  doc["led"] = on;
  doc["ts"] = (uint32_t)millis();
  publishJson(TOPIC_LED_STATE, doc);
}

bool parseLedCommand(const String &payload, bool &outOn)
{
  String p = payload;
  p.trim();
  p.toLowerCase();

  // Plain Text case, e.g. "on"
  if (p == "on" || p == "1" || p == "true")
  {
    outOn = true;
    return true;
  }
  if (p == "off" || p == "0" || p == "false")
  {
    outOn = false;
    return true;
  }

  // JSON case, e.g. { "led" : true }
  StaticJsonDocument<128> doc;
  DeserializationError e = deserializeJson(doc, p);
  if (e)
    return false;

  if (doc.containsKey("led"))
  {
    outOn = doc["led"].as<bool>();
    return true;
  }
  return false;
}

void onMqttMessage(int messageSize)
{
  String topic = mqttClient.messageTopic();

  // Read entire payload
  String payload;
  while (mqttClient.available())
  {
    payload += (char)mqttClient.read();
  }

  Serial.println(topic);
  Serial.println(payload);

  if (topic == TOPIC_LED_SET)
  {
    bool on;
    if (parseLedCommand(payload, on))
    {
      setLed(on);
    }
    else
    {
    }
  }
}

bool mqttConnect()
{
  tlsClient.setCACert(AWS_ROOT_CA);
  tlsClient.setCertificate(DEVICE_CERT);
  tlsClient.setPrivateKey(DEVICE_PRIVATE_KEY);

  Serial.print(AWS_IOT_ENDPOINT);
  Serial.print(":");
  Serial.println(AWS_IOT_PORT);

  mqttClient.setId(MQTT_CLIENT_ID);

  if (!mqttClient.connect(AWS_IOT_ENDPOINT, AWS_IOT_PORT))
  {
    Serial.println(mqttClient.connectError());
    return false;
  }

  mqttClient.onMessage(onMqttMessage);

  Serial.println(TOPIC_LED_SET);
  mqttClient.subscribe(TOPIC_LED_SET);

  // Initial Heartbeat
  StaticJsonDocument<128> hb;
  hb["status"] = "online";
  hb["ip"] = WiFi.localIP().toString();
  hb["ts"] = (uint32_t)millis();
  publishJson(TOPIC_HEARTBEAT, hb);

  return true;
}

// ==================== BUTTONS ====================
void handleButton(ButtonState &b, int id)
{
  bool readLevel = digitalRead(b.pin);

  if (readLevel != b.lastReadLevel)
  {
    b.lastReadLevel = readLevel;
    b.lastChangeMs = millis();
  }

  if ((millis() - b.lastChangeMs) >= DEBOUNCE_MS && readLevel != b.stableLevel)
  {
    b.stableLevel = readLevel;

    bool pressed = (b.stableLevel == LOW);

    StaticJsonDocument<128> doc;
    doc["button"] = id;
    doc["pressed"] = pressed;
    doc["ts"] = (uint32_t)millis();

    publishJson(TOPIC_BUTTONS, doc);
  }
}

// ==================== AP WEB SERVER ====================
void startAP()
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress apIP = WiFi.softAPIP();

  server.stop();
  server.close();

  server.on("/", HTTP_GET, [apIP]()
            {

    StaticJsonDocument<512> doc;
    doc["device"] = "smartThing";
    doc["mode"] = "AP_MODE";
    doc["ap_ip"] = apIP.toString();
    doc["post_to"] = "/wifi";
    doc["example"] = "{\"ssid\":\"YourWiFi2.4G\",\"pwd\":\"YourPass\"}";
    doc["clear_saved"] = "POST /clear";
    sendJson(200, doc); });

  server.on("/wifi", HTTP_POST, []()
            {

  if (!server.hasArg("plain")) {
    StaticJsonDocument<180> err;
    err["ok"] = false;
    err["error"] = "Body is missing (plain)";
    sendJson(400, err);
    return;
  }

  String body = server.arg("plain");

  StaticJsonDocument<256> doc;
  DeserializationError e = deserializeJson(doc, body);
  if (e) {
    StaticJsonDocument<240> err;
    err["ok"] = false;
    err["error"] = "JSON not valid";
    err["detail"] = e.c_str();
    sendJson(400, err);
    return;
  }

  const char* ssid = doc["ssid"];
  const char* pwd  = doc["pwd"];

  if (!ssid || String(ssid).length() == 0) {
    StaticJsonDocument<200> err;
    err["ok"] = false;
    err["error"] = "Missing or empty 'ssid'";
    sendJson(400, err);
    return;
  }
  if (!pwd) pwd = "";

  Serial.print("[PROV] SSID received: ");
  Serial.println(ssid);

  // Save credentials and respond (DO NOT connect right away)
  saveCreds(ssid, pwd);
  savedSsid = ssid;
  savedPwd  = pwd;

  StaticJsonDocument<384> resp;
  resp["ok"] = true;
  resp["ssid"] = ssid;
  resp["status"] = "rebooting";
  resp["note"] = "Device rebooting";
  sendJson(200, resp);
  delay(50);
  pendingReboot = true; });

  // Route /clear
  server.on("/clear", HTTP_POST, []()
            {

    clearCreds();
    savedSsid = "";
    savedPwd  = "";

    StaticJsonDocument<180> resp;
    resp["ok"] = true;
    resp["status"] = "deleted";
    sendJson(200, resp); });

  server.begin();
}

void serialBTCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  mode = NetMode::BT_CLS;
  mqttReady = mqttConnect();
}

// ==================== SETUP / LOOP ==================================
void setup()
{
  Serial.begin(115200);
  delay(250);

  // GPIO
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  attachWiFiEvents();
  loadCreds();

  // When stored credentials exist: try connecting to STA with retry fallback
  if (savedSsid.length() > 0)
  {

    bool ok = false;
    for (uint8_t i = 1; i <= WIFI_CONNECT_RETRIES; i++)
    {
      ok = connectSTA(savedSsid.c_str(), savedPwd.c_str(), WIFI_CONNECT_TIMEOUT_MS);
      if (ok)
        break;
      delay(1000);
    }

    if (ok)
    {
      mode = NetMode::STA_OK;
      mqttReady = mqttConnect();
      return;
    }
  }
  else
  {
  }

  SerialBT.register_callback(serialBTCallback);

  if (!SerialBT.begin("smartThing"))
  {
  }
  else
  {
  }

  // Fallback: AP provisioning
  mode = NetMode::AP_MODE;
  startAP();
}

void loop()
{
  if (mode == NetMode::AP_MODE)
  {
    server.handleClient();

    if (pendingReboot)
    {
      pendingReboot = false;
      delay(800);
      ESP.restart();
    }

    delay(10);
    return;
  }

  if (mode == NetMode::BT_CLS)
  {
    if (Serial.available())
    {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available())
    {
      Serial.write(SerialBT.read());
    }
  }

  // STA_OK mode:
  // 1) MQTT poll
  if (mqttClient.connected())
  {
    mqttClient.poll();
  }
  else
  {
    // when MQTT fails, retry every 5s
    static uint32_t lastMqttRetry = 0;
    if (millis() - lastMqttRetry > 5000)
    {
      lastMqttRetry = millis();
      mqttReady = mqttConnect();
    }
  }

  // 2) Buttons management
  handleButton(b1, 1);
  handleButton(b2, 2);

  // 3) when WiFi fails: retry, then switch to AP
  if (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - lastReconnectAttemptMs > RECONNECT_COOLDOWN_MS)
    {
      lastReconnectAttemptMs = millis();

      bool ok = connectSTA(savedSsid.c_str(), savedPwd.c_str(), WIFI_CONNECT_TIMEOUT_MS);
      if (!ok)
      {
        mode = NetMode::AP_MODE;
        startAP();
      }
      else
      {
        mqttReady = mqttConnect();
      }
    }
  }

  delay(10);
}