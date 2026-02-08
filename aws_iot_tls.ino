#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

// ===================== CONFIG =====================
static const char* WIFI_SSID     = "Sunrise_3228896";
static const char* WIFI_PASSWORD = "kbno4mwf4Uaavwtp";

static const char* AWS_IOT_ENDPOINT = "a2eqw4se8i5px2-ats.iot.eu-west-1.amazonaws.com";
static const int   AWS_IOT_PORT     = 8883;

static const char* THING_NAME = "smartThing-test";

// Topics
static const char* TOPIC_LED_SET    = "demo/led/set";     // SUB: on/off
static const char* TOPIC_LED_GET    = "demo/led/get";     // SUB: qualsiasi payload -> rispondi con status
static const char* TOPIC_LED_STATUS = "demo/led/status";  // PUB: on/off

// PUB retained? (default OFF per evitare policy extra)
static const bool STATUS_RETAINED = false;

static const int LED_PIN = 2;

// NTP
static const char* NTP_SERVER_1 = "pool.ntp.org";
static const char* NTP_SERVER_2 = "time.nist.gov";
static const long  GMT_OFFSET_SEC = 0;
static const int   DAYLIGHT_OFFSET_SEC = 0;

// ===================== CERTIFICATES =====================
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

// ===================== GLOBALS =====================
WiFiClientSecure net;
PubSubClient mqtt(net);
static bool ledOn = false;

// ===================== HELPERS =====================
static void waitForTimeSync_NoTimeout() {
  Serial.println("[NTP] waiting for valid epoch (no timeout) ...");
  time_t now = time(nullptr);
  while (now < 1700000000) {
    Serial.print(".");
    delay(500);
    now = time(nullptr);
  }
  Serial.println();
  Serial.print("[NTP] epoch=");
  Serial.println((long)now);
}

static void connectWiFi_NoTimeout() {
  Serial.print("[WiFi] connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  WiFi.setSleep(false);

  Serial.println("[WiFi] CONNECTED");
  Serial.print("[WiFi] IP=");
  Serial.print(WiFi.localIP());
  Serial.print(" RSSI=");
  Serial.println(WiFi.RSSI());
}

static void setupTLS() {
  Serial.println("[TLS] setting certs");
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(DEVICE_CERT);
  net.setPrivateKey(DEVICE_PRIVATE_KEY);
  net.setHandshakeTimeout(30);
  Serial.println("[TLS] certs set");
}

static void setLed(bool on) {
  ledOn = on;
  digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  Serial.print("[LED] set -> ");
  Serial.println(ledOn ? "ON" : "OFF");
}

static void publishStatus(const char* reason) {
  const char* s = ledOn ? "on" : "off";

  // {"message":"on"} / {"message":"off"}
  char msg[32];
  snprintf(msg, sizeof(msg), "{\"message\":\"%s\"}", s);

  bool ok = mqtt.publish(TOPIC_LED_STATUS, msg, STATUS_RETAINED);

  Serial.print("[MQTT] PUB ");
  Serial.print(TOPIC_LED_STATUS);
  Serial.print(" -> ");
  Serial.print(msg);
  Serial.print(" ");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print(" (");
  Serial.print(reason);
  Serial.println(")");
}

static bool parseOnOff(const byte* payload, unsigned int len, bool &out) {
  String s;
  s.reserve(len);
  for (unsigned int i = 0; i < len; i++) s += (char)payload[i];
  s.trim();
  s.toLowerCase();

  // Se non è JSON, prova comunque a supportare i vecchi payload "on/off"
  // (puoi togliere questa parte se vuoi SOLO JSON)
  if (s == "on" || s == "1" || s == "true")  { out = true;  return true; }
  if (s == "off"|| s == "0" || s == "false") { out = false; return true; }

  // Parsing leggero: cerca "message":"on" o "message":"off"
  // Accetta anche spazi: "message" : "on"
  int k = s.indexOf("\"message\"");
  if (k < 0) return false;

  int colon = s.indexOf(':', k);
  if (colon < 0) return false;

  // trova la prima virgolette dopo i due punti
  int q1 = s.indexOf('\"', colon);
  if (q1 < 0) return false;

  // trova la seconda virgolette (fine valore)
  int q2 = s.indexOf('\"', q1 + 1);
  if (q2 < 0) return false;

  String v = s.substring(q1 + 1, q2);
  v.trim();

  if (v == "on" || v == "1" || v == "true")  { out = true;  return true; }
  if (v == "off"|| v == "0" || v == "false") { out = false; return true; }

  return false;
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] RX ");
  Serial.print(topic);
  Serial.print(" payload=\"");
  for (unsigned int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println("\"");

  String t(topic);

  // --- SET: on/off ---
  if (t == TOPIC_LED_SET) {
    bool on;
    if (!parseOnOff(payload, length, on)) {
      Serial.println("[LED] invalid payload -> ignored (no publish)");
      return;
    }
    setLed(on);
    publishStatus("set");
    return;
  }

  // --- GET: qualsiasi payload -> rispondi con status ---
  if (t == TOPIC_LED_GET) {
    publishStatus("get");
    return;
  }
}

static void connectMQTT_NoTimeout() {
  mqtt.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqtt.setSocketTimeout(15);

  Serial.print("[MQTT] connecting to ");
  Serial.print(AWS_IOT_ENDPOINT);
  Serial.print(":");
  Serial.println(AWS_IOT_PORT);

  Serial.print("[MQTT] clientId=");
  Serial.println(THING_NAME);

  while (!mqtt.connected()) {
    Serial.print("[MQTT] CONNECT ... ");
    bool ok = mqtt.connect(THING_NAME);
    if (ok) {
      Serial.println("OK");

      Serial.print("[MQTT] SUB ");
      Serial.print(TOPIC_LED_SET);
      Serial.println(mqtt.subscribe(TOPIC_LED_SET) ? " OK" : " FAIL");

      Serial.print("[MQTT] SUB ");
      Serial.print(TOPIC_LED_GET);
      Serial.println(mqtt.subscribe(TOPIC_LED_GET) ? " OK" : " FAIL");

      Serial.println("[READY] demo/led/set: on | off");
      Serial.println("[READY] demo/led/get: any payload -> replies on demo/led/status");
    } else {
      Serial.print("FAIL state=");
      Serial.println(mqtt.state());
      delay(1500);
    }
  }
}

// ===================== ARDUINO =====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 AWS IoT (set/get + status) ===");

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  connectWiFi_NoTimeout();

  Serial.println("[NTP] configTime()");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
  waitForTimeSync_NoTimeout();

  setupTLS();
  connectMQTT_NoTimeout();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] LOST -> reconnect");
    setLed(false);
    connectWiFi_NoTimeout();
    Serial.println("[NTP] re-sync");
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
    waitForTimeSync_NoTimeout();
  }

  if (!mqtt.connected()) {
    Serial.println("[MQTT] LOST -> reconnect");
    connectMQTT_NoTimeout();
  }

  mqtt.loop();
  delay(5);
}
