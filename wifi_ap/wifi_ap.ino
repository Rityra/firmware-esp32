/*
  ============================================================
  ESP32 - WiFi provisioning via Hotspot + POST JSON (con log)
  ============================================================

  COSA FA:
  - Al boot:
    1) Legge SSID/PWD salvati in NVS (Preferences).
    2) Se trovati, prova a connettersi in STA per 10 secondi.
       - Se OK: resta in STA.
       - Se KO: avvia hotspot (AP) SSID smartThing PASS 12345678.
    3) Se non ci sono credenziali salvate: avvia subito l'hotspot (AP).

  - In modalità AP:
    - Avvia un web server sulla porta 80.
    - Endpoints:
      * GET  /        -> info + esempio
      * POST /wifi    -> riceve JSON: {"ssid":"NomeRete","pwd":"Password"}
      * POST /clear   -> cancella credenziali salvate

  - Quando riceve POST /wifi:
    - Prova a connettersi con quelle credenziali per 10 secondi.
    - Se OK:
      * salva in NVS
      * spegne l'hotspot
      * rimane in STA
    - Se KO:
      * torna/resta in hotspot per riprovare

  - Se in STA la connessione cade:
    - tenta riconnessione con le credenziali salvate per 10 secondi
    - se non riesce: torna in AP (provisioning)

  ------------------------------------------------------------
  COME USARE IL POST (METODI PRATICI)
  ------------------------------------------------------------

  1) Con curl (PC)
     - Connettiti al WiFi hotspot: SSID "smartThing" PASS "12345678"
     - Poi esegui:

     curl -X POST http://192.168.4.1/wifi \
          -H "Content-Type: application/json" \
          -d "{\"ssid\":\"NomeRete\",\"pwd\":\"PasswordRete\"}"

     Se la rete è aperta:
     curl -X POST http://192.168.4.1/wifi \
          -H "Content-Type: application/json" \
          -d "{\"ssid\":\"NomeRete\",\"pwd\":\"\"}"

  2) Con Postman
     - POST http://192.168.4.1/wifi
     - Body -> raw -> JSON:
       {
         "ssid": "NomeRete",
         "pwd":  "PasswordRete"
       }

  3) Verifica stato:
     - Apri http://192.168.4.1/ (GET) quando sei connesso all'hotspot
     - Risponde con JSON e istruzioni.

  NOTE:
  - Il JSON DEVE essere valido: virgolette e virgole.
    Esempio valido: {"ssid":"rete","pwd":"pass"}
    Esempio NON valido: { ssid: rete pwd: pass }
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// -------------------- Config AP --------------------
static const char* AP_SSID = "smartThing";
static const char* AP_PASS = "12345678";

// Timeout connessione WiFi (ms)
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 10000;

// -------------------- Web server --------------------
WebServer server(80);

// -------------------- Storage NVS --------------------
Preferences prefs;

// Credenziali salvate
String savedSsid;
String savedPwd;

// Stato rete
enum class NetMode { STA_OK, AP_MODE };
NetMode mode = NetMode::AP_MODE;

// Per evitare loop di riconnessioni troppo aggressivi
uint32_t lastReconnectAttemptMs = 0;
static const uint32_t RECONNECT_COOLDOWN_MS = 2000;

// -------------------- Prototipi --------------------
void startAP();
bool connectSTA(const char* ssid, const char* pwd, uint32_t timeoutMs);
void stopAPKeepSTA();
void loadCreds();
void saveCreds(const String& ssid, const String& pwd);
void clearCreds();
void printWiFiStatus();

// -------------------- Utility JSON Response --------------------
void sendJson(int code, const JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

// -------------------- Debug status WiFi --------------------
void printWiFiStatus() {
  wl_status_t st = WiFi.status();
  Serial.print("[WiFi] Status = ");
  switch (st) {
    case WL_IDLE_STATUS:     Serial.println("IDLE"); break;
    case WL_NO_SSID_AVAIL:   Serial.println("NO_SSID_AVAIL"); break;
    case WL_SCAN_COMPLETED:  Serial.println("SCAN_COMPLETED"); break;
    case WL_CONNECTED:       Serial.println("CONNECTED"); break;
    case WL_CONNECT_FAILED:  Serial.println("CONNECT_FAILED"); break;
    case WL_CONNECTION_LOST: Serial.println("CONNECTION_LOST"); break;
    case WL_DISCONNECTED:    Serial.println("DISCONNECTED"); break;
    default:                 Serial.println("UNKNOWN"); break;
  }
}

/*
  Prova a connettersi come client (STA) ad una rete WiFi.
  Ritorna true se connesso entro timeoutMs, altrimenti false.
*/
bool connectSTA(const char* ssid, const char* pwd, uint32_t timeoutMs) {
  Serial.println("--------------------------------------------------");
  Serial.println("[STA] Tentativo connessione WiFi");
  Serial.print("[STA] SSID: ");
  Serial.println(ssid);
  Serial.print("[STA] Timeout (ms): ");
  Serial.println(timeoutMs);

  WiFi.mode(WIFI_STA);

  // Pulisce eventuali vecchie connessioni/config
  Serial.println("[STA] WiFi.disconnect(true,true) + delay(200)");
  WiFi.disconnect(true, true);
  delay(200);

  Serial.println("[STA] WiFi.begin(...)");
  WiFi.begin(ssid, pwd);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  bool ok = (WiFi.status() == WL_CONNECTED);
  printWiFiStatus();

  if (ok) {
    Serial.println("[STA] Connessione OK!");
    Serial.print("[STA] IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[STA] RSSI: ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("[STA] Connessione FALLITA.");
  }

  Serial.println("--------------------------------------------------");
  return ok;
}

/*
  Spegne l'AP e resta in STA.
*/
void stopAPKeepSTA() {
  Serial.println("[AP] Spegnimento hotspot (softAPdisconnect)");

  // Passaggio prudenziale
  WiFi.mode(WIFI_AP_STA);
  delay(100);

  bool r = WiFi.softAPdisconnect(true);
  Serial.print("[AP] softAPdisconnect result: ");
  Serial.println(r ? "true" : "false");
  delay(100);

  WiFi.mode(WIFI_STA);
  Serial.println("[STA] Rimasto in modalità STA");
}

/*
  Avvia hotspot + definisce routes HTTP + avvia server
*/
void startAP() {
  Serial.println("==================================================");
  Serial.println("[AP] Avvio hotspot provisioning...");
  Serial.print("[AP] SSID: "); Serial.println(AP_SSID);
  Serial.print("[AP] PASS: "); Serial.println(AP_PASS);

  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[AP] softAP start: ");
  Serial.println(ok ? "OK" : "FAIL");

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("[AP] IP hotspot: ");
  Serial.println(apIP);

  // -------------------- ROUTES --------------------

  // GET /
  server.on("/", HTTP_GET, [apIP]() {
    Serial.println("[HTTP] GET /");

    StaticJsonDocument<320> doc;
    doc["device"] = "smartThing";
    doc["mode"] = "AP_MODE";
    doc["ap_ip"] = apIP.toString();
    doc["post_to"] = "/wifi";
    doc["example"] = "{\"ssid\":\"YourWiFi\",\"pwd\":\"YourPass\"}";
    sendJson(200, doc);
  });

  // POST /wifi
  server.on("/wifi", HTTP_POST, []() {
    Serial.println("[HTTP] POST /wifi");

    if (!server.hasArg("plain")) {
      Serial.println("[HTTP] Body mancante (plain)");
      StaticJsonDocument<160> err;
      err["ok"] = false;
      err["error"] = "Body mancante (plain)";
      sendJson(400, err);
      return;
    }

    String body = server.arg("plain");
    Serial.print("[HTTP] Body ricevuto: ");
    Serial.println(body);

    StaticJsonDocument<256> doc;
    DeserializationError e = deserializeJson(doc, body);
    if (e) {
      Serial.print("[JSON] Errore parsing: ");
      Serial.println(e.c_str());

      StaticJsonDocument<220> err;
      err["ok"] = false;
      err["error"] = "JSON non valido";
      err["detail"] = e.c_str();
      sendJson(400, err);
      return;
    }

    const char* ssid = doc["ssid"];
    const char* pwd  = doc["pwd"];

    if (!ssid || String(ssid).length() == 0) {
      Serial.println("[JSON] Campo ssid mancante/vuoto");

      StaticJsonDocument<160> err;
      err["ok"] = false;
      err["error"] = "Campo 'ssid' mancante o vuoto";
      sendJson(400, err);
      return;
    }
    if (!pwd) pwd = "";

    Serial.print("[PROV] SSID ricevuto: ");
    Serial.println(ssid);
    Serial.print("[PROV] PWD ricevuta: ");
    Serial.println(strlen(pwd) ? "(non vuota)" : "(vuota)");

    // Tenta connessione STA
    bool connected = connectSTA(ssid, pwd, WIFI_CONNECT_TIMEOUT_MS);

    StaticJsonDocument<256> resp;
    resp["ssid"] = ssid;

    if (connected) {
      Serial.println("[PROV] Connessione riuscita -> salvo credenziali e spengo AP");

      saveCreds(ssid, pwd);
      // aggiorno anche le variabili in RAM per riconnessione futura
      savedSsid = ssid;
      savedPwd = pwd;

      stopAPKeepSTA();
      mode = NetMode::STA_OK;

      resp["ok"] = true;
      resp["status"] = "connesso_e_salvato";
      resp["ip"] = WiFi.localIP().toString();
      sendJson(200, resp);
    } else {
      Serial.println("[PROV] Connessione fallita -> torno in AP");

      mode = NetMode::AP_MODE;

      // Importante: connectSTA mette WIFI_STA, quindi riattivo AP/server
      // Nota: richiamare startAP() dentro handler è semplice ma "brutale".
      // In pratica ripristina routes e server; funziona bene per progetti piccoli.
      startAP();

      resp["ok"] = false;
      resp["status"] = "connessione_fallita";
      resp["hint"] = "Controlla SSID/PWD e riprova";
      sendJson(401, resp);
    }
  });

  // POST /clear (opzionale)
  server.on("/clear", HTTP_POST, []() {
    Serial.println("[HTTP] POST /clear -> cancello credenziali");
    clearCreds();

    savedSsid = "";
    savedPwd = "";

    StaticJsonDocument<160> resp;
    resp["ok"] = true;
    resp["status"] = "credenziali_cancellate";
    sendJson(200, resp);
  });

  // Avvia server
  server.begin();
  Serial.println("[HTTP] Server avviato su porta 80");
  Serial.println("==================================================");
}

/*
  Carica credenziali salvate in NVS
*/
void loadCreds() {
  Serial.println("[NVS] Lettura credenziali salvate...");
  prefs.begin("wifi", true);
  savedSsid = prefs.getString("ssid", "");
  savedPwd  = prefs.getString("pwd", "");
  prefs.end();

  if (savedSsid.length() > 0) {
    Serial.print("[NVS] SSID salvato: ");
    Serial.println(savedSsid);
    Serial.print("[NVS] PWD salvata: ");
    Serial.println(savedPwd.length() ? "(non vuota)" : "(vuota)");
  } else {
    Serial.println("[NVS] Nessuna credenziale salvata.");
  }
}

/*
  Salva credenziali in NVS
*/
void saveCreds(const String& ssid, const String& pwd) {
  Serial.println("[NVS] Salvataggio credenziali...");
  Serial.print("[NVS] SSID = "); Serial.println(ssid);
  Serial.print("[NVS] PWD  = "); Serial.println(pwd.length() ? "(non vuota)" : "(vuota)");

  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pwd", pwd);
  prefs.end();

  Serial.println("[NVS] Salvate OK.");
}

/*
  Cancella credenziali in NVS
*/
void clearCreds() {
  Serial.println("[NVS] Rimozione chiavi ssid/pwd...");
  prefs.begin("wifi", false);
  prefs.remove("ssid");
  prefs.remove("pwd");
  prefs.end();
  Serial.println("[NVS] Cancellate.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("###############################################");
  Serial.println("# smartThing - WiFi Provisioning Boot         #");
  Serial.println("###############################################");

  // 1) carico credenziali
  loadCreds();

  // 2) se esistono credenziali provo connessione entro 10s
  if (savedSsid.length() > 0) {
    Serial.println("[BOOT] Trovate credenziali -> provo connessione STA...");
    bool ok = connectSTA(savedSsid.c_str(), savedPwd.c_str(), WIFI_CONNECT_TIMEOUT_MS);

    if (ok) {
      mode = NetMode::STA_OK;
      Serial.println("[BOOT] Avvio completato in modalità STA_OK");
      Serial.println("###############################################");
      return;
    } else {
      Serial.println("[BOOT] Connessione fallita -> avvio AP provisioning");
    }
  } else {
    Serial.println("[BOOT] Nessuna credenziale -> avvio AP provisioning");
  }

  // 3) se non connesso, avvio AP
  mode = NetMode::AP_MODE;
  startAP();

  Serial.println("[BOOT] Avvio completato in modalità AP_MODE");
  Serial.println("###############################################");
}

void loop() {
  // In modalità AP gestiamo richieste HTTP
  if (mode == NetMode::AP_MODE) {
    server.handleClient();
    delay(10);
    return;
  }

  // In modalità STA_OK qui andrà la tua logica (MQTT ecc.)
  // In più controlliamo se cade il WiFi:
  if (mode == NetMode::STA_OK) {
    if (WiFi.status() != WL_CONNECTED) {
      // cooldown per non martellare continuamente
      if (millis() - lastReconnectAttemptMs > RECONNECT_COOLDOWN_MS) {
        lastReconnectAttemptMs = millis();
        Serial.println("[STA] WiFi non connesso -> tento riconnessione...");

        bool ok = connectSTA(savedSsid.c_str(), savedPwd.c_str(), WIFI_CONNECT_TIMEOUT_MS);
        if (!ok) {
          Serial.println("[STA] Riconnessione fallita -> torno in AP provisioning");
          mode = NetMode::AP_MODE;
          startAP();
        } else {
          Serial.println("[STA] Riconnessione OK");
        }
      }
    }
  }

  delay(10);
}