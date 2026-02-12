### IoT Firmware

#### Arduino CLI

Run on [ArchLinux](https://wiki.archlinux.org/title/Arduino)
Run sketches on [ESP32](https://wellys.com/posts/esp32_cli/)

Make sure core is installed

```
arduino-cli core list
sudo chmod a+rw /dev/ttyUSB0 # (or any port you want to use)
```

Install missing packages

```
arduino-cli lib install PubSubClient
```

Compile for esp32

```
arduino-cli compile --fqbn esp32:esp32:esp32 aws_iot_tls.ino

arduino-cli compile --fqbn esp32:esp32:esp32 wifi_ap.ino
```

Upload to board:

```
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 aws_iot_tls.ino

arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 wifi_ap.ino
```

Reset ESP32 flash memory:

- [step by step](https://randomnerdtutorials.com/esp32-erase-flash-memory/)
- [dependencies](https://docs.espressif.com/projects/esptool/en/latest/esp32/installation.html)
- [Python venv in Arch](https://stackoverflow.com/a/79304690/10708345)
- TL;TR: press the board's boot button + `python -m esptool --chip esp32 erase_flash`

### Sketch Documentation

=====================================================================
ESP32 smartThing - WiFi Provisioning (AP + POST JSON) + MQTT (AWS IoT)
=====================================================================

---

A) HELP - PROVISIONING WiFi (POST JSON)

---

HOTSPOT (AP):
SSID: smartThing
PASS: 12345678
IP: 192.168.4.1

ENDPOINTS:
GET / -> info + esempio
POST /wifi -> provisioning WiFi
POST /clear -> cancella credenziali salvate

1. POSTMAN
   - Connettiti al WiFi: smartThing (pass 12345678)
   - POST http://192.168.4.1/wifi
   - Body -> raw -> JSON:
     {
     "ssid": "NomeRete2.4G",
     "pwd": "PasswordRete"
     }
   - Headers:
     Content-Type: application/json

2. CURL
   curl -X POST http://192.168.4.1/wifi \
    -H "Content-Type: application/json" \
    -d "{\"ssid\":\"NomeRete2.4G\",\"pwd\":\"PasswordRete\"}"

NOTE:

- ESP32 supporta 2.4GHz (non 5GHz)
- JSON valido: virgolette + virgola

---

B) HELP - MQTT (AWS IoT Core)

---

TOPIC usati: - Pubblica stato LED: demo/led/state (JSON) - Riceve comando LED: demo/led/set ("on"/"off" oppure JSON) - Pubblica pulsanti: demo/buttons (JSON) - (opzionale) heartbeat: demo/heartbeat (JSON)

Comando LED - esempi payload accettati: 1) Testo:
on
off 2) JSON:
{"led":true}
{"led":false}

Come testare da PC (mosquitto) _se usi un broker locale_: - Subscribe:
mosquitto_sub -h <broker> -t "demo/#" -v - Publish:
mosquitto_pub -h <broker> -t "demo/led/set" -m "on"

Per AWS IoT Core: - In console AWS IoT -> MQTT test client:
Subscribe: demo/#
Publish su demo/led/set con payload "on" o {"led":true}

IMPORTANTISSIMO:

- Su AWS IoT la connessione MQTT TLS richiede:
  - Root CA
  - Certificato device
  - Private key
  * E policy che permetta i topic demo/...

---

NOTE TECNICHE

---

- Salvataggio credenziali: Preferences (NVS)
- Log seriale ovunque
- WiFi connect robusto: persistent(false), sleep(false)
