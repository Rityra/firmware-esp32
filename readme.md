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

