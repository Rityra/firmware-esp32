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
arduino-cli compile --fqbn esp32:esp32:nodemcu-32s aws_iot_tls.ino
```

Upload to board:
```
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:nodemcu-32s aws_iot_tls.ino
```
