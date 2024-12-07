# esp32-microros

using ESP32 with Micro-ROS

## Quick Start

1. Run the Micro-ROS Agent and the listener node on you laptop

```bash
cd demo/
docker compose up
```

2. Put your Wi-Fi SSID and Password into the `credentials.h` file:

```bash
cp src/credentials-template.h src/credentials.h
# and edit credentials.h
```

3. Setup the platformio

```bash
just setup
just udev
```

4. Built and upload the project to ESP32

```bash
just flash
```

## Platformio VSC extension setup

To use locacl Python venv edit the `/home/$USER/.config/Code/User/settings.json` and add:

```json
{
    "platformio-ide.customPATH": "/home/$USER/repo/esp32-microros/python_venv/bin/pio"
}
```