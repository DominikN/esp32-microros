setup:
    #!/bin/bash
    python3 -m venv ./python_venv
    source ./python_venv/bin/activate
    python3 -m pip install -U platformio

build:
    #!/bin/bash
    source ./python_venv/bin/activate
    pio run --target build

flash:
    #!/bin/bash
    source ./python_venv/bin/activate
    pio run --target upload

monitor:
    #!/bin/bash
    source ./python_venv/bin/activate
    pio run --target monitor

udev:
    #!/bin/bash
    # https://docs.platformio.org/en/latest/core/installation/udev-rules.html
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    sudo service udev restart