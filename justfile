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