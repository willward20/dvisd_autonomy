# DVISD Autonomy

## Installation
Before installing this package, ensure that the Raspberry Pi is properly configured. Follow the instructions in `docs/rpi_setup.md`.

### Create a Virtual Environment

```
cd
python3 -m venv myvenv
source ~/myvenv/bin/activate
```

### Install Dependencies
```
source ~/myvenv/bin/activate
pip3 install adafruit-circuitpython-servokit
pip3 install RPi.GPIO
pip3 install PyYAML
```

## Usage
1. Open `~/dvisd_autonomy/scripts/control_example` and change the config file to the current vehicle.
2. Connect battery to ESC.
3. Turn on ESC by pressing the EZ Set button once.
4. Execute the following commands:
```
source ~/myvenv/bin/activate
cd ~/dvisd_autonomy
python3 -m scripts.control_example
```
