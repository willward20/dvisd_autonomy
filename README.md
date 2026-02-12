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

## Driving the Cars
1. Open `~/dvisd_autonomy/scripts/control_example` and change the config file to the current vehicle.
2. Connect battery to ESC.
3. Turn on ESC by pressing the EZ Set button once.
4. Execute the following commands:
```
source ~/myvenv/bin/activate
cd ~/dvisd_autonomy
python3 -m scripts.control_example
```

## Interfaces
### Control Commands
`dvisd_autonomy/control.py` provides an API enabling open-loop vehicle control.

| Method                    | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| `turn(angle: int)`        | Sets the steering angle (clamps input `angle` based on the servo limits). |
| `forward(pulse_us: int)`  | Sets the throttle pulse width (if no input, defaults to low speed).       |
| `straight()`              | Resets steering angle to center. Does not change throttle value.          |
| `stop()`                  | Resets throttle to neutral. Does not change steering angle.               |
| `shutdown()`              | Safely stops and deinitializes hardware.                                  |
 
