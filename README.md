# DVISD Autonomy

Development note: Do not install any vs code extensions into the rasberry pis. They are not fast enough to handle this. If the pi is slow, go to extensions and uninstall all.

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

## Electronic Wiring
![wiring](/images/CardinalWiring.png)

### Power System
![power system](/images/power_system.png)

The robot is powered by a 7.2 V battery (stored in the left chassis compartment). Using a splitter, the battery powers the ESC and a buck converter (stored in the right compartment) in parallel. The buck convterer (tuned to 5V using its potentiometer) powers the RPi via its GPIO pins (5V power and GND).

### Motor Control
The RPi controls the steering servo and ESC via a PCA9685 motor control board. Note that the PCA board recieves power from the ESC when plugged into Channel 2. Then, the board provides power the steering servo through Channel 1. 

### Ultrasonic Sensor Circuit
![power system](/images/ultrasonic_circuit.png)


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
`dvisd_autonomy/control/control.py` provides an API enabling open-loop vehicle control.

| Method                    | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| `turn(angle: int)`        | Sets the steering angle (clamps input `angle` based on the servo limits). |
| `forward(pulse_us: int)`  | Sets the throttle pulse width (if no input, defaults to low speed).       |
| `straight()`              | Resets steering angle to center. Does not change throttle value.          |
| `stop()`                  | Resets throttle to neutral. Does not change steering angle.               |
| `shutdown()`              | Safely stops and deinitializes hardware.                                  |


## Troubleshooting
### Raspberry Pi won't stay on
If the Raspberry Pi won't stay turned on (the red light will turn on and off), check the voltage level of the LiPo battery. It it's too low, the buck converter can't provide enough current to maintain 5V output for the Pi. Try swapping out the battery for one with more charge. 