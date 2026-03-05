import time
import RPi.GPIO as GPIO

TRIG = 20
ECHO = 21
BUZZ = 18  # change if your buzzer pin differs

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BUZZ, GPIO.OUT)

pwm = GPIO.PWM(BUZZ, 1000)
pwm.start(0)

GPIO.output(TRIG, False)
time.sleep(0.2)

SPEED_OF_SOUND = 34300  # cm/s
START_BEEP_CM = 10
MIN_CM = 2  # avoid weird readings when too close

def get_distance_cm():
    # Trigger 10 µs pulse
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo start
    start = time.time()
    timeout = start + 0.02
    while GPIO.input(ECHO) == 0 and time.time() < timeout:
        start = time.time()

    # Wait for echo end
    stop = time.time()
    timeout = stop + 0.02
    while GPIO.input(ECHO) == 1 and time.time() < timeout:
        stop = time.time()

    pulse = stop - start

    # speed of sound: 34300 cm/s, divide by 2 for round trip
    distance_cm = (pulse * 34300) / 2.0
    return distance_cm

try:
    while True:
        d = get_distance_cm()
        print(f"{d:5.1f} cm")

        # Example threshold
        if MIN_CM < d <= START_BEEP_CM:
            # Map 10 cm → low pitch
            # Map 2 cm → high pitch

            # Linear mapping
            scale = (START_BEEP_CM - d) / (START_BEEP_CM - MIN_CM)
            freq = 500 + scale * 2500   # 500 Hz to 3000 Hz

            pwm.ChangeFrequency(freq)
            pwm.ChangeDutyCycle(50)

        else:
            pwm.ChangeDutyCycle(0)

        time.sleep(0.05)

finally:
    pwm.stop()
    GPIO.cleanup()
