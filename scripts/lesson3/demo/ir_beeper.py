import RPi.GPIO as GPIO
import time

IR_PIN = 21
BUZZ = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZ, GPIO.OUT)

pwm = GPIO.PWM(BUZZ, 1000)
pwm.start(0)

print("Move sensor over different shades...")

try:
    while True:
        state = GPIO.input(IR_PIN)
        print(GPIO.input(IR_PIN))

        if state == 0:  # white
            pwm.ChangeDutyCycle(0)
        else:  # black
            pwm.ChangeFrequency(2000)
            pwm.ChangeDutyCycle(50)


        time.sleep(0.05)

finally:
    pwm.stop()
    GPIO.cleanup()
