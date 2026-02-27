#Libraries
import RPi.GPIO as GPIO
import time 

#Disable warnings
GPIO.setwarnings(False)

#Select GPIO mode
GPIO.setmode(GPIO.BCM)

#Set buzzer output pin
buzzer=18
GPIO.setup(buzzer,GPIO.OUT)

# Create PWM object at 1000 Hz
pwm = GPIO.PWM(buzzer, 1000)
pwm.start(50)   # 50% duty cycle

#Run forever loop
try:
    while True:
        print("Beep")
        pwm.ChangeDutyCycle(50)   # buzzer ON
        time.sleep(0.5)

        print("No Beep")
        pwm.ChangeDutyCycle(0)    # buzzer OFF
        time.sleep(0.5)

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
