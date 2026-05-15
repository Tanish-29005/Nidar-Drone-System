import RPi.GPIO as GPIO
import time

# Set GPIO pin
servo_pin = 17

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM on the pin with 50Hz frequency
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    print("Rotating to drop position...")
    set_angle(5) # Rotate to open (drop) position
    time.sleep(3)
    set_angle(15) # Rotate back to closed position
    print("Drop complete!")

except KeyboardInterrupt:
    pass

finally:
    pwm.stop()
    GPIO.cleanup()
