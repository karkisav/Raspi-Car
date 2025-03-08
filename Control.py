import RPi.GPIO as GPIO
import time

# GPIO pin setup
# Motor A (left side)
ENA = 12  # PWM pin for controlling speed
IN1 = 23  # Direction control
IN2 = 22  # Direction control

# Motor B (right side)
ENB = 13  # PWM pin for controlling speed
IN3 = 18  # Direction control
IN4 = 17  # Direction control

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Create PWM objects for speed control
pwm_a = GPIO.PWM(ENA, 100)  # 100 Hz frequency
pwm_b = GPIO.PWM(ENB, 100)

# Start PWM with 0% duty cycle
pwm_a.start(0)
pwm_b.start(0)


class PiCar:
    def __init__(self):
        self.speed = 10

    def forward(self, speed = None):
        if speed is not None:
            self.speed = speed
        
        # Set motor directions for forward
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        # Set speed
        pwm_a.ChangeDutyCycle(self.speed)
        pwm_b.ChangeDutyCycle(self.speed)

    def backward(self, speed = None):
        if speed is not None:
            self.speed = speed
        
        # Set motor directions for forward
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        # Set speed
        pwm_a.ChangeDutyCycle(self.speed)
        pwm_b.ChangeDutyCycle(self.speed)


    def turn_left(self, turn_speed = 5):
        # Slow down left side, maintain right side
        pwm_a.ChangeDutyCycle(self.speed - turn_speed)
        pwm_b.ChangeDutyCycle(self.speed)

    def turn_right(self, turn_speed = 5):
        # Maintian down left side, slow down right side
        pwm_a.ChangeDutyCycle(self.speed)
        pwm_b.ChangeDutyCycle(self.speed - turn_speed)

    def change_lane_left(self):
        # Quick left turn then straighten
        self.turn_left(1)
        time.sleep(0.5)
        self.forward()  # Back to normal
    
    def change_lane_right(self):
        # Quick right turn then straighten
        self.turn_right(1)
        time.sleep(0.5)
        self.forward()

    def stop(self):
        # Stop both motors
        pwm_a.ChangeDutyCycle(0)
        pwm_b.ChangeDutyCycle(0)

    def cleanup(self):
        # Clean up GPIO
        pwm_a.stop()
        pwm_b.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    car = PiCar()
    try:
        # print("Moving forward ...")
        # car.forward(1)
        # time.sleep(2)
        # print("Done !!!")

        # print("Moving backwards")
        # car.backward(1)
        # time.sleep(2)
        # print("Done !!!")

        print("Turning right ...")
        car.turn_left()
        time.sleep(2)
        print("Done !!!")

        # print("Changing lane To left lane")
        # car.change_lane_left()
        # time.sleep(1)
        # print("Done !!!")

        print("All tests done now !!")
        print("stopping the car now!")
        car.stop()

    except KeyboardInterrupt:
        print("Stopping program")

    finally:
        car.stop()
        car.cleanup()
        print("Clean up done: GPIO Pins are fresh now")
    