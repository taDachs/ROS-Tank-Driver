#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from typing import Final
from . import motors
from . import encoders

MOTOR_REDUCTION_RATIO: Final = 30
MOTOR_WHEEL_RADIUS: Final = (55.5 / 1000) / 2

MOTOR_LEFT_PWM_PIN: Final = 21
MOTOR_LEFT_DIR_PIN: Final = 23
MOTOR_LEFT_MIN_DC: Final = 20
MOTOR_LEFT_MAX_DC: Final = 80
MOTOR_LEFT_ENCODER_ID: Final = 1

MOTOR_RIGHT_PWM_PIN: Final = 22
MOTOR_RIGHT_DIR_PIN: Final = 24
MOTOR_RIGHT_MIN_DC: Final = 20
MOTOR_RIGHT_MAX_DC: Final = 80
MOTOR_RIGHT_ENCODER_ID: Final = 2

SMBUS: Final = 1
I2C_ADDRESS: Final = 0x10


def setup_gpio() -> None:
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)  # just annoying
    GPIO.setup(MOTOR_LEFT_PWM_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_LEFT_DIR_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_PWM_PIN, GPIO.OUT)
    GPIO.setup(MOTOR_RIGHT_DIR_PIN, GPIO.OUT)


class MotorDriverNode(rclpy.node.Node):

    def __init__(self, driver: motors.MotorDriver):
        super().__init__('tank_motor_driver_node')

        self.driver = driver
        self.init_subs()

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_subs(self):
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel',
                                                  self.twist_callback, 1)

    def twist_callback(self, msg: Twist) -> None:
        self.driver.set_vel(msg.linear.x, msg.linear.x)

    def timer_callback(self):
        self.driver.update()


def main(args=None):
    rclpy.init(args=args)
    setup_gpio()

    hat_wrapper = encoders.DFRobotDriverHATWrapper(SMBUS, I2C_ADDRESS)
    hat_wrapper.init()
    left_encoder = encoders.DFRobotDriverHATEncoder(
        hat_wrapper,
        MOTOR_LEFT_ENCODER_ID,
        MOTOR_REDUCTION_RATIO,
        wheel_radius=MOTOR_WHEEL_RADIUS,
        reverse=True)
    left_encoder.init()

    right_encoder = encoders.DFRobotDriverHATEncoder(
        hat_wrapper,
        MOTOR_RIGHT_ENCODER_ID,
        MOTOR_REDUCTION_RATIO,
        wheel_radius=MOTOR_WHEEL_RADIUS)
    right_encoder.init()

    left_encoder.get_current_vel()
    right_encoder.get_current_vel()

    left_motor = motors.Motor(MOTOR_LEFT_PWM_PIN,
                              MOTOR_LEFT_DIR_PIN,
                              left_encoder,
                              name='Left',
                              min_dc=MOTOR_LEFT_MIN_DC,
                              max_dc=MOTOR_LEFT_MAX_DC,
                              reverse=True)
    right_motor = motors.Motor(MOTOR_RIGHT_PWM_PIN,
                               MOTOR_RIGHT_DIR_PIN,
                               right_encoder,
                               name='Right',
                               min_dc=MOTOR_RIGHT_MIN_DC,
                               max_dc=MOTOR_RIGHT_MAX_DC)
    driver = motors.MotorDriver(left_motor, right_motor)
    driver.init()
    driver_node = MotorDriverNode(driver)
    rclpy.spin(driver_node)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
    finally:
        GPIO.cleanup()
        rclpy.shutdown()
