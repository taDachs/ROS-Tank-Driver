import RPi.GPIO as GPIO
from typing import Final
from . import encoders

FREQ: Final = 1000


# enum for directions
class Direction:
    NONE = 0
    FORWARDS = 1
    BACKWARDS = 2


class Motor:

    def __init__(self,
                 pwm_pin: int,
                 dir_pin: int,
                 encoder: encoders.Encoder,
                 name: str = None,
                 max_dc: int = 100,
                 min_dc: int = 0,
                 reverse: bool = False):
        assert max_dc <= 100 and max_dc >= 0
        assert min_dc <= 100 and min_dc >= 0
        assert min_dc <= max_dc
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.encoder = encoder
        self.gpio_pwm_pin = None
        self._dc = 0
        self._direction = Direction.NONE
        self.name = name
        self.max_dc = max_dc
        self.min_dc = min_dc
        self.reverse = reverse
        self.target_vel = 0
        self.dc_hat = 0
        self.gain = 1

    def init(self) -> None:
        self._dc = 0
        self.gpio_pwm_pin = GPIO.PWM(self.pwm_pin, FREQ)
        self.gpio_pwm_pin.start(self._dc)

    def _set_pins(self) -> None:
        assert self._dc >= 0 and self._dc <= 100
        assert (self._dc == 0
                or self._dc >= self.min_dc) and self._dc <= self.max_dc
        if self.gpio_pwm_pin:
            if self._direction != Direction.NONE:
                self.gpio_pwm_pin.start(self._dc)
            else:
                self.gpio_pwm_pin.start(0)
                print('No direction set')

        if self._direction == Direction.FORWARDS:
            GPIO.output(self.dir_pin, GPIO.LOW if self.reverse else GPIO.HIGH)
        elif self._direction == Direction.BACKWARDS:
            GPIO.output(self.dir_pin, GPIO.HIGH if self.reverse else GPIO.LOW)

    def set_dc(self, dc: int) -> None:
        dc = int(dc)
        assert dc >= 0 and dc <= 100
        if not self.gpio_pwm_pin:
            raise Exception('Error, motor not initialized, run init first')

        if dc > self.max_dc:
            print(f'Error, dc of {dc} is higher than max dc ({self.max_dc})')
            return

        if dc != 0 and dc < self.min_dc:
            print(f'Error, dc of {dc} is lower than min dc ({self.min_dc})')
            return

        self._dc = dc
        self._set_pins()

    def update(self):
        speed = self.encoder.get_current_vel()

        dv = self.target_vel - speed

        print(f'[{self.name}] target: {self.target_vel}m/s, '
              f'current: {speed}m/s, '
              f'diff: {dv}m/s, dc_hat: {self.dc_hat}, dc: {self._dc}, '
              f'correction: {dv * self.gain}')

        if self.target_vel == 0:
            self._direction = Direction.NONE
            self.set_dc(0)
            return

        self.dc_hat += dv * self.gain
        if self.dc_hat < 0:
            self._direction = Direction.BACKWARDS
        elif self.dc_hat > 0:
            self._direction = Direction.FORWARDS
        else:
            self._direction = Direction.NONE

        self.set_dc(max(self.min_dc, (min(self.max_dc, abs(self.dc_hat)))))


class MotorDriver:

    def __init__(self, left: Motor, right: Motor, max_vel: float = 2):
        self.left = left
        self.right = right
        self.max_vel = max_vel

    def init(self) -> None:
        self.left.init()
        self.right.init()

    def set_max_dc(self, dc: int):
        self.left.max_dc = dc
        self.right.max_dc = dc

    def set_min_dc(self, dc: int):
        self.left.min_dc = dc
        self.right.min_dc = dc

    def update(self):
        self.left.update()
        self.right.update()

    def set_vel(self, left: float, right: float) -> None:
        '''
        Sets the velocity for each track
        :param left: left velocity in m/s
        :param right: right velociry in m/s
        '''

        self.left.target_vel = left
        self.right.target_vel = right
