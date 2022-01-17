from DFRobotDCMotorDriverHAT.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
from abc import ABC, abstractmethod
import math

import time


class Encoder(ABC):

    def __init__(self, wheel_radius: float, reverse: bool = False):
        self.wheel_radius = wheel_radius
        self.reverse = reverse
        self.last_time = None

    @abstractmethod
    def get_rpm(self) -> float:
        pass

    def get_current_vel(self) -> float:
        if not self.last_time:
            self.last_time = time.time()
            return 0

        rpm = self.get_rpm()

        speed = ((rpm / 60) * self.wheel_radius * math.pi * 2)
        if self.reverse:
            speed *= -1

        return speed


class DFRobotDriverHATWrapper:

    def __init__(self, bus: int, address: int):
        self.bus = bus
        self.address = address

    def init(self) -> None:
        self.board = Board(self.bus, self.address)
        # Board begin and check board status
        while self.board.begin() != Board.STA_OK:
            print('board begin faild')
            time.sleep(2)
        print('board begin success')
        self.board.set_encoder_enable(Board.ALL)

    def get_rpm(self, enc_id: int) -> float:
        return self.board.get_encoder_speed([enc_id])[0]

    def set_reduction_ratio(self, enc_id: int, reduction_ratio: float) -> None:
        self.board.set_encoder_reduction_ratio([enc_id], reduction_ratio)


class DFRobotDriverHATEncoder(Encoder):

    def __init__(self, board: DFRobotDriverHATWrapper, enc_id: int,
                 reduction_ratio: float, **kwargs):
        '''
        :param wheel_radius: wheel radius in m
        '''
        super().__init__(**kwargs)
        self.board = board
        if enc_id == 1:
            self.id = Board.M1
        elif enc_id == 2:
            self.id = Board.M2
        else:
            raise Exception(
                f'{enc_id} is an invalid id, must be in range 1 to 2')
        self.reduction_ratio = reduction_ratio

    def init(self) -> None:
        self.board.set_reduction_ratio(self.id, self.reduction_ratio)

    def get_rpm(self) -> float:
        return self.board.get_rpm(self.id)
