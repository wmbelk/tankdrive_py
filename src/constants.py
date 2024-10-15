from wpilib import CAN
from enum import IntEnum, auto

#region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()

class Rio_Pnue(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()


class Rio_PWM(IntEnum):
    ONE = 0
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()

class Rio_Relay(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()

class Rio_Analog(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
#endregion

#region CAN Constants
class CAN_Address(IntEnum):
    ZERRO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()

#endregion
class DriveConstant:
    LEFT_MOTOR1 = CAN_Address.ONE
    LEFT_MOTOR2 = CAN_Address.TWO
    RIGHT_MOTOR1 = CAN_Address.THREE
    RIGHT_MOTOR2 = CAN_Address.FOUR
    kTrackWidth = 1.0
    
class ControllerConstant:
    XBOX_driver = 0