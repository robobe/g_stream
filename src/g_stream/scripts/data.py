from enum import Enum

class RobotMode(Enum):
    IDLE = "idle"
    ACTIVE = "active"
    ERROR = "error"

mode = RobotMode("active")
print(mode)
print(mode.value)
print(int(mode.value.int))