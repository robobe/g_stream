from enum import Enum

class Presets(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


for item in Presets:
    print(item.value)