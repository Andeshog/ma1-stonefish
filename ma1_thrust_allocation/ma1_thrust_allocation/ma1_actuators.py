from dataclasses import dataclass
import skadipy as sk
import numpy as np

@dataclass(slots=True)
class Actuators:
    ma_bow_port: sk.actuator.Azimuth = None
    ma_bow_starboard: sk.actuator.Azimuth = None
    ma_stern_starboard: sk.actuator.Azimuth = None
    ma_stern_port: sk.actuator.Azimuth = None
    
    def __init__(self):
        self.ma_bow_port = sk.actuator.Azimuth(
            position=sk.toolbox.Point([1.8, -0.8, 0.0]),
            orientation=sk.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), angle=(-np.pi / 4)
            ),
            extra_attributes={
                "rate_limit": 10,
                "saturation_limit": 300.0,
            }
        )

        self.ma_bow_starboard = sk.actuator.Azimuth(
            position=sk.toolbox.Point([1.8, 0.8, 0.0]),
            orientation=sk.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), angle=(np.pi / 4)
            ),
            extra_attributes={
                "rate_limit": 10.0,
                "saturation_limit": 300.0,
            }
        )

        self.ma_stern_starboard = sk.actuator.Azimuth(
            position=sk.toolbox.Point([-1.8, 0.8, 0.0]),
            orientation=sk.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), angle=(3 * np.pi / 4)
            ),
            extra_attributes={
                "rate_limit": 10,
                "saturation_limit": 300.0,
            }
        )

        self.ma_stern_port = sk.actuator.Azimuth(
            position=sk.toolbox.Point([-1.8, -0.8, 0.0]),
            orientation=sk.toolbox.Quaternion(
                axis=(0.0, 0.0, 1.0), angle=(-3 * np.pi / 4)
            ),
            extra_attributes={
                "rate_limit": 10,
                "saturation_limit": 300.0,
            }
        )

    def bow_actuators(self):
        return [self.ma_bow_port, self.ma_bow_starboard]
    
    def stern_actuators(self):
        return [self.ma_stern_port, self.ma_stern_starboard]
    
    def all_actuators(self):
        return [self.ma_bow_port, self.ma_bow_starboard, self.ma_stern_starboard, self.ma_stern_port]
    
actuator = Actuators()
print(actuator.__dir__)