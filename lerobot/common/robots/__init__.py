from .config import RobotConfig
from .robot import Robot
from .utils import make_robot_from_config

# Import robot types to register them
from .koch_follower import koch_follower
from .so100_follower import so100_follower
from .so101_follower import so101_follower
from .so100_Double_follower import so100_dual_follower
