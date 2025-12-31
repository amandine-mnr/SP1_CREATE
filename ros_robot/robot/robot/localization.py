import time
import math

from .lib import *

ROBOT_SPEED_LOC = 0.8 #m/s, used for dead reckonning localization

# Localization 
class DeadReckoningLoc:
    #d = v * dt
    #theta = bearing_to_goal

    def __init__(self, x0, y0, theta0=0.0):
        self.x = float(x0)
        self.y = float(y0)
        self.theta = float(theta0)

        self.speed = 0

        self.goal_x = None
        self.goal_y = None

        self.last_t = time.time()

    def set_pose(self, x, y, theta=None):
        self.x = float(x)
        self.y = float(y)
        if theta is not None:
            self.theta = float(theta)
        self.last_t = time.time()

    def set_goal(self, gx, gy):
        self.goal_x = float(gx)
        self.goal_y = float(gy)
        self.theta = pi_bracket(self.bearing_to_goal())
    
    def moving(self):
        self.speed = ROBOT_SPEED_LOC
        
    def stopped(self):
        self.speed = 0

    def clear_goal(self):
        self.goal_x = None
        self.goal_y = None

    def bearing_to_goal(self):
        if self.goal_x is None or self.goal_y is None:
            return self.theta
        return math.atan2(self.goal_y - self.y, self.goal_x - self.x)

    def update(self):
        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        d = float(self.speed) * dt
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        return self.x, self.y, self.theta, dt