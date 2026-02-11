# hardware.py
from controller import Supervisor
import math
import config

class RobotInterface(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Node Handling
        self.ball_node = self.getFromDef("BALL")
        self.robot_node = self.getFromDef("MY_ROBOT")
        self.ghost_node = self.getFromDef("GHOST_BALL")

        # Motor Setup
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def get_sensor_data(self):
        """Returns dictionary of robot state and truth data."""
        # 1. Get Positions
        ball_pos = self.ball_node.getPosition()
        robot_pos = self.robot_node.getPosition()
        
        # 2. Calculate Heading
        rot_matrix = self.robot_node.getOrientation()
        heading = math.atan2(-rot_matrix[3], -rot_matrix[0])

        # 3. Calculate Relative Ball Vector
        dx = ball_pos[0] - robot_pos[0]
        dy = ball_pos[1] - robot_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        global_ball_angle = math.atan2(dy, dx)
        rel_angle = global_ball_angle - heading

        # Normalize Angle
        while rel_angle > math.pi: rel_angle -= 2 * math.pi
        while rel_angle < -math.pi: rel_angle += 2 * math.pi

        return {
            'x': robot_pos[0],
            'y': robot_pos[1],
            'heading': heading,
            'ball_dist': dist,
            'ball_angle': rel_angle
        }

    def simulate_ir_reading(self, data):
        """Simulates the 16-sector IR sensor."""
        if data['ball_dist'] > 3.0: return None 
        sector_size = (2 * math.pi) / 16
        index = round(data['ball_angle'] / sector_size) % 16
        return int(index)

    def draw_ghost(self, x, y):
        """Visual debugging."""
        if self.ghost_node:
            self.ghost_node.getField("translation").setSFVec3f([x, y, 0.1])

    def set_holonomic_velocity(self, v_x, v_y, omega):
        """
        Input: v_x (forward m/s), v_y (left m/s), omega (rotation rad/s)
        """
        rot = self.robot_node.getOrientation()
        
        # Transform local velocity to global
        global_vx = (rot[0] * v_x) + (rot[1] * v_y)
        global_vy = (rot[3] * v_x) + (rot[4] * v_y)

        self.robot_node.setVelocity([
            global_vx, global_vy, 0, 
            0, 0, omega
        ])