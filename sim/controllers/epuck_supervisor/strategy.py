# strategy.py
import math

class Strategy:
    def __init__(self):
        self.state = "SEARCH"
        self.timer = 0

    def decide(self, robot_x, robot_y, robot_heading, ball_x, ball_y):
        """
        Returns: vx, vy, omega (angular velocity)
        """
        
        # Calculate vector to estimated ball
        dx = ball_x - robot_x
        dy = ball_y - robot_y
        dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_heading
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        # --- STATE MACHINE EXAMPLE ---
        
        if self.state == "SEARCH":
            # Spin until we are confident (or just spin to scan)
            print("State: SEARCH")
            if dist < 0.5:
                self.state = "ATTACK"
            return 0, 0, 1.0 # Spin

        elif self.state == "ATTACK":
            print("State: ATTACK")
            
            # Simple P-Controller for rotation
            omega = angle_diff * 2.0
            
            # Move forward
            vx = 0.5
            
            if dist > 1.0:
                self.state = "SEARCH"
                
            return vx, 0, omega

        return 0, 0, 0