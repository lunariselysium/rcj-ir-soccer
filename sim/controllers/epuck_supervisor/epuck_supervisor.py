# main.py (Ensure this matches your folder name if using folder execution)
from hardware import RobotInterface
from tracker import ParticleFilter
from strategy import Strategy
import config

def run_robot():
    # 1. Initialize Modules
    robot = RobotInterface()
    tracker = ParticleFilter(config.FIELD_X, config.FIELD_Y, config.NUM_PARTICLES)
    brain = Strategy()

    print("System Online. Starting simulation loop...")

    # 2. Main Loop
    while robot.step(robot.timestep) != -1:
        
        # --- INPUT ---
        # Get ground truth from God Mode (Hardware)
        sensor_data = robot.get_sensor_data()
        
        # Simulate the limited sensor (IR)
        ir_index = robot.simulate_ir_reading(sensor_data)

        # --- ESTIMATION ---
        # Update Particle Filter
        tracker.predict()
        tracker.update(
            robot_x=sensor_data['x'],
            robot_y=sensor_data['y'],
            robot_heading=sensor_data['heading'],
            sensor_index=ir_index
        )
        tracker.resample()
        
        # Get where we THINK the ball is
        est_ball_x, est_ball_y = tracker.get_estimated_pos()
        robot.draw_ghost(est_ball_x, est_ball_y)

        # --- DECISION (Your Code) ---
        vx, vy, omega = brain.decide(
            robot_x=sensor_data['x'],
            robot_y=sensor_data['y'],
            robot_heading=sensor_data['heading'],
            ball_x=est_ball_x,
            ball_y=est_ball_y
        )

        # --- ACTION ---
        robot.set_holonomic_velocity(vx, vy, omega)

if __name__ == "__main__":
    run_robot()