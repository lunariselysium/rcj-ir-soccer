from controller import Robot, Supervisor
import math
from tracker import ParticleFilter

# E-puck Constants
MAX_SPEED = 6.28  # Maximum speed in rad/s
AXLE_LENGTH = 1
WHEEL_RADIUS = 1

class EpuckTracker(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # 1. FIND OBJECTS (God Mode)
        self.ball_node = self.getFromDef("BALL")
        self.robot_node = self.getFromDef("MY_ROBOT")
        self.ghost_node = self.getFromDef("GHOST_BALL")

        if self.ball_node is None:
            print("ERROR: Ball not found! Did you set DEF to 'BALL'?")
        if self.robot_node is None:
            print("ERROR: Robot not found! Did you set DEF to 'MY_ROBOT'?")
        if self.ghost_node is None:
            print("ERROR: Ghost ball not found! Did you set DEF?")

        # 2. SETUP MOTORS (Standard E-puck names)
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        
        # Set to velocity control mode (infinite position)
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        self.tracker = ParticleFilter(field_x=4.0, field_y=3.0, num_particles=150)

         # --- VISUALIZATION SETUP ---
        # 1. Get the Root node of the scene tree
        root_node = self.getRoot()
        root_children = root_node.getField("children")

        # 2. Storage for our particle translation fields
        self.particle_nodes = []

        # 3. Create a string representing a single Red Particle
        # We use NO 'boundingObject' so the robot drives through them (ghosts)
        # We set transparency so they look like a cloud
        for i in range(self.tracker.n):
            # Define a unique name for retrieval
            def_name = f"PARTICLE_{i}"
            
            particle_str = f"""
            DEF {def_name} Transform {{
              translation 0 0 -10
              children [
                Shape {{
                  appearance Appearance {{
                    material Material {{ 
                        diffuseColor 0 0 1 
                        transparency 0.9
                        emissiveColor 0 0 1
                    }}
                  }}
                  geometry Sphere {{ radius 0.01 }}
                }}
              ]
            }}
            """
            # Inject into the world
            root_children.importMFNodeFromString(-1, particle_str)
            
            # Get the specific translation field for this new node to update later
            node = self.getFromDef(def_name)
            self.particle_nodes.append(node.getField("translation"))
        

        # --- SETUP COLOR CONTROL ---
        self.ghost_color_field = None
        
        if self.ghost_node:
            # 1. Get the Shape node (usually the first child of the object)
            children = self.ghost_node.getField("children")
            if children.getCount() > 0:
                shape_node = children.getMFNode(0) # Get the Shape
                
                # 2. Get Appearance -> Material
                appearance_node = shape_node.getField("appearance").getSFNode()
                if appearance_node:
                    material_node = appearance_node.getField("material").getSFNode()
                    
                    # 3. Store the specific field that controls color
                    if material_node:
                        self.ghost_color_field = material_node.getField("diffuseColor")


    def get_data(self):
        """
        Calculates angle/distance.
        """
        # 1. Get positions
        ball_pos = self.ball_node.getPosition()
        robot_pos = self.robot_node.getPosition()
        
        # 2. Get Orientation Matrix
        # [0 1 2]
        # [3 4 5]
        # [6 7 8]
        rot_matrix = self.robot_node.getOrientation()

        forward_x = -rot_matrix[0] 
        forward_y = -rot_matrix[3]
        
        # Calculate Robot Heading (Yaw)
        robot_heading = math.atan2(forward_y, forward_x) 

        # 3. Calculate Target Vector
        dx = ball_pos[0] - robot_pos[0]
        dy = ball_pos[1] - robot_pos[1]
        
        # 4. Calculate Angle Difference
        global_ball_angle = math.atan2(dy, dx)
        relative_angle = global_ball_angle - robot_heading

        # 5. Normalize to -PI to +PI
        while relative_angle > math.pi: relative_angle -= 2 * math.pi
        while relative_angle < -math.pi: relative_angle += 2 * math.pi

        return {
            'angle_degrees': math.degrees(relative_angle),
            'angle_rad': relative_angle,
            'distance': math.sqrt(dx**2 + dy**2),
            'x': robot_pos[0],
            'y':robot_pos[1],
            'z': robot_pos[2],
            'heading': robot_heading,
            'time': self.getTime()
        }
    
    def simulate_ir_sensor(self, sensor_data):
        """
        Uses the output of get_data to find which of the 16 
        photodiodes would see the ball.
        """
        rel_angle = sensor_data['angle_rad']
        dist = sensor_data['distance']
        
        # If ball is too far, sensors see nothing
        if dist > 3.0: return None 
        
        # 16 sensors = 22.5 degrees each (0.3927 rad)
        sector_size = (2 * math.pi) / 16
        
        # Convert angle to 0-15 index
        # Adding (sector_size/2) handles the rounding so 0 is center-front
        index = round(rel_angle / sector_size) % 16
        return int(index)

    def set_speed(self, left, right):
        # Clip speed to max capability of e-puck
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def set_holonomic_velocity(self, v, theta, omega):
        """
        Sets velocity directly on the physics body (God Mode).
        
        :param v:     Linear speed scalar (m/s)
        :param theta: Direction relative to robot nose (radians)
        :param omega: Angular velocity (rad/s) 
        """
        
        # --- 1. Calculate Local Linear Velocity (X-Y Plane) ---
        # Theta 0 -> +X (Forward)
        # Theta 90 -> +Y (Left)
        local_vx = v * math.cos(theta)
        local_vy = v * math.sin(theta)
        
        # --- 2. Convert Local -> Global Coordinates ---
        # Get current rotation matrix to transform local forward/left to world coordinates
        rot = self.robot_node.getOrientation()
        
        # Rotation Matrix Layout for getOrientation():
        # [ R00, R01, R02,
        #   R10, R11, R12,
        #   R20, R21, R22 ]
        
        # Global X component: (R00 * local_x) + (R01 * local_y)
        global_vx = (rot[0] * local_vx) + (rot[1] * local_vy)
        
        # Global Y component: (R3 * local_x) + (R4 * local_y)  <- Fixed indices (3,4 = R10, R11)
        global_vy = (rot[3] * local_vx) + (rot[4] * local_vy)
        
        # Global Z component (Height): Keep it 0 (or read existing velocity if needed)
        global_vz = 0 

        # --- 3. Apply to Physics Engine ---
        # We must pass a list of 6 values: [vx, vy, vz, wx, wy, wz]
        # Since the plane is X-Y, rotation is around the Z-axis (index 5)

        # print(global_vx,global_vy)

        velocity_vector = [
            global_vx,   # Linear X
            global_vy,   # Linear Y
            global_vz,   # Linear Z
            0,           # Angular X
            0,           # Angular Y
            omega        # Angular Z (Rotation)
        ]
        
        self.robot_node.setVelocity(velocity_vector)
    





    def run(self):
        print("Starting RCJ Particle Filter Simulation...")
        counter = 0
        
        while self.step(self.timestep) != -1:
            
            # --- A. GET DATA ---
            data = self.get_data()
            
            # --- B. SIMULATE HARDWARE ---
            # This 'ir_index' is the ONLY thing your algorithm is allowed to know about the ball
            ir_index = self.simulate_ir_sensor(data)
            
            # --- C. RUN ALGORITHM ---
            self.tracker.predict() # 1. Move particles slightly
            self.tracker.update(   # 2. Kill particles that don't match sensor
                robot_x=data['x'], 
                robot_y=data['y'], 
                robot_heading=data['heading'], 
                sensor_index=ir_index
            )
            self.tracker.resample() # 3. Re-spawn good particles
            
            # --- D. VISUALIZE ---
            est_x, est_y, stdev = self.tracker.get_estimated_pos()
            if stdev < 0.8:
                print("ok")
            else:
                print("nah")
            if self.ghost_color_field:
                    if stdev < 0.8: 
                        # CONFIDENT -> GREEN
                        green = [0, 1, 0]
                        self.ghost_color_field.setSFColor(green)
                        if hasattr(self, 'ghost_emissive_field'): 
                            self.ghost_emissive_field.setSFColor(green)
                    else:
                        # UNCERTAIN -> RED
                        blue = [1, 0, 0]
                        self.ghost_color_field.setSFColor(blue)
                        if hasattr(self, 'ghost_emissive_field'): 
                            self.ghost_emissive_field.setSFColor(blue)
            
            
            if self.ghost_node:
                # Move the blue ghost sphere to where the robot thinks the ball is
                self.ghost_node.getField("translation").setSFVec3f([est_x, est_y, 0.1])
            
            # 2. Update the Particle Cloud
            # We lift them slightly (z=0.01) so they sit on top of the carpet
            for i, p in enumerate(self.tracker.particles):
                self.particle_nodes[i].setSFVec3f([p[0], p[1], 0.01])

            # --- E. MOVE ROBOT ---
            if counter < 100:
                self.set_holonomic_velocity(1,0,0)
            elif counter < 200:
                self.set_holonomic_velocity(1,3.14,0)
            else:
                counter = 0
            counter += 1
            # self.set_holonomic_velocity(0.5,0,1)

# --- MAIN LOOP ---
robot = EpuckTracker()
robot.run()

# print("Simulation Started. Tracking Ball...")
# last = 0
# while robot.step(robot.timestep) != -1:
    
#     # 1. READ SENSORS
#     data = robot.get_data()
#     angle = round(data['angle_degrees'],2)
#     dist = data['distance']

#     # if angle < 0:
#     #     robot.set_speed(10,-10)
#     # elif angle > 0:
#     #     robot.set_speed(-10,10)
#     # else:
#     #     robot.set_speed(0,0)
#     #robot.set_speed(1,-1)
#     robot.set_holonomic_velocity(0,0,15)
    
#     # Debug print every 500ms roughly
    
#     if data['time'] - last >= 0.01:
#         # print(data['time'])
#         #print(f"Angle: {angle:.1f}° | Dist: {dist:.2f}m")
#         #print(data['heading'])
#         # print(data['x'],data['y'])
#         #print(data['angle_degrees'])
#         print(robot.simulate_ir_sensor(robot.get_data()))
#         last = data['time']