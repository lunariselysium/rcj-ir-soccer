import math
import random
import statistics

class ParticleFilter:
    def __init__(self, field_x=4.0, field_y=3.0, num_particles=200):
        self.n = num_particles
        # Initialize particles randomly on the field (X, Y)
        self.particles = []
        for _ in range(self.n):
            px = random.uniform(-field_x/2, field_x/2)
            py = random.uniform(-field_y/2, field_y/2)
            self.particles.append([px, py])
            
        self.weights = [1.0 / self.n] * self.n

    def normalize_angle(self, angle):
        """
        Forces an angle into the range -PI to +PI.
        Crucial for preventing wrap-around errors.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def predict(self):
        """
        Step 1: Prediction
        Since we don't know the ball's velocity, we assume it stays roughly 
        where it is, but we add noise. This 'spreads' the particles out 
        slightly every frame, representing uncertainty.
        """
        noise = 0.009 # 5cm of uncertainty per step
        for i in range(self.n):
            self.particles[i][0] += random.gauss(0, noise)
            self.particles[i][1] += random.gauss(0, noise)

    def update(self, robot_x, robot_y, robot_heading, sensor_index):
        """
        Step 2: Correction (Measurement Update)
        We verify which particles align with the IR sensor reading.
        """
        if sensor_index is None:
            return # Robot sees nothing, trust prediction only

        # 1. Convert the Sensor Index (0-15) back to an Angle (-PI to PI)
        # Index 0 = 0 rad, Index 4 = 1.57 rad, Index 12 = 4.71 rad (which is -1.57 rad)
        sensor_angle = sensor_index * (2 * math.pi / 16)
        
        # Normalize this angle to match your get_data format (-PI to PI)
        sensor_angle = self.normalize_angle(sensor_angle)

        for i in range(self.n):
            # 2. Calculate Angle from Robot to this specific Particle
            dx = self.particles[i][0] - robot_x
            dy = self.particles[i][1] - robot_y
            
            # Global angle on the field
            particle_global_angle = math.atan2(dy, dx)
            
            # Relative angle (where is the particle relative to robot nose?)
            particle_rel_angle = particle_global_angle - robot_heading
            
            # Normalize to -PI to PI
            particle_rel_angle = self.normalize_angle(particle_rel_angle)
            
            # 3. Calculate difference between Particle Angle and Sensor Angle
            angle_diff = abs(particle_rel_angle - sensor_angle)
            
            # 4. Handle the "Date Line" problem
            # If Particle is at +179 deg (3.12 rad) and Sensor is at -179 deg (-3.12 rad),
            # The math says diff is 6.24 rad. The REAL diff is 0.04 rad (2 degrees).
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # 5. Assign Weight (Gaussian)
            # sigma is how wide we trust the sensor to be (approx 22 degrees or 0.4 rad)
            sigma = 0.2
            probability = math.exp(-(angle_diff**2) / (2 * sigma**2))

            # WEIGHT FLOOR: Prevent weight from reaching absolute zero
            self.weights[i] *= probability + 1e-10
            
            # self.weights[i] *= probability

        # 6. Normalize Weights (ensure they sum to 1.0)
        total_weight = sum(self.weights) + 1.e-300 # avoid divide by zero
        self.weights = [w / total_weight for w in self.weights]

    def resample(self):
        """
        Step 3: Resampling
        Kill particles with low weight (wrong direction).
        Duplicate particles with high weight (correct direction).
        """
        new_particles = []
        # --- ANTI-STUBBORN LOGIC: Random Injection ---
        # We replace 10% of particles with totally random ones 
        # so the filter can 'discover' the ball if it was wrong.
        num_random = int(self.n * 0.10) 
        for _ in range(num_random):
            px = random.uniform(-2.0, 2.0) # Field width
            py = random.uniform(-1.5, 1.5) # Field height
            new_particles.append([px, py])

        # Resample the remaining 90% using the standard wheel
        index = int(random.random() * self.n)
        beta = 0.0
        max_weight = max(self.weights)
        
        for _ in range(self.n - num_random):
            beta += random.random() * 2.0 * max_weight
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % self.n
            new_particles.append(list(self.particles[index]))
            
        self.particles = new_particles
        self.weights = [1.0 / self.n] * self.n

    def get_estimated_pos(self):
        """
        Returns the average X, Y of all particles.
        """
        avg_x = sum(p[0] for p in self.particles) / self.n
        avg_y = sum(p[1] for p in self.particles) / self.n
        stdev = statistics.stdev(p[0] for p in self.particles) + statistics.stdev(p[1] for p in self.particles)
        return avg_x, avg_y, stdev