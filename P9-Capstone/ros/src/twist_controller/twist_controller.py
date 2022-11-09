
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0.0, mn = 0.0, mx=0.2)
        self.vel_lpf = LowPassFilter(tau=0.5, ts=0.02)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time() # for PID sampling


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # If the car is in safety mode
        if not dbw_enabled:
            # PID reset
            self.throttle_controller.reset() 
            return 0., 0., 0.
        
        # Filter out velocity noise
        current_vel = self.vel_lpf.filt(current_vel)
        # Calculate yaw control
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # P error
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        # Sampling timestep
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        # Calculate throttle
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # If car needs to stop completely, apply brake to hold the car.
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700 #N*m
        
        # Decelerate if throttle is less than threshold.
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # N*m. Simple physics

        return throttle, brake, steering
