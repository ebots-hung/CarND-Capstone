from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
STOP_TORQUE = 400   # Torque needed to keep the car from moving when stationary


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Throttle PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.0
        
        # rospy.loginfo("PID Kp :{0}".format(kp))
        # rospy.loginfo("PID Ki :{0}".format(ki))
        # rospy.loginfo("PID Kd :{0}".format(kd))

        mn = 0.0  # Min throttle
        mx = 0.2  # Max throttle

        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Velocity low-pass filter - to filter noise
        tau = 0.5  # Time constant
        ts = 0.02  # Sampling time
        
        # rospy.loginfo("Low pass filter Time constant :{0}".format(tau))
        # rospy.loginfo("Low pass filter Sampling time :{0}".format(ts))

        self.vel_lpf = LowPassFilter(tau, ts)

        # Other parameters
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # In case of drive-bt-wire NOT enabled, reset PID and return 0
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # Otherwise, first filter velocity
        current_vel = self.vel_lpf.filt(current_vel)

        #rospy.logwarn("Angular vel: {0}".format(angular_vel))
        #rospy.logwarn("Target velocity: {0}".format(linear_vel))
        #rospy.logwarn("Target angular velocity: {0}\n".format(angular_vel))
        #rospy.logwarn("Current velocity: {0}".format(current_vel))
        #rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        # Calculate steering
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # Calculate throttle
        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        # Calculate brake
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            # Vehcile stopped
            throttle = 0.0
            brake = STOP_TORQUE # N*m - needed to keep the car in place

        elif throttle < 0.1 and vel_error < 0.0:
            # Stop throttle and calculate brake
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius

        return throttle, brake, steering
