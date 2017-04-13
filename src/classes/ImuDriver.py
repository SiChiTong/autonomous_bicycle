import time
import math
import numpy
from classes.BNO055 import *


class ImuDriver(object):
    def __init__(self, serial_port="/dev/ttyUSB0"):
        self.degrees2rad = math.pi/180.0

        self.debug = False
        self.string_debug = ''

        self.serial_port = serial_port
        self.is_init_imu = False
        self.is_init_device = False
        self.is_init_calibration = False

        self.offset_yaw = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0

        self.bno = BNO055(serial_port=serial_port)

        # IMU info
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0

        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        self.euler_yaw = 0.0
        self.euler_roll = 0.0
        self.euler_pitch = 0.0

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        self.temperature = 0

    def init_imu(self):
        while not self.is_init_imu:
            try:
                self.bno.begin()
                self.is_init_imu = True
                self.string_debug = 'Connected to BNO055 at port: ' + str(self.serial_port)
            except:
                self.string_debug = 'Failed to initialize BNO055 at port: ' + str(self.serial_port)
                time.sleep(0.1)
            if self.debug:
                print self.string_debug

        while not self.is_init_device:
            status, self_test, error = self.bno.get_system_status(False)
            if error == 0 and status != 0x01:
                self.is_init_device = True
            else:
                self.string_debug = 'Failed to initialize IMU port: ' + str(self.serial_port) + ' error: ' + error
                time.sleep(0.1)

            if self.debug:
                print self.string_debug

        if not self.is_init_calibration:
            # Load precomputed calibration values
            self.load_calibration()

        while not self.is_init_calibration:
            cal_sys, cal_gyro, cal_accel, cal_mag = self.bno.get_calibration_status()

            if cal_gyro > 0 and cal_accel > 0:
                self.is_init_calibration = True
            else:
                self.load_calibration()
                self.string_debug = "Waiting for IMU calibration: [S %f, G %f, A %f, M %f]" % (cal_sys, cal_gyro, cal_accel, cal_mag)
                time.sleep(0.1)

            if self.debug:
                print self.string_debug

    def load_calibration(self):
        # computed using tutorial:
        # https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/webgl-example
        data = [253, 255, 5, 0, 166, 0, 205, 246, 93, 252, 95, 1, 254, 255, 255, 255, 1, 0, 232, 3, 163, 1]
        # data = [0, 0, 1, 0, 218, 255, 66, 254, 58, 2, 222, 255, 253, 255, 255, 255, 255, 255, 232, 3, 64, 3]

        self.bno.set_calibration(data)

        return True

    def read(self):
        [yaw, roll, pitch, lx, ly, lz, gx, gy, gz, temp] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        try:
            # qx, qy, qz, qw = self.bno.read_quaterion()    # Orientation as a quaternion
            # mx, my, mz = self.bno.read_magnetometer()     # Magnetometer data (in micro-Teslas)
            # ax, ay, az = self.bno.read_accelerometer()    # Accelerometer data (in meters per second squared)

            # Gravity acceleration data (i.e. acceleration just from gravity--returned
            # in meters per second squared):
            # x,y,z = bno.read_gravity()
            # temp = self.bno.read_temp()                     # Temperature in degrees Celsius

            yaw, roll, pitch = self.bno.read_euler()        # Euler angles for heading, roll, pitch (degrees)
            gx, gy, gz = self.bno.read_gyroscope()          # Gyroscope data (in degrees per second)

            # Linear acceleration data (i.e. acceleration from movement, not gravity--
            # returned in meters per second squared)
            lx, ly, lz = self.bno.read_linear_acceleration()

            # IMU info
            self.linear_acceleration_x = lx
            self.linear_acceleration_y = ly
            self.linear_acceleration_z = lz

            self.angular_velocity_x = gx
            self.angular_velocity_y = gy
            self.angular_velocity_z = gz

            self.euler_yaw = yaw
            self.euler_roll = roll
            self.euler_pitch = pitch

            self.temperature = temp

            self.euler_yaw = self.euler_yaw * self.degrees2rad
            self.euler_pitch = self.euler_pitch * self.degrees2rad
            self.euler_roll = self.euler_roll * self.degrees2rad

            q = self.quaternion_from_euler(self.euler_roll, self.euler_pitch, self.euler_yaw)
            self.orientation_x = q[0]
            self.orientation_y = q[1]
            self.orientation_z = q[2]
            self.orientation_w = q[3]

        except:
            self.is_init_device = True
            self.is_init_imu = False
            self.is_init_calibration = False
            self.init_imu()

    def quaternion_from_euler(self, ai, aj, ak, axes='sxyz'):
        """Return quaternion from Euler angles and axis sequence.
        ai, aj, ak : Euler's roll, pitch and yaw angles
        axes : One of 24 axis sequences as string or encoded tuple
        
        Source: ROS tf transformations
        https://github.com/ros/geometry        
        """
        # axis sequences for Euler angles
        _NEXT_AXIS = [1, 2, 0, 1]

        # map axes strings to/from tuples of inner axis, parity, repetition, frame
        _AXES2TUPLE = {
            'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
            'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
            'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
            'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
            'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
            'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
            'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
            'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

        _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

        try:
            firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
        except (AttributeError, KeyError):
            _ = _TUPLE2AXES[axes]
            firstaxis, parity, repetition, frame = axes

        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]

        if frame:
            ai, ak = ak, ai
        if parity:
            aj = -aj

        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        quaternion = numpy.empty((4, ), dtype=numpy.float64)
        if repetition:
            quaternion[i] = cj*(cs + sc)
            quaternion[j] = sj*(cc + ss)
            quaternion[k] = sj*(cs - sc)
            quaternion[3] = cj*(cc - ss)
        else:
            quaternion[i] = cj*sc - sj*cs
            quaternion[j] = cj*ss + sj*cc
            quaternion[k] = cj*cs - sj*sc
            quaternion[3] = cj*cc + sj*ss
        if parity:
            quaternion[j] *= -1

        return quaternion