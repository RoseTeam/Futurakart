#!/usr/bin/python2

#
# Emulate MBED KartMotorsController
#
import rospy
from math import pi

DIR_RAD_FACTOR = -2.655426455
DIR_RAD_OFFSET = 1.368429766
DIR_MIN_LIMIT_RAD = -0.6
DIR_MAX_LIMIT_RAD = 0.6


class Motor:
    def __init__(self):
        self._pos = 0.0
        self._vel = 0.0
        self._prev_pos = 0.0
        self._prev_vel = 0.0

    def set_cmd_vel(self, cmd_vel, delta):
        self._vel = self._prev_vel + 0.75 * (cmd_vel - self._prev_vel)
        self._pos += self._prev_vel * delta
        self._prev_vel = self._vel

    def set_cmd_pos(self, cmd_pos):
        self._pos = self._prev_pos + 0.2 * (cmd_pos - self._prev_pos)
        self._prev_pos = self._pos

    def get_pos(self):
        return self._pos

    def get_vel(self):
        return self._vel


class QEI:
    def __init__(self, motor, pulses_per_rev):
        self._pulses_per_rev = pulses_per_rev
        self._motor = motor

    def get_pulses(self):
        return self._pulses_per_rev / (2.0 * pi) * self._motor.get_pos()


class PotentioMeter:
    def __init__(self, motor):
        self._motor = motor

    def read(self):
        return (self._motor.get_pos() / DIR_RAD_FACTOR) - (DIR_RAD_OFFSET / DIR_RAD_FACTOR)


class KartMotorsController:

    def __init__(self, wait_time, feedback, prop_encoder, pulses_per_rev, dir_potentiometer):
        self._wait_time = wait_time
        self._last_time = rospy.Time()
        self._feedback = feedback  # [feedback_dir_pos_, feedback_prop_pos_, feedback_prop_vel_]
        self._prev_feedback = [0.0, 0.0]
        self._prop_wheel_encoder = prop_encoder
        self._dir_potentiometer = dir_potentiometer
        self._pulses_per_rev = pulses_per_rev
        self.pid_coeff = [0.6, 0.1, 0.0, 0.3, 0.0, 0.6, 0.1, 0.0, 0.3, 0.0]
        self._prop_prev_error = 0.0
        self._prop_error_sum = 0.0
        self._dir_prev_error = 0.0
        self._dir_error_sum = 0.0
        self._last_time = 0.0

    def ticks_to_rad(self, ticks):
        return ticks * 2.0 * pi / self._pulses_per_rev

    def potunit_to_rad(self, value):
        return DIR_RAD_FACTOR * value + DIR_RAD_OFFSET

    def control(self, current_time, setpoint_dir_pos, setpoint_prop_vel, cmd_dir_pos, cmd_prop_vel):
        delta = current_time - self._last_time
        if delta > self._wait_time:
            # Propulstion control
            # Update feedback propulsion position in rad
            self._feedback[1] = self.ticks_to_rad(self._prop_wheel_encoder.get_pulses())
            self._feedback[2] = (self._feedback[1] - self._prev_feedback[1]) / delta
            # Compute PID:
            error = setpoint_prop_vel - self._feedback[2]
            error_diff = (error - self._prop_prev_error) / delta
            self._prop_error_sum += error

            # Remove cum error if no error
            if error*error < 1e-10:
                rospy.loginfo("Remove cumulative error for propulsion")
                self._prop_error_sum = 0.0

            ki_term = 0.3 if self.pid_coeff[1] * self._prop_error_sum > 0.3 else self.pid_coeff[1] * self._prop_error_sum
            cmd_prop_vel = self.pid_coeff[0] * error + ki_term + self.pid_coeff[2] * error_diff

            self._prev_feedback[1] = self._feedback[1]
            self._prop_prev_error = error

            # Direction control
            self._feedback[0] = self.potunit_to_rad(self._dir_potentiometer.read())
            # Compute PID:
            error = setpoint_dir_pos - self._feedback[0]
            error_diff = (error - self._dir_prev_error) / delta
            self._dir_error_sum += error
            if error*error < 1e-2:
                rospy.loginfo("Remove cumulative error for propulsion")
                self._dir_error_sum = 0.0

            ki_term = 0.3 if self.pid_coeff[6] * self._dir_error_sum > 0.3 else self.pid_coeff[6] * self._dir_error_sum
            dir_pos = self.pid_coeff[5] * error + ki_term + self.pid_coeff[7] * error_diff

            # For motor sanity
            if -self.pid_coeff[8] < dir_pos < self.pid_coeff[8]:
                dir_pos = 0.0

            # For security reasons
            if self._feedback[0] > DIR_MAX_LIMIT_RAD or self._feedback[0] < DIR_MIN_LIMIT_RAD:
                rospy.logerr("Direction security is applied")
                cmd_dir_pos = 0.0
            else:
                cmd_dir_pos = dir_pos

            self._prev_feedback[0] = self._feedback[0]
            self._dir_prev_error = error

            self._last_time = current_time

        print "1.1 : ", cmd_dir_pos, cmd_prop_vel
        return cmd_dir_pos, cmd_prop_vel


if __name__ == "__main__":
    import time

    prop_motor = Motor()
    dir_motor = Motor()

    prop_encoder = QEI(prop_motor, 64)
    dir_potmeter = PotentioMeter(dir_motor)

    cmd_vels = [1.0, 2.0, 3.0, 0.0, -4.0, 4.0]
    cmd_pos_s = [0.4, 0.8, 0.0, -0.4, 0.4, -0.8]
    count = 250
    index_vel = 0
    index_pos = 0
    while count > 0:
        print "--- count = ", count, "---"
        print "- index_vel = ", index_vel
        print "- index_pos = ", index_pos
        print "- Prop motor : ", prop_motor._pos, prop_motor._vel
        print "- Dir motor : ", dir_motor._pos

        if abs(prop_motor.get_vel() - cmd_vels[index_vel]) < 1e-2 and index_vel < len(cmd_vels) - 1:
            index_vel += 1
        if abs(dir_motor.get_pos() - cmd_pos_s[index_pos]) < 1e-2 and index_pos < len(cmd_pos_s) - 1:
            index_pos += 1

        prop_motor.set_cmd_vel(cmd_vels[index_vel], 0.2)
        dir_motor.set_cmd_pos(cmd_pos_s[index_pos])

        time.sleep(0.2)
        count -= 1
