# Code for handling the kinematics of linear delta robots
#
# Copyright (C) 2024  Matthew Eldridge <matthew.eldridge@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, mathutil

class ArmKinematics:
    def __init__(self, toolhead, config):
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax = False,
            units_in_radians=True)
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            units_in_radians=True)
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax = False,
            units_in_radians=True)
        self.rails = [rail_a, rail_b, rail_c]
        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        # Read radius and arm lengths
        self.arm_L0 = L0 = config.getfloat('arm_L0')
        self.arm_L2 = L2 = config.getfloat('arm_L2')
        self.arm_Lp = Lp = config.getfloat('arm_Lp')
        self.arm_Ld = Ld = config.getfloat('arm_Ld')
        for r, stype in zip(self.rails, 'abc'):
            r.setup_itersolve('arm_stepper_alloc', L0, L2, Lp, Ld, stype)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        self.limits = [(1.0, -1.0)] * 3
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def _actuator_to_cartesian(self, s0, s1, s2):
        L0, L2, Ld, Lp = self.arm_L0, self.arm_L2, self.arm_Ld, self.arm_Lp
        rd = Lp * math.cos(s1) + Ld * math.sin(s1+s2-math.pi/2.)
        zd = Lp * math.sin(s1) - Ld * math.cos(s1+s2-math.pi/2.)
        x = (rd + L0) * math.cos(s0)
        y = (rd + L0) * math.sin(s0)
        z = zd + L2
        return (x, y, z)
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(*spos)
    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None] * 3
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            self.home_axis(homing_state, axis, self.rails[axis])
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def check_move(self, move):
        pass
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return ArmKinematics(toolhead, config)
