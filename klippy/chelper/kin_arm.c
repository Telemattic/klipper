// Delta kinematics stepper pulse time generation
//
// Copyright (C) 2024 Matthew Eldridge <matthew.eldridge@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct arm_stepper {
    struct stepper_kinematics sk;
    double L0, L2, Lp, Ld;
    char type;
};

static double
arm_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
			  , double move_time)
{
    struct arm_stepper *as = container_of(sk, struct arm_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    if (as->type == 'a')
      return atan2(c.y, c.x);
    
    double rd = hypot(c.x, c.y) - as->L0;
    double zd = c.z - as->L2;
    double Lp = as->Lp;
    double Ld = as->Ld;
    double Lp_sq = Lp * Lp;
    double Ld_sq = Ld * Ld;
    double Lv_sq = rd*rd + zd*zd;

    if (as->type == 'b') {
      double Lv = sqrt(Lv_sq);
      return atan2(zd, rd) + acos((Lp_sq + Lv_sq - Ld_sq)/(2*Lp*Lv));
    }
    
    /* as->type == 'c' */
    return acos((Lp_sq + Ld_sq - Lv_sq) / (2*Lp*Ld));
}

struct stepper_kinematics * __visible
arm_stepper_alloc(double L0, double L2, double Lp, double Ld, char type)
{
    struct arm_stepper *as = malloc(sizeof(*as));
    memset(as, 0, sizeof(*as));
    as->L0 = L0;
    as->L2 = L2;
    as->Lp = Lp;
    as->Ld = Ld;
    as->type = type;
    as->sk.calc_position_cb = arm_stepper_calc_position;
    if (type == 'a')
      as->sk.active_flags = AF_X | AF_Y;
    else
      as->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &as->sk;
}
