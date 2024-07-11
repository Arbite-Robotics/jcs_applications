// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#ifndef HOPPER_2D_KINEMATICS_H_
#define HOPPER_2D_KINEMATICS_H_

#include "helpers.h"

namespace hopper_2d_kinematics {

static double const parameter_l1 = 0.1;
static double const parameter_l2 = 0.1;
// Small section on end of link 2.
// Is 20mm from l2 pivot to surface tip
static double const parameter_l3 = 0.02;

void fk_analytic(helpers::vec2& th, helpers::vec2* xd);
bool ik_analytic(helpers::vec2& xd, helpers::vec2 *th);
void jacobian_numeric(helpers::vec2& th, helpers::mat22* J);

} // End namespace 

#endif