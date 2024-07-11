// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "hopper_2d_kinematics.h"
#include <cmath>

namespace hopper_2d_kinematics {


// FK for 2 link parallel robot with extra bit on the end
void fk_analytic(helpers::vec2& th, helpers::vec2* xd) {
    // Find the positions of the points A and B
    helpers::vec2 a( helpers::vec2(parameter_l1*cos(th[0]), parameter_l1*sin(th[0])) );
    helpers::vec2 b( helpers::vec2(parameter_l1*cos(th[1]), parameter_l1*sin(th[1])) );

    double h = sqrt(helpers::sq(b.x-a.x) + helpers::sq(b.y-a.y));
    double d = acos(h / (2.0*parameter_l2));
    double Y = atan2( b.y-a.y, b.x-a.x );

    double phi_1 = d + Y;
    double phi_2 = M_PI - (phi_1 - 2.0*Y);

    xd->x = b.x + parameter_l2*cos(phi_2) + parameter_l3*cos(phi_1);
    xd->y = b.y + parameter_l2*sin(phi_2) + parameter_l3*sin(phi_1);
}

bool ik_analytic(helpers::vec2& xd, helpers::vec2 *th) {

    // Find the length and angle of xd to origin
    double alpha_d = atan2(xd.y, xd.x);
    double c_d     = sqrt(helpers::sq(xd.x) + helpers::sq(xd.y));
    // Find the top angle of the triangle
    double l_hat  = parameter_l2 + parameter_l3;
    double beta_3 = acos( (helpers::sq(c_d)+helpers::sq(l_hat)-helpers::sq(parameter_l1)) / (2.0*c_d*l_hat) );
    // Then find the angle of the line xd-xc in x0
    double beta_hat = alpha_d - beta_3;
    // Now we can find the coordinates of xc
    helpers::vec2 xc( xd.x-parameter_l3*cos(beta_hat), xd.y-parameter_l3*sin(beta_hat) );

    double alpha = atan2(xc.y, xc.x);
    double c     = sqrt(helpers::sq(xc.x) + helpers::sq(xc.y));
    double gamma = acos( (helpers::sq(parameter_l1)+helpers::sq(c)-helpers::sq(parameter_l2)) / (2.0*parameter_l1*c) );

    if (std::isnan(alpha) || std::isnan(gamma)) {
        return false;
    }
    th->x = alpha + gamma;
    th->y = alpha - gamma;
    return true;
}

// Old:
// FK and IK for a 2 link parallel robot
// void fk_analytic_simplified(helpers::vec2& th, helpers::vec2* xp) {
//     helpers::vec2 r1( helpers::vec2(parameter_l1*cos(th[0]), parameter_l1*sin(th[0])) );
//     helpers::vec2 r2( helpers::vec2(parameter_l1*cos(th[1]), parameter_l1*sin(th[1])) );

//     double v1 = (r1.y - r2.y) / (r2.x - r1.x);

//     double v3 = helpers::sq(v1) + 1.0;
//     double v4 = 2.0*(-v1*r1.x - r1.y);
//     double v5 = helpers::sq(parameter_l1) - helpers::sq(parameter_l2);

//     // Solve the quadratic, but we always want the positive term (elbows out)
//     double y_pos = (-v4 + sqrt(helpers::sq(v4) - 4.0*v3*v5)) / (2.0*v3);
//     xp->x = v1*y_pos; 
//     xp->y = y_pos; 
// }

// bool ik_analytic(helpers::vec2& xp, helpers::vec2 *th) {
//     double alpha = atan2(xp.y, xp.x);
//     double c     = sqrt(helpers::sq(xp.x) + helpers::sq(xp.y));
//     double gamma = acos( (helpers::sq(parameter_l1)+helpers::sq(c)-helpers::sq(parameter_l2)) / (2.0*parameter_l1*c) );

//     // Overloads dont work that well here... Should have just used Eigen...
//     // *th[0] = alpha + gamma;
//     // *th[1] = alpha - gamma;
//     if (std::isnan(alpha) || std::isnan(gamma)) {
//         return false;
//     }
//     th->x = alpha + gamma;
//     th->y = alpha - gamma;
//     return true;
// }

// Numeric jacobian
// dxp/dqi = (1/delta_q) * fk(q1, qi + delta_q, qn) - fk(q1,..., qn)
//
// https://www.youtube.com/watch?v=wI5FQD3uIDg
void jacobian_numeric(helpers::vec2& th, helpers::mat22* J) {
    // Find x for current joint position
    helpers::vec2 x_i;
    fk_analytic(th, &x_i);

    double const delta_q = 0.001;

    helpers::vec2 j11_j12;
    helpers::vec2 th_perturbed = helpers::vec2(th[0]+delta_q, th[1]);
    fk_analytic(th_perturbed, &j11_j12);
    j11_j12 = j11_j12 - x_i;

    helpers::vec2 j21_j22;
    th_perturbed = helpers::vec2(th[0], th[1]+delta_q);
    fk_analytic(th_perturbed, &j21_j22);
    j21_j22 = j21_j22 - x_i;

    helpers::mat22 JT(j11_j12.x, j11_j12.y, j21_j22.x, j21_j22.y);
    JT.divide(delta_q);

    *J = JT.T();
}

} // End namespace