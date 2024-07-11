// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "hopper_2d_vmc_simple_xy_ctl.h"
#include "hopper_2d_kinematics.h"
#include <cmath>

#include "jcs_host.h"

hopper_2d_vmc_simple_xy_ctl::hopper_2d_vmc_simple_xy_ctl() :
    hopper_2d("Simple VMC xy control")
{
    // Starting tip postion
    x_tip_ = {0.0, 0.14121};
    // Set initial commanded position
    x_tip_ref_ = {0.0, 0.15};

    // XY limits
    x_tip_x_coord_limits_ = {-0.17, 0.17};
    x_tip_y_coord_limits_ = {0.0, 0.21};

    // Initial gains are soft
    cart_pd_x_.kp = 10.0;
    cart_pd_x_.kd = 1.5;
    cart_pd_y_.kp = 10.0;
    cart_pd_y_.kd = 1.5;

    tau_cmd_ = {0.0, 0.0};

    thd_source_ = thd_source::motor_s;

    outputs_active_ = false;
}

int hopper_2d_vmc_simple_xy_ctl::startup(double sample_rate_hz) {
    return jcs::RET_OK;
}

bool hopper_2d_vmc_simple_xy_ctl::can_start() {
    // Check the signal sizes are ok for what we need
    if (f32_ctl_osig_->size() < 2) { return false; }
    if (f32_ctl_isig_->size() < 6) { return false; }
    return true;
}

int hopper_2d_vmc_simple_xy_ctl::render() {
    const double default_step_val = 1.0;

    ImGui::PushID(this);

    ImGui::Text("VMC based simple 2D XY control");
    ImGui::Text("Use configuration: 04_cnf_vmc_hopping");

    ImGui::Checkbox("Output signals active", &outputs_active_);

    ImGui::Separator();
    ImGui::SliderScalar("X cmd (m)", ImGuiDataType_Double, &x_tip_ref_[0], &x_tip_x_coord_limits_[0], &x_tip_x_coord_limits_[1]);
    ImGui::SliderScalar("Y cmd (m)", ImGuiDataType_Double, &x_tip_ref_[1], &x_tip_y_coord_limits_[0], &x_tip_y_coord_limits_[1]);

    const double zero = 0.0;
    const double p_max = 300.0;
    const double d_max = 10.0;

    ImGui::Separator();

    ImGui::Text("X axis gains");
    ImGui::SliderScalar("X axis P gain",   ImGuiDataType_Double, &cart_pd_x_.kp, &zero, &p_max);
    ImGui::SliderScalar("X axis D gain",   ImGuiDataType_Double, &cart_pd_x_.kd, &zero, &d_max);

    ImGui::Text("Y axis gains");
    ImGui::SliderScalar("Y axis P gain",   ImGuiDataType_Double, &cart_pd_y_.kp, &zero, &p_max);
    ImGui::SliderScalar("Y axis D gain",   ImGuiDataType_Double, &cart_pd_y_.kd, &zero, &d_max);

    ImGui::Separator();
    ImGui::Text("Select joint theta_dot source");
    if (ImGui::RadioButton("Joint", thd_source_ == thd_source::joint_s)) { thd_source_ = thd_source::joint_s; } ImGui::SameLine();
    if (ImGui::RadioButton("Motor", thd_source_ == thd_source::motor_s)) { thd_source_ = thd_source::motor_s; }

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

    ImGui::Text("Info");
    if (ImGui::BeginTable("Info", 3, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("th 0/1 Actual");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", th_[0]);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", th_[1]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("thd 0/1 Actual");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", thd_[0]);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", thd_[1]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position Command");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_ref_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_ref_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position Error");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_ref_.x - x_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_ref_.y - x_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Velcocity");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", v_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", v_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Force command");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", f_tip_cmd_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", f_tip_cmd_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("tau 0/1 commands");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", tau_cmd_[0]);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", tau_cmd_[1]);

        ImGui::EndTable();
    }

    ImGui::PopID();

    return jcs::RET_OK;
}

int hopper_2d_vmc_simple_xy_ctl::step_rt_init() {
    return jcs::RET_OK;
}

int hopper_2d_vmc_simple_xy_ctl::step_rt_run() {

    // Assemble joint feedback positions into a vector
    th_[0] = f32_ctl_isig_->at(static_cast<int>(isig_map::j0_th));
    th_[1] = -f32_ctl_isig_->at(static_cast<int>(isig_map::j1_th));
    // Kinematics output has theta=0 along the x axis.
    // Robot has theta=0 along y axis
    th_ = th_ + helpers::vec2(M_PI_2, M_PI_2);

    // Assemble joint feedback velocities into a vector
    switch (thd_source_) {
        case thd_source::joint_s:
            thd_[0] = f32_ctl_isig_->at(static_cast<int>(isig_map::j0_thd));
            thd_[1] = -f32_ctl_isig_->at(static_cast<int>(isig_map::j1_thd));
            break;        
        case thd_source::motor_s:
            thd_[0] = f32_ctl_isig_->at(static_cast<int>(isig_map::m0_w_m));
            thd_[1] = -f32_ctl_isig_->at(static_cast<int>(isig_map::m1_w_m));
            // Account for gear ratio
            thd_[0] *= (1.0/3.0);
            thd_[1] *= (1.0/3.0);
    }

    // FK from joint positions to get the x_top position
    hopper_2d_kinematics::fk_analytic(th_, &x_tip_);

    // Compute the leg jacobian to get the x_tip velocity
    hopper_2d_kinematics::jacobian_numeric(th_, &J);
    v_tip_ = J*thd_;

    // Step PD controller for y direction only. Output is a cartesian force
    f_tip_cmd_.x = cart_pd_x_.step_rt(x_tip_ref_.x - x_tip_.x, 0.0-v_tip_.x);
    f_tip_cmd_.y = cart_pd_y_.step_rt(x_tip_ref_.y - x_tip_.y, 0.0-v_tip_.y);

    // Transform commanded force into joint torque commands
    tau_cmd_ = J.T()*f_tip_cmd_;

    // Write the output joint torques
    if (outputs_active_) {
        // Torque commands are negative
        f32_ctl_osig_->at(static_cast<int>(osig_map::j0_tau)) = static_cast<float>(tau_cmd_[0]);
        f32_ctl_osig_->at(static_cast<int>(osig_map::j1_tau)) = static_cast<float>(-tau_cmd_[1]);
    } else {
        f32_ctl_osig_->at(static_cast<int>(osig_map::j0_tau)) = static_cast<float>(0.0);
        f32_ctl_osig_->at(static_cast<int>(osig_map::j1_tau)) = static_cast<float>(0.0);
    }
    return jcs::RET_OK;
}