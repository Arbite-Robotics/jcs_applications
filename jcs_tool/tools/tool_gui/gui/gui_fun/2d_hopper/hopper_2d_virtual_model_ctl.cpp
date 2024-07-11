// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "hopper_2d_virtual_model_ctl.h"
#include "hopper_2d_kinematics.h"
#include <cmath>

#include "jcs_host.h"

hopper_2d_virtual_model_ctl::hopper_2d_virtual_model_ctl() :
    hopper_2d("VMC hopper")
{
    hop_state_ = hop_state::stop_s;

    // Starting tip postion
    x_tip_ = {0.0, 0.14121};
    // Tip position xy limits
    x_tip_x_coord_limits_ = {-0.17, 0.17};
    x_tip_y_coord_limits_ = {0.01, hopper_2d_kinematics::parameter_l1 + hopper_2d_kinematics::parameter_l2 - 0.005};
    // Loading and unloading limits
    x_tip_y_loading_ = 0.1;
    x_tip_y_unloading_ = x_tip_y_coord_limits_[1] - 0.005;

    x_tip_y_loading_thresh_ = 0.0015;
    // x_tip_y_unloading_thresh_ = 0.0015;
    x_tip_y_flight_to_loading_thresh_ = 0.002;

    // Set initial commanded position
    x_tip_ref_ = {0.02, 0.5 * (x_tip_y_loading_ + x_tip_y_unloading_)};

    // Loading: Robot is coming in for landing and compressing
    // Soft gains
    loading_gain_ = {130.0, 1.0};
    // Unloading: Robot is springing off to jump
    // Stiff gains
    unloading_gain_ = {150.0, 2.1};

    // X axis gains are stiff to hold x at 0
    cart_pd_x_.kp = 170.0;
    cart_pd_x_.kd = 1.5;
    // Initial y gains are loading gains
    cart_pd_y_.kp = loading_gain_[0];
    cart_pd_y_.kd = loading_gain_[1];

    tau_cmd_ = {0.0, 0.0};

    thd_source_ = thd_source::motor_s;

    do_ff_mass_ = false;
    ff_mass_kg_ = 1.0;

    f_tip_y_unloading_ = 0.0;

    outputs_active_ = false;
    stop_requested_ = false;
}

int hopper_2d_virtual_model_ctl::startup(double sample_rate_hz) {
    return jcs::RET_OK;
}

bool hopper_2d_virtual_model_ctl::can_start() {
    // Check the signal sizes are ok for what we need
    if (f32_ctl_osig_->size() < 2) { return false; }
    if (f32_ctl_isig_->size() < 6) { return false; }
    return true;
}

int hopper_2d_virtual_model_ctl::render() {
    const double default_step_val = 1.0;

    ImGui::PushID(this);

    ImGui::Text("VMC based 2D hopping controller");
    ImGui::Text("Use configuration: 04_cnf_vmc_hopping");

    if (ImGui::Button("Start hopping")) {
        next_hop_rt(hop_state::loading_s);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop hopping")) {
        next_hop_rt(hop_state::stop_s);
    }
    ImGui::SameLine();
    ImGui::Checkbox("Output signals active", &outputs_active_);

    ImGui::Separator();
    ImGui::Checkbox("Add mass feedforward to y force", &do_ff_mass_);
    ImGui::InputScalar("Feedforward mass (kg)", ImGuiDataType_Double, &ff_mass_kg_, &default_step_val);

    const double zero = 0.0;
    const double p_max = 300.0;
    const double d_max = 10.0;

    ImGui::Text("Loading length");
    ImGui::SliderScalar("Loading length (m)", ImGuiDataType_Double, &x_tip_y_loading_, &x_tip_y_coord_limits_[0], &x_tip_y_coord_limits_[1]);
    ImGui::InputScalar( "Loading threshold", ImGuiDataType_Double, &x_tip_y_loading_thresh_, &default_step_val);
    ImGui::Text("Loading gains: Robot is coming in for landing and compressing down - soft gains");
    ImGui::SliderScalar("Loading P gain",   ImGuiDataType_Double, &loading_gain_[0], &zero, &p_max);
    ImGui::SliderScalar("Loading D gain",   ImGuiDataType_Double, &loading_gain_[1], &zero, &d_max);

    ImGui::Separator();

    ImGui::Text("Unloading length");
    ImGui::SliderScalar("Unloading length (m)", ImGuiDataType_Double, &x_tip_y_unloading_, &x_tip_y_coord_limits_[0], &x_tip_y_coord_limits_[1]);
    // ImGui::InputScalar( "Unloading threshold", ImGuiDataType_Double, &x_tip_y_unloading_thresh_, &default_step_val);
    ImGui::Text("Unloading gains: Robot is springing off for a jump - stiff gains");
    ImGui::SliderScalar("Unloading P gain", ImGuiDataType_Double, &unloading_gain_[0], &zero, &p_max);
    ImGui::SliderScalar("Unloading D gain", ImGuiDataType_Double, &unloading_gain_[1], &zero, &d_max);

    const double uf_max = 100.0;
    ImGui::Text("Unloading force");
    ImGui::Text("Force added during unloading phase");
    ImGui::SliderScalar("Unloading force (N)", ImGuiDataType_Double, &f_tip_y_unloading_, &zero, &uf_max);

    ImGui::Text("Flight");
    ImGui::InputScalar( "Flight to loading threshold", ImGuiDataType_Double, &x_tip_y_flight_to_loading_thresh_, &default_step_val);

    ImGui::Separator();
    ImGui::Text("X tip position");
    ImGui::SliderScalar("X cmd (m)", ImGuiDataType_Double, &x_tip_ref_[0], &x_tip_x_coord_limits_[0], &x_tip_x_coord_limits_[1]);
    ImGui::Text("X axis gains");
    ImGui::SliderScalar("X axis P gain",   ImGuiDataType_Double, &cart_pd_x_.kp, &zero, &p_max);
    ImGui::SliderScalar("X axis D gain",   ImGuiDataType_Double, &cart_pd_x_.kd, &zero, &d_max);

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
        ImGui::Text("XY Position");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Velocity");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", v_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", v_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position Command");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_ref_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_ref_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position Error");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_ref_.x - x_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_ref_.y - x_tip_.y);

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

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("State");
        ImGui::TableSetColumnIndex(1);
        print_state();
        ImGui::TableSetColumnIndex(2);

        ImGui::EndTable();
    }

    ImGui::PopID();

    return jcs::RET_OK;
}

int hopper_2d_virtual_model_ctl::step_rt_init() {
    return jcs::RET_OK;
}

int hopper_2d_virtual_model_ctl::step_rt_run() {

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

    switch(hop_state_) {
        default:
        case hop_state::stop_s:
            cart_pd_y_.kp = loading_gain_[0];
            cart_pd_y_.kd = loading_gain_[1];
            maybe_next_flight();
            break;

        case hop_state::loading_s:
            maybe_next_unloading();
            break;

        case hop_state::unloading_s:
            maybe_next_flight();
            break;

        case hop_state::flight_s:
            maybe_next_loading();
            break;
    }

    // Compute the leg jacobian to get the x_tip velocity
    hopper_2d_kinematics::jacobian_numeric(th_, &J);
    v_tip_ = J*thd_;

    // Step PD controller for y direction only. Output is a cartesian force
    f_tip_cmd_.x = cart_pd_x_.step_rt(x_tip_ref_.x - x_tip_.x, 0.0-v_tip_.x);
    f_tip_cmd_.y = cart_pd_y_.step_rt(x_tip_ref_.y - x_tip_.y, 0.0-v_tip_.y);

    // Add mass feedforward? Only when in contact with the ground
    switch(hop_state_) {
        default:
        case hop_state::flight_s:
            break;

        case hop_state::stop_s:
        case hop_state::loading_s:
        case hop_state::unloading_s:
            if (do_ff_mass_) {
                f_tip_cmd_.y += ff_mass_kg_ * 9.81;
            }
            break;
    }

    // Add unloading force?
    if (hop_state_ == hop_state::unloading_s) {
        f_tip_cmd_.y +=  f_tip_y_unloading_;
    }

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

void hopper_2d_virtual_model_ctl::maybe_next_flight() {
    // When the leg is fully extended, we are flying
    // if (std::abs(x_tip_ref_.y - x_tip_.y) < x_tip_y_unloading_thresh_) {
    // if (x_tip_.y > (x_tip_y_unloading_-x_tip_y_unloading_thresh_)) {
    if (x_tip_.y > x_tip_y_unloading_) {
        next_hop_rt(hop_state::flight_s);
    }
}
void hopper_2d_virtual_model_ctl::maybe_next_unloading() {
    // When we are loaded down fully, do the hop
    if (std::abs(x_tip_ref_.y - x_tip_.y) < x_tip_y_loading_thresh_) {
        next_hop_rt(hop_state::unloading_s);
    }
}
void hopper_2d_virtual_model_ctl::maybe_next_loading() {
    // We have stopped flying when the leg starts to compress - Note: Not abs
    // if ((x_tip_ref_.y - x_tip_.y)  > x_tip_y_flight_to_loading_thresh_) {
    if (x_tip_.y  < (x_tip_y_unloading_-x_tip_y_flight_to_loading_thresh_)) {
        if (stop_requested_) {
            next_hop_rt(hop_state::stop_s);
        } else {
            next_hop_rt(hop_state::loading_s);
        }
    }
}

void hopper_2d_virtual_model_ctl::next_hop_rt(hop_state next) {
    switch(next) {
        default:
        case hop_state::stop_s:
            stop_requested_ = true;
            // Stop gains are soft loading gains
            cart_pd_y_.kp = loading_gain_[0];
            cart_pd_y_.kd = loading_gain_[1];
            // Y Position somewhere in the middle
            x_tip_ref_.y = 0.5 * (x_tip_y_loading_ + x_tip_y_unloading_);
            break;

        case hop_state::loading_s:
            stop_requested_ = false;
            // Set controller loading gains
            cart_pd_y_.kp = loading_gain_[0];
            cart_pd_y_.kd = loading_gain_[1];
            // Required position is loading position
            x_tip_ref_.y = x_tip_y_loading_;
            break;

        case hop_state::unloading_s:
            // Set controller unloading gains
            cart_pd_y_.kp = unloading_gain_[0];
            cart_pd_y_.kd = unloading_gain_[1];
            // Required position is the unloading position
            x_tip_ref_.y = x_tip_y_unloading_;
            break;

        case hop_state::flight_s:
            // Flight gains are soft loading gains
            cart_pd_y_.kp = unloading_gain_[0];
            cart_pd_y_.kd = unloading_gain_[1];
            // Flight position is the unloading position
            x_tip_ref_.y = x_tip_y_unloading_;
            break;
    }

    hop_state_ = next;
}

void hopper_2d_virtual_model_ctl::print_state() {
    switch(hop_state_) {
        default:
        case hop_state::stop_s:      ImGui::Text("Stop");      break;
        case hop_state::loading_s:   ImGui::Text("Loading");   break;
        case hop_state::unloading_s: ImGui::Text("Unloading"); break;
        case hop_state::flight_s:    ImGui::Text("Flight");    break;
    }
}