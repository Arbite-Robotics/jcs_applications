// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include "hopper_2d_simple_kine.h"
#include "hopper_2d_kinematics.h"

#include "jcs_host.h"

hopper_2d_simple_kine::hopper_2d_simple_kine() :
    hopper_2d("Simple kinematics"),
    // Some default dt
    sine_x_(1.0 / 1000.0),
    sine_y_(1.0 / 1000.0)
{
    // Negative limits will make sure we dont hit singularity at th=0
    // j0_limits_[0] = 0.7853;
    j0_limits_[0] = -0.05;
    j0_limits_[1] = -0.85;
    j1_limits_[0] = 0.85;
    j1_limits_[1] = 0.05;

    x_tip_.x = 0.0;
    x_tip_.y = 0.14121;

    j0_gain_[0] = 20.0;
    j0_gain_[1] = 0.125;
    j1_gain_[0] = 20.0;
    j1_gain_[1] = 0.125;

    outputs_active_ = false;
}

int hopper_2d_simple_kine::startup(double sample_rate_hz) {
    sine_x_.dt = 1.0 / sample_rate_hz;
    sine_y_.dt = 1.0 / sample_rate_hz;

    return jcs::RET_OK;
}

bool hopper_2d_simple_kine::can_start() {
    // Check the signal sizes are ok for what we need
    if (f32_ctl_osig_->size() < 6) { return false; }
    if (f32_ctl_isig_->size() < 2) { return false; }
    return true;
}

int hopper_2d_simple_kine::render() {
    const double default_step_val = 0.1;

    ImGui::PushID(this);

    ImGui::Text("Simple parallel 2D robot inverse kinematics");
    ImGui::Text("Use configuration: 04_cnf_ik_position_ctl");

    ImGui::Text("XY tip position");
    const double y_pos_min = 0.01;
    const double pos_max = hopper_2d_kinematics::parameter_l1 + hopper_2d_kinematics::parameter_l2 - 0.005; 
    const double x_pos_min = -pos_max; 
    ImGui::SliderScalar("X Position", ImGuiDataType_Double, &x_tip_.x, &x_pos_min, &pos_max);
    ImGui::SliderScalar("Y Position", ImGuiDataType_Double, &x_tip_.y, &y_pos_min, &pos_max);

    ImGui::Checkbox("Output signals active", &outputs_active_);

    ImGui::Separator();
    ImGui::Text("XY tip sine modulation");
    const double zero = 0.0;
    const double amp_max = 0.05;
    const double freq_max = 50.0;
    ImGui::SliderScalar("X sine amplitude", ImGuiDataType_Double, &sine_x_.amplitude, &zero, &amp_max);
    ImGui::SliderScalar("X sine frequency", ImGuiDataType_Double, &sine_x_.frequency, &zero, &freq_max);
    ImGui::SliderScalar("Y sine amplitude", ImGuiDataType_Double, &sine_y_.amplitude, &zero, &amp_max);
    ImGui::SliderScalar("Y sine frequency", ImGuiDataType_Double, &sine_y_.frequency, &zero, &freq_max);

    ImGui::Separator();

    static ImGuiTableFlags table_flags = ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_NoSavedSettings;

    ImGui::Text("Info");
    if (ImGui::BeginTable("Info", 3, table_flags)) {
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthStretch);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("XY Position");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", x_tip_.x);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", x_tip_.y);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("th0/1 Kinematics");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", kin_th_[0]);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", kin_th_[1]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("th0/1 Command");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", th_[0]);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", th_[1]);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("th0/1 Actual");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", f32_ctl_isig_->at(static_cast<int>(isig_map::j0_th)));
        ImGui::TableSetColumnIndex(2);
        // Recall joint 2 is backwards to joint 1
        ImGui::Text("%.6f", -f32_ctl_isig_->at(static_cast<int>(isig_map::j1_th)));

        ImGui::EndTable();
    }

    // ImGui::Separator();
    // if (ImGui::Button("Zero joint encoder 0")) {
    //     PARAM_CHECK_WHOOPS_RETOK( host_->write_command("joint_encoder_0", "encoder_position_zero"), "Parameter failed: encoder_position_zero" )
    // }
    // ImGui::SameLine();
    // if (ImGui::Button("Zero joint encoder 1")) {
    //     PARAM_CHECK_WHOOPS_RETOK( host_->write_command("joint_encoder_1", "encoder_position_zero"), "Parameter failed: encoder_position_zero" )
    // }

    ImGui::Separator();
    ImGui::Text("Joint position limits");
    ImGui::InputScalar("Joint 0 limit positive", ImGuiDataType_Double, &j0_limits_[0], &default_step_val);
    ImGui::InputScalar("Joint 0 limit negative", ImGuiDataType_Double, &j0_limits_[1], &default_step_val);
    ImGui::InputScalar("Joint 1 limit positive", ImGuiDataType_Double, &j1_limits_[0], &default_step_val);
    ImGui::InputScalar("Joint 1 limit negative", ImGuiDataType_Double, &j1_limits_[1], &default_step_val);

    ImGui::Separator();
    ImGui::Text("Joint position controller gains");
    const double p_max = 50.0;
    const double d_max = 50.0;
    ImGui::SliderScalar("J0 P gain", ImGuiDataType_Double, &j0_gain_[0], &zero, &p_max);
    ImGui::SliderScalar("J0 D gain", ImGuiDataType_Double, &j0_gain_[1], &zero, &d_max);
    ImGui::SliderScalar("J1 P gain", ImGuiDataType_Double, &j1_gain_[0], &zero, &p_max);
    ImGui::SliderScalar("J1 D gain", ImGuiDataType_Double, &j1_gain_[1], &zero, &d_max);

    ImGui::PopID();

    return jcs::RET_OK;
}

int hopper_2d_simple_kine::step_rt_init() {
    return jcs::RET_OK;
}

int hopper_2d_simple_kine::step_rt_run() {

    // Modulate the tip
    x_tip_mod_ = x_tip_ + helpers::vec2(sine_x_.step_rt(), sine_y_.step_rt());

    // Get joint positions. True for valid solution
    if (!hopper_2d_kinematics::ik_analytic(x_tip_mod_, &kin_th_)) {
        return jcs::RET_OK;
    }
    // Kinematics output has theta=0 along the x axis.
    // Robot has theta=0 along y axis
    th_ = helpers::vec2(M_PI_2, M_PI_2) - kin_th_;

    bool j0_clamped = false;
    bool j1_clamped = false;

    // Check and clamp joint limits
    if (th_[0] > j0_limits_[0]) { th_[0] = j0_limits_[0]; j0_clamped = true; }
    if (th_[0] < j0_limits_[1]) { th_[0] = j0_limits_[1]; j0_clamped = true; }

    if (th_[1] > j1_limits_[0]) { th_[1] = j1_limits_[0]; j1_clamped = true; }
    if (th_[1] < j1_limits_[1]) { th_[1] = j1_limits_[1]; j1_clamped = true; }

    // Joint 2 is backwards to joint 1
    double th1_out = -th_[1];


    // Set output joint positions
    if (outputs_active_) {
        // If one joint output is at its limit, then the resulting joint
        // theta combinations may not be valid. Do not output.
        if (!j0_clamped && !j0_clamped) {
            f32_ctl_osig_->at(static_cast<int>(osig_map::j0_th)) = static_cast<float>(th_[0]);
            f32_ctl_osig_->at(static_cast<int>(osig_map::j1_th)) = static_cast<float>(th1_out);
        }
        // Set joint gains
        f32_ctl_osig_->at(static_cast<int>(osig_map::j0_gain_p)) = static_cast<float>(j0_gain_[0]);
        f32_ctl_osig_->at(static_cast<int>(osig_map::j0_gain_d)) = static_cast<float>(j0_gain_[1]);
        f32_ctl_osig_->at(static_cast<int>(osig_map::j1_gain_p)) = static_cast<float>(j1_gain_[0]);
        f32_ctl_osig_->at(static_cast<int>(osig_map::j1_gain_d)) = static_cast<float>(j1_gain_[1]);
    }

    return jcs::RET_OK;
}

