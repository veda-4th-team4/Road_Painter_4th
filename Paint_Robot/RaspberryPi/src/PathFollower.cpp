#include "PathFollower.h"
#include <algorithm>
#include <cmath>
#include <iostream>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

PathFollower::PathFollower()
    : current_waypoint_idx(0), drift_offset_deg(0.0f), is_turning(false),
      turn_target_angle_deg(0.0f), turn_target_steps(0),
      turn_start_left_steps(0), turn_start_right_steps(0),
      is_moving_straight(false), move_target_steps(0), move_start_left_steps(0),
      move_start_right_steps(0), wheel_diameter_m(0.066f), // 66mm wheels
      wheelbase_m(0.166f),                                 // 166mm wheel track
      gear_ratio(1.0f),                                    // Direct drive
      steps_per_rev(3200) // 200 full steps * 16 microsteps = 3200
{}

PathFollower::~PathFollower() {}

void PathFollower::SetPath(const std::vector<Segment_t> &new_path) {
  path = new_path;
  current_waypoint_idx = 0;
  drift_offset_deg = 0.0f;
  is_turning = false;
  is_moving_straight = false;
  is_arc = false;
  std::cout << GetTimestampStr() << "[PathFollower] Path loaded with " << path.size() << " segments."
            << std::endl;
}

bool PathFollower::GetCurrentSegment(Segment_t &out_seg) const {
  if (path.empty() || current_waypoint_idx >= path.size())
    return false;
  out_seg = path[current_waypoint_idx];
  return true;
}

void PathFollower::AdvanceSegment() {
  if (current_waypoint_idx < path.size()) {
    current_waypoint_idx++;
    drift_offset_deg = 0.0f; // Reset drift offset on segment change
    is_turning = false;
    is_moving_straight = false;
    std::cout << "[PathFollower] Advanced to segment index: "
              << current_waypoint_idx << std::endl;
  }
}

bool PathFollower::IsPathFinished() const {
  return path.empty() || current_waypoint_idx >= path.size();
}

void PathFollower::SetDriftOffset(float offset_deg) {
  drift_offset_deg = offset_deg;
}

uint32_t PathFollower::CalculateTurnSteps(float angle_deg) const {
  // Turn angle with 1.050f hardware calibration factor (adjusted from 1.053f to fix 0.3-deg over-turn)
  float turn_circ = M_PI * wheelbase_m;
  float arc_len = turn_circ * (std::fabs(angle_deg) * 1.050f / 360.0f);
  float wheel_circ = M_PI * wheel_diameter_m;
  float steps = (arc_len / wheel_circ) * steps_per_rev * gear_ratio;
  return static_cast<uint32_t>(std::round(steps));
}

void PathFollower::StartTurn(float angle_deg, int32_t start_left_steps,
                             int32_t start_right_steps, float start_imu_yaw) {
  is_turning = true;
  turn_target_angle_deg = angle_deg;
  turn_target_steps = CalculateTurnSteps(angle_deg);
  turn_start_left_steps = start_left_steps;
  turn_start_right_steps = start_right_steps;
  turn_start_imu_yaw = start_imu_yaw;
  std::cout << "[PathFollower] StartTurn: target angle=" << angle_deg
            << " deg | target_steps=" << turn_target_steps 
            << " | start IMU Yaw=" << start_imu_yaw << " deg" << std::endl;
}

bool PathFollower::UpdateTurn(int32_t cur_left_steps, int32_t cur_right_steps,
                              Msg_SetSpeed_t &out_speed, float cur_imu_yaw, bool has_imu) {
  if (!is_turning)
    return true;

  int32_t diff_left = cur_left_steps - turn_start_left_steps;
  int32_t diff_right = cur_right_steps - turn_start_right_steps;
  uint32_t delta_left = static_cast<uint32_t>(std::abs(diff_left));
  uint32_t delta_right = static_cast<uint32_t>(std::abs(diff_right));
  uint32_t progress_steps = (delta_left + delta_right) / 2;

  bool turn_finished = false;

  if (has_imu) {
    // IMU Closed-loop feedback termination
    float target_yaw = turn_start_imu_yaw + turn_target_angle_deg;
    
    // Fix: Snap large structural turns (90, 180, 270) to the absolute 90-degree grid 
    // to eliminate cumulative drift from previous straight moves.
    if (std::fabs(turn_target_angle_deg) >= 45.0f) {
        target_yaw = std::round(target_yaw / 90.0f) * 90.0f;
    }

    float yaw_error = std::fabs(cur_imu_yaw - target_yaw);

    // Check if IMU reached within 0.25 deg of target angle OR step guard safety limit (1.35x target steps)
    if (yaw_error <= 0.25f || progress_steps >= static_cast<uint32_t>(turn_target_steps * 1.35f)) {
      turn_finished = true;
      std::cout << "[PathFollower] Turn IMU Closed-loop finished! Target Yaw: " << target_yaw
                << " deg | Final IMU Yaw: " << cur_imu_yaw << " deg (Error: " 
                << (cur_imu_yaw - target_yaw) << " deg) | Steps: " << progress_steps << "/" << turn_target_steps << std::endl;
    }
  } else {
    // Step-odometry fallback termination (when IMU is offline)
    if (progress_steps >= turn_target_steps) {
      turn_finished = true;
      std::cout << "[PathFollower] Turn Step-Fallback finished! Reached " << progress_steps
                << "/" << turn_target_steps << " steps." << std::endl;
    }
  }

  if (turn_finished) {
    out_speed.left_sps = 0;
    out_speed.right_sps = 0;
    is_turning = false;
    return true;
  }

  // Turn in progress
  float base_turn_sps = 400.0f; // Raised to 400 SPS for large angles
  if (std::fabs(turn_target_angle_deg) <= 10.0f) {
    base_turn_sps = 200.0f; // Gentle turn speed for ALIGN micro-turns
  }

  // 1-Step Deceleration logic for Turn
  uint32_t remaining_steps = (turn_target_steps > progress_steps) ? (turn_target_steps - progress_steps) : 0;
  uint32_t decel_threshold = (turn_target_steps * 8) / 100; // 8% remaining threshold
  
  float current_turn_sps = base_turn_sps;

  // Apply deceleration only if it's a large turn where base speed is high
  if (base_turn_sps > 250.0f && decel_threshold > 0 && remaining_steps <= decel_threshold) {
    current_turn_sps = 250.0f; // 1-step floor turn speed at 250 SPS
  }

  int16_t turn_sps = static_cast<int16_t>(current_turn_sps);

  if (turn_target_angle_deg > 0.0f) {
    // Left turn: Left wheel backward (-sps), Right wheel forward (+sps)
    out_speed.left_sps = -turn_sps;
    out_speed.right_sps = turn_sps;
  } else {
    // Right turn: Left wheel forward (+sps), Right wheel backward (-sps)
    out_speed.left_sps = turn_sps;
    out_speed.right_sps = -turn_sps;
  }

  return false;
}

bool PathFollower::TrimTurn(float target_yaw, float cur_imu_yaw, Msg_SetSpeed_t &out_speed) {
  float residual_error = cur_imu_yaw - target_yaw;

  if (std::fabs(residual_error) <= 0.15f) {
    out_speed.left_sps = 0;
    out_speed.right_sps = 0;
    return true; // Micro-trim complete!
  }

  // Ultra-slow micro pulse speed (raised to 350 sps to prevent resonance)
  int16_t trim_sps = 350;
  if (cur_imu_yaw < target_yaw) {
    // cur_imu_yaw < target_yaw (e.g. -90.70 deg < -90.00 deg) -> needs positive rotation back towards -90.00 deg
    out_speed.left_sps = -trim_sps;
    out_speed.right_sps = trim_sps;
  } else {
    // cur_imu_yaw > target_yaw (e.g. -89.30 deg > -90.00 deg) -> needs negative rotation further towards -90.00 deg
    out_speed.left_sps = trim_sps;
    out_speed.right_sps = -trim_sps;
  }
  return false;
}

uint32_t PathFollower::CalculateMoveSteps(float dist_m) const {
  // Straight move distance with 1.040f hardware calibration factor (updated for 1.195m -> 1.2m correction)
  float abs_dist = std::fabs(dist_m) * 1.040f;
  float steps = abs_dist * 15433.09f;
  return static_cast<uint32_t>(std::round(steps));
}

void PathFollower::StartMove(float dist_m, int32_t start_left_steps,
                             int32_t start_right_steps, float target_v_m_s) {
  is_moving_straight = true;
  move_dist_m = dist_m;
  move_speed_m_s = target_v_m_s;
  move_target_steps = CalculateMoveSteps(dist_m);
  move_start_left_steps = start_left_steps;
  move_start_right_steps = start_right_steps;
  std::cout << GetTimestampStr() << "[PathFollower] StartMove: target dist=" << dist_m
            << " m | speed=" << target_v_m_s << " m/s | target_steps=" << move_target_steps << std::endl;
}

bool PathFollower::UpdateMove(int32_t cur_left_steps, int32_t cur_right_steps,
                              Msg_SetSpeed_t &out_speed,
                              float imu_yaw_deg) {
  if (!is_moving_straight)
    return true;

  int32_t diff_left = cur_left_steps - move_start_left_steps;
  int32_t diff_right = cur_right_steps - move_start_right_steps;
  uint32_t delta_left = static_cast<uint32_t>(std::abs(diff_left));
  uint32_t delta_right = static_cast<uint32_t>(std::abs(diff_right));
  uint32_t progress_steps = (delta_left + delta_right) / 2;

  if (progress_steps >= move_target_steps) {
    // Move target reached
    out_speed.left_sps = 0;
    out_speed.right_sps = 0;
    is_moving_straight = false;
    std::cout << GetTimestampStr() << "[PathFollower] Move completed! Reached " << progress_steps
              << "/" << move_target_steps << " steps." << std::endl;
    return true;
  }

  // Straight move velocity (positive dist_m -> +move_speed_m_s forward, negative dist_m -> -move_speed_m_s reverse)
  float target_v = (move_dist_m >= 0.0f) ? move_speed_m_s : -move_speed_m_s;

  // 1-Step Deceleration: When 8% or less of target steps remains, step down speed instantly
  uint32_t remaining_steps = move_target_steps - progress_steps;
  uint32_t decel_threshold = (move_target_steps * 8) / 100; // 8% remaining threshold
  if (decel_threshold > 0 && remaining_steps <= decel_threshold) {
    float base_sps = (std::abs(move_speed_m_s) / (M_PI * wheel_diameter_m)) * steps_per_rev * gear_ratio;
    float min_factor = 300.0f / (base_sps > 0.1f ? base_sps : 0.1f); // Raised to 300 SPS floor speed
    if (min_factor > 1.0f) min_factor = 1.0f; // Do not exceed max speed

    target_v *= min_factor; // Drop instantly to minimum speed
  }

  // Protocol v2 sign convention: positive drift_offset_deg = turn right (CW) -> negative angular velocity (w)
  float drift_rad = drift_offset_deg * (3.14159265f / 180.0f);
  float target_w = -0.5f * drift_rad; // P-gain 0.5 for smooth steering correction

  out_speed = velocity_to_sps(target_v, target_w);

  static int move_log_count = 0;
  if (++move_log_count % 10 == 0) {
    std::cout << GetTimestampStr() << "[PathFollower MOVE] Progress: " << progress_steps << "/"
              << move_target_steps << " steps | IMU Yaw: " << imu_yaw_deg
              << " deg | Server Drift: " << drift_offset_deg
              << " deg -> Target SPS (L: " << out_speed.left_sps
              << ", R: " << out_speed.right_sps << ")" << std::endl;
  }

  return false;
}

void PathFollower::StartArc(float radius_m, float angle_deg,
                            const std::string &direction, int32_t start_l_steps,
                            int32_t start_r_steps) {
  is_arc = true;
  bool is_left = (direction == "left");
  arc_is_left = is_left;

  float r_robot = radius_m;
  float r_left =
      is_left ? (r_robot - wheelbase_m / 2.0f) : (r_robot + wheelbase_m / 2.0f);
  float r_right =
      is_left ? (r_robot + wheelbase_m / 2.0f) : (r_robot - wheelbase_m / 2.0f);

  float angle_rad = angle_deg * (M_PI / 180.0f);
  float pulses_per_m = (steps_per_rev * gear_ratio) / (M_PI * wheel_diameter_m);

  arc_target_l_steps =
      static_cast<uint32_t>(std::abs(r_left * angle_rad) * pulses_per_m);
  arc_target_r_steps =
      static_cast<uint32_t>(std::abs(r_right * angle_rad) * pulses_per_m);
  arc_start_l_steps = start_l_steps;
  arc_start_r_steps = start_r_steps;

  float base_sps = 771.65f; // 0.05 m/s base speed
  const float kMinArcRadius = 2e-3f; // 2mm threshold to prevent UB division overflow
  if (std::fabs(r_robot) < kMinArcRadius) {
      arc_sps_l = is_left ? static_cast<int16_t>(-base_sps) : static_cast<int16_t>(+base_sps);
      arc_sps_r = is_left ? static_cast<int16_t>(+base_sps) : static_cast<int16_t>(-base_sps);
  } else {
      arc_sps_l = static_cast<int16_t>(base_sps * (r_left / r_robot));
      arc_sps_r = static_cast<int16_t>(base_sps * (r_right / r_robot));
  }

  std::cout << GetTimestampStr() << "[PathFollower ARC] StartArc: R_paint=" << radius_m
            << "m -> R_robot=" << r_robot << "m | angle=" << angle_deg
            << " deg (" << direction << ") | SPS_L=" << arc_sps_l
            << ", SPS_R=" << arc_sps_r << std::endl;
}

bool PathFollower::UpdateArc(int32_t cur_l_steps, int32_t cur_r_steps,
                             Msg_SetSpeed_t &out_speed) {
  if (!is_arc)
    return true;

  int32_t dl = std::abs(cur_l_steps - arc_start_l_steps);
  int32_t dr = std::abs(cur_r_steps - arc_start_r_steps);

  uint32_t outer_progress = arc_is_left ? static_cast<uint32_t>(dr) : static_cast<uint32_t>(dl);
  uint32_t outer_target = arc_is_left ? arc_target_r_steps : arc_target_l_steps;

  // Outer wheel completion evaluation: Prevents premature termination when r_in ~ 0
  if (outer_progress >= outer_target) {
    out_speed.left_sps = 0;
    out_speed.right_sps = 0;
    is_arc = false;
    std::cout << GetTimestampStr() << "[PathFollower ARC] Arc motion completed!" << std::endl;
    return true;
  }

  // 1-Step Deceleration: When 8% or less of target steps remains, step down speed instantly
  uint32_t remaining_steps = outer_target - outer_progress;
  uint32_t decel_threshold = (outer_target * 8) / 100; // 8% remaining threshold
  float speed_factor = 1.0f;

  if (decel_threshold > 0 && remaining_steps <= decel_threshold) {
    float base_sps = std::max(std::abs((float)arc_sps_l), std::abs((float)arc_sps_r));
    float min_factor = 300.0f / (base_sps > 0.1f ? base_sps : 0.1f); // Raised to 300 SPS floor speed
    if (min_factor > 1.0f) min_factor = 1.0f; // Do not exceed max speed

    speed_factor = min_factor; // Drop instantly to minimum speed
  }

  out_speed.left_sps = static_cast<int16_t>(arc_sps_l * speed_factor);
  out_speed.right_sps = static_cast<int16_t>(arc_sps_r * speed_factor);
  return false;
}



Msg_SetSpeed_t PathFollower::velocity_to_sps(float v, float w) {
  Msg_SetSpeed_t speed_cmd;

  // Differential drive kinematics
  float v_left = v - (w * wheelbase_m) / 2.0f;
  float v_right = v + (w * wheelbase_m) / 2.0f;

  // Wheel circumference
  float wheel_circ = M_PI * wheel_diameter_m;

  // Convert linear speed to steps-per-second (sps)
  float left_rps = v_left / wheel_circ;
  float right_rps = v_right / wheel_circ;

  speed_cmd.left_sps =
      static_cast<int16_t>(left_rps * steps_per_rev * gear_ratio);
  speed_cmd.right_sps =
      static_cast<int16_t>(right_rps * steps_per_rev * gear_ratio);

  return speed_cmd;
}
