#include "proximity_guard.h"

ProximityGuard::ProximityGuard()
    : state_(State::kSafe),
      last_transition_ms_(0),
      pending_state_(State::kSafe),
      pending_since_ms_(0),
      caution_enter_mm_(0.0),
      caution_exit_mm_(0.0),
      danger_enter_mm_(0.0),
      danger_exit_mm_(0.0),
      min_dwell_ms_(0) {}

void ProximityGuard::Configure(double caution_enter_mm, double caution_exit_mm,
                               double danger_enter_mm, double danger_exit_mm,
                               long min_dwell_ms) {
  caution_enter_mm_ = caution_enter_mm;
  caution_exit_mm_ = caution_exit_mm;
  danger_enter_mm_ = danger_enter_mm;
  danger_exit_mm_ = danger_exit_mm;
  min_dwell_ms_ = min_dwell_ms;
}

ProximityGuard::State ProximityGuard::Update(double distance_mm, long now_ms) {
  // 현재 상태에 따라 enter/exit 중 어느 임계값을 쓸지 갈린다 — 이미 그 구역
  // 안에 있으면 더 넓은(exit) 문턱으로 버티고, 밖에 있으면 더 좁은(enter)
  // 문턱을 넘어야 들어간다. 그래서 경계값 근처에서 매 프레임 왔다갔다하지 않는다.
  State candidate;
  const bool in_danger = (state_ == State::kDanger);
  const bool in_caution_or_worse = (state_ != State::kSafe);

  if (distance_mm <= danger_enter_mm_ ||
      (in_danger && distance_mm <= danger_exit_mm_)) {
    candidate = State::kDanger;
  } else if (distance_mm <= caution_enter_mm_ ||
            (in_caution_or_worse && distance_mm <= caution_exit_mm_)) {
    candidate = State::kCaution;
  } else {
    candidate = State::kSafe;
  }

  if (candidate != pending_state_) {
    pending_state_ = candidate;
    pending_since_ms_ = now_ms;
  }
  if (candidate != state_ && now_ms - pending_since_ms_ >= min_dwell_ms_) {
    state_ = candidate;
    last_transition_ms_ = now_ms;
  }
  return state_;
}
