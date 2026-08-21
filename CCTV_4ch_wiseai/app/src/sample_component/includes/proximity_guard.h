#ifndef PROXIMITY_GUARD_H
#define PROXIMITY_GUARD_H

/**
 * 사람-로봇 근접 상태머신 (안전 -> 주의 -> 위험 -> 해제).
 *
 * 입력은 이미 계산된 거리(mm)뿐이다 — bbox 파싱이나 호모그래피와는 무관하게 독립적으로
 * 짜고 검증할 수 있게 분리했다 (WISEAI_METADATA_INTEGRATION_DESIGN.md §4).
 *
 * enter/exit 임계값을 다르게 둔 히스테리시스 + min_dwell_ms 동안 후보 상태가
 * 유지돼야 실제로 전환하는 디바운스, 두 가지로 경계에서의 떨림을 막는다.
 * marker_hold_ms_(SampleComponent)와 같은 발상 — 순간값 하나로 바로 반응하지 않는다.
 */
class ProximityGuard {
 public:
  enum class State { kSafe, kCaution, kDanger };

  ProximityGuard();

  // enter < exit 여야 정상적인 히스테리시스가 된다 (틀리면 Update()가 그대로
  // 받아들이되 두 값이 뒤집힌 만큼 이상하게 동작한다 — 호출자 책임).
  void Configure(double caution_enter_mm, double caution_exit_mm,
                 double danger_enter_mm, double danger_exit_mm,
                 long min_dwell_ms);

  // 이번 사이클에 유효한 거리값이 있을 때 호출. 새 상태를 반환.
  State Update(double distance_mm, long now_ms);

  // 이번 사이클에 유효한 거리값이 없을 때(검출 실패, 신뢰도 미달, 지평선 밖 등)
  // 호출. 상태를 그대로 유지한다 — 끊긴 검출을 "안전"으로 오판하지 않기 위함.
  State Hold() const { return state_; }

  State state() const { return state_; }
  long last_transition_ms() const { return last_transition_ms_; }

 private:
  State state_;
  long last_transition_ms_;

  State pending_state_;
  long pending_since_ms_;

  double caution_enter_mm_;
  double caution_exit_mm_;
  double danger_enter_mm_;
  double danger_exit_mm_;
  long min_dwell_ms_;
};

#endif  // PROXIMITY_GUARD_H
