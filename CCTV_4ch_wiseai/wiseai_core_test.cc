#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "proximity_guard.h"
#include "wiseai_metadata.h"

namespace {

void Require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << std::endl;
  std::exit(1);
}

void TestDetectionMetadata() {
  const std::string xml =
      "<tt:MetadataStream><tt:VideoAnalytics>"
      "<tt:Frame UtcTime=\"2026-08-18T06:01:16.678Z\">"
      "<tt:Object ObjectId=\"1077\"><tt:Appearance><tt:Shape>"
      "<tt:BoundingBox left=\"0.0\" top=\"395.0\" right=\"83.0\" bottom=\"998.0\"/>"
      "</tt:Shape><tt:Class><tt:Type Likelihood=\"0.48\">Human</tt:Type>"
      "</tt:Class></tt:Appearance></tt:Object>"
      "<tt:Object ObjectId=\"1078\"><tt:Appearance><tt:Shape>"
      "<tt:BoundingBox left=\"100\" top=\"200\" right=\"300\" bottom=\"600\"/>"
      "</tt:Shape><tt:Class><tt:Type Likelihood=\"0.91\">Human</tt:Type>"
      "</tt:Class></tt:Appearance></tt:Object></tt:Frame>"
      "<tt:Frame UtcTime=\"2026-08-18T06:01:17.678Z\">"
      "<tt:Object ObjectId=\"2000\"><tt:Appearance><tt:Shape>"
      "<tt:BoundingBox left=\"10\" top=\"20\" right=\"30\" bottom=\"40\"/>"
      "</tt:Shape><tt:Class><tt:Type Likelihood=\"0.75\">Vehicle</tt:Type>"
      "</tt:Class></tt:Appearance></tt:Object></tt:Frame>"
      "</tt:VideoAnalytics></tt:MetadataStream>";

  std::vector<WiseAiDetection> out;
  ParseWiseAiMetadata(xml, 2, &out);
  Require(out.size() == 3, "three objects parsed");
  Require(out[0].channel == 2 && out[0].object_id == "1077", "channel/id preserved");
  Require(out[0].class_type == "Human" && std::fabs(out[0].likelihood - 0.48f) < 1e-5f,
          "class and likelihood parsed");
  Require(std::fabs(out[0].foot_u - 41.5f) < 1e-5f && out[0].foot_v == 998.0f,
          "foot point is bbox bottom centre");
  Require(out[0].utc_ms != 0 && out[0].utc_ms == out[1].utc_ms,
          "objects in one frame share capture time");
  Require(out[2].utc_ms - out[1].utc_ms == 1000, "separate frame time parsed");

  std::vector<WiseAiDetection> malformed;
  ParseWiseAiMetadata(
      "<tt:Frame><tt:Object><tt:BoundingBox left=\"1\" top=\"2\" right=\"3\"/>"
      "</tt:Object></tt:Frame>",
      0, &malformed);
  Require(malformed.empty(), "incomplete bbox rejected");
}

void TestIvaEventMetadata() {
  const std::string xml =
      "<tt:Event><wsnt:NotificationMessage>"
      "<wsnt:Topic>tns1:OpenApp/WiseAI/IvaArea</wsnt:Topic>"
      "<wsnt:Message><tt:Message UtcTime=\"2026-08-19T00:53:19.184Z\">"
      "<tt:Source><tt:SimpleItem Name=\"RuleName\" Value=\"name1\"/></tt:Source>"
      "<tt:Data><tt:SimpleItem Name=\"State\" Value=\"true\"/>"
      "<tt:SimpleItem Name=\"ObjectId\" Value=\"783\"/>"
      "<tt:SimpleItem Name=\"Action\" Value=\"Exit\"/></tt:Data>"
      "</tt:Message></wsnt:Message></wsnt:NotificationMessage></tt:Event>";

  std::vector<WiseAiIvaAreaEvent> out;
  ParseWiseAiIvaAreaEvents(xml, &out);
  Require(out.size() == 1, "IVA event parsed");
  Require(out[0].rule_name == "name1" && out[0].object_id == "783", "IVA ids parsed");
  Require(out[0].action == "Exit" && out[0].state && out[0].utc_ms != 0,
          "IVA action/state/time parsed");
}

void TestProximityGuard() {
  ProximityGuard guard;
  guard.Configure(1500.0, 1800.0, 700.0, 900.0, 300);

  Require(guard.Update(1400.0, 0) == ProximityGuard::State::kSafe,
          "caution waits for dwell");
  Require(guard.Update(1400.0, 299) == ProximityGuard::State::kSafe,
          "caution still waiting");
  Require(guard.Update(1400.0, 300) == ProximityGuard::State::kCaution,
          "caution enters after dwell");
  Require(guard.Update(600.0, 400) == ProximityGuard::State::kCaution,
          "danger waits for dwell");
  Require(guard.Update(600.0, 700) == ProximityGuard::State::kDanger,
          "danger enters after dwell");
  Require(guard.Update(800.0, 800) == ProximityGuard::State::kDanger,
          "danger exit hysteresis holds");
  Require(guard.Update(1000.0, 900) == ProximityGuard::State::kDanger,
          "danger exit waits for dwell");
  Require(guard.Update(1000.0, 1200) == ProximityGuard::State::kCaution,
          "danger exits to caution");
  Require(guard.Hold() == ProximityGuard::State::kCaution,
          "missing measurement preserves state");
}

}  // namespace

int main() {
  TestDetectionMetadata();
  TestIvaEventMetadata();
  TestProximityGuard();
  std::cout << "wiseai_core_test: PASS" << std::endl;
  return 0;
}
