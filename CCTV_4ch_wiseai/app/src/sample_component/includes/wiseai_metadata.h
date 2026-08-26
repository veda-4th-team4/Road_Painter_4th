#pragma once

#include <string>
#include <vector>

// Parses the ONVIF metadata XML that WiseAI's MetadataManager delivers per
// channel (see docs/08.18/2026-08-18_WISEAI_METADATA_SAMPLE.md for the format
// this was written against: pixel bbox, (left,top,right,bottom), ISO 8601 UTC
// frame timestamp). Deliberately a lightweight substring scan rather than a
// real XML parser -- this project pulls in no XML library anywhere else, and
// the fields this needs are a handful of numeric attributes at fixed tag
// names, not a general document.

struct WiseAiDetection {
  int channel = -1;
  std::string object_id;    // <tt:Object ObjectId="..."> -- WiseAI's per-track id.
                             // Shared with WiseAiIvaAreaEvent::object_id below:
                             // an Enter/Exit event names the same id the bbox
                             // stream used for that object, which is how the
                             // two get matched up (see the recent-bbox cache
                             // in sample_component.h/.cc).
  std::string class_type;   // "Human", "Face", "Head", ... (outer <tt:Type Likelihood=...>)
  float likelihood = 0.0f;
  float left = 0.0f, top = 0.0f, right = 0.0f, bottom = 0.0f;  // pixels
  float foot_u = 0.0f, foot_v = 0.0f;  // (left+right)/2, bottom -- NOT CenterOfGravity
  // <tt:Frame UtcTime="..."> of the frame this object was seen in, as epoch
  // milliseconds. 0 = the frame carried no parseable timestamp.
  //
  // This is when the CAMERA SAW it, which is the whole point of carrying it:
  // the moment we handle the callback is later by an unknown decode/queue
  // delay, and the moment the Pi receives the resulting event is later again
  // by the link (the control line has a retry queue, so that gap is not even
  // bounded). Only this value can be lined up against camera recordings, or
  // against a CAM_POSE sample, to ask what else was true at that instant.
  long utc_ms = 0;
};

// Appends one WiseAiDetection per <tt:Object> in `xml` that has a
// <tt:BoundingBox>. Frames with no objects (nobody in view) add nothing.
// `channel` is stamped from the caller (MetadataOutput::channel()), not
// re-derived from the XML.
void ParseWiseAiMetadata(const std::string& xml, int channel,
                          std::vector<WiseAiDetection>* out);

// One <tt:Event> whose Topic names IvaArea -- a WiseAI IVA-area rule firing
// (object entered/exited/intruded the polygon set via PUT
// /configuration/ivaarea). The shape below was confirmed against live
// Enter/Exit captures on 2026-08-19; see the parser declaration below.
struct WiseAiIvaAreaEvent {
  std::string rule_name;    // <tt:Source> SimpleItem "RuleName" -- matches
                             // definedArea[].name from the PUT that set it up
  std::string object_id;    // <tt:Data> SimpleItem "ObjectId"
  std::string action;       // <tt:Data> SimpleItem "Action" -- live-confirmed
                             // "Enter"/"Exit" (other configured modes pass through)
  bool state = false;       // <tt:Data> SimpleItem "State"
  // <tt:Message UtcTime="..."> -- when WiseAI decided the rule fired, epoch
  // milliseconds, 0 if absent. Same clock as WiseAiDetection::utc_ms, which
  // is what makes the two comparable: WiseAI judges with the bbox centre and
  // we judge with the foot point (see iva_zone_ in sample_component.h), and
  // "how far apart do the two verdicts land" is only answerable if both
  // carry a time from the same source.
  long utc_ms = 0;
};

// Appends one WiseAiIvaAreaEvent per <tt:Event> in `xml` whose
// <wsnt:Topic>...</wsnt:Topic> contains "IvaArea". Frames with no rule
// firing (nothing crossed the polygon) add nothing.
//
// Confirmed live 2026-08-19 (ch0/index0, 10-minute walk-through: 26 Enter +
// 33 Exit, all parsed correctly) -- field names and the "IvaArea" topic
// substring check both matched real captures. Written originally against
// WiseAI.html's documented ONVIF event schema alone, before that capture.
void ParseWiseAiIvaAreaEvents(const std::string& xml,
                              std::vector<WiseAiIvaAreaEvent>* out);
