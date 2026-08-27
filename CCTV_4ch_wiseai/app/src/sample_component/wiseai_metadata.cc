#include "wiseai_metadata.h"

#include <cstdio>   // sscanf -- ParseOnvifUtcMs
#include <cstdlib>
#include <cstring>

namespace {

// Finds `attr` (e.g. "left=") in `s` and reads the number inside the quotes
// that follow. Returns false if the attribute isn't present at all --
// callers use this to bail out of an object that doesn't have every field
// they need rather than silently reporting an object at (0,0).
bool ExtractFloatAttr(const std::string& s, const char* attr, float* out) {
  size_t p = s.find(attr);
  if (p == std::string::npos) return false;
  p = s.find('"', p);
  if (p == std::string::npos) return false;
  *out = strtof(s.c_str() + p + 1, nullptr);
  return true;
}

// Same shape as ExtractFloatAttr, for a quoted string attribute instead of a
// number (WiseAI's ObjectId looks numeric in every sample seen so far, but
// nothing says it always will -- kept as a string rather than parsed with
// strtol so a future non-numeric id round-trips instead of becoming 0).
std::string ExtractStringAttr(const std::string& s, const char* attr) {
  size_t p = s.find(attr);
  if (p == std::string::npos) return "";
  p = s.find('"', p);
  if (p == std::string::npos) return "";
  const size_t end = s.find('"', p + 1);
  if (end == std::string::npos) return "";
  return s.substr(p + 1, end - p - 1);
}

// ONVIF SimpleItem lookup: finds Name="`name`" within `s`, then reads the
// Value="..." that follows it. "" if the item isn't present at all --
// callers treat a missing field as "didn't say", not as an empty string
// that was deliberately sent.
std::string ExtractSimpleItemValue(const std::string& s, const char* name) {
  const std::string needle = std::string("Name=\"") + name + "\"";
  size_t p = s.find(needle);
  if (p == std::string::npos) return "";
  p = s.find("Value=\"", p);
  if (p == std::string::npos) return "";
  p += 7;  // strlen("Value=\"")
  const size_t end = s.find('"', p);
  if (end == std::string::npos) return "";
  return s.substr(p, end - p);
}

// Days from 1970-01-01 to y-m-d (proleptic Gregorian), by Howard Hinnant's
// days_from_civil. Written out instead of calling into libc on purpose:
//
//   - mktime() is WRONG here. It reads the fields as LOCAL time, and this
//     camera runs on KST -- every timestamp would land 9 hours off, silently
//     and consistently enough to look like a real clock skew.
//   - timegm() is the correct call but is a GNU extension, and nothing
//     guarantees it in this cross toolchain's libc.
//
// The input is already UTC (ONVIF stamps these with a trailing Z), so there
// is no zone question to answer -- only arithmetic, which is worth 8 lines to
// own outright.
long DaysFromCivil(long y, unsigned m, unsigned d) {
  y -= (m <= 2);
  const long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);              // [0, 399]
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;  // [0, 365]
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;  // [0, 146096]
  return era * 146097L + (long)doe - 719468L;
}

// "2026-08-19T07:30:52.632Z" -> epoch milliseconds. 0 if `s` doesn't look
// like that at all, which callers treat as "this frame didn't say" rather
// than as the epoch itself. Fractional seconds are optional -- ONVIF permits
// omitting them, and a whole-second stamp is still worth having.
long ParseOnvifUtcMs(const std::string& s) {
  if (s.empty()) return 0;
  int Y = 0, M = 0, D = 0, h = 0, mi = 0, sec = 0;
  int consumed = 0;
  if (sscanf(s.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d%n",
             &Y, &M, &D, &h, &mi, &sec, &consumed) != 6) {
    return 0;
  }
  if (M < 1 || M > 12 || D < 1 || D > 31) return 0;
  long ms = 0;
  if (s.size() > (size_t)consumed && s[consumed] == '.') {
    // Take exactly 3 digits, padding a shorter fraction rather than
    // misreading ".6" as 6ms when it means 600ms.
    for (int i = 0; i < 3; ++i) {
      const size_t p = (size_t)consumed + 1 + (size_t)i;
      const bool have = p < s.size() && s[p] >= '0' && s[p] <= '9';
      ms = ms * 10 + (have ? (s[p] - '0') : 0);
    }
  }
  const long days = DaysFromCivil(Y, (unsigned)M, (unsigned)D);
  return ((days * 24L + h) * 60L + mi) * 60L * 1000L + (long)sec * 1000L + ms;
}

}  // namespace

void ParseWiseAiMetadata(const std::string& xml, int channel,
                          std::vector<WiseAiDetection>* out) {
  size_t pos = 0;
  size_t cached_frame_pos = std::string::npos;
  long cached_frame_ms = 0;
  while (true) {
    size_t obj_start = xml.find("<tt:Object", pos);
    if (obj_start == std::string::npos) break;
    size_t obj_end = xml.find("<tt:Object", obj_start + 1);
    if (obj_end == std::string::npos) obj_end = xml.find("</tt:Frame>", obj_start);
    if (obj_end == std::string::npos) obj_end = xml.size();
    const std::string obj = xml.substr(obj_start, obj_end - obj_start);
    pos = obj_end;

    const size_t bbox_pos = obj.find("<tt:BoundingBox");
    if (bbox_pos == std::string::npos) continue;
    const size_t bbox_tag_end = obj.find('>', bbox_pos);
    const std::string bbox_tag =
        obj.substr(bbox_pos, bbox_tag_end == std::string::npos
                                  ? std::string::npos
                                  : bbox_tag_end - bbox_pos);

    // <tt:Object ObjectId="..."> is the opening tag `obj` itself starts
    // with (obj_start is where "<tt:Object" begins), so its own first '>'
    // bounds it -- same pattern as bbox_tag just above, one level out.
    const size_t obj_tag_end = obj.find('>');
    const std::string obj_tag =
        obj.substr(0, obj_tag_end == std::string::npos ? std::string::npos : obj_tag_end);

    // The frame timestamp lives on the <tt:Frame> that ENCLOSES this object,
    // one level out from the loop, so walk back to the nearest one before
    // this object rather than reading the first in the document -- a single
    // callback is one frame in every sample seen so far, but nothing in the
    // schema says a payload cannot carry several, and reading frame 1's time
    // for frame 2's objects would be wrong in a way nothing downstream could
    // detect.
    long frame_ms = 0;
    const size_t frame_pos = xml.rfind("<tt:Frame", obj_start);
    if (frame_pos != std::string::npos) {
      // A frame can contain several objects. Parse its timestamp once and
      // reuse it for every object in that frame instead of allocating and
      // reparsing the same opening tag N times.
      if (frame_pos != cached_frame_pos) {
        cached_frame_pos = frame_pos;
        cached_frame_ms = 0;
        const size_t frame_tag_end = xml.find('>', frame_pos);
        if (frame_tag_end != std::string::npos) {
          cached_frame_ms = ParseOnvifUtcMs(
              ExtractStringAttr(xml.substr(frame_pos, frame_tag_end - frame_pos),
                                "UtcTime="));
        }
      }
      frame_ms = cached_frame_ms;
    }

    WiseAiDetection d;
    d.channel = channel;
    d.utc_ms = frame_ms;
    d.object_id = ExtractStringAttr(obj_tag, "ObjectId=");
    const bool ok = ExtractFloatAttr(bbox_tag, "left=", &d.left) &&
                     ExtractFloatAttr(bbox_tag, "top=", &d.top) &&
                     ExtractFloatAttr(bbox_tag, "right=", &d.right) &&
                     ExtractFloatAttr(bbox_tag, "bottom=", &d.bottom);
    if (!ok) continue;

    // The outer <tt:Type Likelihood="...">Name</tt:Type> -- not the
    // <tt:ClassCandidate><tt:Type>Name</tt:Type> one, which has no
    // Likelihood attribute and is one of possibly several candidates.
    const size_t type_pos = obj.find("<tt:Type Likelihood=");
    if (type_pos != std::string::npos) {
      ExtractFloatAttr(obj.substr(type_pos), "Likelihood=", &d.likelihood);
      const size_t name_start = obj.find('>', type_pos);
      const size_t name_end =
          name_start == std::string::npos ? std::string::npos
                                           : obj.find("</tt:Type>", name_start);
      if (name_start != std::string::npos && name_end != std::string::npos) {
        d.class_type = obj.substr(name_start + 1, name_end - name_start - 1);
      }
    }

    d.foot_u = (d.left + d.right) * 0.5f;
    d.foot_v = d.bottom;
    out->push_back(d);
  }
}

void ParseWiseAiIvaAreaEvents(const std::string& xml,
                              std::vector<WiseAiIvaAreaEvent>* out) {
  size_t pos = 0;
  while (true) {
    const size_t ev_start = xml.find("<tt:Event", pos);
    if (ev_start == std::string::npos) break;
    size_t ev_end = xml.find("</tt:Event>", ev_start);
    if (ev_end == std::string::npos) { pos = ev_start + 1; continue; }
    ev_end += 11;  // strlen("</tt:Event>")
    const std::string block = xml.substr(ev_start, ev_end - ev_start);
    pos = ev_end;

    const size_t topic_pos = block.find("<wsnt:Topic");
    if (topic_pos == std::string::npos) continue;
    const size_t topic_end = block.find("</wsnt:Topic>", topic_pos);
    if (topic_end == std::string::npos) continue;
    if (block.substr(topic_pos, topic_end - topic_pos).find("IvaArea") ==
        std::string::npos) {
      continue;  // some other rule's event (e.g. a future ObjectDetection one)
    }

    WiseAiIvaAreaEvent e;
    // First UtcTime in the block is <tt:Message>'s -- the only one an event
    // carries (unlike a VideoAnalytics frame, there is no enclosing tag with
    // a competing timestamp here).
    e.utc_ms = ParseOnvifUtcMs(ExtractStringAttr(block, "UtcTime="));
    e.rule_name = ExtractSimpleItemValue(block, "RuleName");
    e.object_id = ExtractSimpleItemValue(block, "ObjectId");
    e.action = ExtractSimpleItemValue(block, "Action");
    const std::string state_str = ExtractSimpleItemValue(block, "State");
    e.state = (state_str == "true" || state_str == "1" || state_str == "True");
    out->push_back(e);
  }
}
