/*
 *  Yifei Chen
 *  Motion Matching Demo - scripted self test
 *
 *  `Runtime_demo_mm --selftest [out_dir]` replays a fixed sequence of stick inputs and path
 *  edits at 60 Hz, prints how well the character follows them, and saves screenshots.
 *  Handy to check a build without a gamepad in hand.
 */
#ifndef MM_SELFTEST_H
#define MM_SELFTEST_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

namespace selftest {

// virtual controller state for one tick
struct Input {
  float mx = 0.f, my = 0.f;  // left stick, +y forward (camera relative)
  float run = 0.f;           // right trigger
  bool strafe = false;       // left trigger
  float cam_orbit = 0.f;     // right stick x
};

struct Phase {
  const char* name;
  float seconds;
  Input input;
  const char* screenshot;  // taken at the end of the phase (nullptr: none)
  float tolerance_deg = 35.f;  // facing / moving direction error allowed at the end of the phase
};

inline std::vector<Phase> ControllerScript() {
  return {
      {"idle", 2.0f, {}, "01_idle"},
      {"walk forward", 4.0f, {0.f, 1.f, 0.f, false, 0.f}, "02_walk"},
      {"turn right", 3.0f, {1.f, 0.f, 0.f, false, 0.f}, "02b_turn"},
      {"run forward", 4.0f, {0.f, 1.f, 1.f, false, 0.f}, "03_run"},
      {"run + orbit camera", 3.0f, {0.f, 1.f, 1.f, false, 0.6f}, nullptr, 75.f},  // the goal keeps rotating
      {"stop", 2.5f, {}, nullptr},
      {"strafe left", 3.0f, {-1.f, 0.f, 0.f, true, 0.f}, "04_strafe"},
      {"walk back toward camera", 3.0f, {0.f, -1.f, 0.f, false, 0.f}, nullptr},
      {"half stick", 3.0f, {0.f, 0.5f, 0.f, false, 0.f}, nullptr},
      {"stop", 2.5f, {}, "05_stop"},
  };
}

// --------------------------------------------------------------- minimal PNG writer (stored deflate)

inline uint32_t Crc32(const uint8_t* d, size_t n, uint32_t c = 0xFFFFFFFFu) {
  static uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t k = i;
      for (int j = 0; j < 8; ++j) { k = (k & 1) ? 0xEDB88320u ^ (k >> 1) : k >> 1; }
      table[i] = k;
    }
    init = true;
  }
  for (size_t i = 0; i < n; ++i) { c = table[(c ^ d[i]) & 0xFF] ^ (c >> 8); }
  return c;
}

inline bool WritePng(const std::string& file, int w, int h, const std::vector<uint8_t>& rgb_bottom_up) {
  std::vector<uint8_t> raw;
  raw.reserve(static_cast<size_t>(h) * (w * 3 + 1));
  for (int y = h - 1; y >= 0; --y) {
    raw.push_back(0);
    const uint8_t* row = rgb_bottom_up.data() + static_cast<size_t>(y) * w * 3;
    raw.insert(raw.end(), row, row + w * 3);
  }
  std::vector<uint8_t> z = {0x78, 0x01};
  size_t pos = 0;
  while (pos < raw.size() || raw.empty()) {
    const size_t n = std::min<size_t>(65535, raw.size() - pos);
    z.push_back(pos + n == raw.size() ? 1 : 0);
    z.push_back(n & 0xFF); z.push_back((n >> 8) & 0xFF);
    z.push_back(~n & 0xFF); z.push_back((~n >> 8) & 0xFF);
    z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + n);
    pos += n;
    if (raw.empty()) { break; }
  }
  uint32_t a = 1, b = 0;
  for (uint8_t v : raw) { a = (a + v) % 65521; b = (b + a) % 65521; }
  const uint32_t adler = (b << 16) | a;
  for (int s = 24; s >= 0; s -= 8) { z.push_back((adler >> s) & 0xFF); }

  FILE* fp = std::fopen(file.c_str(), "wb");
  if (!fp) { return false; }
  auto be32 = [](std::vector<uint8_t>& v, uint32_t x) { for (int s = 24; s >= 0; s -= 8) { v.push_back((x >> s) & 0xFF); } };
  auto chunk = [&](const char* type, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> c;
    be32(c, static_cast<uint32_t>(data.size()));
    c.insert(c.end(), type, type + 4);
    c.insert(c.end(), data.begin(), data.end());
    const uint32_t crc = Crc32(c.data() + 4, c.size() - 4) ^ 0xFFFFFFFFu;
    be32(c, crc);
    std::fwrite(c.data(), 1, c.size(), fp);
  };
  static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  std::fwrite(sig, 1, 8, fp);
  std::vector<uint8_t> ihdr;
  be32(ihdr, static_cast<uint32_t>(w));
  be32(ihdr, static_cast<uint32_t>(h));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
  chunk("IHDR", ihdr);
  chunk("IDAT", z);
  chunk("IEND", {});
  std::fclose(fp);
  return true;
}

// read the back buffer (call after rendering, before swapping)
inline bool Screenshot(const std::string& file) {
  GLint vp[4];
  ::glGetIntegerv(GL_VIEWPORT, vp);
  const int w = vp[2], h = vp[3];
  if (w <= 0 || h <= 0) { return false; }
  std::vector<uint8_t> px(static_cast<size_t>(w) * h * 3);
  ::glPixelStorei(GL_PACK_ALIGNMENT, 1);
  ::glReadBuffer(GL_BACK);
  ::glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
  return WritePng(file, w, h, px);
}

}  // namespace selftest

#endif
