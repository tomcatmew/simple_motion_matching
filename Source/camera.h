/*
 *  Yifei Chen
 *  Motion Matching Demo - camera
 *
 *  Two orbit rigs share one perspective camera:
 *   - Follow : third person camera that trails the character (gamepad right stick / RMB drag)
 *   - Free   : detached camera for looking around and editing the path with the mouse
 *              (RMB orbit, MMB / Shift+RMB pan on the ground, wheel zoom)
 *  The rendered view is damped toward the active rig, so switching modes is a smooth move
 *  instead of a cut.
 */
#ifndef MM_CAMERA_H
#define MM_CAMERA_H

#include <algorithm>
#include <cmath>

struct V3 {
  float x = 0.f, y = 0.f, z = 0.f;
  V3() = default;
  V3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  V3 operator+(const V3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  V3 operator-(const V3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  V3 operator*(float s) const { return {x * s, y * s, z * s}; }
  V3& operator+=(const V3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  V3& operator-=(const V3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  [[nodiscard]] float Dot(const V3& o) const { return x * o.x + y * o.y + z * o.z; }
  [[nodiscard]] V3 Cross(const V3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  [[nodiscard]] float Length() const { return std::sqrt(Dot(*this)); }
  [[nodiscard]] V3 Normalized() const { float l = Length(); return l > 1e-9f ? *this * (1.f / l) : V3(); }
};

struct Ray {
  V3 origin;
  V3 dir;
  // intersection with the horizontal plane y = h
  bool HitGround(V3& hit, float h = 0.f) const {
    if (std::fabs(dir.y) < 1e-6f) { return false; }
    const float t = (h - origin.y) / dir.y;
    if (t <= 0.f) { return false; }
    hit = origin + dir * t;
    return true;
  }
};

namespace cam_util {

constexpr float kPi = 3.14159265358979f;

inline float WrapAngle(float a) {
  while (a > kPi) { a -= 2.f * kPi; }
  while (a < -kPi) { a += 2.f * kPi; }
  return a;
}

// exponential smoothing that is frame rate independent
inline float DampFactor(float halflife, float dt) {
  if (halflife <= 1e-5f) { return 1.f; }
  return 1.f - std::exp(-0.69314718f * dt / halflife);
}

}  // namespace cam_util

class OrbitCamera {
 public:
  enum class Mode { Follow = 0, Free = 1 };

  struct Rig {
    V3 target;
    float yaw = 0.f;    // around +Y, 0 = camera on the +Z side looking toward -Z
    float pitch = 0.f;  // elevation, positive = above the target
    float distance = 500.f;
  };

  Mode mode = Mode::Follow;
  Rig follow{V3(0, 90, 0), 0.6f, 0.28f, 520.f};
  Rig free{V3(300, 0, 300), 0.7f, 0.95f, 1700.f};
  Rig current = follow;  // what is rendered

  // lens
  float fovy_deg = 45.f;
  float znear = 5.f;
  float zfar = 40000.f;

  // follow settings
  float follow_height = 95.f;      // look-at height above the character root
  float follow_lag = 0.10f;        // seconds, half-life of the target smoothing
  bool follow_auto_align = false;  // slowly swing behind the character while it moves
  float auto_align_delay = 1.5f;   // seconds without manual camera input
  // general
  float transition_halflife = 0.15f;
  float orbit_speed = 2.6f;        // rad / s at full stick
  float mouse_sensitivity = 0.006f;
  bool invert_y = false;

  // ---------------- control

  Rig& Active() { return mode == Mode::Follow ? follow : free; }

  void SetMode(Mode m) {
    if (m == mode) { return; }
    if (m == Mode::Free) {
      // start the free camera from the current view, so nothing jumps
      free = current;
      // keep the pivot on the ground so that panning is intuitive
      const V3 eye = EyeOf(current);
      const V3 fwd = (current.target - eye).Normalized();
      if (fwd.y < -0.05f) {
        const float t = -eye.y / fwd.y;
        free.target = eye + fwd * t;
        free.distance = t;
      }
    }
    mode = m;
    transition_timer_ = 0.8f;
  }

  void Toggle() { SetMode(mode == Mode::Follow ? Mode::Free : Mode::Follow); }

  void Orbit(float dyaw, float dpitch) {
    Rig& r = Active();
    r.yaw = cam_util::WrapAngle(r.yaw + dyaw);
    r.pitch += invert_y ? -dpitch : dpitch;
    if (mode == Mode::Follow) {
      r.pitch = std::clamp(r.pitch, -0.25f, 1.35f);
    } else {
      r.pitch = std::clamp(r.pitch, 0.02f, 1.553f);
    }
    idle_time_ = 0.f;
  }

  void Zoom(float factor) {
    Rig& r = Active();
    r.distance *= factor;
    if (mode == Mode::Follow) {
      r.distance = std::clamp(r.distance, 120.f, 2500.f);
    } else {
      r.distance = std::clamp(r.distance, 60.f, 20000.f);
    }
  }

  // move the free camera pivot on the ground plane (world units)
  void PanGround(float dx, float dz) {
    free.target.x += dx;
    free.target.z += dz;
  }

  // pan so that the ground point under `from` moves under `to` (screen pixels)
  void PanByDrag(float from_x, float from_y, float to_x, float to_y, float width, float height) {
    const Ray r0 = ScreenRay(free, from_x, from_y, width, height);
    const Ray r1 = ScreenRay(free, to_x, to_y, width, height);
    V3 h0, h1;
    if (r0.HitGround(h0) && r1.HitGround(h1) && (h0 - h1).Length() < free.distance * 4.f) {
      PanGround(h0.x - h1.x, h0.z - h1.z);
    } else {  // looking at the horizon, fall back to screen space
      const float s = free.distance * 0.0015f;
      PanGround(-(to_x - from_x) * s * std::cos(free.yaw) + (to_y - from_y) * s * std::sin(free.yaw),
                (to_x - from_x) * s * std::sin(free.yaw) + (to_y - from_y) * s * std::cos(free.yaw));
    }
  }

  void TopView() {
    free.pitch = 1.553f;
  }

  void Focus(const V3& p) {
    free.target = V3(p.x, 0.f, p.z);
  }

  // look direction projected on the ground: what "forward" means for the left stick
  void GroundBasis(float& fwd_x, float& fwd_z, float& right_x, float& right_z) const {
    fwd_x = -std::sin(current.yaw);
    fwd_z = -std::cos(current.yaw);
    right_x = std::cos(current.yaw);
    right_z = -std::sin(current.yaw);
  }

  void NotifyManualInput() { idle_time_ = 0.f; }

  // ---------------- per frame

  void Update(float dt, const V3& character_root, float character_yaw, bool character_moving) {
    using cam_util::DampFactor;
    // follow rig tracks the character
    const V3 goal = character_root + V3(0.f, follow_height, 0.f);
    const float kf = DampFactor(follow_lag, dt);
    follow.target = follow.target + (goal - follow.target) * kf;
    if (std::fabs(follow.target.x - goal.x) > 2000.f || std::fabs(follow.target.z - goal.z) > 2000.f) {
      follow.target = goal;  // teleported
    }
    idle_time_ += dt;
    if (mode == Mode::Follow && follow_auto_align && character_moving && idle_time_ > auto_align_delay) {
      // camera behind the character: camera yaw == character yaw + pi
      const float behind = cam_util::WrapAngle(character_yaw + cam_util::kPi);
      follow.yaw = cam_util::WrapAngle(follow.yaw + cam_util::WrapAngle(behind - follow.yaw) * DampFactor(1.2f, dt));
    }
    // rendered rig damps toward the active one
    const Rig& a = mode == Mode::Follow ? follow : free;
    transition_timer_ = std::max(0.f, transition_timer_ - dt);
    const float halflife = transition_timer_ > 0.f ? transition_halflife : (mode == Mode::Follow ? 0.015f : 0.035f);
    const float k = DampFactor(halflife, dt);
    current.target = current.target + (a.target - current.target) * k;
    current.yaw = cam_util::WrapAngle(current.yaw + cam_util::WrapAngle(a.yaw - current.yaw) * k);
    current.pitch += (a.pitch - current.pitch) * k;
    current.distance += (a.distance - current.distance) * k;
  }

  void SnapToActive() { current = Active(); }

  // ---------------- matrices (column major, ready for glLoadMatrixf)

  static V3 EyeOf(const Rig& r) {
    return r.target + V3(std::sin(r.yaw) * std::cos(r.pitch),
                         std::sin(r.pitch),
                         std::cos(r.yaw) * std::cos(r.pitch)) * r.distance;
  }
  [[nodiscard]] V3 Eye() const { return EyeOf(current); }

  void ViewMatrix(float m[16]) const { LookAt(m, Eye(), current.target); }

  void ProjectionMatrix(float m[16], float aspect) const {
    const float f = 1.f / std::tan(fovy_deg * cam_util::kPi / 360.f);
    std::fill(m, m + 16, 0.f);
    m[0] = f / aspect;
    m[5] = f;
    m[10] = (zfar + znear) / (znear - zfar);
    m[11] = -1.f;
    m[14] = 2.f * zfar * znear / (znear - zfar);
  }

  // ray through a pixel (origin top-left) of the rendered view
  [[nodiscard]] Ray ScreenRay(float px, float py, float width, float height) const {
    return ScreenRay(current, px, py, width, height);
  }

  // project a world point to pixels (origin top-left); false if behind the camera
  bool WorldToScreen(const V3& p, float width, float height, float& sx, float& sy) const {
    float v[16], pr[16];
    ViewMatrix(v);
    ProjectionMatrix(pr, width / std::max(height, 1.f));
    const float ex = v[0] * p.x + v[4] * p.y + v[8] * p.z + v[12];
    const float ey = v[1] * p.x + v[5] * p.y + v[9] * p.z + v[13];
    const float ez = v[2] * p.x + v[6] * p.y + v[10] * p.z + v[14];
    const float cx = pr[0] * ex;
    const float cy = pr[5] * ey;
    const float cw = -ez;
    if (cw <= znear) { return false; }
    sx = (cx / cw * 0.5f + 0.5f) * width;
    sy = (1.f - (cy / cw * 0.5f + 0.5f)) * height;
    return true;
  }

 private:
  [[nodiscard]] Ray ScreenRay(const Rig& r, float px, float py, float width, float height) const {
    const V3 eye = EyeOf(r);
    const V3 fwd = (r.target - eye).Normalized();
    const V3 right = fwd.Cross(V3(0, 1, 0)).Normalized();
    const V3 up = right.Cross(fwd);
    const float th = std::tan(fovy_deg * cam_util::kPi / 360.f);
    const float aspect = width / std::max(height, 1.f);
    const float nx = (2.f * px / width - 1.f) * th * aspect;
    const float ny = (1.f - 2.f * py / height) * th;
    return {eye, (fwd + right * nx + up * ny).Normalized()};
  }

  static void LookAt(float m[16], const V3& eye, const V3& target) {
    const V3 f = (target - eye).Normalized();
    V3 s = f.Cross(V3(0, 1, 0));
    if (s.Length() < 1e-6f) { s = V3(1, 0, 0); }
    s = s.Normalized();
    const V3 u = s.Cross(f);
    m[0] = s.x; m[4] = s.y; m[8] = s.z;   m[12] = -s.Dot(eye);
    m[1] = u.x; m[5] = u.y; m[9] = u.z;   m[13] = -u.Dot(eye);
    m[2] = -f.x; m[6] = -f.y; m[10] = -f.z; m[14] = f.Dot(eye);
    m[3] = 0.f; m[7] = 0.f; m[11] = 0.f;  m[15] = 1.f;
  }

  float idle_time_ = 0.f;
  float transition_timer_ = 0.f;
};

#endif
