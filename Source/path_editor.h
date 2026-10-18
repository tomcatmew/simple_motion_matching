/*
 *  Yifei Chen
 *  Motion Matching Demo - path model + interactive path editor
 *
 *  Mouse (in the 3D view)
 *    LMB on empty ground ...... add a waypoint (after the selected one, or at the end)
 *    LMB on a path segment .... insert a waypoint there
 *    LMB drag a waypoint ...... move it
 *    RMB click a waypoint ..... context menu (delete / duplicate / speed / start here)
 *    Del / Backspace .......... delete the selected waypoint
 *    Ctrl+Z / Ctrl+Y .......... undo / redo
 */
#ifndef MM_PATH_EDITOR_H
#define MM_PATH_EDITOR_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "imgui.h"
#include "delfem2/vec2.h"
#include "camera.h"
#include "render_util.h"

namespace dfm2 = delfem2;

struct KeyPoint {
  dfm2::CVec2d Position;  // (x, z) on the ground
  float Velocity = 150.f;  // speed (units / s) used on the segment that starts at this point
};

// where the path "rabbit" currently is: segment index + normalized position on that segment
struct PathFollower {
  int keypoint = 0;
  float t = 0.f;
};

class MotionPath {
 public:
  std::vector<KeyPoint> points;
  bool loop = true;

  [[nodiscard]] int Size() const { return static_cast<int>(points.size()); }
  [[nodiscard]] bool Usable() const { return points.size() >= 2; }
  [[nodiscard]] int NumSegments() const {
    const int n = Size();
    if (n < 2) { return 0; }
    return loop ? n : n - 1;
  }
  [[nodiscard]] int Next(int i) const {
    const int n = Size();
    return loop ? (i + 1) % n : std::min(i + 1, n - 1);
  }
  [[nodiscard]] bool IsLastPoint(int i) const { return !loop && i >= Size() - 1; }

  [[nodiscard]] dfm2::CVec2d PointAt(int kp, float t) const {
    const KeyPoint& a = points[kp];
    const KeyPoint& b = points[Next(kp)];
    return a.Position + (b.Position - a.Position) * t;
  }

  [[nodiscard]] dfm2::CVec2d DirectionAt(int kp) const {
    // direction of the segment starting at kp (last point of an open path: previous segment)
    int i = kp;
    if (IsLastPoint(i)) { i = std::max(0, i - 1); }
    for (int k = 0; k < Size(); ++k) {
      dfm2::CVec2d d = points[Next(i)].Position - points[i].Position;
      if (d.norm() > 1e-6) { return d.normalized(); }
      i = Next(i);
    }
    return {1.0, 0.0};
  }

  [[nodiscard]] double Length() const {
    double len = 0.0;
    for (int i = 0; i < NumSegments(); ++i) { len += (points[Next(i)].Position - points[i].Position).norm(); }
    return len;
  }

  [[nodiscard]] double Duration() const {
    double sec = 0.0;
    for (int i = 0; i < NumSegments(); ++i) {
      sec += (points[Next(i)].Position - points[i].Position).norm() / std::max(1.0f, points[i].Velocity);
    }
    return sec;
  }

  /**
   * Move along the path for `time` seconds starting from `from`
   * (generalization of the original SimulatePath(): supports open paths and degenerate segments).
   */
  void Simulate(float time, const PathFollower& from, PathFollower& to,
                dfm2::CVec2d& pos, dfm2::CVec2d& dir) const {
    to = from;
    if (!Usable()) {
      pos = points.empty() ? dfm2::CVec2d(0, 0) : points[0].Position;
      dir = dfm2::CVec2d(1, 0);
      return;
    }
    to.keypoint = std::clamp(to.keypoint, 0, Size() - 1);
    to.t = std::clamp(to.t, 0.f, 1.f);
    float remaining = std::max(0.f, time);
    int guard = 4 * Size() + 4;
    while (guard-- > 0) {
      if (IsLastPoint(to.keypoint)) {  // reached the end of an open path
        to.t = 0.f;
        pos = points[to.keypoint].Position;
        dir = DirectionAt(to.keypoint);
        return;
      }
      const KeyPoint& cur = points[to.keypoint];
      const KeyPoint& nxt = points[Next(to.keypoint)];
      const dfm2::CVec2d seg = nxt.Position - cur.Position;
      const double seg_len = seg.norm();
      const double speed = std::max(1.0f, cur.Velocity);
      const double left = seg_len * (1.0 - to.t);
      const double time_to_next = left / speed;
      if (seg_len > 1e-6 && remaining <= time_to_next) {
        to.t = static_cast<float>(to.t + remaining * speed / seg_len);
        pos = cur.Position + seg * to.t;
        dir = seg.normalized();
        return;
      }
      remaining -= static_cast<float>(time_to_next);
      to.keypoint = Next(to.keypoint);
      to.t = 0.f;
    }
    pos = PointAt(to.keypoint, to.t);
    dir = DirectionAt(to.keypoint);
  }

  // closest location on the path
  PathFollower Closest(const dfm2::CVec2d& p, double* out_dist = nullptr) const {
    PathFollower best;
    double best_d = 1e30;
    for (int i = 0; i < NumSegments(); ++i) {
      const dfm2::CVec2d a = points[i].Position;
      const dfm2::CVec2d ab = points[Next(i)].Position - a;
      const double l2 = ab.squaredNorm();
      double t = l2 > 1e-9 ? (p - a).dot(ab) / l2 : 0.0;
      t = std::clamp(t, 0.0, 1.0);
      const double d = (a + ab * t - p).norm();
      if (d < best_d) {
        best_d = d;
        best.keypoint = i;
        best.t = static_cast<float>(t);
      }
    }
    if (out_dist) { *out_dist = best_d; }
    return best;
  }

  void Reverse() {
    if (points.size() < 2) { return; }
    // speeds belong to the segment that *starts* at a point, shift them with the reversal
    std::vector<KeyPoint> r(points.rbegin(), points.rend());
    const int n = static_cast<int>(r.size());
    std::vector<float> sp(n);
    for (int i = 0; i < n; ++i) { sp[i] = r[(i + 1) % n].Velocity; }
    for (int i = 0; i < n; ++i) { r[i].Velocity = sp[i]; }
    points = r;
  }

  // ---------------------------------------------------------- presets

  enum Preset { kSquare = 0, kRectangle, kCircle, kFigureEight, kStar, kSlalom, kZigzag, kRandom, kNumPresets };

  static const char* PresetName(int p) {
    static const char* names[kNumPresets] = {"Square", "Rectangle", "Circle", "Figure eight", "Star",
                                             "Slalom", "Zigzag (open)", "Random"};
    return names[p];
  }

  void ApplyPreset(int preset, float size, float speed, const dfm2::CVec2d& center) {
    points.clear();
    loop = true;
    const double h = size * 0.5;
    auto add = [&](double x, double z) {
      KeyPoint k;
      k.Position = dfm2::CVec2d(center.x + x, center.y + z);
      k.Velocity = speed;
      points.push_back(k);
    };
    switch (preset) {
      case kSquare:
        add(-h, -h); add(h, -h); add(h, h); add(-h, h);
        break;
      case kRectangle:
        add(-h * 1.4, -h * 0.5); add(h * 1.4, -h * 0.5); add(h * 1.4, h * 0.5); add(-h * 1.4, h * 0.5);
        break;
      case kCircle:
        for (int i = 0; i < 12; ++i) {
          const double a = 2.0 * M_PI * i / 12.0;
          add(h * std::cos(a), h * std::sin(a));
        }
        break;
      case kFigureEight:
        for (int i = 0; i < 16; ++i) {  // lemniscate of Gerono
          const double a = 2.0 * M_PI * (i + 0.5) / 16.0;  // no waypoint on the crossing
          add(h * 1.3 * std::sin(a), h * 0.75 * std::sin(a) * std::cos(a) * 2.0);
        }
        break;
      case kStar:
        for (int i = 0; i < 10; ++i) {
          const double a = 2.0 * M_PI * i / 10.0 - M_PI * 0.5;
          const double r = (i % 2 == 0) ? h : h * 0.45;
          add(r * std::cos(a), r * std::sin(a));
        }
        break;
      case kSlalom:
        for (int i = 0; i < 6; ++i) { add(-h * 1.5 + i * h * 0.6, (i % 2 == 0) ? -h * 0.35 : h * 0.35); }
        add(h * 1.5, h); add(-h * 1.5, h);
        break;
      case kZigzag:
        loop = false;
        for (int i = 0; i < 7; ++i) { add(-h * 1.5 + i * h * 0.5, (i % 2 == 0) ? -h * 0.4 : h * 0.4); }
        break;
      case kRandom:
      default: {
        unsigned int s = static_cast<unsigned int>(rand()) | 1u;
        auto rnd = [&s]() { s = s * 1664525u + 1013904223u; return ((s >> 8) & 0xFFFF) / 65535.0; };
        const int n = 6 + static_cast<int>(rnd() * 4);
        for (int i = 0; i < n; ++i) {
          const double a = 2.0 * M_PI * i / n;
          const double r = h * (0.45 + 0.55 * rnd());
          add(r * std::cos(a), r * std::sin(a));
        }
        for (auto& k : points) { k.Velocity = static_cast<float>(speed * (0.6 + 0.8 * rnd())); }
        break;
      }
    }
  }

  // ---------------------------------------------------------- file io

  [[nodiscard]] bool Save(const std::string& file) const {
    std::ofstream ofs(file);
    if (!ofs) { return false; }
    ofs << "# motion matching path: x z speed\n";
    ofs << "loop " << (loop ? 1 : 0) << "\n";
    for (const auto& k : points) { ofs << k.Position.x << " " << k.Position.y << " " << k.Velocity << "\n"; }
    return true;
  }

  bool Load(const std::string& file) {
    std::ifstream ifs(file);
    if (!ifs) { return false; }
    std::vector<KeyPoint> pts;
    bool lp = true;
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.empty() || line[0] == '#') { continue; }
      std::istringstream ss(line);
      if (line.rfind("loop", 0) == 0) {
        std::string tag;
        int v = 1;
        ss >> tag >> v;
        lp = v != 0;
        continue;
      }
      KeyPoint k;
      if (ss >> k.Position.x >> k.Position.y) {
        float v;
        if (ss >> v) { k.Velocity = v; }
        pts.push_back(k);
      }
    }
    if (pts.size() < 2) { return false; }
    points = pts;
    loop = lp;
    return true;
  }
};

// ============================================================================

class PathEditor {
 public:
  // settings
  bool click_to_add = true;
  bool snap = false;
  float grid = 50.f;
  bool show_labels = true;
  bool color_by_speed = true;
  float new_point_speed = 150.f;
  int preset = MotionPath::kSquare;
  float preset_size = 600.f;
  char filename[256] = "path.txt";

  // state
  int selected = -1;
  int hovered = -1;
  int hovered_segment = -1;
  bool dragging = false;
  bool has_ground_hit = false;
  V3 ground_hit;

  // events for the application, cleared by the application
  bool structure_changed = false;  // points added/removed/reordered -> re-project the follower
  bool request_restart = false;    // put the character at the start of the path
  bool request_join = false;       // put the follower at the closest point to the character
  int request_start_at = -1;       // put the follower at this waypoint

  // ------------------------------------------------------------------ undo

  void PushUndo(const MotionPath& path) {
    undo_.push_back(path);
    if (undo_.size() > 100) { undo_.erase(undo_.begin()); }
    redo_.clear();
  }
  void Undo(MotionPath& path) {
    if (undo_.empty()) { return; }
    redo_.push_back(path);
    path = undo_.back();
    undo_.pop_back();
    AfterStructuralEdit(path);
  }
  void Redo(MotionPath& path) {
    if (redo_.empty()) { return; }
    undo_.push_back(path);
    path = redo_.back();
    redo_.pop_back();
    AfterStructuralEdit(path);
  }

  // ------------------------------------------------------------------ editing primitives

  int InsertPoint(MotionPath& path, int index, const dfm2::CVec2d& p, float speed) {
    PushUndo(path);
    KeyPoint k;
    k.Position = Snapped(p);
    k.Velocity = speed;
    index = std::clamp(index, 0, path.Size());
    path.points.insert(path.points.begin() + index, k);
    selected = index;
    structure_changed = true;
    return index;
  }

  void DeletePoint(MotionPath& path, int index) {
    if (index < 0 || index >= path.Size()) { return; }
    PushUndo(path);
    path.points.erase(path.points.begin() + index);
    if (selected == index) {
      selected = path.points.empty() ? -1 : std::min(index, path.Size() - 1);
    } else if (selected > index) {
      --selected;
    }
    structure_changed = true;
  }

  // ------------------------------------------------------------------ mouse in the 3D view

  /**
   * @param viewport_mouse  true when the mouse is over the 3D view (not over an ImGui window)
   * @return true if the left mouse button is used by the editor this frame
   */
  bool HandleMouse(MotionPath& path, const OrbitCamera& cam, bool viewport_mouse) {
    const ImGuiIO& io = ImGui::GetIO();
    const float w = io.DisplaySize.x, h = io.DisplaySize.y;
    const float mx = io.MousePos.x, my = io.MousePos.y;
    hovered = -1;
    hovered_segment = -1;
    has_ground_hit = false;
    if (selected >= path.Size()) { selected = -1; }

    const bool mouse_valid = mx >= 0.f && my >= 0.f && mx < w && my < h;  // cursor inside the window
    if (mouse_valid) {
      has_ground_hit = cam.ScreenRay(mx, my, w, h).HitGround(ground_hit);
    }

    // finish a drag even if the mouse left the view
    if (dragging && !io.MouseDown[0]) { dragging = false; }
    if (dragging) {
      if (has_ground_hit && selected >= 0) {
        path.points[selected].Position = Snapped(dfm2::CVec2d(ground_hit.x + drag_offset_.x, ground_hit.z + drag_offset_.y));
      }
      hovered = selected;
      return true;
    }
    if (!viewport_mouse || !mouse_valid) {
      has_ground_hit = false;  // no click preview while the mouse is over a panel
      return false;
    }

    // hover: waypoints first (screen space), then segments
    float best = kPickPixels;
    for (int i = 0; i < path.Size(); ++i) {
      float sx, sy;
      if (!cam.WorldToScreen(V3((float)path.points[i].Position.x, 0.f, (float)path.points[i].Position.y), w, h, sx, sy)) { continue; }
      const float d = std::hypot(sx - mx, sy - my);
      if (d < best) { best = d; hovered = i; }
    }
    if (hovered < 0) {
      float best_seg = kSegmentPickPixels;
      for (int i = 0; i < path.NumSegments(); ++i) {
        const auto& a = path.points[i].Position;
        const auto& b = path.points[path.Next(i)].Position;
        float ax, ay, bx, by;
        if (!cam.WorldToScreen(V3((float)a.x, 0.f, (float)a.y), w, h, ax, ay)) { continue; }
        if (!cam.WorldToScreen(V3((float)b.x, 0.f, (float)b.y), w, h, bx, by)) { continue; }
        const float d = PointSegmentDistance2D(mx, my, ax, ay, bx, by);
        if (d < best_seg) { best_seg = d; hovered_segment = i; }
      }
    }

    // left click
    if (io.MouseClicked[0]) {
      if (hovered >= 0) {
        selected = hovered;
        PushUndo(path);
        dragging = true;
        drag_offset_ = has_ground_hit
                           ? dfm2::CVec2d(path.points[hovered].Position.x - ground_hit.x,
                                          path.points[hovered].Position.y - ground_hit.z)
                           : dfm2::CVec2d(0, 0);
        return true;
      }
      if (hovered_segment >= 0 && has_ground_hit) {
        const auto& a = path.points[hovered_segment];
        const auto& b = path.points[path.Next(hovered_segment)];
        const dfm2::CVec2d p = ClosestOnSegment(dfm2::CVec2d(ground_hit.x, ground_hit.z), a.Position, b.Position);
        InsertPoint(path, hovered_segment + 1, p, a.Velocity);
        dragging = true;
        drag_offset_ = dfm2::CVec2d(0, 0);
        return true;
      }
      if (click_to_add && has_ground_hit) {
        const int at = selected >= 0 ? selected + 1 : path.Size();
        InsertPoint(path, at, dfm2::CVec2d(ground_hit.x, ground_hit.z), new_point_speed);
        dragging = true;
        drag_offset_ = dfm2::CVec2d(0, 0);
        return true;
      }
      selected = -1;
    }
    return false;
  }

  // RMB released without dragging: open the context menu for what is under the mouse
  void OpenContextMenu() {
    context_point_ = hovered;
    context_hit_ = ground_hit;
    context_has_hit_ = has_ground_hit;
    if (hovered >= 0) { selected = hovered; }
    ImGui::OpenPopup("##path_context");
  }

  void DrawContextMenu(MotionPath& path) {
    if (!ImGui::BeginPopup("##path_context")) { return; }
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) { ImGui::CloseCurrentPopup(); }
    if (context_point_ >= 0 && context_point_ < path.Size()) {
      const int i = context_point_;
      ImGui::TextDisabled("Waypoint %d", i);
      ImGui::Separator();
      ImGui::SetNextItemWidth(160);
      float sp = path.points[i].Velocity;
      if (ImGui::SliderFloat("Speed", &sp, 20.f, 600.f, "%.0f")) {
        if (!speed_edit_active_) { PushUndo(path); speed_edit_active_ = true; }
        path.points[i].Velocity = sp;
      }
      if (!ImGui::IsItemActive()) { speed_edit_active_ = false; }
      if (ImGui::MenuItem("Start character here")) { request_start_at = i; }
      if (ImGui::MenuItem("Duplicate")) {
        const auto& a = path.points[i].Position;
        const auto& b = path.points[path.Next(i)].Position;
        InsertPoint(path, i + 1, path.Next(i) == i ? a + dfm2::CVec2d(grid, 0) : (a + b) * 0.5, path.points[i].Velocity);
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::MenuItem("Delete", "Del")) {
        DeletePoint(path, i);
        ImGui::CloseCurrentPopup();
      }
    } else {
      ImGui::TextDisabled("Ground");
      ImGui::Separator();
      if (ImGui::MenuItem("Add waypoint here", nullptr, false, context_has_hit_)) {
        const int at = selected >= 0 ? selected + 1 : path.Size();
        InsertPoint(path, at, dfm2::CVec2d(context_hit_.x, context_hit_.z), new_point_speed);
      }
      if (ImGui::MenuItem("Deselect", nullptr, false, selected >= 0)) { selected = -1; }
      if (ImGui::MenuItem("Join path from character")) { request_join = true; }
    }
    ImGui::EndPopup();
  }

  // keyboard shortcuts, call when ImGui does not want the keyboard
  void HandleKeys(MotionPath& path, bool del, bool undo, bool redo) {
    if (del && selected >= 0) { DeletePoint(path, selected); }
    if (undo) { Undo(path); }
    if (redo) { Redo(path); }
  }

  // ------------------------------------------------------------------ drawing (3D)

  void DrawScene(const MotionPath& path, bool editing) const {
    const int n = path.Size();
    ::glDisable(GL_LIGHTING);
    ::glDisable(GL_TEXTURE_2D);
    const float y = 1.0f;
    // segments
    ::glLineWidth(4.f);
    ::glBegin(GL_LINES);
    for (int i = 0; i < path.NumSegments(); ++i) {
      const auto& a = path.points[i];
      const auto& b = path.points[path.Next(i)];
      float c[3];
      SegmentColor(a.Velocity, i == hovered_segment, c);
      ::glColor3fv(c);
      ::glVertex3d(a.Position.x, y, a.Position.y);
      ::glVertex3d(b.Position.x, y, b.Position.y);
    }
    ::glEnd();
    // chevrons showing the travel direction
    ::glLineWidth(2.f);
    ::glBegin(GL_LINES);
    for (int i = 0; i < path.NumSegments(); ++i) {
      const auto& a = path.points[i].Position;
      const auto& b = path.points[path.Next(i)].Position;
      const dfm2::CVec2d d = b - a;
      const double len = d.norm();
      if (len < 1e-3) { continue; }
      const dfm2::CVec2d u = d / len;
      const dfm2::CVec2d v(-u.y, u.x);
      const int nchev = std::max(1, static_cast<int>(len / 120.0));
      float c[3];
      SegmentColor(path.points[i].Velocity, false, c);
      ::glColor3f(c[0] * 0.6f, c[1] * 0.6f, c[2] * 0.6f);
      for (int k = 0; k < nchev; ++k) {
        const dfm2::CVec2d m = a + d * ((k + 0.5) / nchev);
        const dfm2::CVec2d p0 = m - u * 10.0 + v * 9.0;
        const dfm2::CVec2d p1 = m - u * 10.0 - v * 9.0;
        ::glVertex3d(p0.x, y + 0.5, p0.y); ::glVertex3d(m.x, y + 0.5, m.y);
        ::glVertex3d(p1.x, y + 0.5, p1.y); ::glVertex3d(m.x, y + 0.5, m.y);
      }
    }
    ::glEnd();
    // waypoints
    for (int i = 0; i < n; ++i) {
      const auto& p = path.points[i].Position;
      float r = 7.f;
      if (i == selected) {
        ::glColor3f(1.0f, 0.55f, 0.1f);
        r = 10.f;
      } else if (i == hovered) {
        ::glColor3f(1.0f, 1.0f, 1.0f);
        r = 9.f;
      } else if (i == 0) {
        ::glColor3f(0.2f, 0.85f, 0.35f);
      } else {
        ::glColor3f(0.95f, 0.85f, 0.15f);
      }
      gl_util::Sphere(p.x, y + r * 0.5, p.y, r);
    }
    if (!editing) { return; }
    // preview of what a left click would do
    if (!dragging && hovered < 0 && has_ground_hit) {
      if (hovered_segment >= 0) {
        const auto& a = path.points[hovered_segment].Position;
        const auto& b = path.points[path.Next(hovered_segment)].Position;
        const dfm2::CVec2d q = Snapped(ClosestOnSegment(dfm2::CVec2d(ground_hit.x, ground_hit.z), a, b));
        ::glColor3f(1.f, 1.f, 1.f);
        gl_util::Sphere(q.x, y + 3.0, q.y, 5.0);
      } else if (click_to_add) {
        const dfm2::CVec2d q = Snapped(dfm2::CVec2d(ground_hit.x, ground_hit.z));
        ::glEnable(GL_LINE_STIPPLE);
        ::glLineStipple(3, 0xAAAA);
        ::glLineWidth(2.f);
        ::glColor3f(1.f, 1.f, 1.f);
        ::glBegin(GL_LINES);
        if (n > 0) {
          const int anchor = selected >= 0 ? selected : n - 1;
          const auto& a = path.points[anchor].Position;
          ::glVertex3d(a.x, y, a.y); ::glVertex3d(q.x, y, q.y);
          if (path.loop && n > 1) {
            const auto& b = path.points[(anchor + 1) % n].Position;
            ::glVertex3d(q.x, y, q.y); ::glVertex3d(b.x, y, b.y);
          }
        }
        ::glEnd();
        ::glDisable(GL_LINE_STIPPLE);
        gl_util::Sphere(q.x, y + 3.0, q.y, 6.0);
      }
    }
    if (snap && has_ground_hit) {  // small grid around the cursor
      const double g = grid;
      const double cx = std::round(ground_hit.x / g) * g, cz = std::round(ground_hit.z / g) * g;
      ::glLineWidth(1.f);
      ::glColor4f(1.f, 1.f, 1.f, 0.35f);
      ::glBegin(GL_LINES);
      for (int k = -3; k <= 3; ++k) {
        ::glVertex3d(cx + k * g, y, cz - 3 * g); ::glVertex3d(cx + k * g, y, cz + 3 * g);
        ::glVertex3d(cx - 3 * g, y, cz + k * g); ::glVertex3d(cx + 3 * g, y, cz + k * g);
      }
      ::glEnd();
    }
  }

  // index / speed labels + hint next to the cursor, drawn by ImGui on top of the scene
  void DrawOverlay(const MotionPath& path, const OrbitCamera& cam, bool editing) const {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    const float w = io.DisplaySize.x, h = io.DisplaySize.y;
    if (show_labels) {
      for (int i = 0; i < path.Size(); ++i) {
        const auto& p = path.points[i];
        float sx, sy;
        if (!cam.WorldToScreen(V3((float)p.Position.x, 0.f, (float)p.Position.y), w, h, sx, sy)) { continue; }
        char buf[64];
        if (i == 0) {
          std::snprintf(buf, sizeof(buf), "start  %.0f", p.Velocity);
        } else {
          std::snprintf(buf, sizeof(buf), "%d  %.0f", i, p.Velocity);
        }
        const ImVec2 ts = ImGui::CalcTextSize(buf);
        const ImVec2 p0(sx + 10, sy - ts.y - 6);
        dl->AddRectFilled(ImVec2(p0.x - 3, p0.y - 1), ImVec2(p0.x + ts.x + 3, p0.y + ts.y + 1),
                          i == selected ? IM_COL32(200, 110, 20, 220) : IM_COL32(20, 20, 20, 170), 3.f);
        dl->AddText(p0, IM_COL32(255, 255, 255, 255), buf);
      }
    }
    if (!editing || ImGui::GetIO().WantCaptureMouse) { return; }
    const char* hint = nullptr;
    if (dragging) { hint = "drag to move"; }
    else if (hovered >= 0) { hint = "LMB drag: move   RMB: menu   Del: delete"; }
    else if (hovered_segment >= 0) { hint = "click: insert waypoint"; }
    else if (click_to_add && has_ground_hit) { hint = selected >= 0 ? "click: add after selected" : "click: add waypoint"; }
    if (hint) {
      dl->AddText(ImVec2(io.MousePos.x + 18, io.MousePos.y + 14), IM_COL32(0, 0, 0, 160), hint);
      dl->AddText(ImVec2(io.MousePos.x + 17, io.MousePos.y + 13), IM_COL32(255, 255, 255, 255), hint);
    }
  }

  // ------------------------------------------------------------------ panel

  void DrawPanel(MotionPath& path, const dfm2::CVec2d& character_pos, bool* paused, float s = 1.f) {
    ImGui::TextDisabled("LMB: add / insert / drag     RMB: menu     Del: delete");

    ImGui::Checkbox("Click adds waypoints", &click_to_add);
    ImGui::SameLine();
    ImGui::BeginDisabled(undo_.empty());
    if (ImGui::Button("Undo")) { Undo(path); }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(redo_.empty());
    if (ImGui::Button("Redo")) { Redo(path); }
    ImGui::EndDisabled();

    if (ImGui::Checkbox("Loop", &path.loop)) { structure_changed = true; }
    ImGui::SameLine();
    ImGui::Checkbox("Snap", &snap);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(80 * s);
    ImGui::DragFloat("##grid", &grid, 1.f, 5.f, 500.f, "grid %.0f");
    ImGui::SameLine();
    ImGui::Checkbox("Labels", &show_labels);

    // character
    if (ImGui::Button("Restart at start")) { request_restart = true; }
    ImGui::SameLine();
    if (ImGui::Button("Join from character")) { request_join = true; }
    if (paused) {
      ImGui::SameLine();
      ImGui::Checkbox("Pause", paused);
    }

    ImGui::Separator();
    // presets
    ImGui::SetNextItemWidth(120 * s);
    if (ImGui::BeginCombo("##preset", MotionPath::PresetName(preset))) {
      for (int p = 0; p < MotionPath::kNumPresets; ++p) {
        if (ImGui::Selectable(MotionPath::PresetName(p), p == preset)) { preset = p; }
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90 * s);
    ImGui::DragFloat("##size", &preset_size, 5.f, 100.f, 3000.f, "size %.0f");
    ImGui::SameLine();
    if (ImGui::Button("Apply preset")) {
      PushUndo(path);
      path.ApplyPreset(preset, preset_size, new_point_speed, preset_at_character_ ? character_pos : dfm2::CVec2d(0, 0));
      AfterStructuralEdit(path);
      request_join = true;
    }
    ImGui::SameLine();
    ImGui::Checkbox("at character", &preset_at_character_);

    ImGui::SetNextItemWidth(120 * s);
    ImGui::SliderFloat("##newspeed", &new_point_speed, 20.f, 600.f, "speed %.0f");
    ImGui::SameLine();
    if (ImGui::Button("Set all speeds")) {
      PushUndo(path);
      for (auto& k : path.points) { k.Velocity = new_point_speed; }
    }
    if (color_by_speed) {
      ImGui::SameLine();
      DrawSpeedLegend();
    }

    // table of waypoints
    ImGui::Separator();
    ImGui::Text("%d waypoints   length %.0f   lap %.1f s", path.Size(), path.Length(), path.Duration());
    const ImGuiTableFlags tflags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                   ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit;
    const float row_h = ImGui::GetFrameHeightWithSpacing();
    const float table_h = std::min(10.f, std::max(3.f, (float)path.Size() + 1.f)) * row_h;
    int move_from = -1, move_to = -1, delete_at = -1, insert_after = -1;
    if (ImGui::BeginTable("##waypoints", 5, tflags, ImVec2(0, table_h))) {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 26 * s);
      ImGui::TableSetupColumn("X", ImGuiTableColumnFlags_WidthFixed, 64 * s);
      ImGui::TableSetupColumn("Z", ImGuiTableColumnFlags_WidthFixed, 64 * s);
      ImGui::TableSetupColumn("Speed", ImGuiTableColumnFlags_WidthFixed, 50 * s);
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 92 * s);
      ImGui::TableHeadersRow();
      for (int i = 0; i < path.Size(); ++i) {
        ImGui::PushID(i);
        ImGui::TableNextRow();
        if (i == selected) {
          ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(200, 110, 20, 110));
        }
        ImGui::TableSetColumnIndex(0);
        char label[16];
        std::snprintf(label, sizeof(label), "%d", i);
        if (ImGui::Selectable(label, i == selected, ImGuiSelectableFlags_None)) { selected = i; }
        if (i == selected && scroll_to_selected_) {
          ImGui::SetScrollHereY();
          scroll_to_selected_ = false;
        }
        auto& k = path.points[i];
        float x = static_cast<float>(k.Position.x) + 0.f, z = static_cast<float>(k.Position.y) + 0.f;
        if (std::fabs(x) < 0.5f) { x = 0.f; }  // avoid "-0"
        if (std::fabs(z) < 0.5f) { z = 0.f; }
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat("##x", &x, 2.f, 0.f, 0.f, "%.0f")) { k.Position.x = x; }
        TrackItemEdit(path, i);
        ImGui::TableSetColumnIndex(2);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat("##z", &z, 2.f, 0.f, 0.f, "%.0f")) { k.Position.y = z; }
        TrackItemEdit(path, i);
        ImGui::TableSetColumnIndex(3);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat("##v", &k.Velocity, 1.f, 20.f, 600.f, "%.0f");
        TrackItemEdit(path, i);
        ImGui::TableSetColumnIndex(4);
        if (ImGui::SmallButton("^") && i > 0) { move_from = i; move_to = i - 1; }
        ImGui::SameLine(0, 2);
        if (ImGui::SmallButton("v") && i + 1 < path.Size()) { move_from = i; move_to = i + 1; }
        ImGui::SameLine(0, 2);
        if (ImGui::SmallButton("+")) { insert_after = i; }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Insert a waypoint after %d", i); }
        ImGui::SameLine(0, 2);
        if (ImGui::SmallButton("x")) { delete_at = i; }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Delete waypoint %d", i); }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
    if (move_from >= 0) {
      PushUndo(path);
      std::swap(path.points[move_from], path.points[move_to]);
      if (selected == move_from) { selected = move_to; } else if (selected == move_to) { selected = move_from; }
      structure_changed = true;
    }
    if (insert_after >= 0) {
      const auto& a = path.points[insert_after].Position;
      const int nx = path.Next(insert_after);
      const dfm2::CVec2d p = nx == insert_after ? a + dfm2::CVec2d(grid * 2, 0) : (a + path.points[nx].Position) * 0.5;
      InsertPoint(path, insert_after + 1, p, path.points[insert_after].Velocity);
    }
    if (delete_at >= 0) { DeletePoint(path, delete_at); }

    if (ImGui::Button("Add point")) {
      dfm2::CVec2d p(0, 0);
      if (path.Size() > 0) {
        const auto& last = path.points.back().Position;
        const dfm2::CVec2d d = path.Size() > 1 ? path.points.back().Position - path.points[path.Size() - 2].Position
                                               : dfm2::CVec2d(1, 0);
        p = last + (d.norm() > 1e-6 ? d.normalized() : dfm2::CVec2d(1, 0)) * 200.0;
      }
      InsertPoint(path, path.Size(), p, new_point_speed);
      scroll_to_selected_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reverse")) {
      PushUndo(path);
      path.Reverse();
      if (selected >= 0) { selected = path.Size() - 1 - selected; }
      structure_changed = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
      PushUndo(path);
      path.points.clear();
      selected = -1;
      structure_changed = true;
    }
    if (!path.Usable()) {
      ImGui::TextColored(ImVec4(1.f, 0.6f, 0.2f, 1.f), "Need at least 2 waypoints: click on the ground to add some.");
    }

    // file
    ImGui::SetNextItemWidth(150 * s);
    ImGui::InputText("##file", filename, sizeof(filename));
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
      SetStatus(path.Save(filename) ? std::string("saved ") + filename : std::string("cannot write ") + filename);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
      MotionPath tmp = path;
      if (tmp.Load(filename)) {
        PushUndo(path);
        path = tmp;
        AfterStructuralEdit(path);
        request_join = true;
        SetStatus(std::string("loaded ") + filename);
      } else {
        SetStatus(std::string("cannot read ") + filename);
      }
    }
    if (!status_.empty() && ImGui::GetTime() - status_time_ < 4.0) {
      ImGui::SameLine();
      ImGui::TextDisabled("%s", status_.c_str());
    }
  }

  void SelectionChangedFrom3D() { scroll_to_selected_ = true; }

 private:
  static constexpr float kPickPixels = 16.f;
  static constexpr float kSegmentPickPixels = 9.f;

  [[nodiscard]] dfm2::CVec2d Snapped(const dfm2::CVec2d& p) const {
    if (!snap || grid <= 0.f) { return p; }
    return {std::round(p.x / grid) * grid, std::round(p.y / grid) * grid};
  }

  static dfm2::CVec2d ClosestOnSegment(const dfm2::CVec2d& p, const dfm2::CVec2d& a, const dfm2::CVec2d& b) {
    const dfm2::CVec2d ab = b - a;
    const double l2 = ab.squaredNorm();
    if (l2 < 1e-9) { return a; }
    const double t = std::clamp((p - a).dot(ab) / l2, 0.0, 1.0);
    return a + ab * t;
  }

  static float PointSegmentDistance2D(float px, float py, float ax, float ay, float bx, float by) {
    const float abx = bx - ax, aby = by - ay;
    const float l2 = abx * abx + aby * aby;
    float t = l2 > 1e-6f ? ((px - ax) * abx + (py - ay) * aby) / l2 : 0.f;
    t = std::clamp(t, 0.f, 1.f);
    return std::hypot(px - (ax + abx * t), py - (ay + aby * t));
  }

  void SegmentColor(float speed, bool highlight, float c[3]) const {
    if (highlight) { c[0] = 1.f; c[1] = 1.f; c[2] = 1.f; return; }
    if (!color_by_speed) { c[0] = 0.95f; c[1] = 0.85f; c[2] = 0.15f; return; }
    // slow = teal, walk = yellow, run = red
    const float t = std::clamp((speed - 60.f) / 340.f, 0.f, 1.f);
    if (t < 0.5f) {
      const float s = t * 2.f;
      c[0] = 0.15f + 0.8f * s; c[1] = 0.75f + 0.1f * s; c[2] = 0.75f - 0.6f * s;
    } else {
      const float s = (t - 0.5f) * 2.f;
      c[0] = 0.95f; c[1] = 0.85f - 0.65f * s; c[2] = 0.15f;
    }
  }

  void DrawSpeedLegend() const {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float wbar = 70.f, hbar = ImGui::GetFrameHeight() * 0.5f;
    const float y0 = p.y + hbar * 0.5f;
    for (int i = 0; i < 14; ++i) {
      float c0[3], c1[3];
      SegmentColor(60.f + 340.f * i / 14.f, false, c0);
      SegmentColor(60.f + 340.f * (i + 1) / 14.f, false, c1);
      dl->AddRectFilledMultiColor(ImVec2(p.x + wbar * i / 14.f, y0), ImVec2(p.x + wbar * (i + 1) / 14.f, y0 + hbar),
                                  ImGui::ColorConvertFloat4ToU32(ImVec4(c0[0], c0[1], c0[2], 1)),
                                  ImGui::ColorConvertFloat4ToU32(ImVec4(c1[0], c1[1], c1[2], 1)),
                                  ImGui::ColorConvertFloat4ToU32(ImVec4(c1[0], c1[1], c1[2], 1)),
                                  ImGui::ColorConvertFloat4ToU32(ImVec4(c0[0], c0[1], c0[2], 1)));
    }
    ImGui::Dummy(ImVec2(wbar, ImGui::GetFrameHeight()));
    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("segment color: slow (60) -> fast (400)"); }
  }

  // one undo step per drag of a table cell
  void TrackItemEdit(MotionPath& path, int i) {
    if (ImGui::IsItemActivated()) {
      PushUndo(path);
      selected = i;
    }
  }

  void AfterStructuralEdit(const MotionPath& path) {
    if (selected >= path.Size()) { selected = path.Size() - 1; }
    structure_changed = true;
  }

  void SetStatus(const std::string& s) {
    status_ = s;
    status_time_ = ImGui::GetTime();
  }

  std::vector<MotionPath> undo_, redo_;
  dfm2::CVec2d drag_offset_;
  int context_point_ = -1;
  V3 context_hit_;
  bool context_has_hit_ = false;
  bool speed_edit_active_ = false;
  bool scroll_to_selected_ = false;
  bool preset_at_character_ = false;
  std::string status_;
  double status_time_ = 0.0;
};

#endif
