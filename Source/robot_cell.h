/*
 *  Yifei Chen
 *  Motion Viewer - robot workcell
 *
 *  Reads the .scene file next to a robot clip (written by tools/make_robot_clips.py) and draws
 *  the workcell, the 6-axis arm with its tool, the props it moves and the dirt it wipes away.
 *
 *    tool <gripper|sponge|screwdriver> <length>   tcp <bone>   surface <y>
 *    camera <target x y z> <yaw> <pitch> <distance>
 *    box <center x y z> <size x y z> <r g b>      cylinder <base x y z> <radius> <height> <r g b>
 *    prop <bone> box <size x y z> <r g b>         prop <bone> screw <head radius> <head height> <length> <r g b>
 *    dirt <x y z> <radius> <seconds of wiping to remove it>
 *    step <frame> <text>
 */
#ifndef MM_ROBOT_CELL_H
#define MM_ROBOT_CELL_H

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "motion_clip.h"
#include "render_util.h"

namespace viewer {

struct Shape {
  enum Kind { kBox, kCylinder, kScrew } kind = kBox;
  int ibone = -1;               // props: the bone it follows
  double p[3] = {}, s[3] = {};  // static position, size
  float c[3] = {0.7f, 0.7f, 0.7f};
};

struct Dirt {
  double x = 0, y = 0, z = 0, r = 1, seconds = 0.1;
};

class RobotCell {
 public:
  std::string tool = "none";
  double tool_length = 0, surface = 0;
  bool has_camera = false;
  float camera[6] = {};  // target x y z, yaw, pitch, distance
  std::vector<Shape> statics, props;
  std::vector<Dirt> dirt;
  std::vector<std::pair<int, std::string>> steps;
  std::vector<Vec3> tcp;      // tool center point of every frame
  std::vector<char> contact;  // the tool touches the work surface

  bool Load(const std::string& file, const MotionClip& clip, std::string& log) {
    std::ifstream in(file);
    if (!in) {
      log += "no scene file " + file + "\n";
      return false;
    }
    std::string line, tcp_bone = "TCP";
    while (std::getline(in, line)) {
      std::istringstream ss(line);
      std::string key;
      if (!(ss >> key) || key[0] == '#') { continue; }
      Shape sh;
      if (key == "tool") {
        ss >> tool >> tool_length;
      } else if (key == "tcp") {
        ss >> tcp_bone;
      } else if (key == "surface") {
        ss >> surface;
      } else if (key == "camera") {
        for (float& v : camera) { ss >> v; }
        has_camera = static_cast<bool>(ss);
      } else if (key == "box") {
        ss >> sh.p[0] >> sh.p[1] >> sh.p[2] >> sh.s[0] >> sh.s[1] >> sh.s[2] >> sh.c[0] >> sh.c[1] >> sh.c[2];
        statics.push_back(sh);
      } else if (key == "cylinder") {
        sh.kind = Shape::kCylinder;
        ss >> sh.p[0] >> sh.p[1] >> sh.p[2] >> sh.s[0] >> sh.s[1] >> sh.c[0] >> sh.c[1] >> sh.c[2];
        statics.push_back(sh);
      } else if (key == "prop") {
        std::string bone, kind;
        ss >> bone >> kind;
        sh.ibone = clip.FindBone({bone.c_str()});
        sh.kind = kind == "screw" ? Shape::kScrew : (kind == "cylinder" ? Shape::kCylinder : Shape::kBox);
        const int n = sh.kind == Shape::kCylinder ? 2 : 3;
        for (int k = 0; k < n; ++k) { ss >> sh.s[k]; }
        ss >> sh.c[0] >> sh.c[1] >> sh.c[2];
        if (sh.ibone >= 0) { props.push_back(sh); }
      } else if (key == "dirt") {
        Dirt d;
        ss >> d.x >> d.y >> d.z >> d.r >> d.seconds;
        d.seconds = std::max(d.seconds, 1e-3);
        dirt.push_back(d);
      } else if (key == "step") {
        int f = 0;
        std::string text;
        ss >> f >> std::ws;
        std::getline(ss, text);
        steps.emplace_back(f, text);
      }
    }
    for (const char* n : {"Base", "Turret", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3", "Tool", "FingerL", "FingerR"}) {
      ib_.push_back(clip.FindBone({n}));
    }
    ib_tcp_ = clip.FindBone({tcp_bone.c_str()});
    if (ib_tcp_ < 0) { ib_tcp_ = ib_[7]; }
    // where the tool is in every frame, and how long each dirt spot has been wiped so far
    tcp.resize(clip.nframes);
    contact.assign(clip.nframes, 0);
    wiped_.assign(static_cast<size_t>(clip.nframes) * dirt.size(), 0.f);
    std::vector<float> sum(dirt.size(), 0.f);
    std::vector<dfm2::CRigBone> pose;
    for (int f = 0; f < clip.nframes; ++f) {
      clip.Pose(f * clip.frame_time, pose);
      const dfm2::CRigBone& b = pose[ib_tcp_];
      tcp[f] = BonePos(b);
      contact[f] = tool == "sponge" && tcp[f].y < surface + 0.6;
      if (contact[f]) {
        // sponge footprint: 10 cm along the tool x axis, 6 cm along its z axis
        const double* m = b.affmat3Global;
        for (size_t i = 0; i < dirt.size(); ++i) {
          const double dx = dirt[i].x - tcp[f].x, dz = dirt[i].z - tcp[f].z;
          const double u = dx * m[0] + dz * m[8], v = dx * m[2] + dz * m[10];
          if (std::fabs(u) < 5.0 + 0.5 * dirt[i].r && std::fabs(v) < 3.0 + 0.5 * dirt[i].r) {
            sum[i] += static_cast<float>(clip.frame_time);
          }
        }
      }
      std::copy(sum.begin(), sum.end(), wiped_.begin() + static_cast<long>(static_cast<size_t>(f) * dirt.size()));
    }
    return true;
  }

  [[nodiscard]] const char* StepAt(int frame) const {
    const char* s = "";
    for (const auto& st : steps) {
      if (st.first <= frame) { s = st.second.c_str(); }
    }
    return s;
  }

  // share of the dirt that is gone at a frame
  [[nodiscard]] double Cleaned(int frame) const {
    if (dirt.empty()) { return 0.0; }
    double total = 0, gone = 0;
    for (size_t i = 0; i < dirt.size(); ++i) {
      const double area = dirt[i].r * dirt[i].r;
      total += area;
      gone += area * std::min(1.0, Wiped(frame, i) / dirt[i].seconds);
    }
    return gone / total;
  }

  void DrawStatic() const {
    for (const Shape& s : statics) {
      ::glColor3fv(s.c);
      if (s.kind == Shape::kBox) {
        gl_util::Box(s.p[0], s.p[1], s.p[2], s.s[0] * 0.5, s.s[1] * 0.5, s.s[2] * 0.5);
      } else {
        gl_util::Cylinder(s.p[0], s.p[1], s.p[2], s.p[0], s.p[1] + s.s[1], s.p[2], s.s[0], 24);
      }
    }
  }

  // the visible part of the work surface, marked in the stencil buffer as the shadow receiver
  void DrawReceiver() const {
    ::glEnable(GL_STENCIL_TEST);
    ::glStencilFunc(GL_ALWAYS, 1, ~0u);
    ::glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    ::glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    ::glDepthMask(GL_FALSE);
    ::glBegin(GL_QUADS);
    for (const Shape& s : statics) {
      if (s.kind != Shape::kBox || std::fabs(s.p[1] + s.s[1] * 0.5 - surface) > 0.05) { continue; }
      const double x0 = s.p[0] - s.s[0] * 0.5, x1 = s.p[0] + s.s[0] * 0.5;
      const double z0 = s.p[2] - s.s[2] * 0.5, z1 = s.p[2] + s.s[2] * 0.5;
      ::glVertex3d(x0, surface, z0);
      ::glVertex3d(x0, surface, z1);
      ::glVertex3d(x1, surface, z1);
      ::glVertex3d(x1, surface, z0);
    }
    ::glEnd();
    ::glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    ::glDepthMask(GL_TRUE);
    ::glDisable(GL_STENCIL_TEST);
  }

  // dirt still on the table, and the wet streak the sponge left in the last seconds
  void DrawDirt(int frame) const {
    ::glDisable(GL_LIGHTING);
    ::glEnable(GL_POLYGON_OFFSET_FILL);
    ::glPolygonOffset(-1.f, -2.f);
    ::glDepthMask(GL_FALSE);
    const int nf = static_cast<int>(tcp.size());
    Vec3 last(1e9, 0, 0);
    for (int f = std::max(0, frame - 240); f <= frame && f < nf; ++f) {
      if (!contact[f] || std::hypot(tcp[f].x - last.x, tcp[f].z - last.z) < 1.0) { continue; }  // even, also where it pauses
      last = tcp[f];
      const float age = static_cast<float>(frame - f) / 240.f;
      ::glColor4f(0.30f, 0.34f, 0.38f, 0.10f * (1.f - age));
      gl_util::Disc(tcp[f].x, surface + 0.02, tcp[f].z, 4.0, 14);
    }
    for (size_t i = 0; i < dirt.size(); ++i) {
      const double left = 1.0 - std::min(1.0, Wiped(frame, i) / dirt[i].seconds);
      if (left <= 0.0) { continue; }
      ::glColor4f(0.33f, 0.20f, 0.09f, static_cast<float>(0.92 * left));
      gl_util::Disc(dirt[i].x, dirt[i].y, dirt[i].z, dirt[i].r * (0.75 + 0.25 * left), 16);
    }
    ::glDepthMask(GL_TRUE);
    ::glDisable(GL_POLYGON_OFFSET_FILL);
  }

  void DrawTcpPath(int frame) const {
    ::glDisable(GL_LIGHTING);
    ::glLineWidth(2.f);
    ::glBegin(GL_LINE_STRIP);
    for (int f = std::max(0, frame - 180); f <= frame && f < static_cast<int>(tcp.size()); ++f) {
      ::glColor4f(0.1f, 0.6f, 0.9f, 0.15f + 0.8f * static_cast<float>(f - frame + 180) / 180.f);
      ::glVertex3d(tcp[f].x, tcp[f].y, tcp[f].z);
    }
    ::glEnd();
  }

  // the arm and its tool, drawn in the frames of the posed bones
  void DrawRobot(const std::vector<dfm2::CRigBone>& bones) const {
    const float orange[3] = {0.93f, 0.47f, 0.10f}, dark[3] = {0.20f, 0.21f, 0.24f}, steel[3] = {0.70f, 0.72f, 0.75f};
    auto in = [&](int k, auto&& draw) {
      if (k >= static_cast<int>(ib_.size()) || ib_[k] < 0) { return; }
      ::glPushMatrix();
      MultBone(bones[ib_[k]]);
      draw();
      ::glPopMatrix();
    };
    auto cyl_y = [](double y0, double y1, double r, const float* c) {
      ::glColor3fv(c);
      gl_util::Cylinder(0, y0, 0, 0, y1, 0, r, 24);
    };
    auto cyl_x = [](double y, double half, double r, const float* c) {
      ::glColor3fv(c);
      gl_util::Cylinder(-half, y, 0, half, y, 0, r, 24);
    };
    in(0, [&] {  // base on its mounting plate
      ::glColor3fv(dark);
      gl_util::Box(0, 0.6, 0, 13, 0.6, 13);
      cyl_y(0, 9, 9.5, dark);
    });
    in(1, [&] {  // turret + shoulder motor
      cyl_y(0, 8, 7.6, orange);
      cyl_x(8, 8.5, 6.8, dark);
    });
    in(2, [&] {  // upper arm
      cyl_y(0, 42.5, 5.2, orange);
      cyl_x(42.5, 7.0, 5.8, dark);
    });
    in(3, [&] { cyl_y(0, 39.2, 4.3, orange); });  // forearm
    in(4, [&] { cyl_y(0, 6.0, 4.1, dark); });     // wrist 1
    in(5, [&] { cyl_x(0, 5.0, 4.1, orange); });   // wrist 2
    in(6, [&] {                                   // wrist 3 + flange (+ screwdriver body, it does not spin)
      cyl_y(0, 7.2, 3.7, dark);
      cyl_y(7.2, 8.0, 4.4, steel);
      if (tool == "screwdriver") {
        const float body[3] = {0.22f, 0.30f, 0.42f};
        cyl_y(8.0, 18.0, 3.3, body);
        ::glColor3fv(dark);
        gl_util::Box(4.6, 13.5, 0, 1.6, 3.6, 2.0);
      }
    });
    in(7, [&] {  // tool, turning with the spindle
      if (tool == "gripper") {
        ::glColor3fv(dark);
        gl_util::Box(0, 4.6, 0, 5.5, 4.0, 2.8);
        ::glColor3fv(steel);
        gl_util::Box(0, 8.9, 0, 6.6, 0.4, 1.5);
      } else if (tool == "sponge") {
        cyl_y(0, 4.0, 2.2, steel);
        ::glColor3fv(dark);
        gl_util::Box(0, 4.5, 0, 4.6, 0.5, 2.6);
        ::glColor3f(0.96f, 0.82f, 0.30f);
        gl_util::Box(0, 6.6, 0, 5.0, 1.6, 3.0);
        ::glColor3f(0.20f, 0.52f, 0.30f);
        gl_util::Box(0, 8.6, 0, 5.0, 0.4, 3.0);
      } else if (tool == "screwdriver") {
        cyl_y(10.0, 12.0, 1.1, steel);
        const float bit[3] = {0.50f, 0.52f, 0.56f};
        cyl_y(12.0, 17.6, 0.32, bit);
        ::glColor3fv(bit);
        gl_util::Box(0, 17.8, 0, 0.45, 0.2, 0.12);
      }
    });
    for (int k : {8, 9}) {  // gripper fingers
      in(k, [&] {
        ::glColor3fv(steel);
        gl_util::Box(0, 2.5, 0, 0.6, 2.5, 1.3);
      });
    }
  }

  void DrawProps(const std::vector<dfm2::CRigBone>& bones) const {
    for (const Shape& s : props) {
      ::glPushMatrix();
      MultBone(bones[s.ibone]);
      ::glColor3fv(s.c);
      if (s.kind == Shape::kBox) {
        gl_util::Box(0, 0, 0, s.s[0] * 0.5, s.s[1] * 0.5, s.s[2] * 0.5);
      } else if (s.kind == Shape::kCylinder) {
        gl_util::Cylinder(0, 0, 0, 0, s.s[1], 0, s.s[0], 20);
      } else {  // screw: head with a cross slot, and the shaft
        gl_util::Cylinder(0, 0, 0, 0, s.s[1], 0, s.s[0], 20);
        gl_util::Cylinder(0, 0, 0, 0, -s.s[2], 0, s.s[0] * 0.4, 12);
        ::glColor3f(0.15f, 0.15f, 0.17f);
        gl_util::Box(0, s.s[1], 0, s.s[0] * 0.8, 0.06, 0.12);
        gl_util::Box(0, s.s[1], 0, 0.12, 0.06, s.s[0] * 0.8);
      }
      ::glPopMatrix();
    }
  }

 private:
  std::vector<int> ib_;
  int ib_tcp_ = -1;
  std::vector<float> wiped_;  // nframes * dirt, seconds of wiping up to the frame

  [[nodiscard]] double Wiped(int frame, size_t i) const {
    if (tcp.empty()) { return 0.0; }
    frame = std::clamp(frame, 0, static_cast<int>(tcp.size()) - 1);
    return wiped_[static_cast<size_t>(frame) * dirt.size() + i];
  }

  static void MultBone(const dfm2::CRigBone& b) {
    double m[16];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) { m[j * 4 + i] = b.affmat3Global[i * 4 + j]; }
    }
    ::glMultMatrixd(m);
  }
};

}  // namespace viewer

#endif
