/*
 *  Yifei Chen
 *  Motion Viewer - drawing people
 *
 *  A capsule mannequin (the radius of each body part comes from the bone names, so common
 *  skeletons work), a walking stick, footprints, and a treadmill belt for the in-place view.
 */
#ifndef MM_MANNEQUIN_H
#define MM_MANNEQUIN_H

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "motion_clip.h"
#include "render_util.h"

namespace viewer {

inline bool Has(const std::string& s, const char* key) { return s.find(key) != std::string::npos; }

// radius (cm) of the body part from a joint to its child; 0 = the head, drawn as a sphere
inline double PartRadius(const std::string& joint, const std::string& child, double length) {
  if (Has(joint, "Head")) { return 0.0; }
  for (const char* k : {"Finger", "Thumb", "Index", "Middle", "Ring", "Pinky"}) {
    if (Has(joint, k)) { return 1.0; }
  }
  if (Has(joint, "Hips") || Has(joint, "Pelvis")) {
    return Has(child, "Hip") || Has(child, "UpLeg") || Has(child, "Thigh") ? 6.4 : 10.5;
  }
  if (Has(joint, "Spine") || Has(joint, "Chest")) {
    return Has(child, "Collar") || Has(child, "Shoulder") || Has(child, "Clavicle") ? 5.0 : 10.5;
  }
  if (Has(joint, "Neck") || Has(joint, "Collar") || Has(joint, "Clavicle")) { return 4.6; }
  if (Has(joint, "ForeArm") || Has(joint, "Elbow") || Has(joint, "LowerArm")) { return 3.6; }
  if (Has(joint, "Wrist") || Has(joint, "Hand")) { return 2.8; }
  if (Has(joint, "UpLeg") || Has(joint, "Thigh") || Has(joint, "UpperLeg") || Has(joint, "Hip")) { return 6.3; }
  if (Has(joint, "Knee") || Has(joint, "Leg") || Has(joint, "Shin")) { return 4.8; }
  if (Has(joint, "Toe")) { return kToeRadius; }
  if (Has(joint, "Shoulder") || Has(joint, "Arm")) { return 4.4; }
  return std::clamp(length * 0.12, 1.2, 5.0);
}

inline void Capsule(const Vec3& a, const Vec3& b, double r) {
  gl_util::Cylinder(a.x, a.y, a.z, b.x, b.y, b.z, r, 18);
  gl_util::Sphere(a.x, a.y, a.z, r);
  gl_util::Sphere(b.x, b.y, b.z, r);
}

// posed skeleton as a mannequin, in the current color
inline void DrawMannequin(const MotionClip& clip, const std::vector<dfm2::CRigBone>& bones) {
  for (size_t i = 0; i < bones.size(); ++i) {
    const int p = bones[i].ibone_parent;
    if (p < 0) { continue; }
    const Vec3 a = BonePos(bones[p]), c = BonePos(bones[i]);
    const double len = (c - a).Length();
    if (len < 1e-6) { continue; }
    const int side = p == clip.ib_foot[0] ? 0 : (p == clip.ib_foot[1] ? 1 : -1);
    if (side >= 0 && static_cast<int>(i) == clip.ib_toe[side]) {  // foot with a heel
      const Vec3 heel = BonePoint(bones[p], clip.heel_local[side]);
      Capsule(a, heel, kSoleRadius + 0.5);
      Capsule(heel, c, kSoleRadius);
      continue;
    }
    const std::string& joint = bones[p].name;
    const double r = PartRadius(joint, bones[i].name, len);
    if (r == 0.0) {
      const Vec3 h = a + (c - a) * 0.45;
      gl_util::Sphere(h.x, h.y, h.z, std::clamp(len * 0.62, 8.0, 12.0));
      continue;
    }
    Vec3 e = c;
    if (bones[i].name == "Site" && (Has(joint, "Wrist") || Has(joint, "Hand"))) {  // a hand, not the finger tips
      e = a + (c - a) * std::min(1.0, 9.0 / len);
    }
    Capsule(a, e, r);
  }
}

// walking stick from the hand to its tip, with a handle pointing forward
inline void DrawCane(const Vec3& grip, const Vec3& tip, double yaw) {
  ::glColor3f(0.32f, 0.20f, 0.11f);
  gl_util::Cylinder(tip.x, tip.y + 1.5, tip.z, grip.x, grip.y, grip.z, 1.1, 14);
  const double fx = std::sin(yaw), fz = std::cos(yaw);
  gl_util::Cylinder(grip.x - fx * 3.0, grip.y + 1.2, grip.z - fz * 3.0, grip.x + fx * 7.0, grip.y + 1.2, grip.z + fz * 7.0, 1.3, 14);
  ::glColor3f(0.08f, 0.08f, 0.09f);
  gl_util::Cylinder(tip.x, tip.y, tip.z, tip.x, tip.y + 2.0, tip.z, 1.5, 14);
}

// sole print on the floor; left feet blue, right feet red
inline void DrawFootprint(const Vec3& p, double yaw, int foot, float alpha) {
  const double fx = std::sin(yaw), fz = std::cos(yaw);
  if (foot == 0) {
    ::glColor4f(0.20f, 0.42f, 0.85f, alpha);
  } else {
    ::glColor4f(0.88f, 0.32f, 0.22f, alpha);
  }
  ::glBegin(GL_TRIANGLE_FAN);
  ::glVertex3d(p.x, 0.4, p.z);
  for (int i = 0; i <= 24; ++i) {
    const double a = 2 * mm::kPi * i / 24;
    const double l = std::cos(a) * 12.0, w = std::sin(a) * (std::cos(a) > 0 ? 4.6 : 3.8);  // wider at the toes
    ::glVertex3d(p.x + fx * l + fz * w, 0.4, p.z + fz * l - fx * w);
  }
  ::glEnd();
}

// treadmill under a character walking in place (facing +z); the stripes move with the distance walked
inline void DrawTreadmill(double cx, double cz, double walked) {
  const double hw = 38.0, hl = 105.0;
  ::glColor3f(0.24f, 0.25f, 0.28f);
  gl_util::Box(cx - hw - 3.0, 2.5, cz, 3.0, 2.5, hl + 4.0);
  gl_util::Box(cx + hw + 3.0, 2.5, cz, 3.0, 2.5, hl + 4.0);
  ::glDisable(GL_LIGHTING);
  ::glEnable(GL_POLYGON_OFFSET_FILL);
  ::glPolygonOffset(-1.f, -1.f);
  ::glColor3f(0.14f, 0.15f, 0.16f);
  ::glBegin(GL_QUADS);
  ::glVertex3d(cx - hw, 0.15, cz - hl);
  ::glVertex3d(cx - hw, 0.15, cz + hl);
  ::glVertex3d(cx + hw, 0.15, cz + hl);
  ::glVertex3d(cx + hw, 0.15, cz - hl);
  ::glEnd();
  ::glPolygonOffset(-2.f, -2.f);
  ::glColor3f(0.30f, 0.31f, 0.33f);
  const double gap = 25.0;
  double shift = std::fmod(walked, gap);
  if (shift < 0) { shift += gap; }
  ::glBegin(GL_QUADS);
  for (double z = -hl - shift; z < hl; z += gap) {
    const double z0 = std::max(z, -hl), z1 = std::min(z + 3.0, hl);
    if (z1 <= z0) { continue; }
    ::glVertex3d(cx - hw, 0.2, cz + z0);
    ::glVertex3d(cx - hw, 0.2, cz + z1);
    ::glVertex3d(cx + hw, 0.2, cz + z1);
    ::glVertex3d(cx + hw, 0.2, cz + z0);
  }
  ::glEnd();
  ::glDisable(GL_POLYGON_OFFSET_FILL);
}

}  // namespace viewer

#endif
