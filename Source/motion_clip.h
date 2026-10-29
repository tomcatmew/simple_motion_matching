/*
 *  Yifei Chen
 *  Motion Viewer - one BVH clip
 *
 *  Loads a clip and poses it at any time (neighbouring frames are interpolated). For human
 *  clips it also extracts
 *    - a root on the floor (smoothed hips, facing from shoulders + hips), for the treadmill view
 *    - foot contacts and the usual gait measures: speed, cadence, step length / width, double support
 *    - the tip of a walking stick held in one hand, planted on the floor like a real cane
 */
#ifndef MM_MOTION_CLIP_H
#define MM_MOTION_CLIP_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

#include "motion_matching.h"  // small math, delfem2 BVH reader

namespace viewer {

namespace dfm2 = delfem2;
using mm::Vec3;

struct FootStep {
  int foot = 0;            // 0 left, 1 right
  int start = 0, end = 0;  // frames of the contact (end exclusive)
  Vec3 pos;                // middle of the foot on the floor
  double yaw = 0;          // heel -> toe direction
};

struct GaitStats {
  bool walking = false;       // enough walking in the clip to measure
  double speed = 0;           // cm/s while walking
  double cadence = 0;         // steps / min
  double step_length = 0;     // cm, along the walking direction
  double step_width = 0;      // cm, sideways between the feet
  double double_support = 0;  // % of the walking time with both feet down
  int steps = 0;
};

// rigid motion in the floor plane: p -> Ry(-yaw) (p - origin) + offset
struct Placement {
  double ox = 0, oz = 0, yaw = 0;
  double tx = 0, ty = 0, tz = 0;
  [[nodiscard]] Vec3 Apply(const Vec3& p) const {
    double lx, lz;
    mm::WorldToLocal(yaw, p.x - ox, p.z - oz, lx, lz);
    return {lx + tx, p.y + ty, lz + tz};
  }
  [[nodiscard]] double Yaw(double world_yaw) const { return world_yaw - yaw; }
};

inline Vec3 BonePos(const dfm2::CRigBone& b) { return {b.affmat3Global[3], b.affmat3Global[7], b.affmat3Global[11]}; }
// a point given in the frame of a posed bone
inline Vec3 BonePoint(const dfm2::CRigBone& b, const Vec3& local) {
  const double* m = b.affmat3Global;
  return {m[0] * local.x + m[1] * local.y + m[2] * local.z + m[3], m[4] * local.x + m[5] * local.y + m[6] * local.z + m[7],
          m[8] * local.x + m[9] * local.y + m[10] * local.z + m[11]};
}

// mannequin foot: capsules ankle -> heel -> toe -> toe tip (cm); the floor is found from them
constexpr double kSoleRadius = 2.6, kToeRadius = 2.3;

class MotionClip {
 public:
  std::string name, path;
  std::vector<dfm2::CRigBone> bind;  // skeleton in the bind pose
  std::vector<dfm2::CChannel_BioVisionHierarchy> channels;
  std::vector<double> values;  // nframes * channels
  int nframes = 0;
  double frame_time = 1.0 / 60.0;

  // human clips only
  bool human = false;
  int ib_lshoulder = -1, ib_rshoulder = -1, ib_lhip = -1, ib_rhip = -1;
  int ib_foot[2] = {-1, -1}, ib_toe[2] = {-1, -1}, ib_toe_end[2] = {-1, -1};
  Vec3 heel_local[2];                          // heel in the frame of the ankle
  double floor = 0.0;                          // add -floor to stand on y = 0
  std::vector<Vec3> joints;                    // nframes * nbones, global joint positions
  std::vector<double> root_x, root_z, root_yaw, travel;  // root on the floor, distance walked
  std::vector<char> contact[2];                // per frame, foot on the floor
  std::vector<FootStep> steps;
  GaitStats gait;
  int cane_hand = -1;                          // bone holding a walking stick
  std::vector<Vec3> cane_grip, cane_tip;

  [[nodiscard]] int NumBones() const { return static_cast<int>(bind.size()); }
  [[nodiscard]] double Duration() const { return std::max(0, nframes - 1) * frame_time; }
  [[nodiscard]] int FrameAt(double t) const {
    return std::clamp(static_cast<int>(std::floor(t / frame_time + 1e-6)), 0, std::max(0, nframes - 1));
  }
  [[nodiscard]] const Vec3& Joint(int f, int b) const { return joints[static_cast<size_t>(f) * bind.size() + b]; }

  [[nodiscard]] int FindBone(std::initializer_list<const char*> names) const {
    for (const char* n : names) {
      for (size_t i = 0; i < bind.size(); ++i) {
        std::string bn = bind[i].name;
        const size_t colon = bn.find_last_of(':');  // mixamorig:LeftArm
        if (colon != std::string::npos) { bn = bn.substr(colon + 1); }
        if (bn == n) { return static_cast<int>(i); }
      }
    }
    return -1;
  }
  // the End Site under a bone (delfem2 names them "Site")
  [[nodiscard]] int EndSiteOf(int ib) const {
    for (size_t i = 0; i < bind.size(); ++i) {
      if (bind[i].ibone_parent == ib && bind[i].name == "Site") { return static_cast<int>(i); }
    }
    return -1;
  }

  /**
   * @param scale  multiply all lengths (e.g. 5.644 for the CMU / PFNN skeleton -> cm)
   * @param cane   bone name of the hand holding a walking stick, or nullptr
   */
  bool Load(const std::string& file, const std::string& clip_name, double scale, const char* cane, std::string& log) {
    path = file;
    name = clip_name;
    size_t nf = 0;
    std::string header;
    dfm2::Read_BioVisionHierarchy(bind, channels, nf, frame_time, values, header, file);
    if (nf == 0 || bind.empty() || values.size() != nf * channels.size()) {
      log += "cannot read " + file + "\n";
      return false;
    }
    nframes = static_cast<int>(nf);
    frame_time = std::max(frame_time, 1e-4);
    if (scale != 1.0) { Scale(scale); }
    ib_lshoulder = FindBone({"LeftArm", "LeftUpperArm", "L_UpperArm", "LeftShoulder"});
    ib_rshoulder = FindBone({"RightArm", "RightUpperArm", "R_UpperArm", "RightShoulder"});
    ib_lhip = FindBone({"LeftUpLeg", "LeftUpperLeg", "LeftThigh", "L_Thigh", "LeftHip"});
    ib_rhip = FindBone({"RightUpLeg", "RightUpperLeg", "RightThigh", "R_Thigh", "RightHip"});
    ib_foot[0] = FindBone({"LeftFoot", "LeftAnkle", "L_Foot"});
    ib_foot[1] = FindBone({"RightFoot", "RightAnkle", "R_Foot"});
    ib_toe[0] = FindBone({"LeftToe", "LeftToeBase", "LeftToes", "L_Toe"});
    ib_toe[1] = FindBone({"RightToe", "RightToeBase", "RightToes", "R_Toe"});
    const int found[6] = {ib_lshoulder, ib_rshoulder, ib_lhip, ib_rhip, ib_foot[0], ib_foot[1]};
    human = std::all_of(found, found + 6, [](int i) { return i >= 0; });
    if (!human) { return true; }
    for (int s = 0; s < 2; ++s) {
      if (ib_toe[s] < 0) { ib_toe[s] = ib_foot[s]; }
      ib_toe_end[s] = EndSiteOf(ib_toe[s]);
      if (ib_toe_end[s] < 0) { ib_toe_end[s] = ib_toe[s]; }
      // the bind pose has no rotations, so offsets there are offsets in the ankle frame:
      // the heel is under the ankle, a bit behind it, level with the toe tip
      const Vec3 ankle = BonePos(bind[ib_foot[s]]);
      const Vec3 toe = BonePos(bind[ib_toe[s]]) - ankle, tip = BonePos(bind[ib_toe_end[s]]) - ankle;
      heel_local[s] = Vec3(-0.25 * toe.x, tip.y + kSoleRadius - kToeRadius, -0.25 * toe.z);
    }
    if (cane) { cane_hand = FindBone({cane}); }
    Analyze();
    return true;
  }

  // pose at time t (seconds), interpolating between frames
  void Pose(double t, std::vector<dfm2::CRigBone>& out) const {
    const double ft = std::clamp(t / frame_time, 0.0, static_cast<double>(nframes - 1));
    const int f0 = static_cast<int>(std::floor(ft));
    const int f1 = std::min(f0 + 1, nframes - 1);
    const double a = ft - f0;
    out = bind;
    dfm2::SetPose_BioVisionHierarchy(out, channels, values.data() + static_cast<size_t>(f0) * channels.size());
    if (f1 == f0 || a < 1e-4) { return; }
    tmp_ = bind;
    dfm2::SetPose_BioVisionHierarchy(tmp_, channels, values.data() + static_cast<size_t>(f1) * channels.size());
    for (size_t b = 0; b < out.size(); ++b) {
      Slerp(out[b].quatRelativeRot, tmp_[b].quatRelativeRot, a);
      for (int k = 0; k < 3; ++k) { out[b].transRelative[k] += (tmp_[b].transRelative[k] - out[b].transRelative[k]) * a; }
    }
    dfm2::UpdateBoneRotTrans(out);
  }

  // clip space -> view: the placement in the floor plane, and the floor moved to y = 0
  [[nodiscard]] Vec3 ToView(const Vec3& p, const Placement& pl) const {
    Vec3 q = pl.Apply(p);
    q.y -= floor;
    return q;
  }

  // move the whole pose (the root bone) by a placement, and onto the floor
  void Place(std::vector<dfm2::CRigBone>& bones, const Placement& pl) const {
    for (auto& b : bones) {
      if (b.ibone_parent >= 0) { continue; }
      const Vec3 q = ToView(Vec3(b.transRelative[0], b.transRelative[1], b.transRelative[2]), pl);
      b.transRelative[0] = q.x;
      b.transRelative[1] = q.y;
      b.transRelative[2] = q.z;
      const mm::Quat r = mm::NormalizeQ(mm::YawQuat(-pl.yaw) * mm::Quat(b.quatRelativeRot));
      r.CopyTo(b.quatRelativeRot);
    }
    dfm2::UpdateBoneRotTrans(bones);
  }

  // root on the floor at time t (interpolated), for the treadmill view and the follow camera
  void Root(double t, double& x, double& z, double& yaw, double& walked) const {
    const double ft = std::clamp(t / frame_time, 0.0, static_cast<double>(nframes - 1));
    const int f0 = static_cast<int>(std::floor(ft));
    const int f1 = std::min(f0 + 1, nframes - 1);
    const double a = ft - f0;
    x = root_x[f0] + (root_x[f1] - root_x[f0]) * a;
    z = root_z[f0] + (root_z[f1] - root_z[f0]) * a;
    yaw = root_yaw[f0] + mm::WrapAngle(root_yaw[f1] - root_yaw[f0]) * a;
    walked = travel[f0] + (travel[f1] - travel[f0]) * a;
  }

 private:
  mutable std::vector<dfm2::CRigBone> tmp_;

  static void Slerp(double* q, const double* r, double a) {
    double d = q[0] * r[0] + q[1] * r[1] + q[2] * r[2] + q[3] * r[3];
    double s = 1.0;
    if (d < 0) { d = -d; s = -1.0; }
    double w0 = 1.0 - a, w1 = a;
    if (d < 0.9995) {
      const double th = std::acos(d);
      w0 = std::sin((1.0 - a) * th) / std::sin(th);
      w1 = std::sin(a * th) / std::sin(th);
    }
    double l = 0;
    for (int k = 0; k < 4; ++k) {
      q[k] = q[k] * w0 + r[k] * s * w1;
      l += q[k] * q[k];
    }
    l = std::sqrt(l);
    for (int k = 0; k < 4; ++k) { q[k] /= l; }
  }

  void Scale(double s) {
    for (auto& b : bind) {
      b.invBindMat[3] *= s;
      b.invBindMat[7] *= s;
      b.invBindMat[11] *= s;
    }
    for (auto& b : bind) {
      if (b.ibone_parent < 0) { continue; }
      const auto& p = bind[b.ibone_parent];
      b.transRelative[0] = -b.invBindMat[3] + p.invBindMat[3];
      b.transRelative[1] = -b.invBindMat[7] + p.invBindMat[7];
      b.transRelative[2] = -b.invBindMat[11] + p.invBindMat[11];
    }
    dfm2::UpdateBoneRotTrans(bind);
    const size_t nch = channels.size();
    for (size_t i = 0; i < values.size(); ++i) {
      if (!channels[i % nch].isrot) { values[i] *= s; }
    }
  }

  static void Smooth(std::vector<Vec3>& v, double sigma) {
    const int n = static_cast<int>(v.size());
    const int r = static_cast<int>(std::ceil(sigma * 3));
    std::vector<Vec3> out(n);
    for (int f = 0; f < n; ++f) {
      Vec3 s;
      double ws = 0;
      for (int i = -r; i <= r; ++i) {
        const double k = std::exp(-0.5 * i * i / (sigma * sigma));
        s = s + v[std::clamp(f + i, 0, n - 1)] * k;
        ws += k;
      }
      out[f] = s * (1.0 / ws);
    }
    v.swap(out);
  }

  static double Percentile(std::vector<double> v, double p) {
    if (v.empty()) { return 0.0; }
    const size_t k = std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())));
    std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
    return v[k];
  }

  void Analyze() {
    const int nb = NumBones();
    // global joint positions of every frame, and the floor: where the soles are when they are lowest
    joints.resize(static_cast<size_t>(nframes) * nb);
    std::vector<dfm2::CRigBone> pose;
    std::vector<double> lows;
    for (int f = 0; f < nframes; ++f) {
      pose = bind;
      dfm2::SetPose_BioVisionHierarchy(pose, channels, values.data() + static_cast<size_t>(f) * channels.size());
      for (int b = 0; b < nb; ++b) { joints[static_cast<size_t>(f) * nb + b] = BonePos(pose[b]); }
      double low = 1e30;
      for (int s = 0; s < 2; ++s) {
        low = std::min({low, BonePoint(pose[ib_foot[s]], heel_local[s]).y - kSoleRadius,
                        BonePos(pose[ib_toe[s]]).y - kSoleRadius, BonePos(pose[ib_toe_end[s]]).y - kToeRadius});
      }
      lows.push_back(low);
    }
    floor = Percentile(lows, 0.02);
    // root: smoothed hips on the floor, facing the smoothed body forward direction
    std::vector<Vec3> rp(nframes), fw(nframes);
    for (int f = 0; f < nframes; ++f) {
      rp[f] = Vec3(Joint(f, 0).x, 0, Joint(f, 0).z);
      const Vec3 across = (Joint(f, ib_lshoulder) - Joint(f, ib_rshoulder)) + (Joint(f, ib_lhip) - Joint(f, ib_rhip));
      fw[f] = Vec3(-across.z, 0, across.x);
      const double l = std::hypot(fw[f].x, fw[f].z);
      fw[f] = l > 1e-9 ? fw[f] * (1.0 / l) : Vec3(0, 0, 1);
    }
    Smooth(rp, 30.0);  // slow enough to leave the sway of each step in the body
    Smooth(fw, 20.0);
    root_x.resize(nframes);
    root_z.resize(nframes);
    root_yaw.resize(nframes);
    travel.assign(nframes, 0.0);
    for (int f = 0; f < nframes; ++f) {
      root_x[f] = rp[f].x;
      root_z[f] = rp[f].z;
      root_yaw[f] = std::atan2(fw[f].x, fw[f].z);
      if (f > 0) {
        double lx, lz;
        mm::WorldToLocal(root_yaw[f - 1], rp[f].x - rp[f - 1].x, rp[f].z - rp[f - 1].z, lx, lz);
        travel[f] = travel[f - 1] + lz;
      }
    }
    FindContacts();
    MeasureGait();
    if (cane_hand >= 0) { PlantCane(); }
  }

  void FindContacts() {
    steps.clear();
    for (int s = 0; s < 2; ++s) {
      std::vector<double> ha, ht;
      for (int f = 0; f < nframes; ++f) {
        ha.push_back(Joint(f, ib_foot[s]).y);
        ht.push_back(Joint(f, ib_toe[s]).y);
      }
      const double rest_a = Percentile(ha, 0.05), rest_t = Percentile(ht, 0.05);
      std::vector<char>& c = contact[s];
      c.assign(nframes, 0);
      for (int f = 0; f < nframes; ++f) {
        const int f0 = std::max(0, f - 1), f1 = std::min(nframes - 1, f + 1);
        auto speed = [&](int ib) {
          const Vec3 d = Joint(f1, ib) - Joint(f0, ib);
          return std::hypot(d.x, d.z) / (std::max(1, f1 - f0) * frame_time);
        };
        // heel down (heel strike, flat foot) or toes down (push off)
        c[f] = (ha[f] < rest_a + 3.0 && speed(ib_foot[s]) < 30.0) || (ht[f] < rest_t + 2.5 && speed(ib_toe[s]) < 30.0);
      }
      std::vector<std::pair<int, int>> touches;  // [start, end) of each contact
      for (int f = 0; f < nframes;) {
        int g = f;
        while (g < nframes && c[g] == c[f]) { ++g; }
        if (c[f]) { touches.emplace_back(f, g); }
        f = g;
      }
      auto where = [&](int f) {
        const Vec3 a = Joint(f, ib_foot[s]), t = Joint(f, ib_toe_end[s]);
        return Vec3((a.x + t.x) * 0.5, floor, (a.z + t.z) * 0.5);
      };
      // touches with a tiny gap, or where the foot did not move, are one contact (shuffling, weight shifts)
      std::vector<std::pair<int, int>> merged;
      for (const auto& t : touches) {
        if (!merged.empty()) {
          const Vec3 d = where((t.first + t.second) / 2) - where((merged.back().first + merged.back().second) / 2);
          if (t.first - merged.back().second <= 3 || std::hypot(d.x, d.z) < 5.0) {
            merged.back().second = t.second;
            continue;
          }
        }
        merged.push_back(t);
      }
      c.assign(nframes, 0);
      for (const auto& t : merged) {
        if (t.second - t.first < 9) { continue; }  // the foot brushed the floor during a swing
        std::fill(c.begin() + t.first, c.begin() + t.second, static_cast<char>(1));
        const int mid = (t.first + t.second) / 2;
        FootStep st;
        st.foot = s;
        st.start = t.first;
        st.end = t.second;
        st.pos = where(mid);
        const Vec3 a = Joint(mid, ib_foot[s]), tip = Joint(mid, ib_toe_end[s]);
        st.yaw = std::atan2(tip.x - a.x, tip.z - a.z);
        steps.push_back(st);
      }
    }
    std::sort(steps.begin(), steps.end(), [](const FootStep& a, const FootStep& b) { return a.start < b.start; });
  }

  void MeasureGait() {
    gait = GaitStats();
    std::vector<char> walking(nframes, 0);
    double speed_sum = 0;
    int nwalk = 0, ndouble = 0;
    for (int f = 0; f < nframes; ++f) {
      const int f0 = std::max(0, f - 6), f1 = std::min(nframes - 1, f + 6);
      const double v = std::hypot(root_x[f1] - root_x[f0], root_z[f1] - root_z[f0]) / (std::max(1, f1 - f0) * frame_time);
      walking[f] = v > 10.0;
      if (!walking[f]) { continue; }
      ++nwalk;
      speed_sum += v;
      ndouble += contact[0][f] && contact[1][f];
    }
    const double walk_time = nwalk * frame_time;
    if (walk_time < 2.0) { return; }
    double len = 0, width = 0;
    int npairs = 0;
    for (size_t i = 1; i < steps.size(); ++i) {
      const FootStep& s = steps[i];
      if (!walking[s.start]) { continue; }
      ++gait.steps;
      for (size_t j = i; j-- > 0;) {  // the last step of the other foot
        if (steps[j].foot == s.foot) { continue; }
        // measured along the way the body moved between the two steps (turns included)
        const double wx = root_x[s.start] - root_x[steps[j].start], wz = root_z[s.start] - root_z[steps[j].start];
        const double wl = std::hypot(wx, wz);
        if ((s.start - steps[j].start) * frame_time < 1.6 && wl > 3.0) {
          const double dx = s.pos.x - steps[j].pos.x, dz = s.pos.z - steps[j].pos.z;
          len += (dx * wx + dz * wz) / wl;
          width += std::fabs(dx * wz - dz * wx) / wl;
          ++npairs;
        }
        break;
      }
    }
    gait.walking = gait.steps >= 4;
    gait.speed = speed_sum / nwalk;
    gait.cadence = gait.steps / walk_time * 60.0;
    gait.step_length = npairs ? len / npairs : 0.0;
    gait.step_width = npairs ? width / npairs : 0.0;
    gait.double_support = 100.0 * ndouble / nwalk;
  }

  // the stick hangs from the hand; its tip is planted when it reaches the floor and stays there
  // until the hand lifts it or leans too far over it
  void PlantCane() {
    const int end = EndSiteOf(cane_hand);
    cane_grip.resize(nframes);
    cane_tip.resize(nframes);
    std::vector<double> heights;
    for (int f = 0; f < nframes; ++f) {
      const Vec3 w = Joint(f, cane_hand);
      cane_grip[f] = end >= 0 ? w + (Joint(f, end) - w) * 0.3 : w;
      heights.push_back(cane_grip[f].y);
    }
    const double floor_y = floor;
    const double length = Percentile(heights, 0.04) - floor_y;
    bool planted = false;
    Vec3 tip = cane_grip[0] - Vec3(0, length, 0);
    for (int f = 0; f < nframes; ++f) {
      const Vec3 g = cane_grip[f];
      const double fx = std::sin(root_yaw[f]), fz = std::cos(root_yaw[f]);
      // hanging slightly forward and away from the body
      double ox = g.x - root_x[f], oz = g.z - root_z[f];
      const double along = ox * fx + oz * fz;
      ox -= fx * along;
      oz -= fz * along;
      const double ol = std::max(1e-6, std::hypot(ox, oz));
      Vec3 hang(g.x + fx * 5.0 + ox / ol * 3.0, g.y - length, g.z + fz * 5.0 + oz / ol * 3.0);
      if (planted) {
        const double lean = std::hypot(g.x - tip.x, g.z - tip.z);
        if (hang.y > floor_y + 2.0 || lean > 30.0) { planted = false; }
      } else if (hang.y <= floor_y + 0.8) {
        planted = true;
        tip = Vec3(hang.x, floor_y, hang.z);
      }
      if (!planted) {
        hang.y = std::max(hang.y, floor_y);
        tip = f == 0 ? hang : tip + (hang - tip) * 0.35;  // swing toward the hanging position
      }
      cane_tip[f] = tip;
    }
  }
};

}  // namespace viewer

#endif
