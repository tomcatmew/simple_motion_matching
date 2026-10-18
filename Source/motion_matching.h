/*
 *  Yifei Chen
 *  Motion Matching Demo - database, search and playback
 *
 *  Feature vector (27 floats, same layout as the original demo)
 *    [ 0.. 5]  future root positions   (x,z) at +20, +40, +60 frames, character space
 *    [ 6..11]  future facing directions (x,z) at +20, +40, +60 frames, character space
 *    [12..14]  left foot position        [15..17] left foot velocity
 *    [18..20]  right foot position       [21..23] right foot velocity
 *    [24..26]  hip velocity
 *
 *  "Root" is a simulation bone on the ground: the smoothed hip position projected on the floor,
 *  facing the smoothed body forward direction (computed from shoulders and hips).
 *  Playback integrates the database root motion, so the character can go anywhere,
 *  and transitions are smoothed with inertialization (offsets that decay with a spring).
 */
#ifndef MM_MOTION_MATCHING_H
#define MM_MOTION_MATCHING_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "delfem2/rig_geo3.h"
#include "delfem2/rig_bvh.h"

namespace mm {

namespace dfm2 = delfem2;

constexpr double kPi = 3.14159265358979323846;
constexpr int kFeatureDim = 27;
constexpr int kTrajCount = 3;
constexpr int kTrajFrames[kTrajCount] = {20, 40, 60};
constexpr double kFps = 60.0;
constexpr double kDt = 1.0 / kFps;

// ------------------------------------------------------------------ small math

struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  [[nodiscard]] double Length() const { return std::sqrt(x * x + y * y + z * z); }
};

struct Quat {  // same memory layout as delfem2: x, y, z, w
  double x = 0, y = 0, z = 0, w = 1;
  Quat() = default;
  Quat(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
  explicit Quat(const double* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}
  void CopyTo(double* p) const { p[0] = x; p[1] = y; p[2] = z; p[3] = w; }
};

inline Quat operator*(const Quat& p, const Quat& q) {
  return {p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
          p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
          p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w,
          p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z};
}
inline Quat Conj(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Quat NormalizeQ(const Quat& q) {
  const double l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return l > 1e-12 ? Quat(q.x / l, q.y / l, q.z / l, q.w / l) : Quat();
}
inline Quat Abs(const Quat& q) { return q.w < 0 ? Quat(-q.x, -q.y, -q.z, -q.w) : q; }
inline Quat YawQuat(double yaw) { return {0, std::sin(yaw * 0.5), 0, std::cos(yaw * 0.5)}; }

// rotation vector (axis * angle) <-> unit quaternion
inline Vec3 Log(const Quat& q_in) {
  const Quat q = Abs(NormalizeQ(q_in));
  const double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (s < 1e-9) { return {2 * q.x, 2 * q.y, 2 * q.z}; }
  const double angle = 2.0 * std::atan2(s, q.w);
  return {q.x / s * angle, q.y / s * angle, q.z / s * angle};
}
inline Quat Exp(const Vec3& v) {
  const double angle = v.Length();
  if (angle < 1e-9) { return NormalizeQ(Quat(v.x * 0.5, v.y * 0.5, v.z * 0.5, 1.0)); }
  const double s = std::sin(angle * 0.5) / angle;
  return {v.x * s, v.y * s, v.z * s, std::cos(angle * 0.5)};
}

inline double WrapAngle(double a) {
  while (a > kPi) { a -= 2 * kPi; }
  while (a < -kPi) { a += 2 * kPi; }
  return a;
}

// world (x,z) -> character space of a root with this yaw (forward = +z)
inline void WorldToLocal(double yaw, double wx, double wz, double& lx, double& lz) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  lx = wx * c - wz * s;
  lz = wx * s + wz * c;
}
inline void LocalToWorld(double yaw, double lx, double lz, double& wx, double& wz) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  wx = lx * c + lz * s;
  wz = -lx * s + lz * c;
}

// critically damped spring that brings an offset back to zero (Holden, "Spring-It-On")
inline void DecaySpring(double& x, double& v, double halflife, double dt) {
  const double y = (4.0 * 0.69314718056) / (halflife + 1e-5) / 2.0;
  const double j1 = v + x * y;
  const double eydt = std::exp(-y * dt);
  x = eydt * (x + j1 * dt);
  v = eydt * (v - j1 * y * dt);
}

// ------------------------------------------------------------------ settings

struct FeatureWeights {
  float traj_pos = 1.0f;
  float traj_dir = 1.5f;
  float foot_pos = 0.75f;
  float foot_vel = 1.0f;
  float hip_vel = 1.0f;
  bool operator!=(const FeatureWeights& o) const {
    return traj_pos != o.traj_pos || traj_dir != o.traj_dir || foot_pos != o.foot_pos ||
           foot_vel != o.foot_vel || hip_vel != o.hip_vel;
  }
};

struct ClipSource {
  std::string path;
  std::string name;
};

// ------------------------------------------------------------------ database

class MotionDatabase {
 public:
  struct Clip {
    std::string name;
    int start = 0;  // first frame (inclusive)
    int end = 0;    // last frame (exclusive)
  };

  // skeleton (bind pose, scaled)
  std::vector<dfm2::CRigBone> bones;
  std::vector<dfm2::CChannel_BioVisionHierarchy> channels;
  int ibone_lfoot = -1, ibone_rfoot = -1, ibone_lshoulder = -1, ibone_rshoulder = -1, ibone_lhip = -1, ibone_rhip = -1;

  // per frame data (all clips concatenated, resampled to 60 fps)
  int nframes = 0;
  std::vector<Clip> clips;
  std::vector<int> clip_of_frame;
  std::vector<Quat> local_rot;   // nframes * nbones; bone 0 is relative to the root yaw
  std::vector<Vec3> hips_pos;    // hips position in root space (includes the height)
  std::vector<double> root_x, root_z, root_yaw;
  std::vector<double> root_dx, root_dz, root_dyaw;  // root motion to the next frame, root space

  // features
  std::vector<float> features_raw;  // nframes * kFeatureDim
  std::vector<float> features;      // normalized and weighted
  std::vector<char> valid;          // usable as a search result (full future inside the clip)
  std::array<float, kFeatureDim> mean{}, scale{}, weight{};
  int nvalid = 0;
  double load_seconds = 0.0;
  double ground_height = 0.0;

  [[nodiscard]] int NumBones() const { return static_cast<int>(bones.size()); }
  [[nodiscard]] bool Empty() const { return nframes == 0; }
  [[nodiscard]] const Quat* Rot(int frame) const { return local_rot.data() + static_cast<size_t>(frame) * bones.size(); }
  [[nodiscard]] const float* Feature(int frame) const { return features.data() + static_cast<size_t>(frame) * kFeatureDim; }
  [[nodiscard]] const float* FeatureRaw(int frame) const { return features_raw.data() + static_cast<size_t>(frame) * kFeatureDim; }
  [[nodiscard]] bool SameClip(int a, int b) const { return clip_of_frame[a] == clip_of_frame[b]; }
  [[nodiscard]] int ClipEnd(int frame) const { return clips[clip_of_frame[frame]].end; }

  /**
   * @param unit_scale  multiply all lengths (the CMU/PFNN skeleton is ~18 units tall at the hips; 5.644 gives cm)
   * @param fallback_bone_index  used when the bone names are not found (original hard coded skeleton)
   */
  bool Load(const std::vector<ClipSource>& sources, double unit_scale, const std::array<int, 6>& fallback_bone_index,
            std::string& log) {
    const auto t0 = std::chrono::high_resolution_clock::now();
    *this = MotionDatabase();
    std::vector<Vec3> hips_w, lfoot_w, rfoot_w, fwd_w;
    std::vector<Quat> hips_rot_w;
    char buf[512];
    for (const ClipSource& src : sources) {
      std::vector<dfm2::CRigBone> b;
      std::vector<dfm2::CChannel_BioVisionHierarchy> ch;
      size_t nf = 0;
      double frame_time = 0;
      std::vector<double> values;
      std::string header;
      dfm2::Read_BioVisionHierarchy(b, ch, nf, frame_time, values, header, src.path);
      if (nf == 0 || b.empty() || values.size() != nf * ch.size()) {
        log += "  cannot read " + src.path + "\n";
        continue;
      }
      if (bones.empty()) {
        bones = b;
        channels = ch;
        ScaleSkeleton(bones, unit_scale);
        ResolveBones(fallback_bone_index, log);
        if (ibone_lfoot < 0) { log += "  skeleton not supported\n"; return false; }
      } else if (b.size() != bones.size() || ch.size() != channels.size()) {
        log += "  skipped " + src.name + " (different skeleton)\n";
        continue;
      }
      // resample to 60 fps
      const int step = std::max(1, static_cast<int>(std::lround(kDt / std::max(frame_time, 1e-4))));
      Clip clip;
      clip.name = src.name;
      clip.start = nframes;
      std::vector<dfm2::CRigBone> pose = bones;
      const size_t nch = channels.size();
      for (size_t f = 0; f < nf; f += step) {
        double* v = values.data() + f * nch;
        for (size_t ich = 0; ich < nch; ++ich) {
          if (!channels[ich].isrot) { v[ich] *= unit_scale; }
        }
        dfm2::SetPose_BioVisionHierarchy(pose, channels, v);
        for (const auto& bone : pose) { local_rot.emplace_back(bone.quatRelativeRot); }
        hips_rot_w.emplace_back(pose[0].quatRelativeRot);
        hips_w.push_back(Pos(pose[0]));
        lfoot_w.push_back(Pos(pose[ibone_lfoot]));
        rfoot_w.push_back(Pos(pose[ibone_rfoot]));
        const Vec3 across = (Pos(pose[ibone_lshoulder]) - Pos(pose[ibone_rshoulder])) +
                            (Pos(pose[ibone_lhip]) - Pos(pose[ibone_rhip]));
        fwd_w.emplace_back(-across.z, 0.0, across.x);  // across x up
        clip_of_frame.push_back(static_cast<int>(clips.size()));
        ++nframes;
      }
      clip.end = nframes;
      if (clip.end - clip.start < kTrajFrames[kTrajCount - 1] + 2) {  // too short to be useful
        log += "  skipped " + src.name + " (too short)\n";
        nframes = clip.start;
        local_rot.resize(static_cast<size_t>(nframes) * bones.size());
        hips_rot_w.resize(nframes); hips_w.resize(nframes); lfoot_w.resize(nframes);
        rfoot_w.resize(nframes); fwd_w.resize(nframes); clip_of_frame.resize(nframes);
        continue;
      }
      clips.push_back(clip);
      std::snprintf(buf, sizeof(buf), "  %-28s %6d frames (%.0f fps -> 60)\n", src.name.c_str(), clip.end - clip.start,
                    1.0 / frame_time);
      log += buf;
    }
    if (nframes == 0) {
      log += "  no motion loaded\n";
      return false;
    }
    // the floor: where the feet are when they are lowest
    {
      std::vector<double> lows;
      lows.reserve(nframes);
      for (int f = 0; f < nframes; ++f) { lows.push_back(std::min(lfoot_w[f].y, rfoot_w[f].y)); }
      std::nth_element(lows.begin(), lows.begin() + nframes / 20, lows.end());
      ground_height = lows[nframes / 20];
    }
    BuildRoot(hips_w, hips_rot_w, fwd_w);
    BuildFeatures(hips_w, lfoot_w, rfoot_w);
    load_seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t0).count();
    std::snprintf(buf, sizeof(buf), "  %d frames, %d searchable, %zu bones, %.1f s\n", nframes, nvalid, bones.size(),
                  load_seconds);
    log += buf;
    return true;
  }

  void Normalize(const FeatureWeights& w) {
    // the original demo shares one deviation per feature group, so that no axis dominates
    const int groups[][2] = {{0, 6}, {6, 12}, {12, 15}, {15, 18}, {18, 21}, {21, 24}, {24, 27}};
    const float gw[] = {w.traj_pos, w.traj_dir, w.foot_pos, w.foot_vel, w.foot_pos, w.foot_vel, w.hip_vel};
    std::array<double, kFeatureDim> sum{}, sum2{};
    int count = 0;
    for (int f = 0; f < nframes; ++f) {
      if (!valid[f]) { continue; }
      const float* r = FeatureRaw(f);
      for (int i = 0; i < kFeatureDim; ++i) { sum[i] += r[i]; sum2[i] += double(r[i]) * r[i]; }
      ++count;
    }
    count = std::max(count, 1);
    std::array<double, kFeatureDim> sd{};
    for (int i = 0; i < kFeatureDim; ++i) {
      mean[i] = static_cast<float>(sum[i] / count);
      sd[i] = std::sqrt(std::max(0.0, sum2[i] / count - double(mean[i]) * mean[i]));
    }
    for (int g = 0; g < 7; ++g) {
      double s = 0;
      for (int i = groups[g][0]; i < groups[g][1]; ++i) { s += sd[i]; }
      s /= (groups[g][1] - groups[g][0]);
      for (int i = groups[g][0]; i < groups[g][1]; ++i) {
        scale[i] = static_cast<float>(std::max(s, 1e-6));
        weight[i] = gw[g];
      }
    }
    features.resize(features_raw.size());
    for (int f = 0; f < nframes; ++f) {
      const float* r = FeatureRaw(f);
      float* o = features.data() + static_cast<size_t>(f) * kFeatureDim;
      for (int i = 0; i < kFeatureDim; ++i) { o[i] = NormalizeValue(i, r[i]); }
    }
  }

  [[nodiscard]] float NormalizeValue(int i, double raw) const {
    return static_cast<float>((raw - mean[i]) / scale[i] * weight[i]);
  }

  [[nodiscard]] float Cost(const float* query, int frame) const {
    const float* f = Feature(frame);
    float c = 0.f;
    for (int i = 0; i < kFeatureDim; ++i) {
      const float d = query[i] - f[i];
      c += d * d;
    }
    return c;
  }

  // brute force nearest neighbour (with early out), ~1 ms for 50k frames
  int Search(const float* query, float& best_cost) const {
    int best = -1;
    best_cost = std::numeric_limits<float>::max();
    for (int f = 0; f < nframes; ++f) {
      if (!valid[f]) { continue; }
      const float* row = Feature(f);
      float c = 0.f;
      int i = 0;
      for (; i < kFeatureDim; ++i) {
        const float d = query[i] - row[i];
        c += d * d;
        if (c >= best_cost) { break; }
      }
      if (i == kFeatureDim && c < best_cost) {
        best_cost = c;
        best = f;
      }
    }
    return best;
  }

 private:
  static Vec3 Pos(const dfm2::CRigBone& b) { return {b.affmat3Global[3], b.affmat3Global[7], b.affmat3Global[11]}; }

  static void ScaleSkeleton(std::vector<dfm2::CRigBone>& bs, double s) {
    for (auto& b : bs) {
      b.invBindMat[3] *= s;
      b.invBindMat[7] *= s;
      b.invBindMat[11] *= s;
    }
    for (auto& b : bs) {
      if (b.ibone_parent < 0) { continue; }
      const auto& p = bs[b.ibone_parent];
      b.transRelative[0] = -b.invBindMat[3] + p.invBindMat[3];
      b.transRelative[1] = -b.invBindMat[7] + p.invBindMat[7];
      b.transRelative[2] = -b.invBindMat[11] + p.invBindMat[11];
    }
    dfm2::UpdateBoneRotTrans(bs);
  }

  [[nodiscard]] int FindBone(std::initializer_list<const char*> names) const {
    for (const char* n : names) {
      for (size_t i = 0; i < bones.size(); ++i) {
        std::string bn = bones[i].name;
        const size_t colon = bn.find_last_of(':');  // mixamorig:LeftArm
        if (colon != std::string::npos) { bn = bn.substr(colon + 1); }
        if (bn == n) { return static_cast<int>(i); }
      }
    }
    return -1;
  }

  void ResolveBones(const std::array<int, 6>& fallback, std::string& log) {
    ibone_lshoulder = FindBone({"LeftArm", "LeftUpperArm", "L_UpperArm", "LeftShoulder"});
    ibone_rshoulder = FindBone({"RightArm", "RightUpperArm", "R_UpperArm", "RightShoulder"});
    ibone_lhip = FindBone({"LeftUpLeg", "LeftUpperLeg", "LeftThigh", "L_Thigh", "LeftHip"});
    ibone_rhip = FindBone({"RightUpLeg", "RightUpperLeg", "RightThigh", "R_Thigh", "RightHip"});
    ibone_lfoot = FindBone({"LeftFoot", "LeftAnkle", "L_Foot"});
    ibone_rfoot = FindBone({"RightFoot", "RightAnkle", "R_Foot"});
    const int found[6] = {ibone_lshoulder, ibone_rshoulder, ibone_lhip, ibone_rhip, ibone_lfoot, ibone_rfoot};
    if (std::all_of(found, found + 6, [](int i) { return i >= 0; })) { return; }
    const int nb = NumBones();
    if (std::all_of(fallback.begin(), fallback.end(), [nb](int i) { return i >= 0 && i < nb; })) {
      ibone_lshoulder = fallback[0];
      ibone_rshoulder = fallback[1];
      ibone_lhip = fallback[2];
      ibone_rhip = fallback[3];
      ibone_lfoot = fallback[4];
      ibone_rfoot = fallback[5];
      log += "  bone names not recognized, using hard coded bone indices\n";
      return;
    }
    ibone_lfoot = -1;
  }

  static void Smooth(std::vector<Vec3>& v, int start, int end, double sigma) {
    const int r = static_cast<int>(std::ceil(sigma * 3));
    std::vector<double> k(2 * r + 1);
    for (int i = -r; i <= r; ++i) { k[i + r] = std::exp(-0.5 * i * i / (sigma * sigma)); }
    std::vector<Vec3> out(end - start);
    for (int f = start; f < end; ++f) {
      Vec3 s;
      double ws = 0;
      for (int i = -r; i <= r; ++i) {
        const int g = std::clamp(f + i, start, end - 1);
        s = s + v[g] * k[i + r];
        ws += k[i + r];
      }
      out[f - start] = s * (1.0 / ws);
    }
    std::copy(out.begin(), out.end(), v.begin() + start);
  }

  void BuildRoot(const std::vector<Vec3>& hips_w, const std::vector<Quat>& hips_rot_w, std::vector<Vec3>& fwd_w) {
    const int nb = NumBones();
    root_x.resize(nframes);
    root_z.resize(nframes);
    root_yaw.resize(nframes);
    std::vector<Vec3> rootp(hips_w);
    for (auto& p : rootp) { p.y = 0; }
    for (auto& d : fwd_w) {
      const double l = std::sqrt(d.x * d.x + d.z * d.z);
      d = l > 1e-9 ? Vec3(d.x / l, 0, d.z / l) : Vec3(0, 0, 1);
    }
    for (const Clip& c : clips) {
      Smooth(rootp, c.start, c.end, 4.0);
      Smooth(fwd_w, c.start, c.end, 8.0);
    }
    for (int f = 0; f < nframes; ++f) {
      root_x[f] = rootp[f].x;
      root_z[f] = rootp[f].z;
      root_yaw[f] = std::atan2(fwd_w[f].x, fwd_w[f].z);
    }
    // sanity check: people mostly walk forward. If not, the left/right names are swapped
    double fwd_motion = 0;
    for (int f = 0; f + 1 < nframes; ++f) {
      if (!SameClipRaw(f, f + 1)) { continue; }
      double lx, lz;
      WorldToLocal(root_yaw[f], root_x[f + 1] - root_x[f], root_z[f + 1] - root_z[f], lx, lz);
      fwd_motion += lz;
    }
    if (fwd_motion < 0) {
      for (auto& y : root_yaw) { y = WrapAngle(y + kPi); }
    }
    hips_pos.resize(nframes);
    root_dx.assign(nframes, 0.0);
    root_dz.assign(nframes, 0.0);
    root_dyaw.assign(nframes, 0.0);
    for (int f = 0; f < nframes; ++f) {
      const double yaw = root_yaw[f];
      // hips relative to the root
      double lx, lz;
      WorldToLocal(yaw, hips_w[f].x - root_x[f], hips_w[f].z - root_z[f], lx, lz);
      hips_pos[f] = Vec3(lx, hips_w[f].y, lz);
      local_rot[static_cast<size_t>(f) * nb] = Abs(NormalizeQ(Conj(YawQuat(yaw)) * hips_rot_w[f]));
      // root motion to the next frame (last frame of a clip: reuse the previous one)
      int a = f, b = f + 1;
      if (b >= nframes || !SameClipRaw(a, b)) { a = f - 1; b = f; }
      if (a >= 0 && SameClipRaw(a, b)) {
        WorldToLocal(root_yaw[a], root_x[b] - root_x[a], root_z[b] - root_z[a], lx, lz);
        root_dx[f] = lx;
        root_dz[f] = lz;
        root_dyaw[f] = WrapAngle(root_yaw[b] - root_yaw[a]);
      }
    }
    // make all local rotations continuous (same hemisphere as the previous frame)
    for (int f = 1; f < nframes; ++f) {
      if (!SameClipRaw(f - 1, f)) { continue; }
      for (int b = 0; b < nb; ++b) {
        Quat& q = local_rot[static_cast<size_t>(f) * nb + b];
        const Quat& p = local_rot[static_cast<size_t>(f - 1) * nb + b];
        if (q.x * p.x + q.y * p.y + q.z * p.z + q.w * p.w < 0) { q = Quat(-q.x, -q.y, -q.z, -q.w); }
      }
    }
  }

  [[nodiscard]] bool SameClipRaw(int a, int b) const { return clip_of_frame[a] == clip_of_frame[b]; }

  void BuildFeatures(const std::vector<Vec3>& hips_w, const std::vector<Vec3>& lfoot_w, const std::vector<Vec3>& rfoot_w) {
    features_raw.assign(static_cast<size_t>(nframes) * kFeatureDim, 0.f);
    valid.assign(nframes, 0);
    nvalid = 0;
    for (const Clip& c : clips) {
      for (int f = c.start; f < c.end; ++f) {
        float* o = features_raw.data() + static_cast<size_t>(f) * kFeatureDim;
        const double yaw = root_yaw[f];
        const double rx = root_x[f], rz = root_z[f];
        for (int k = 0; k < kTrajCount; ++k) {
          const int g = std::min(f + kTrajFrames[k], c.end - 1);
          double lx, lz;
          WorldToLocal(yaw, root_x[g] - rx, root_z[g] - rz, lx, lz);
          o[k * 2 + 0] = static_cast<float>(lx);
          o[k * 2 + 1] = static_cast<float>(lz);
          WorldToLocal(yaw, std::sin(root_yaw[g]), std::cos(root_yaw[g]), lx, lz);
          o[6 + k * 2 + 0] = static_cast<float>(lx);
          o[6 + k * 2 + 1] = static_cast<float>(lz);
        }
        const int n = std::min(f + 1, c.end - 1);
        const int p = n == f ? f - 1 : f;
        auto local3 = [&](const Vec3& v, float* out) {
          double lx, lz;
          WorldToLocal(yaw, v.x, v.z, lx, lz);
          out[0] = static_cast<float>(lx);
          out[1] = static_cast<float>(v.y);
          out[2] = static_cast<float>(lz);
        };
        local3(lfoot_w[f] - Vec3(rx, ground_height, rz), o + 12);
        local3((lfoot_w[n] - lfoot_w[p]) * kFps, o + 15);
        local3(rfoot_w[f] - Vec3(rx, ground_height, rz), o + 18);
        local3((rfoot_w[n] - rfoot_w[p]) * kFps, o + 21);
        local3((hips_w[n] - hips_w[p]) * kFps, o + 24);
        // need a full future trajectory, and one frame of history for velocities
        if (f + kTrajFrames[kTrajCount - 1] < c.end - 1 && f > c.start) {
          valid[f] = 1;
          ++nvalid;
        }
      }
    }
  }
};

// ------------------------------------------------------------------ playback

class CharacterPlayer {
 public:
  // world state of the simulation bone
  double pos_x = 0, pos_z = 0, yaw = 0;
  int frame = 0;
  std::vector<dfm2::CRigBone> bones;  // posed skeleton for rendering

  // settings
  bool inertialize = true;
  float blend_halflife = 0.10f;

  // stats
  int transitions = 0;
  double vel_x = 0, vel_z = 0;  // world velocity of the root (units / s)

  void Init(const MotionDatabase& db, int start_frame, double x, double z, double y) {
    db_ = &db;
    bones = db.bones;
    pos_x = x;
    pos_z = z;
    yaw = y;
    frame = std::clamp(start_frame, 0, db.nframes - 1);
    const int nb = db.NumBones();
    off_rot_.assign(nb, Vec3());
    off_vel_.assign(nb, Vec3());
    off_hips_ = off_hips_vel_ = Vec3();
    out_rot_.assign(db.Rot(frame), db.Rot(frame) + nb);
    prev_out_rot_ = out_rot_;
    out_hips_ = prev_out_hips_ = db.hips_pos[frame];
    vel_x = vel_z = 0;
    ApplyPose();
  }

  void Teleport(double x, double z, double y) {
    pos_x = x;
    pos_z = z;
    yaw = y;
    vel_x = vel_z = 0;
    ApplyPose();
  }

  /**
   * advance one 60 Hz tick
   * @param next_frame  frame + 1 when continuing, otherwise the new database frame
   */
  void Step(int next_frame, bool transition) {
    const MotionDatabase& db = *db_;
    const int nb = db.NumBones();
    const double dt = kDt;
    if (transition && inertialize) {
      // offsets that make the new animation start exactly where the current output is
      const Quat* src = db.Rot(next_frame);
      const Quat* src_next = db.Rot(next_frame + 1);
      for (int b = 0; b < nb; ++b) {
        const Vec3 out_vel = Log(out_rot_[b] * Conj(prev_out_rot_[b])) * (1.0 / dt);
        const Vec3 src_vel = Log(src_next[b] * Conj(src[b])) * (1.0 / dt);
        off_rot_[b] = Log(Abs(out_rot_[b] * Conj(src[b])));
        off_vel_[b] = out_vel - src_vel;
      }
      off_hips_ = out_hips_ - db.hips_pos[next_frame];
      off_hips_vel_ = (out_hips_ - prev_out_hips_) * (1.0 / dt) -
                      (db.hips_pos[next_frame + 1] - db.hips_pos[next_frame]) * (1.0 / dt);
    } else if (transition) {
      for (int b = 0; b < nb; ++b) { off_rot_[b] = off_vel_[b] = Vec3(); }
      off_hips_ = off_hips_vel_ = Vec3();
    }
    if (transition) { ++transitions; }

    // root motion of the database, applied in the current facing
    const int motion_frame = transition ? next_frame : frame;
    double wx, wz;
    LocalToWorld(yaw, db.root_dx[motion_frame], db.root_dz[motion_frame], wx, wz);
    pos_x += wx;
    pos_z += wz;
    yaw = WrapAngle(yaw + db.root_dyaw[motion_frame]);
    vel_x = wx / dt;
    vel_z = wz / dt;
    frame = next_frame;

    // decay the offsets and build the output pose
    prev_out_rot_ = out_rot_;
    prev_out_hips_ = out_hips_;
    const Quat* src = db.Rot(frame);
    for (int b = 0; b < nb; ++b) {
      DecaySpring(off_rot_[b].x, off_vel_[b].x, blend_halflife, dt);
      DecaySpring(off_rot_[b].y, off_vel_[b].y, blend_halflife, dt);
      DecaySpring(off_rot_[b].z, off_vel_[b].z, blend_halflife, dt);
      out_rot_[b] = NormalizeQ(Exp(off_rot_[b]) * src[b]);
    }
    DecaySpring(off_hips_.x, off_hips_vel_.x, blend_halflife, dt);
    DecaySpring(off_hips_.y, off_hips_vel_.y, blend_halflife, dt);
    DecaySpring(off_hips_.z, off_hips_vel_.z, blend_halflife, dt);
    out_hips_ = db.hips_pos[frame] + off_hips_;
    ApplyPose();
  }

  void ApplyPose() {
    const MotionDatabase& db = *db_;
    const int nb = db.NumBones();
    const Quat qy = YawQuat(yaw);
    for (int b = 1; b < nb; ++b) { out_rot_[b].CopyTo(bones[b].quatRelativeRot); }
    NormalizeQ(qy * out_rot_[0]).CopyTo(bones[0].quatRelativeRot);
    double hx, hz;
    LocalToWorld(yaw, out_hips_.x, out_hips_.z, hx, hz);
    bones[0].transRelative[0] = pos_x + hx;
    bones[0].transRelative[1] = out_hips_.y - db.ground_height;
    bones[0].transRelative[2] = pos_z + hz;
    dfm2::UpdateBoneRotTrans(bones);
  }

  [[nodiscard]] double BlendAmount() const {  // how far the pose is from the database (radians)
    double m = 0;
    for (const auto& o : off_rot_) { m = std::max(m, o.Length()); }
    return m;
  }

 private:
  const MotionDatabase* db_ = nullptr;
  std::vector<Vec3> off_rot_, off_vel_;
  Vec3 off_hips_, off_hips_vel_;
  std::vector<Quat> out_rot_, prev_out_rot_;
  Vec3 out_hips_, prev_out_hips_;
};

}  // namespace mm

#endif
