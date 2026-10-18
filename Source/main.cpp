/*
 *  Yifei Chen
 *  Motion Matching Demo
 *
 *  Control modes
 *    Controller : gamepad (or WASD) drives a spring-damper trajectory, motion matching follows it
 *    Path       : the character follows a looped path that can be edited with the mouse
 *  Camera modes
 *    Follow     : third person camera behind the character (right stick / RMB drag to orbit)
 *    Free       : detached orbit camera for looking around and editing the path
 */
#include <algorithm>
#include <chrono>
#include <cctype>
#include <climits>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "../traj.h"  // spring_character_update_dt()
#include "delfem2/opengl/old/funcs.h"
#include "delfem2/opengl/old/rigv3.h"
#include "camera.h"
#include "gamepad.h"
#include "motion_matching.h"
#include "path_editor.h"
#include "render_util.h"
#include "selftest.h"

namespace fs = std::filesystem;

// ============================================================================ data

struct DataSet {
  std::vector<mm::ClipSource> clips;
  double scale = 1.0;
  std::string description;
};

static std::string FindDataDir() {
  const std::vector<fs::path> candidates = {fs::path(PATH_SOURCE_DIR) / ".." / "data", "data", "../data",
                                            "../../data", "../../../data"};
  for (const auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c / "jl_motionMatching_hd.bvh", ec) || fs::exists(c / "LocomotionFlat01_000.bvh", ec)) {
      return fs::weakly_canonical(c, ec).string();
    }
  }
  return "";
}

/**
 * 1) .bvh files given on the command line (optionally `--scale s`)
 * 2) data/jl_motionMatching_hd.bvh, the capture used by the original demo
 * 3) the PFNN locomotion captures shipped in data/
 */
static DataSet ChooseData(int argc, char* argv[], const std::string& dir) {
  DataSet ds;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--scale" && i + 1 < argc) {
      ds.scale = std::atof(argv[++i]);
      continue;
    }
    if (a == "--selftest") {
      if (i + 1 < argc && argv[i + 1][0] != '-') { ++i; }
      continue;
    }
    std::string ext = fs::path(a).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    std::error_code ec;
    if ((ext == ".bvh" || ext == ".txt") && fs::exists(a, ec)) { ds.clips.push_back({a, fs::path(a).stem().string()}); }
  }
  if (!ds.clips.empty()) {
    ds.description = "command line";
    return ds;
  }
  ds.scale = 1.0;
  if (dir.empty()) { return ds; }
  const fs::path jl = fs::path(dir) / "jl_motionMatching_hd.bvh";
  std::error_code ec;
  if (fs::exists(jl, ec)) {
    ds.clips.push_back({jl.string(), "jl_motionMatching_hd"});
    ds.description = "jl_motionMatching_hd.bvh";
    return ds;
  }
  static const char* kClips[] = {"LocomotionFlat01_000", "LocomotionFlat02_000", "LocomotionFlat03_000",
                                 "LocomotionFlat05_000", "LocomotionFlat06_000", "LocomotionFlat08_000",
                                 "LocomotionFlat10_000"};
  for (const char* c : kClips) {
    for (const char* suffix : {"", "_mirror"}) {
      const fs::path p = fs::path(dir) / (std::string(c) + suffix + ".bvh");
      if (fs::exists(p, ec)) { ds.clips.push_back({p.string(), std::string(c) + suffix}); }
    }
  }
  ds.scale = 5.644;  // CMU skeleton units -> roughly centimeters
  ds.description = "PFNN locomotion (data/LocomotionFlat*.bvh)";
  return ds;
}

// ============================================================================ trajectory

struct Trajectory {
  double px[mm::kTrajCount] = {}, pz[mm::kTrajCount] = {};  // world positions
  double dx[mm::kTrajCount] = {}, dz[mm::kTrajCount] = {};  // world facing directions
};

// critically damped spring on the velocity (Holden) + exponential facing, positions are relative
class ControllerSim {
 public:
  float vx = 0, vz = 0, ax = 0, az = 0;
  float facing = 0;
  float goal_vx = 0, goal_vz = 0, goal_facing = 0;

  void Reset(float vel_x, float vel_z, float yaw) {
    vx = vel_x; vz = vel_z; ax = az = 0;
    goal_vx = goal_vz = 0;
    facing = goal_facing = yaw;
  }

  void Tick(float vel_halflife, float rot_halflife, float dt) {
    float x = 0.f, z = 0.f;
    spring_character_update_dt(x, vx, ax, goal_vx, vel_halflife, dt);
    spring_character_update_dt(z, vz, az, goal_vz, vel_halflife, dt);
    facing = static_cast<float>(goal_facing + mm::WrapAngle(facing - goal_facing) * std::exp(-0.693147f * dt / rot_halflife));
  }

  void Predict(double ox, double oz, float vel_halflife, float rot_halflife, Trajectory& t) const {
    for (int k = 0; k < mm::kTrajCount; ++k) {
      const float tk = static_cast<float>(mm::kTrajFrames[k] / mm::kFps);
      float x = 0, v = vx, a = ax;
      spring_character_update_dt(x, v, a, goal_vx, vel_halflife, tk);
      float z = 0, w = vz, b = az;
      spring_character_update_dt(z, w, b, goal_vz, vel_halflife, tk);
      t.px[k] = ox + x;
      t.pz[k] = oz + z;
      const double ang = goal_facing + mm::WrapAngle(facing - goal_facing) * std::exp(-0.693147 * tk / rot_halflife);
      t.dx[k] = std::sin(ang);
      t.dz[k] = std::cos(ang);
    }
  }
};

// ============================================================================ app

enum class ControlMode { Controller = 0, Path = 1 };

class App {
 public:
  int Run(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--selftest") {
        st_.enabled = true;
        st_.dir = (i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1] : "selftest";
      }
    }
    if (!InitWindow()) { return EXIT_FAILURE; }
    RenderLoadingFrame();
    LoadDatabase(argc, argv);
    SetupDefaultPath();
    ResetCharacter();
    cam.SnapToActive();
    if (st_.enabled) {
      if (!db_ok_) { return EXIT_FAILURE; }
      std::error_code ec;
      fs::create_directories(st_.dir, ec);
      glfwSwapInterval(0);
      show_help = false;
    }

    double last = glfwGetTime();
    double accumulator = 0.0;
    while (!glfwWindowShouldClose(window_)) {
      glfwPollEvents();
      const double now = glfwGetTime();
      // the self test runs exactly one 60 Hz tick per rendered frame
      const float frame_dt = st_.enabled ? static_cast<float>(mm::kDt) : static_cast<float>(std::min(now - last, 0.1));
      last = now;
      accumulator += st_.enabled ? 0.0 : frame_dt;

      pad.Update();
      ImGui_ImplOpenGL2_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      if (st_.enabled) { SelftestBeforeTick(); }
      HandleInput(frame_dt);
      if (st_.enabled && !st_.done) {
        Tick();
        SelftestAfterTick();
      }
      int steps = 0;
      while (accumulator >= mm::kDt) {
        if (db_ok_ && (!paused || step_once)) {
          Tick();
          step_once = false;
        }
        accumulator -= mm::kDt;
        if (++steps >= 4) {  // can't keep up: drop time instead of spiraling
          accumulator = 0.0;
          break;
        }
      }
      const bool moving = std::hypot(player.vel_x, player.vel_z) > 20.0;
      cam.Update(frame_dt, V3((float)player.pos_x, 0.f, (float)player.pos_z), (float)player.yaw, moving);

      Render();
      DrawUI();

      ImGui::Render();
      ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
      if (!st_.screenshot.empty()) {
        const std::string file = st_.dir + "/" + st_.screenshot + ".png";
        std::printf("  screenshot %s\n", selftest::Screenshot(file) ? file.c_str() : "FAILED");
        st_.screenshot.clear();
      }
      glfwSwapBuffers(window_);
      UpdateKeyHistory();
      if (st_.enabled && st_.done) { break; }
    }
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
    if (st_.enabled) { return st_.failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE; }
    return EXIT_SUCCESS;
  }

 private:
  // ------------------------------------------------------------------ state
  GLFWwindow* window_ = nullptr;
  float ui_scale_ = 1.f;

  mm::MotionDatabase db;
  mm::CharacterPlayer player;
  mm::FeatureWeights weights, applied_weights_;
  bool db_ok_ = false;
  std::string db_log_;
  DataSet data_;
  int idle_frame_ = 0;

  OrbitCamera cam;
  Gamepad pad;
  MotionPath path;
  PathEditor editor;
  PathFollower follower;
  double follower_x_ = 0, follower_z_ = 0;
  ControllerSim sim;
  ControlMode mode = ControlMode::Controller;

  // settings
  float walk_speed = 150.f;
  float run_speed = 380.f;
  float vel_halflife = 0.27f;
  float rot_halflife = 0.27f;
  int search_interval = 10;
  int ignore_range = 20;
  bool force_search_on_input = true;
  float path_max_lead = 220.f;
  bool auto_camera = true;
  bool paused = false;
  bool step_once = false;
  bool show_help = true;
  bool show_ui = true;
  bool draw_debug = true;
  bool draw_query = true, draw_match = true, draw_history = true, draw_input = true, draw_shadow = true;

  // runtime
  Trajectory traj_;
  float query_[mm::kFeatureDim] = {};
  int ticks_since_search_ = 1000;
  float best_cost_ = 0.f, current_cost_ = 0.f;
  double search_ms_ = 0.0;
  int searches_ = 0;
  float searched_goal_vx_ = 0, searched_goal_vz_ = 0, searched_goal_facing_ = 0;
  std::deque<std::pair<float, float>> history_;
  bool keys_prev_[GLFW_KEY_LAST + 1] = {};
  bool rmb_active_ = false, mmb_active_ = false, lmb_orbit_ = false;
  float rmb_travel_ = 0.f;
  bool context_menu_requested_ = false;
  float pad_toast_ = 0.f;
  bool pad_was_connected_ = false;

  // scripted self test (--selftest)
  struct SelftestState {
    bool enabled = false, done = false;
    std::string dir;
    std::string screenshot;  // taken after this frame is rendered
    int stage = 0, phase = 0, tick = 0, failures = 0, warnings = 0;
    std::vector<selftest::Phase> script;
    selftest::Input input;
    double speed_sum = 0, goal_sum = 0, dir_err_sum = 0;
    int samples = 0, dir_samples = 0, transitions0 = 0;
    double path_err_sum = 0, path_err_max = 0, search_ms_max = 0;
    int path_samples = 0;
  } st_;

  // ------------------------------------------------------------------ init

  bool InitWindow() {
    glfwSetErrorCallback([](int, const char* d) { std::fprintf(stderr, "GLFW: %s\n", d); });
    if (!glfwInit()) { return false; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_SAMPLES, 4);
    int w = 1600, h = 900;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor()) {
      int mx, my, mw, mh;
      glfwGetMonitorWorkarea(mon, &mx, &my, &mw, &mh);
      if (mw > 0 && mh > 0) {  // 85% of the screen, 16:9
        h = mh * 85 / 100;
        w = std::min(mw * 85 / 100, h * 16 / 9);
        h = w * 9 / 16;
      }
    }
    window_ = glfwCreateWindow(w, h, "Motion Matching Demo", nullptr, nullptr);
    if (!window_) {
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // fixed layout every start
    float xs = 1.f, ys = 1.f;
    glfwGetWindowContentScale(window_, &xs, &ys);
    ui_scale_ = std::clamp(xs, 1.f, 3.f);
    ImFontConfig cfg;
    cfg.SizePixels = std::round(13.f * ui_scale_);
    io.Fonts->AddFontDefault(&cfg);
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.f;
    style.FrameRounding = 4.f;
    style.GrabRounding = 4.f;
    style.WindowBorderSize = 0.f;
    style.Colors[ImGuiCol_WindowBg].w = 0.90f;
    style.ScaleAllSizes(ui_scale_);
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL2_Init();

    // lighting (positions are set every frame in world space)
    ::glEnable(GL_LIGHT0);
    ::glEnable(GL_LIGHT1);
    const GLfloat amb[4] = {0.38f, 0.38f, 0.40f, 1.f};
    const GLfloat d0[4] = {0.70f, 0.68f, 0.62f, 1.f};
    const GLfloat d1[4] = {0.22f, 0.25f, 0.30f, 1.f};
    const GLfloat zero[4] = {0, 0, 0, 1};
    ::glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    ::glLightfv(GL_LIGHT0, GL_DIFFUSE, d0);
    ::glLightfv(GL_LIGHT0, GL_SPECULAR, zero);
    ::glLightfv(GL_LIGHT1, GL_DIFFUSE, d1);
    ::glLightfv(GL_LIGHT1, GL_SPECULAR, zero);
    ::glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.f);
    return true;
  }

  void RenderLoadingFrame() {
    for (int i = 0; i < 2; ++i) {
      glfwPollEvents();
      int fbw, fbh;
      glfwGetFramebufferSize(window_, &fbw, &fbh);
      ::glViewport(0, 0, fbw, fbh);
      ::glClearColor(0.20f, 0.22f, 0.25f, 1.f);
      ::glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL2_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      const ImVec2 ds = ImGui::GetIO().DisplaySize;
      ImGui::SetNextWindowPos(ImVec2(ds.x * 0.5f, ds.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
      ImGui::Begin("##loading", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Text("Loading motion database ...");
      ImGui::End();
      ImGui::Render();
      ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
      glfwSwapBuffers(window_);
    }
  }

  void LoadDatabase(int argc, char* argv[]) {
    const std::string dir = FindDataDir();
    data_ = ChooseData(argc, argv, dir);
    db_log_ = "data: " + (dir.empty() ? std::string("<not found>") : dir) + "\n";
    if (data_.clips.empty()) {
      db_log_ += "  no .bvh files found\n";
      std::cout << db_log_;
      return;
    }
    db_log_ += "set:  " + data_.description + "\n";
    // bone indices of the skeleton used by the original demo (lshoulder, rshoulder, lhip, rhip, lfoot, rfoot)
    const std::array<int, 6> fallback = {14, 9, 23, 18, 25, 20};
    db_ok_ = db.Load(data_.clips, data_.scale, fallback, db_log_);
    std::cout << db_log_ << std::flush;
    if (!db_ok_) { return; }
    db.Normalize(weights);
    applied_weights_ = weights;
    // walk / run speeds that exist in this capture
    {
      std::vector<double> speeds;
      for (int f = 0; f < db.nframes; ++f) {
        const double v = std::hypot(db.root_dx[f], db.root_dz[f]) * mm::kFps;
        if (db.valid[f] && v > 40.0) { speeds.push_back(v); }
      }
      if (speeds.size() > 100) {
        std::sort(speeds.begin(), speeds.end());
        walk_speed = static_cast<float>(std::round(speeds[speeds.size() * 45 / 100] / 10.0) * 10.0);
        run_speed = static_cast<float>(std::round(speeds[speeds.size() * 97 / 100] / 10.0) * 10.0);
        char buf[128];
        std::snprintf(buf, sizeof(buf), "  speeds in the data: walk ~%.0f, run ~%.0f\n", walk_speed, run_speed);
        db_log_ += buf;
        std::cout << buf;
      }
    }
    // a standing frame to start from
    float best = 1e30f;
    for (int f = 0; f < db.nframes; ++f) {
      if (!db.valid[f]) { continue; }
      const float* r = db.FeatureRaw(f);
      float s = 0;
      for (int i = 0; i < 6; ++i) { s += std::fabs(r[i]); }
      s += std::fabs(r[24]) + std::fabs(r[26]);
      if (s < best) { best = s; idle_frame_ = f; }
    }
  }

  void SetupDefaultPath() {
    // same square as the original demo
    const double pts[4][2] = {{0, 0}, {600, 0}, {600, 600}, {0, 600}};
    path.points.clear();
    for (auto& p : pts) {
      KeyPoint k;
      k.Position = dfm2::CVec2d(p[0], p[1]);
      k.Velocity = 150.f;
      path.points.push_back(k);
    }
    path.loop = true;
    follower = PathFollower();
  }

  // ------------------------------------------------------------------ character

  void ResetCharacter() {
    if (!db_ok_) { return; }
    history_.clear();
    ticks_since_search_ = 1000;
    if (mode == ControlMode::Path && path.Usable()) {
      follower = PathFollower();
      const auto p0 = path.points[0].Position;
      const auto d0 = path.DirectionAt(0);
      player.Init(db, idle_frame_, p0.x, p0.y, std::atan2(d0.x, d0.y));
    } else {
      player.Init(db, idle_frame_, 0.0, 0.0, 0.0);
    }
    sim.Reset(0, 0, static_cast<float>(player.yaw));
    cam.follow.yaw = static_cast<float>(mm::WrapAngle(player.yaw + mm::kPi));
    cam.follow.target = V3((float)player.pos_x, cam.follow_height, (float)player.pos_z);
  }

  void SetMode(ControlMode m) {
    if (m == mode) { return; }
    mode = m;
    if (mode == ControlMode::Path) {
      JoinPathFromCharacter();
      if (auto_camera) {
        cam.SetMode(OrbitCamera::Mode::Free);
        // frame the whole path
        if (path.Usable()) {
          double minx = 1e30, maxx = -1e30, minz = 1e30, maxz = -1e30;
          for (const auto& k : path.points) {
            minx = std::min(minx, k.Position.x); maxx = std::max(maxx, k.Position.x);
            minz = std::min(minz, k.Position.y); maxz = std::max(maxz, k.Position.y);
          }
          cam.free.target = V3((float)(minx + maxx) * 0.5f, 0.f, (float)(minz + maxz) * 0.5f);
          cam.free.distance = std::clamp((float)std::max(maxx - minx, maxz - minz) * 1.6f + 400.f, 600.f, 8000.f);
          cam.free.pitch = std::max(cam.free.pitch, 0.9f);
        }
      }
    } else {
      sim.Reset(static_cast<float>(player.vel_x), static_cast<float>(player.vel_z), static_cast<float>(player.yaw));
      if (auto_camera) { cam.SetMode(OrbitCamera::Mode::Follow); }
    }
    ticks_since_search_ = 1000;
  }

  void JoinPathFromCharacter() {
    if (!path.Usable()) { return; }
    follower = path.Closest(dfm2::CVec2d(player.pos_x, player.pos_z));
  }

  // ------------------------------------------------------------------ simulation tick (60 Hz)

  void Tick() {
    const float dt = static_cast<float>(mm::kDt);
    bool input_changed = false;
    if (mode == ControlMode::Controller) {
      sim.Tick(vel_halflife, rot_halflife, dt);
      sim.Predict(player.pos_x, player.pos_z, vel_halflife, rot_halflife, traj_);
      const float dv = std::hypot(sim.goal_vx - searched_goal_vx_, sim.goal_vz - searched_goal_vz_);
      const float df = std::fabs(static_cast<float>(mm::WrapAngle(sim.goal_facing - searched_goal_facing_)));
      input_changed = dv > 0.35f * walk_speed || df > 0.4f;
    } else {
      TickPath(dt);
    }

    // query = desired trajectory (character space) + current pose features
    for (int k = 0; k < mm::kTrajCount; ++k) {
      double lx, lz;
      mm::WorldToLocal(player.yaw, traj_.px[k] - player.pos_x, traj_.pz[k] - player.pos_z, lx, lz);
      query_[k * 2 + 0] = db.NormalizeValue(k * 2 + 0, lx);
      query_[k * 2 + 1] = db.NormalizeValue(k * 2 + 1, lz);
      mm::WorldToLocal(player.yaw, traj_.dx[k], traj_.dz[k], lx, lz);
      query_[6 + k * 2 + 0] = db.NormalizeValue(6 + k * 2 + 0, lx);
      query_[6 + k * 2 + 1] = db.NormalizeValue(6 + k * 2 + 1, lz);
    }
    const float* pose = db.Feature(player.frame);
    std::copy(pose + 12, pose + mm::kFeatureDim, query_ + 12);

    ++ticks_since_search_;
    const bool must = !db.valid[player.frame];
    const bool want = ticks_since_search_ >= search_interval ||
                      (force_search_on_input && input_changed && ticks_since_search_ >= 3);
    int next = player.frame + 1;
    bool transition = false;
    if (must || want) {
      const auto t0 = std::chrono::high_resolution_clock::now();
      float best_cost = 0.f;
      const int best = db.Search(query_, best_cost);
      search_ms_ = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0).count();
      ++searches_;
      current_cost_ = must ? INFINITY : db.Cost(query_, player.frame);
      best_cost_ = best_cost;
      if (best >= 0) {
        const bool is_near = db.SameClip(best, player.frame) && std::abs(best - player.frame) <= ignore_range;
        transition = must || (!is_near && best_cost < current_cost_);
        if (transition) { next = best; }
      }
      ticks_since_search_ = 0;
      searched_goal_vx_ = sim.goal_vx;
      searched_goal_vz_ = sim.goal_vz;
      searched_goal_facing_ = sim.goal_facing;
    }
    if (next >= db.nframes || (!transition && !db.SameClip(next, player.frame))) {
      next = idle_frame_;
      transition = true;
    }
    player.Step(next, transition);

    history_.emplace_back(static_cast<float>(player.pos_x), static_cast<float>(player.pos_z));
    while (history_.size() > 300) { history_.pop_front(); }
  }

  void TickPath(float dt) {
    if (!path.Usable()) {  // nothing to follow: stand still
      for (int k = 0; k < mm::kTrajCount; ++k) {
        traj_.px[k] = player.pos_x;
        traj_.pz[k] = player.pos_z;
        traj_.dx[k] = std::sin(player.yaw);
        traj_.dz[k] = std::cos(player.yaw);
      }
      return;
    }
    dfm2::CVec2d pos, dir;
    path.Simulate(0.f, follower, follower, pos, dir);
    // the target waits if the character falls too far behind (e.g. a speed it can't reach)
    const double lead = std::hypot(pos.x - player.pos_x, pos.y - player.pos_z);
    if (lead < path_max_lead) { path.Simulate(dt, follower, follower, pos, dir); }
    follower_x_ = pos.x;
    follower_z_ = pos.y;
    for (int k = 0; k < mm::kTrajCount; ++k) {
      PathFollower tmp;
      path.Simulate(static_cast<float>(mm::kTrajFrames[k] / mm::kFps), follower, tmp, pos, dir);
      traj_.px[k] = pos.x;
      traj_.pz[k] = pos.y;
      traj_.dx[k] = dir.x;
      traj_.dz[k] = dir.y;
    }
  }

  // ------------------------------------------------------------------ input (every rendered frame)

  bool KeyDown(int key) const { return glfwGetKey(window_, key) == GLFW_PRESS; }
  bool KeyPressed(int key) const { return KeyDown(key) && !keys_prev_[key]; }
  void UpdateKeyHistory() {
    for (int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k) { keys_prev_[k] = glfwGetKey(window_, k) == GLFW_PRESS; }
  }

  void HandleInput(float frame_dt) {
    ImGuiIO& io = ImGui::GetIO();
    const bool keyboard_free = !io.WantTextInput && !io.WantCaptureKeyboard;
    const bool ctrl = io.KeyCtrl, shift = io.KeyShift;

    // gamepad (dis)connection toast
    if (pad.connected != pad_was_connected_) {
      pad_toast_ = 3.f;
      pad_was_connected_ = pad.connected;
    }
    pad_toast_ = std::max(0.f, pad_toast_ - frame_dt);

    // ---------- global shortcuts
    if (keyboard_free) {
      if (KeyPressed(GLFW_KEY_TAB)) { cam.Toggle(); }
      if (KeyPressed(GLFW_KEY_M)) { SetMode(mode == ControlMode::Controller ? ControlMode::Path : ControlMode::Controller); }
      if (KeyPressed(GLFW_KEY_R) && !ctrl) { ResetCharacter(); }
      if (KeyPressed(GLFW_KEY_P)) { paused = !paused; }
      if (KeyPressed(GLFW_KEY_PERIOD)) { paused = true; step_once = true; }
      if (KeyPressed(GLFW_KEY_H) || KeyPressed(GLFW_KEY_F1)) { show_help = !show_help; }
      if (KeyPressed(GLFW_KEY_F2)) { show_ui = !show_ui; }
      if (KeyPressed(GLFW_KEY_G)) { draw_debug = !draw_debug; }
      if (KeyPressed(GLFW_KEY_T)) {
        cam.SetMode(OrbitCamera::Mode::Free);
        cam.TopView();
      }
      if (KeyPressed(GLFW_KEY_F)) {
        cam.SetMode(OrbitCamera::Mode::Free);
        cam.Focus(V3((float)player.pos_x, 0.f, (float)player.pos_z));
      }
      if (KeyPressed(GLFW_KEY_ESCAPE)) {
        if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)) {
          // the menu closes itself
        } else if (editor.selected >= 0) {
          editor.selected = -1;
        } else {
          glfwSetWindowShouldClose(window_, GLFW_TRUE);
        }
      }
      if (mode == ControlMode::Path) {
        editor.HandleKeys(path, KeyPressed(GLFW_KEY_DELETE) || KeyPressed(GLFW_KEY_BACKSPACE),
                          ctrl && KeyPressed(GLFW_KEY_Z) && !shift,
                          ctrl && (KeyPressed(GLFW_KEY_Y) || (shift && KeyPressed(GLFW_KEY_Z))));
      }
    }

    // ---------- gamepad buttons
    if (pad.Pressed(Gamepad::Back)) { cam.Toggle(); }
    if (pad.Pressed(Gamepad::Start)) { SetMode(mode == ControlMode::Controller ? ControlMode::Path : ControlMode::Controller); }
    if (pad.Pressed(Gamepad::Y)) { ResetCharacter(); }
    if (pad.Pressed(Gamepad::X)) { draw_debug = !draw_debug; }
    if (pad.Pressed(Gamepad::RStick)) {
      cam.follow.yaw = static_cast<float>(mm::WrapAngle(player.yaw + mm::kPi));
      cam.follow.pitch = 0.28f;
    }

    // ---------- camera: gamepad
    if (std::fabs(pad.rx) > 0.f || std::fabs(pad.ry) > 0.f) {
      cam.Orbit(-pad.rx * cam.orbit_speed * frame_dt, -pad.ry * cam.orbit_speed * 0.6f * frame_dt);
    }
    if (pad.Down(Gamepad::LB)) { cam.Zoom(std::pow(2.2f, frame_dt)); }
    if (pad.Down(Gamepad::RB)) { cam.Zoom(std::pow(1.f / 2.2f, frame_dt)); }

    // ---------- camera + path editing: mouse
    const bool over_ui = io.WantCaptureMouse;
    const bool path_editing = mode == ControlMode::Path;
    const float w = io.DisplaySize.x, h = io.DisplaySize.y;
    const ImVec2 md = io.MouseDelta;
    if (!over_ui && io.MouseWheel != 0.f) { cam.Zoom(std::pow(0.88f, io.MouseWheel)); }
    // right button: orbit (or pan with shift in the free camera); a click opens the waypoint menu
    if (io.MouseClicked[1] && !over_ui) {
      rmb_active_ = true;
      rmb_travel_ = 0.f;
    }
    if (rmb_active_ && io.MouseDown[1]) {
      rmb_travel_ += std::fabs(md.x) + std::fabs(md.y);
      if (rmb_travel_ > 3.f) {
        if (shift && cam.mode == OrbitCamera::Mode::Free) {
          cam.PanByDrag(io.MousePos.x - md.x, io.MousePos.y - md.y, io.MousePos.x, io.MousePos.y, w, h);
        } else {
          cam.Orbit(-md.x * cam.mouse_sensitivity, md.y * cam.mouse_sensitivity);
        }
      }
    }
    if (rmb_active_ && io.MouseReleased[1]) {
      if (rmb_travel_ <= 3.f && path_editing) { context_menu_requested_ = true; }
      rmb_active_ = false;
    }
    // middle button: pan (free camera)
    if (io.MouseClicked[2] && !over_ui) { mmb_active_ = true; }
    if (mmb_active_ && io.MouseDown[2]) {
      if (cam.mode == OrbitCamera::Mode::Follow) { cam.SetMode(OrbitCamera::Mode::Free); }
      cam.PanByDrag(io.MousePos.x - md.x, io.MousePos.y - md.y, io.MousePos.x, io.MousePos.y, w, h);
    }
    if (!io.MouseDown[2]) { mmb_active_ = false; }
    // left button: path editing, or orbit (Alt+LMB always orbits, like the original demo)
    bool lmb_used = false;
    if (path_editing && !io.KeyAlt && !lmb_orbit_) {
      const int prev_sel = editor.selected;
      lmb_used = editor.HandleMouse(path, cam, !over_ui && !rmb_active_);
      if (editor.selected != prev_sel) { editor.SelectionChangedFrom3D(); }
    } else {
      editor.hovered = editor.hovered_segment = -1;
      editor.has_ground_hit = false;
    }
    if (!lmb_used && io.MouseClicked[0] && !over_ui && (io.KeyAlt || !path_editing)) { lmb_orbit_ = true; }
    if (lmb_orbit_ && io.MouseDown[0]) { cam.Orbit(-md.x * cam.mouse_sensitivity, md.y * cam.mouse_sensitivity); }
    if (!io.MouseDown[0]) { lmb_orbit_ = false; }

    if (path_editing) {
      if (editor.structure_changed) {
        editor.structure_changed = false;
        if (path.Usable()) {
          follower = path.Closest(dfm2::CVec2d(follower_x_, follower_z_));
        } else {
          follower = PathFollower();
        }
      }
      if (editor.request_restart) { editor.request_restart = false; ResetCharacter(); }
      if (editor.request_join) { editor.request_join = false; JoinPathFromCharacter(); }
      if (editor.request_start_at >= 0) {
        if (editor.request_start_at < path.Size()) {
          follower.keypoint = editor.request_start_at;
          follower.t = 0.f;
          const auto p = path.points[follower.keypoint].Position;
          const auto d = path.DirectionAt(follower.keypoint);
          player.Teleport(p.x, p.y, std::atan2(d.x, d.y));
          history_.clear();
        }
        editor.request_start_at = -1;
      }
    }

    // ---------- movement input: left stick + WASD, relative to the camera
    float mx = pad.lx, my = pad.ly;
    float run = pad.rt;
    bool strafe = pad.LeftTriggerDown();
    if (st_.enabled) {
      mx = st_.input.mx;
      my = st_.input.my;
      run = st_.input.run;
      strafe = st_.input.strafe;
      if (st_.input.cam_orbit != 0.f) { cam.Orbit(-st_.input.cam_orbit * cam.orbit_speed * frame_dt, 0.f); }
    }
    const bool keys_move_character = mode == ControlMode::Controller;
    if (keyboard_free) {
      float kx = 0.f, ky = 0.f;
      if (KeyDown(GLFW_KEY_W) || KeyDown(GLFW_KEY_UP)) { ky += 1.f; }
      if (KeyDown(GLFW_KEY_S) || KeyDown(GLFW_KEY_DOWN)) { ky -= 1.f; }
      if (KeyDown(GLFW_KEY_D) || KeyDown(GLFW_KEY_RIGHT)) { kx += 1.f; }
      if (KeyDown(GLFW_KEY_A) || KeyDown(GLFW_KEY_LEFT)) { kx -= 1.f; }
      if (kx != 0.f || ky != 0.f) {
        const float l = std::hypot(kx, ky);
        kx /= l;
        ky /= l;
        if (keys_move_character) {
          mx = kx;
          my = ky;
        } else if (cam.mode == OrbitCamera::Mode::Free) {  // path mode: WASD pans the free camera
          float fx, fz, rx, rz;
          cam.GroundBasis(fx, fz, rx, rz);
          const float s = cam.free.distance * 0.9f * frame_dt;
          cam.PanGround((fx * ky + rx * kx) * s, (fz * ky + rz * kx) * s);
        }
      }
      if (shift) { run = 1.f; }
      if (ctrl && !KeyDown(GLFW_KEY_Z) && !KeyDown(GLFW_KEY_Y)) { strafe = true; }
    }
    float fx, fz, rx, rz;
    cam.GroundBasis(fx, fz, rx, rz);
    const float speed = walk_speed + (run_speed - walk_speed) * std::clamp(run, 0.f, 1.f);
    sim.goal_vx = (fx * my + rx * mx) * speed;
    sim.goal_vz = (fz * my + rz * mx) * speed;
    if (strafe) {
      sim.goal_facing = std::atan2(fx, fz);
    } else if (std::hypot(sim.goal_vx, sim.goal_vz) > 1.f) {
      sim.goal_facing = std::atan2(sim.goal_vx, sim.goal_vz);
    } else {
      sim.goal_facing = sim.facing;
    }
  }

  // ------------------------------------------------------------------ rendering

  void Render() {
    int fbw, fbh;
    glfwGetFramebufferSize(window_, &fbw, &fbh);
    if (fbw <= 0 || fbh <= 0) { return; }
    ::glViewport(0, 0, fbw, fbh);
    const float sky[4] = {0.70f, 0.78f, 0.85f, 1.f};
    ::glClearColor(sky[0], sky[1], sky[2], sky[3]);
    ::glClearStencil(0);
    ::glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    ::glEnable(GL_DEPTH_TEST);
    ::glDepthFunc(GL_LEQUAL);
    ::glEnable(GL_NORMALIZE);
    ::glEnable(GL_LINE_SMOOTH);
    ::glEnable(GL_BLEND);
    ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    ::glShadeModel(GL_SMOOTH);

    float m[16];
    ::glMatrixMode(GL_PROJECTION);
    cam.ProjectionMatrix(m, static_cast<float>(fbw) / static_cast<float>(fbh));
    ::glLoadMatrixf(m);
    ::glMatrixMode(GL_MODELVIEW);
    cam.ViewMatrix(m);
    ::glLoadMatrixf(m);
    const GLfloat l0[4] = {0.35f, 1.0f, 0.45f, 0.f};
    const GLfloat l1[4] = {-0.6f, 0.5f, -0.5f, 0.f};
    ::glLightfv(GL_LIGHT0, GL_POSITION, l0);
    ::glLightfv(GL_LIGHT1, GL_POSITION, l1);

    ::glEnable(GL_FOG);
    ::glFogi(GL_FOG_MODE, GL_LINEAR);
    ::glFogfv(GL_FOG_COLOR, sky);
    ::glFogf(GL_FOG_START, 2500.f + cam.current.distance);
    ::glFogf(GL_FOG_END, 7500.f + cam.current.distance);

    const V3 c = cam.current.target;
    DrawGround(c.x, c.z);
    if (db_ok_ && draw_shadow) { DrawShadow(player.bones, c.x, c.z); }
    ::glDisable(GL_FOG);

    if (mode == ControlMode::Path) {
      editor.DrawScene(path, true);
      if (path.Usable() && draw_debug) {
        gl_util::Sphere(follower_x_, 4.0, follower_z_, 6.0, 0.15f, 0.85f, 0.3f);
      }
    }
    if (!db_ok_) { return; }
    if (draw_debug) { DrawDebug(); }
    DrawCharacter(player.bones);
  }

  static void DrawGround(float cx, float cz) {
    const double tile = 100.0;
    const int n = 45;
    const double ox = std::floor(cx / (2 * tile)) * 2 * tile;
    const double oz = std::floor(cz / (2 * tile)) * 2 * tile;
    ::glDisable(GL_LIGHTING);
    ::glEnable(GL_STENCIL_TEST);
    ::glStencilFunc(GL_ALWAYS, 1, ~0u);
    ::glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    ::glEnable(GL_POLYGON_OFFSET_FILL);
    ::glPolygonOffset(1.f, 1.f);
    ::glBegin(GL_QUADS);
    ::glNormal3d(0, 1, 0);
    for (int i = -n; i < n; ++i) {
      for (int j = -n; j < n; ++j) {
        if (((i + j) & 1) == 0) { ::glColor3f(0.62f, 0.62f, 0.56f); }
        else { ::glColor3f(0.56f, 0.56f, 0.50f); }
        const double x0 = ox + i * tile, z0 = oz + j * tile;
        ::glVertex3d(x0, 0, z0);
        ::glVertex3d(x0, 0, z0 + tile);
        ::glVertex3d(x0 + tile, 0, z0 + tile);
        ::glVertex3d(x0 + tile, 0, z0);
      }
    }
    ::glEnd();
    ::glDisable(GL_POLYGON_OFFSET_FILL);
    ::glDisable(GL_STENCIL_TEST);
    // world axes at the origin
    ::glLineWidth(2.f);
    ::glBegin(GL_LINES);
    ::glColor3f(0.8f, 0.2f, 0.2f); ::glVertex3d(0, 0.5, 0); ::glVertex3d(100, 0.5, 0);
    ::glColor3f(0.2f, 0.3f, 0.9f); ::glVertex3d(0, 0.5, 0); ::glVertex3d(0, 0.5, 100);
    ::glEnd();
  }

  // planar projected shadow from a directional light, masked to the floor with the stencil buffer
  static void DrawShadow(const std::vector<dfm2::CRigBone>& bones, float cx, float cz) {
    const float plane[4] = {0.f, 1.f, 0.f, -0.3f};
    const float light[4] = {0.35f, 1.0f, 0.45f, 0.f};
    const float dot = plane[0] * light[0] + plane[1] * light[1] + plane[2] * light[2] + plane[3] * light[3];
    float shadow[16];
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        shadow[col * 4 + row] = (row == col ? dot : 0.f) - light[row] * plane[col];
      }
    }
    ::glEnable(GL_STENCIL_TEST);
    ::glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    ::glDepthMask(GL_FALSE);
    ::glStencilFunc(GL_EQUAL, 1, ~0u);
    ::glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    ::glPushMatrix();
    ::glMultMatrixf(shadow);
    dfm2::opengl::DrawBone_Octahedron(bones, UINT_MAX, UINT_MAX, 0.1, 1.0);
    ::glPopMatrix();
    ::glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    ::glDepthMask(GL_TRUE);
    ::glStencilFunc(GL_EQUAL, 2, ~0u);
    ::glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    ::glDisable(GL_DEPTH_TEST);
    ::glDisable(GL_LIGHTING);
    ::glColor4f(0.05f, 0.05f, 0.08f, 0.35f);
    ::glBegin(GL_QUADS);
    const double r = 4000.0;
    ::glVertex3d(cx - r, 0.2, cz - r);
    ::glVertex3d(cx - r, 0.2, cz + r);
    ::glVertex3d(cx + r, 0.2, cz + r);
    ::glVertex3d(cx + r, 0.2, cz - r);
    ::glEnd();
    ::glEnable(GL_DEPTH_TEST);
    ::glDisable(GL_STENCIL_TEST);
  }

  static void DrawCharacter(const std::vector<dfm2::CRigBone>& bones) {
    ::glEnable(GL_LIGHTING);
    const GLfloat diffuse[4] = {0.90f, 0.66f, 0.38f, 1.f};
    ::glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    dfm2::opengl::DrawBone_Octahedron(bones, UINT_MAX, UINT_MAX, 0.1, 1.0);
    ::glDisable(GL_LIGHTING);
  }

  void DrawDebug() {
    const double px = player.pos_x, pz = player.pos_z;
    ::glDisable(GL_LIGHTING);
    // trail
    if (draw_history && history_.size() > 1) {
      ::glLineWidth(2.f);
      ::glBegin(GL_LINE_STRIP);
      const size_t n = history_.size();
      for (size_t i = 0; i < n; ++i) {
        ::glColor4f(0.25f, 0.25f, 0.3f, 0.15f + 0.6f * float(i) / float(n));
        ::glVertex3d(history_[i].first, 0.6, history_[i].second);
      }
      ::glEnd();
    }
    // root + facing
    ::glColor4f(0.1f, 0.1f, 0.15f, 0.8f);
    gl_util::Ring(px, 0.7, pz, 14.0, 18.0);
    ::glLineWidth(3.f);
    gl_util::GroundArrow(px, pz, std::sin(player.yaw) * 40.0, std::cos(player.yaw) * 40.0, 0.8, 9.0);
    // stick / keyboard input
    if (draw_input && mode == ControlMode::Controller) {
      const double s = 0.25;
      ::glColor4f(0.1f, 0.65f, 0.25f, 0.9f);
      ::glLineWidth(3.f);
      gl_util::GroundArrow(px, pz, sim.goal_vx * s, sim.goal_vz * s, 1.0, 10.0);
    }
    // desired trajectory (query)
    if (draw_query) {
      ::glLineWidth(2.f);
      ::glColor3f(0.95f, 0.45f, 0.1f);
      ::glBegin(GL_LINE_STRIP);
      ::glVertex3d(px, 2.0, pz);
      for (int k = 0; k < mm::kTrajCount; ++k) { ::glVertex3d(traj_.px[k], 2.0, traj_.pz[k]); }
      ::glEnd();
      ::glBegin(GL_LINES);
      for (int k = 0; k < mm::kTrajCount; ++k) {
        ::glVertex3d(traj_.px[k], 2.0, traj_.pz[k]);
        ::glVertex3d(traj_.px[k] + traj_.dx[k] * 25.0, 2.0, traj_.pz[k] + traj_.dz[k] * 25.0);
      }
      ::glEnd();
      for (int k = 0; k < mm::kTrajCount; ++k) { gl_util::Sphere(traj_.px[k], 4.0, traj_.pz[k], 4.5, 0.95f, 0.45f, 0.1f); }
    }
    // trajectory of the matched database frame
    if (draw_match) {
      const float* r = db.FeatureRaw(player.frame);
      double wx[mm::kTrajCount], wz[mm::kTrajCount], ddx[mm::kTrajCount], ddz[mm::kTrajCount];
      for (int k = 0; k < mm::kTrajCount; ++k) {
        mm::LocalToWorld(player.yaw, r[k * 2], r[k * 2 + 1], wx[k], wz[k]);
        wx[k] += px;
        wz[k] += pz;
        mm::LocalToWorld(player.yaw, r[6 + k * 2], r[6 + k * 2 + 1], ddx[k], ddz[k]);
      }
      ::glLineWidth(2.f);
      ::glColor3f(0.15f, 0.35f, 0.95f);
      ::glBegin(GL_LINE_STRIP);
      ::glVertex3d(px, 3.0, pz);
      for (int k = 0; k < mm::kTrajCount; ++k) { ::glVertex3d(wx[k], 3.0, wz[k]); }
      ::glEnd();
      ::glBegin(GL_LINES);
      for (int k = 0; k < mm::kTrajCount; ++k) {
        ::glVertex3d(wx[k], 3.0, wz[k]);
        ::glVertex3d(wx[k] + ddx[k] * 25.0, 3.0, wz[k] + ddz[k] * 25.0);
      }
      ::glEnd();
      for (int k = 0; k < mm::kTrajCount; ++k) { gl_util::Sphere(wx[k], 5.0, wz[k], 3.5, 0.15f, 0.35f, 0.95f); }
    }
  }

  // ------------------------------------------------------------------ self test

  void Check(bool ok, bool hard, const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    std::printf("  [%s] %s\n", ok ? " ok " : (hard ? "FAIL" : "warn"), buf);
    if (!ok) { (hard ? st_.failures : st_.warnings)++; }
    std::fflush(stdout);
  }

  void ResetPathStats() {
    st_.path_err_sum = st_.path_err_max = 0;
    st_.path_samples = 0;
    st_.tick = 0;
  }

  void SamplePathError() {
    double d = 0;
    path.Closest(dfm2::CVec2d(player.pos_x, player.pos_z), &d);
    st_.path_err_sum += d;
    st_.path_err_max = std::max(st_.path_err_max, d);
    ++st_.path_samples;
  }

  void SelftestBeforeTick() {
    if (st_.done) { return; }
    if (st_.stage == 0 && st_.script.empty()) {
      st_.script = selftest::ControllerScript();
      std::printf("[selftest] %d frames in the database, %d clips\n[selftest] controller (scripted left stick)\n",
                  db.nframes, (int)db.clips.size());
    }
    st_.input = st_.stage == 0 ? st_.script[st_.phase].input : selftest::Input();
  }

  void SelftestAfterTick() {
    st_.search_ms_max = std::max(st_.search_ms_max, search_ms_);
    ++st_.tick;
    if (st_.stage == 0) {
      const selftest::Phase& ph = st_.script[st_.phase];
      const int nticks = static_cast<int>(ph.seconds * mm::kFps);
      if (st_.tick > nticks * 0.45) {  // after the character had time to react
        const double speed = std::hypot(player.vel_x, player.vel_z);
        const double goal = std::hypot(sim.goal_vx, sim.goal_vz);
        st_.speed_sum += speed;
        st_.goal_sum += goal;
        ++st_.samples;
        if (goal > 20 && speed > 20) {
          const double a = std::atan2(player.vel_x, player.vel_z) - std::atan2(sim.goal_vx, sim.goal_vz);
          st_.dir_err_sum += std::fabs(mm::WrapAngle(a));
          ++st_.dir_samples;
        }
      }
      if (st_.tick < nticks) { return; }
      const double speed = st_.speed_sum / std::max(1, st_.samples);
      const double goal = st_.goal_sum / std::max(1, st_.samples);
      const double facing_err = std::fabs(mm::WrapAngle(player.yaw - sim.goal_facing)) * 180.0 / mm::kPi;
      const double dir_err = st_.dir_samples ? st_.dir_err_sum / st_.dir_samples * 180.0 / mm::kPi : 0.0;
      if (goal > 20) {
        Check(speed > goal * 0.5 && speed < goal * 1.6 && facing_err < ph.tolerance_deg && dir_err < ph.tolerance_deg,
              false,
              "%-24s speed %5.0f (goal %4.0f)  facing err %4.1f deg  move dir err %4.1f deg", ph.name, speed, goal,
              facing_err, dir_err);
      } else {
        Check(speed < 45, false, "%-24s speed %5.0f (goal 0)  frame %d", ph.name, speed, player.frame);
      }
      if (ph.screenshot) { st_.screenshot = ph.screenshot; }
      st_.speed_sum = st_.goal_sum = st_.dir_err_sum = 0;
      st_.samples = st_.dir_samples = 0;
      st_.tick = 0;
      if (++st_.phase < static_cast<int>(st_.script.size())) { return; }
      Check(player.transitions > st_.transitions0, true, "motion matching transitions: %d, max search %.2f ms",
            player.transitions - st_.transitions0, st_.search_ms_max);
      std::printf("[selftest] path mode (default square)\n");
      SetMode(ControlMode::Path);
      ResetCharacter();
      ResetPathStats();
      st_.stage = 1;
      return;
    }
    if (st_.stage == 1) {
      if (st_.tick > 180) { SamplePathError(); }
      if (st_.tick == 300) { st_.screenshot = "06_path_free_camera"; }
      if (st_.tick == 480) { cam.SetMode(OrbitCamera::Mode::Follow); }
      if (st_.tick == 720) { st_.screenshot = "07_path_follow_camera"; }
      if (st_.tick < 1200) { return; }
      Check(st_.path_err_sum / st_.path_samples < 60, false, "square path: mean distance %.1f, max %.1f",
            st_.path_err_sum / st_.path_samples, st_.path_err_max);
      SelftestEditor();
      std::printf("[selftest] path mode (figure eight preset, top view)\n");
      editor.PushUndo(path);
      path.ApplyPreset(MotionPath::kFigureEight, 900.f, 160.f, dfm2::CVec2d(0, 0));
      editor.structure_changed = true;
      editor.request_restart = true;
      cam.SetMode(OrbitCamera::Mode::Free);
      cam.free.target = V3(0, 0, 0);
      cam.free.distance = 1900.f;
      cam.TopView();
      editor.selected = 4;
      ResetPathStats();
      st_.stage = 2;
      return;
    }
    if (st_.stage == 2) {
      if (st_.tick > 120) { SamplePathError(); }
      if (st_.tick == 120) { SelftestPicking(); }
      if (st_.tick == 598) {  // right click menu on a waypoint + help overlay
        editor.hovered = 4;
        context_menu_requested_ = true;
        show_help = true;
      }
      if (st_.tick == 600) { st_.screenshot = "08_path_editor_figure8"; }
      if (st_.tick < 900) { return; }
      Check(st_.path_err_sum / st_.path_samples < 60, false, "figure eight: mean distance %.1f, max %.1f",
            st_.path_err_sum / st_.path_samples, st_.path_err_max);
      // camera: switching to the free camera must not move the view
      cam.SetMode(OrbitCamera::Mode::Follow);
      for (int i = 0; i < 120; ++i) { cam.Update((float)mm::kDt, V3((float)player.pos_x, 0, (float)player.pos_z), (float)player.yaw, true); }
      const V3 eye_before = cam.Eye();
      cam.SetMode(OrbitCamera::Mode::Free);
      cam.Update((float)mm::kDt, V3((float)player.pos_x, 0, (float)player.pos_z), (float)player.yaw, true);
      Check((cam.Eye() - eye_before).Length() < 1.f, true, "follow -> free camera keeps the view (moved %.2f)",
            (cam.Eye() - eye_before).Length());
      std::printf("[selftest] %s: %d failure(s), %d warning(s)\n", st_.failures ? "FAILED" : "PASSED", st_.failures,
                  st_.warnings);
      std::fflush(stdout);
      st_.done = true;
    }
  }

  // editing operations + mouse picking geometry
  void SelftestEditor() {
    const MotionPath original = path;
    const int n = path.Size();
    editor.InsertPoint(path, 2, dfm2::CVec2d(650, 300), 200.f);
    const bool inserted = path.Size() == n + 1 && path.points[2].Position.x == 650;
    editor.Undo(path);
    const bool undone = path.Size() == n;
    editor.Redo(path);
    const bool redone = path.Size() == n + 1;
    editor.DeletePoint(path, 2);
    path.Reverse();
    path.Reverse();
    bool same = path.Size() == original.Size();
    for (int i = 0; same && i < n; ++i) {
      same = (path.points[i].Position - original.points[i].Position).norm() < 1e-9 &&
             path.points[i].Velocity == original.points[i].Velocity;
    }
    Check(inserted && undone && redone && same, true, "insert / undo / redo / delete / reverse");
    const std::string file = st_.dir + "/path_roundtrip.txt";
    MotionPath loaded;
    const bool io_ok = path.Save(file) && loaded.Load(file) && loaded.Size() == path.Size() && loaded.loop == path.loop;
    Check(io_ok, true, "save / load %s", file.c_str());
    editor.structure_changed = true;
  }

  // a waypoint projected to the screen and picked back must land on itself
  void SelftestPicking() {
    const ImGuiIO& io = ImGui::GetIO();
    double worst = 0;
    int visible = 0;
    for (const auto& k : path.points) {
      float sx, sy;
      V3 hit;
      if (!cam.WorldToScreen(V3((float)k.Position.x, 0, (float)k.Position.y), io.DisplaySize.x, io.DisplaySize.y, sx, sy) ||
          sx < 0 || sy < 0 || sx > io.DisplaySize.x || sy > io.DisplaySize.y) {
        continue;
      }
      ++visible;
      if (cam.ScreenRay(sx, sy, io.DisplaySize.x, io.DisplaySize.y).HitGround(hit)) {
        worst = std::max(worst, (double)std::hypot(hit.x - k.Position.x, hit.z - k.Position.y));
      } else {
        worst = 1e9;
      }
    }
    if (visible == 0) { worst = 1e9; }
    Check(worst < 1.0, true, "mouse picking round trip error %.3f (%d waypoints on screen)", worst, visible);
  }

  // ------------------------------------------------------------------ UI

  void DrawUI() {
    ImGuiIO& io = ImGui::GetIO();
    const float s = ui_scale_;
    if (context_menu_requested_) {
      context_menu_requested_ = false;
      editor.OpenContextMenu();
    }
    if (mode == ControlMode::Path) {
      editor.DrawContextMenu(path);
      editor.DrawOverlay(path, cam, true);
    }
    DrawStatusBadge();

    if (!show_ui) { return; }
    ImGui::SetNextWindowPos(ImVec2(10 * s, 10 * s), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(370 * s, 0), ImGuiCond_Once);
    ImGui::Begin("Motion Matching", &show_ui, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PushItemWidth(150 * s);

    // ---- modes
    ImGui::TextDisabled("Control");
    ImGui::SameLine(80 * s);
    if (ImGui::RadioButton("Controller", mode == ControlMode::Controller)) { SetMode(ControlMode::Controller); }
    ImGui::SameLine();
    if (ImGui::RadioButton("Path", mode == ControlMode::Path)) { SetMode(ControlMode::Path); }
    ImGui::SameLine();
    ImGui::TextDisabled("(M)");
    ImGui::TextDisabled("Camera");
    ImGui::SameLine(80 * s);
    if (ImGui::RadioButton("Follow", cam.mode == OrbitCamera::Mode::Follow)) { cam.SetMode(OrbitCamera::Mode::Follow); }
    ImGui::SameLine();
    if (ImGui::RadioButton("Free", cam.mode == OrbitCamera::Mode::Free)) { cam.SetMode(OrbitCamera::Mode::Free); }
    ImGui::SameLine();
    ImGui::TextDisabled("(Tab)");

    // ---- input
    if (pad.connected) {
      ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1.f), "Gamepad: %s%s", pad.name.c_str(), pad.mapped ? "" : " (raw)");
    } else {
      ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.3f, 1.f), "No gamepad - using keyboard (WASD, Shift, Ctrl)");
    }
    DrawInputWidget();

    if (ImGui::Button(paused ? "Resume (P)" : "Pause (P)")) { paused = !paused; }
    ImGui::SameLine();
    if (ImGui::Button("Step (.)")) { paused = true; step_once = true; }
    ImGui::SameLine();
    if (ImGui::Button("Reset character (R)")) { ResetCharacter(); }
    ImGui::Checkbox("Camera follows the control mode", &auto_camera);

    if (ImGui::CollapsingHeader("Character", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::SliderFloat("Walk speed", &walk_speed, 30.f, 400.f, "%.0f");
      ImGui::SliderFloat("Run speed", &run_speed, 50.f, 600.f, "%.0f");
      ImGui::SliderFloat("Velocity half-life", &vel_halflife, 0.05f, 1.0f, "%.2f s");
      ImGui::SliderFloat("Turning half-life", &rot_halflife, 0.05f, 1.0f, "%.2f s");
      ImGui::Text("Speed %.0f   frame %d", std::hypot(player.vel_x, player.vel_z), player.frame);
      if (db_ok_) {
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", db.clips[db.clip_of_frame[player.frame]].name.c_str());
      }
    }
    if (ImGui::CollapsingHeader("Motion matching")) {
      ImGui::SliderInt("Search every", &search_interval, 1, 30, "%d frames");
      ImGui::SliderInt("Ignore nearby", &ignore_range, 0, 60, "%d frames");
      ImGui::Checkbox("Search immediately on input change", &force_search_on_input);
      ImGui::Checkbox("Inertialization", &player.inertialize);
      ImGui::SameLine();
      ImGui::SetNextItemWidth(120 * s);
      ImGui::SliderFloat("##blend", &player.blend_halflife, 0.02f, 0.4f, "half-life %.2f s");
      ImGui::TextDisabled("Feature weights");
      ImGui::SliderFloat("Trajectory position", &weights.traj_pos, 0.f, 3.f, "%.2f");
      ImGui::SliderFloat("Trajectory direction", &weights.traj_dir, 0.f, 3.f, "%.2f");
      ImGui::SliderFloat("Foot position", &weights.foot_pos, 0.f, 3.f, "%.2f");
      ImGui::SliderFloat("Foot velocity", &weights.foot_vel, 0.f, 3.f, "%.2f");
      ImGui::SliderFloat("Hip velocity", &weights.hip_vel, 0.f, 3.f, "%.2f");
      if (ImGui::Button("Default weights")) { weights = mm::FeatureWeights(); }
      if (db_ok_ && weights != applied_weights_) {
        db.Normalize(weights);
        applied_weights_ = weights;
      }
      ImGui::Text("Search %.2f ms   cost %.2f (current %.2f)", search_ms_, best_cost_,
                  std::isfinite(current_cost_) ? current_cost_ : -1.f);
      ImGui::Text("Transitions %d / searches %d", player.transitions, searches_);
      if (db_ok_) { ImGui::Text("Database %d frames (%d searchable), %zu clips", db.nframes, db.nvalid, db.clips.size()); }
    }
    if (ImGui::CollapsingHeader("Path following")) {
      ImGui::SliderFloat("Max lead distance", &path_max_lead, 50.f, 1000.f, "%.0f");
      ImGui::TextDisabled("the path target waits when the character falls behind");
      if (mode != ControlMode::Path && ImGui::Button("Switch to path mode (M)")) { SetMode(ControlMode::Path); }
    }
    if (ImGui::CollapsingHeader("Camera")) {
      ImGui::SliderFloat("Field of view", &cam.fovy_deg, 20.f, 90.f, "%.0f deg");
      ImGui::SliderFloat("Follow distance", &cam.follow.distance, 120.f, 2500.f, "%.0f");
      ImGui::SliderFloat("Follow height", &cam.follow_height, 0.f, 250.f, "%.0f");
      ImGui::SliderFloat("Follow lag", &cam.follow_lag, 0.f, 0.5f, "%.2f s");
      ImGui::Checkbox("Swing behind the character", &cam.follow_auto_align);
      ImGui::SliderFloat("Stick orbit speed", &cam.orbit_speed, 0.5f, 6.f, "%.1f");
      ImGui::Checkbox("Invert Y", &cam.invert_y);
      if (ImGui::Button("Top view (T)")) { cam.SetMode(OrbitCamera::Mode::Free); cam.TopView(); }
      ImGui::SameLine();
      if (ImGui::Button("Focus character (F)")) {
        cam.SetMode(OrbitCamera::Mode::Free);
        cam.Focus(V3((float)player.pos_x, 0.f, (float)player.pos_z));
      }
    }
    if (ImGui::CollapsingHeader("Display")) {
      ImGui::Checkbox("Debug drawing (G / X)", &draw_debug);
      ImGui::BeginDisabled(!draw_debug);
      ImGui::Checkbox("Desired trajectory", &draw_query);
      ImGui::SameLine();
      ImGui::Checkbox("Matched trajectory", &draw_match);
      ImGui::Checkbox("Input arrow", &draw_input);
      ImGui::SameLine();
      ImGui::Checkbox("Trail", &draw_history);
      ImGui::EndDisabled();
      ImGui::Checkbox("Shadow", &draw_shadow);
      ImGui::SameLine();
      ImGui::Checkbox("Path labels", &editor.show_labels);
      ImGui::SameLine();
      ImGui::Checkbox("Help (H)", &show_help);
    }
    if (!db_ok_) {
      ImGui::Separator();
      ImGui::TextColored(ImVec4(1.f, 0.4f, 0.3f, 1.f), "Motion database not loaded");
      ImGui::TextWrapped("%s", db_log_.c_str());
    }
    ImGui::Separator();
    ImGui::TextDisabled("%.1f FPS   F2: hide panels", io.Framerate);
    ImGui::PopItemWidth();
    ImGui::End();

    if (mode == ControlMode::Path) {
      ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10 * s, 48 * s), ImGuiCond_Once, ImVec2(1.f, 0.f));
      ImGui::Begin("Path Editor", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      editor.DrawPanel(path, dfm2::CVec2d(player.pos_x, player.pos_z), &paused, s);
      ImGui::End();
    }
    if (show_help) { DrawHelp(); }
  }

  // live view of the sticks / triggers / keyboard, so it is obvious what the character is told to do
  void DrawInputWidget() {
    const float s = ui_scale_;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float r = 26.f * s;
    const ImU32 ring = IM_COL32(140, 140, 150, 255), dot = IM_COL32(90, 220, 120, 255), dim = IM_COL32(70, 70, 80, 255);
    auto stick = [&](ImVec2 c, float x, float y, const char* label) {
      dl->AddCircleFilled(c, r, IM_COL32(30, 30, 36, 255), 32);
      dl->AddCircle(c, r, ring, 32, 1.5f);
      dl->AddLine(ImVec2(c.x - r, c.y), ImVec2(c.x + r, c.y), dim);
      dl->AddLine(ImVec2(c.x, c.y - r), ImVec2(c.x, c.y + r), dim);
      dl->AddCircleFilled(ImVec2(c.x + x * r, c.y - y * r), 5.f * s, dot);
      dl->AddText(ImVec2(c.x - ImGui::CalcTextSize(label).x * 0.5f, c.y + r + 2 * s), ring, label);
    };
    auto bar = [&](ImVec2 a, float v, const char* label) {
      const float bw = 12.f * s, bh = 2 * r;
      dl->AddRectFilled(a, ImVec2(a.x + bw, a.y + bh), IM_COL32(30, 30, 36, 255), 3.f);
      dl->AddRectFilled(ImVec2(a.x, a.y + bh * (1.f - v)), ImVec2(a.x + bw, a.y + bh), IM_COL32(230, 150, 60, 255), 3.f);
      dl->AddText(ImVec2(a.x + bw * 0.5f - ImGui::CalcTextSize(label).x * 0.5f, a.y + bh + 2 * s), ring, label);
    };
    // what the simulation receives (stick or keys, after camera-independent mapping)
    const float speed = std::max(run_speed, 1.f);
    float fx, fz, rx, rz;
    cam.GroundBasis(fx, fz, rx, rz);
    const float in_x = (sim.goal_vx * rx + sim.goal_vz * rz) / speed;
    const float in_y = (sim.goal_vx * fx + sim.goal_vz * fz) / speed;
    stick(ImVec2(p.x + r + 4 * s, p.y + r + 2 * s), mode == ControlMode::Controller ? in_x : pad.lx,
          mode == ControlMode::Controller ? in_y : pad.ly, "move");
    stick(ImVec2(p.x + 3 * r + 16 * s, p.y + r + 2 * s), pad.rx, pad.ry, "camera");
    bar(ImVec2(p.x + 4 * r + 30 * s, p.y + 2 * s), pad.lt, "LT");
    bar(ImVec2(p.x + 4 * r + 50 * s, p.y + 2 * s), pad.connected ? pad.rt : (ImGui::GetIO().KeyShift ? 1.f : 0.f), "RT");
    // state text
    const float tx = p.x + 4 * r + 74 * s;
    const bool strafe = pad.LeftTriggerDown() || glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;
    const float spd = std::hypot(sim.goal_vx, sim.goal_vz);
    const char* gait = spd < 1.f ? "idle" : (spd > (walk_speed + run_speed) * 0.5f ? "run" : "walk");
    dl->AddText(ImVec2(tx, p.y + 4 * s), IM_COL32(230, 230, 230, 255), mode == ControlMode::Controller ? gait : "path");
    if (mode == ControlMode::Controller && strafe) {
      dl->AddText(ImVec2(tx, p.y + 20 * s), IM_COL32(230, 180, 90, 255), "strafe");
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.0f", spd);
    if (mode == ControlMode::Controller) { dl->AddText(ImVec2(tx, p.y + 36 * s), IM_COL32(160, 160, 170, 255), buf); }
    ImGui::Dummy(ImVec2(4 * r + 120 * s, 2 * r + 22 * s));
  }

  void DrawStatusBadge() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s  |  %s camera%s", mode == ControlMode::Controller ? "CONTROLLER" : "PATH",
                  cam.mode == OrbitCamera::Mode::Follow ? "follow" : "free", paused ? "  |  PAUSED" : "");
    const ImVec2 ts = ImGui::CalcTextSize(buf);
    const ImVec2 p((io.DisplaySize.x - ts.x) * 0.5f, 10.f * ui_scale_);
    dl->AddRectFilled(ImVec2(p.x - 10, p.y - 5), ImVec2(p.x + ts.x + 10, p.y + ts.y + 5), IM_COL32(20, 20, 25, 190), 6.f);
    dl->AddText(p, paused ? IM_COL32(255, 200, 80, 255) : IM_COL32(255, 255, 255, 255), buf);
    if (pad_toast_ > 0.f) {
      std::snprintf(buf, sizeof(buf), pad.connected ? "Gamepad connected: %s" : "Gamepad disconnected%s",
                    pad.connected ? pad.name.c_str() : "");
      const ImVec2 t2 = ImGui::CalcTextSize(buf);
      const ImVec2 q((io.DisplaySize.x - t2.x) * 0.5f, p.y + ts.y + 16);
      const int a = static_cast<int>(std::min(1.f, pad_toast_) * 220);
      dl->AddRectFilled(ImVec2(q.x - 10, q.y - 5), ImVec2(q.x + t2.x + 10, q.y + t2.y + 5), IM_COL32(30, 90, 50, a), 6.f);
      dl->AddText(q, IM_COL32(255, 255, 255, a + 35), buf);
    }
  }

  void DrawHelp() {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(10 * ui_scale_, io.DisplaySize.y - 10 * ui_scale_), ImGuiCond_Always, ImVec2(0.f, 1.f));
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::Begin("##help", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                                        ImGuiWindowFlags_NoInputs);
    auto row = [](const char* k, const char* v) {
      ImGui::TextColored(ImVec4(1.f, 0.85f, 0.45f, 1.f), "%s", k);
      ImGui::SameLine(150 * ImGui::GetIO().FontGlobalScale * ImGui::GetFontSize() / 13.f);
      ImGui::TextUnformatted(v);
    };
    if (ImGui::BeginTable("##helpcols", 2, ImGuiTableFlags_SizingFixedFit)) {
      ImGui::TableNextColumn();
      ImGui::TextDisabled("GAMEPAD");
      row("Left stick", "move (camera relative)");
      row("Right stick", "orbit camera");
      row("RT (hold)", "run (analog)");
      row("LT (hold)", "strafe, face the camera");
      row("LB / RB", "zoom out / in");
      row("R3", "camera behind character");
      row("View / Back", "follow / free camera");
      row("Menu / Start", "controller / path mode");
      row("Y  /  X", "reset  /  debug drawing");
      ImGui::TableNextColumn();
      ImGui::TextDisabled("KEYBOARD + MOUSE");
      row("W A S D", mode == ControlMode::Path ? "pan the free camera" : "move (camera relative)");
      row("Shift / Ctrl", "run / strafe");
      row("Tab  /  M", "camera mode  /  control mode");
      row("R  P  .", "reset, pause, single step");
      row("T  /  F", "top view  /  focus character");
      row("RMB drag", "orbit (Shift: pan)");
      row("MMB drag, wheel", "pan, zoom");
      if (mode == ControlMode::Path) {
        row("LMB", "add / insert / drag waypoints");
        row("RMB click, Del", "waypoint menu, delete");
        row("Ctrl+Z / Ctrl+Y", "undo / redo");
      } else {
        row("LMB drag", "orbit");
      }
      row("H  /  F2", "this help  /  panels");
      ImGui::EndTable();
    }
    ImGui::End();
  }
};

int main(int argc, char* argv[]) {
  App app;
  return app.Run(argc, argv);
}
