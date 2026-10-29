/*
 *  Yifei Chen
 *  Motion Viewer
 *
 *  Plays the extra clips of this repository with on-screen buttons, no controller needed:
 *    - elderly and assisted walking from 100STYLE (data/100style): elderly gait, walking stick, limp
 *    - a robot arm that cleans a table and assembles parts (data/robot, from tools/make_robot_clips.py)
 *  Two clips can be compared side by side, walked in place on a treadmill, and measured
 *  (walking speed, cadence, step length and width, double support).
 *
 *  `Motion_viewer [file.bvh ...] [--scale s]`  adds BVH files to the list
 *  `Motion_viewer --capture [dir]`             renders the README screenshots and GIF frames, then exits
 */
#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "delfem2/opengl/old/rigv3.h"
#include "camera.h"
#include "mannequin.h"
#include "render_util.h"
#include "robot_cell.h"
#include "selftest.h"  // Screenshot()

namespace fs = std::filesystem;
namespace dfm2 = delfem2;
using viewer::MotionClip;
using viewer::Placement;
using viewer::RobotCell;
using viewer::Vec3;

// ============================================================================ library

struct Entry {
  std::string key;    // file stem, e.g. Old_FW
  std::string group;  // "walk", "robot" or "file"
  std::string title;  // shown above the scene
  std::string about;
  std::string file;
  const char* cane = nullptr;  // bone of the hand holding a walking stick
  double scale = 1.0;
};

struct Style {
  const char* key;
  const char* label;
  const char* about;
  const char* cane;
};
static const Style kStyles[] = {
    {"Neutral", "Neutral", "An ordinary walk, as a reference.", nullptr},
    {"Old", "Elderly", "Elderly gait: a stooped posture, short and slow steps, the arms held forward for balance.", nullptr},
    {"WalkingStickLeft", "Cane, left hand", "Walking with a cane in the left hand.", "LeftWrist"},
    {"WalkingStickRight", "Cane, right hand", "Walking with a cane in the right hand.", "RightWrist"},
    {"LimpLeft", "Limp, left leg", "Limping: short, careful steps on the left leg.", nullptr},
    {"LimpRight", "Limp, right leg", "Limping: short, careful steps on the right leg.", nullptr},
};

struct Kind {
  const char* key;
  const char* label;
  const char* about;
};
static const Kind kKinds[] = {
    {"FW", "Walk", "Walking forward with turns."},
    {"TR1", "Start/stop", "Starting, stopping, speeding up and turning."},
    {"ID", "Stand", "Standing still."},
};

static const Kind kTasks[] = {
    {"Robot_WipeTable", "Clean: wipe the table",
     "Cleaning. The arm wipes a coffee spill with a sponge in overlapping strokes, then scrubs a dried stain in circles."},
    {"Robot_PickPlace", "Assemble: pick and place",
     "Assembly. The arm picks four parts from a tray, turns two of them, and stacks them on a fixture."},
    {"Robot_ScrewDriving", "Assemble: drive screws", "Assembly. An electric screwdriver fastens the four screws of a housing lid."},
};

static void Note(const char* text) {
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
  ImGui::TextWrapped("%s", text);
  ImGui::PopStyleColor();
}

static std::string FindDataDir() {
  const std::vector<fs::path> candidates = {fs::path(PATH_SOURCE_DIR) / ".." / "data", "data", "../data",
                                            "../../data", "../../../data"};
  for (const auto& c : candidates) {
    std::error_code ec;
    if (fs::exists(c / "100style", ec) || fs::exists(c / "robot", ec)) { return fs::weakly_canonical(c, ec).string(); }
  }
  return "";
}

// ============================================================================ capture script

struct Shot {
  const char* name;     // png file, or folder of GIF frames
  const char* clip;     // entry key
  const char* compare;  // entry key or nullptr
  double time;          // seconds into the clip
  bool in_place;
  float yaw, pitch, distance;  // camera; distance 0 = the default view of the clip
  bool ui;                     // show the panels
  double gif_seconds = 0.0;    // > 0: a frame sequence for a GIF
};

static std::vector<Shot> CaptureScript() {
  return {
      {"viewer_ui", "WalkingStickRight_FW", nullptr, 11.0, false, 0.75f, 0.22f, 360.f, true},
      {"elderly_vs_neutral", "Neutral_FW", "Old_FW", 6.0, true, 0.62f, 0.12f, 430.f, false},
      {"walking_stick", "WalkingStickLeft_FW", nullptr, 14.0, false, 0.55f, 0.25f, 380.f, false},
      {"limp", "LimpRight_FW", "Neutral_FW", 9.0, true, 0.35f, 0.18f, 420.f, false},
      {"robot_wipe", "Robot_WipeTable", nullptr, 10.0, false, 0.f, 0.f, 0.f, false},
      {"robot_pick_place", "Robot_PickPlace", nullptr, 25.6, false, 0.f, 0.f, 0.f, false},
      {"robot_screw", "Robot_ScrewDriving", nullptr, 10.3, false, 0.f, 0.f, 0.f, false},
      {"gif_elderly", "Neutral_FW", "Old_FW", 3.0, true, 0.62f, 0.10f, 360.f, false, 4.0},
      {"gif_walking_stick", "WalkingStickLeft_FW", nullptr, 10.0, true, 1.15f, 0.10f, 300.f, false, 5.0},
      {"gif_robot_wipe", "Robot_WipeTable", nullptr, 4.0, false, 0.f, 0.f, 0.f, false, 6.0},
      {"gif_robot_pick_place", "Robot_PickPlace", nullptr, 19.5, false, 0.f, 0.f, 0.f, false, 6.0},
      {"gif_robot_screw", "Robot_ScrewDriving", nullptr, 9.0, false, 0.f, 0.f, 0.f, false, 4.0},
  };
}

// ============================================================================ app

class Viewer {
 public:
  int Run(int argc, char* argv[]) {
    std::string capture_dir;
    double scale = 1.0;
    std::vector<std::string> files;
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--capture") {
        capture_ = true;
        capture_dir = (i + 1 < argc && argv[i + 1][0] != '-') ? argv[++i] : "capture";
      } else if (a == "--scale" && i + 1 < argc) {
        scale = std::atof(argv[++i]);
      } else {
        files.push_back(a);
      }
    }
    BuildLibrary(files, scale);
    if (!InitWindow()) { return EXIT_FAILURE; }
    if (capture_) { return Capture(capture_dir); }
    int first = 0;
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
      if (entries_[i].group == "file" || entries_[i].key == "WalkingStickRight_FW") { first = i; break; }
    }
    Select(first);

    double last = glfwGetTime();
    while (!glfwWindowShouldClose(window_)) {
      glfwPollEvents();
      const double now = glfwGetTime();
      const float dt = static_cast<float>(std::min(now - last, 0.1));
      last = now;
      NewFrame();
      HandleInput();
      if (playing_) { Advance(dt * speed_); }
      UpdateCamera(dt);
      Render();
      DrawUI();
      EndFrame();
      UpdateKeyHistory();
    }
    Shutdown();
    return EXIT_SUCCESS;
  }

 private:
  GLFWwindow* window_ = nullptr;
  float ui_scale_ = 1.f;
  std::string data_dir_, log_;
  std::vector<Entry> entries_;
  std::map<std::string, std::unique_ptr<MotionClip>> clips_;
  std::map<std::string, std::unique_ptr<RobotCell>> cells_;
  int current_ = -1, compare_ = -1;

  // playback
  double time_ = 0.0;
  bool playing_ = true, loop_ = true;
  float speed_ = 1.f;

  // display
  bool in_place_ = false, follow_ = true, mannequin_ = true, footprints_ = true, trail_ = true;
  bool shadow_ = true, labels_ = true, cane_ = true, tool_path_ = false;
  bool show_ui_ = true, show_badge_ = true, show_help_ = false;
  ImFont* overlay_font_ = nullptr;  // title and labels over the scene (nullptr: the UI font)
  ImFont* big_font_ = nullptr;
  OrbitCamera cam_;

  // runtime
  struct Shown {  // a character drawn this frame
    const MotionClip* clip = nullptr;
    Placement place;
    double time = 0;
    Vec3 head;
  };
  Shown shown_[2];
  int nshown_ = 0;
  std::vector<dfm2::CRigBone> pose_[2];
  bool keys_prev_[GLFW_KEY_LAST + 1] = {};
  bool rmb_ = false, mmb_ = false, lmb_ = false;
  bool capture_ = false;

  // ------------------------------------------------------------------ clips

  void BuildLibrary(const std::vector<std::string>& files, double scale) {
    data_dir_ = FindDataDir();
    std::error_code ec;
    for (const std::string& f : files) {
      if (!fs::exists(f, ec)) { continue; }
      Entry e;
      e.key = fs::path(f).stem().string();
      e.group = "file";
      e.title = e.key;
      e.about = f;
      e.file = f;
      e.scale = scale;
      entries_.push_back(e);
    }
    for (const Style& s : kStyles) {
      for (const Kind& k : kKinds) {
        Entry e;
        e.key = std::string(s.key) + "_" + k.key;
        e.group = "walk";
        e.title = std::string(s.label) + " - " + k.label;
        e.about = std::string(s.about) + " " + k.about;
        e.file = (fs::path(data_dir_) / "100style" / (e.key + ".bvh")).string();
        e.cane = s.cane;
        if (fs::exists(e.file, ec)) { entries_.push_back(e); }
      }
    }
    for (const Kind& t : kTasks) {
      Entry e;
      e.key = t.key;
      e.group = "robot";
      e.title = std::string("Robot arm - ") + t.label;
      e.about = t.about;
      e.file = (fs::path(data_dir_) / "robot" / (e.key + ".bvh")).string();
      if (fs::exists(e.file, ec)) { entries_.push_back(e); }
    }
    if (entries_.empty()) { log_ += "no clips found (data: " + (data_dir_.empty() ? "<not found>" : data_dir_) + ")\n"; }
  }

  [[nodiscard]] int Find(const char* key) const {
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
      if (key && entries_[i].key == key) { return i; }
    }
    return -1;
  }

  MotionClip* Clip(int i) {
    if (i < 0 || i >= static_cast<int>(entries_.size())) { return nullptr; }
    const Entry& e = entries_[i];
    auto it = clips_.find(e.file);
    if (it != clips_.end()) { return it->second.get(); }
    auto c = std::make_unique<MotionClip>();
    if (!c->Load(e.file, e.key, e.scale, e.cane, log_)) { return nullptr; }
    std::error_code ec;
    const fs::path scene = fs::path(e.file).replace_extension(".scene");
    if (!c->human && fs::exists(scene, ec)) {
      auto cell = std::make_unique<RobotCell>();
      if (cell->Load(scene.string(), *c, log_)) { cells_[e.file] = std::move(cell); }
    }
    return (clips_[e.file] = std::move(c)).get();
  }

  RobotCell* Cell(int i) {
    if (i < 0) { return nullptr; }
    auto it = cells_.find(entries_[i].file);
    return it == cells_.end() ? nullptr : it->second.get();
  }

  void Select(int i) {
    MotionClip* c = Clip(i);
    if (!c) { return; }
    current_ = i;
    time_ = 0.0;
    playing_ = true;
    if (MotionClip* other = Clip(compare_); !other || !c->human || other->NumBones() != c->NumBones()) { compare_ = -1; }
    ResetCamera();
  }

  void ResetCamera() {
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    if (RobotCell* cell = Cell(current_)) {
      cam_.SetMode(OrbitCamera::Mode::Free);
      if (cell->has_camera) {
        cam_.free.target = V3(cell->camera[0], cell->camera[1], cell->camera[2]);
        cam_.free.yaw = cell->camera[3];
        cam_.free.pitch = cell->camera[4];
        cam_.free.distance = cell->camera[5];
      }
    } else {
      cam_.follow_height = 92.f;
      cam_.follow.yaw = 0.75f;
      cam_.follow.pitch = 0.22f;
      cam_.follow.distance = 430.f;
      cam_.SetMode(follow_ ? OrbitCamera::Mode::Follow : OrbitCamera::Mode::Free);
      if (!follow_) {
        cam_.free = cam_.follow;
        cam_.free.target = V3(0, 90, 0);
      }
    }
    UpdateCamera(0.f, true);
  }

  [[nodiscard]] double Duration() {
    MotionClip* c = Clip(current_);
    return c ? c->Duration() : 0.0;
  }

  void Advance(double dt) {
    const double d = Duration();
    if (d <= 0) { return; }
    time_ += dt;
    if (time_ > d || time_ < 0) {
      if (loop_) {
        time_ = std::fmod(time_ + d, d);
      } else {
        time_ = std::clamp(time_, 0.0, d);
        playing_ = false;
      }
    }
  }

  void StepFrames(int n) {
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    playing_ = false;
    const int f = std::clamp(c->FrameAt(time_) + n, 0, c->nframes - 1);
    time_ = f * c->frame_time;
  }

  // where a character is shown: its own path (starting at the origin) or walking in place
  [[nodiscard]] Placement PlaceFor(const MotionClip& c, double t, double side) const {
    Placement pl;
    if (!c.human) { return pl; }
    if (in_place_) {
      double x, z, yaw, walked;
      c.Root(t, x, z, yaw, walked);
      pl.ox = x;
      pl.oz = z;
      pl.yaw = yaw;
    } else {
      pl.ox = c.root_x[0];
      pl.oz = c.root_z[0];
      pl.yaw = c.root_yaw[0];
    }
    pl.tx = side;
    return pl;
  }

  // the characters of this frame: the selected clip, and the one compared with it
  void Layout() {
    nshown_ = 0;
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    MotionClip* other = compare_ >= 0 ? Clip(compare_) : nullptr;
    const double half = other ? (in_place_ ? 65.0 : 80.0) : 0.0;
    shown_[nshown_++] = {c, PlaceFor(*c, time_, -half), time_, Vec3()};
    if (other) {
      const double t = loop_ ? std::fmod(time_, std::max(other->Duration(), 1e-3)) : std::min(time_, other->Duration());
      shown_[nshown_++] = {other, PlaceFor(*other, t, half), t, Vec3()};
    }
  }

  // ------------------------------------------------------------------ window

  bool InitWindow() {
    glfwSetErrorCallback([](int, const char* d) { std::fprintf(stderr, "GLFW: %s\n", d); });
    if (!glfwInit()) { return false; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_SAMPLES, 4);
    int w = 1600, h = 900;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor(); mon && !capture_) {
      int mx, my, mw, mh;
      glfwGetMonitorWorkarea(mon, &mx, &my, &mw, &mh);
      if (mw > 0 && mh > 0) {  // 85% of the screen, 16:9
        h = mh * 85 / 100;
        w = std::min(mw * 85 / 100, h * 16 / 9);
        h = w * 9 / 16;
      }
    }
    if (capture_) { glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE); }
    window_ = glfwCreateWindow(w, h, "Motion Viewer", nullptr, nullptr);
    if (!window_) {
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(capture_ ? 0 : 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    float xs = 1.f, ys = 1.f;
    glfwGetWindowContentScale(window_, &xs, &ys);
    ui_scale_ = capture_ ? 1.f : std::clamp(xs, 1.f, 3.f);  // the same images on any screen
    ImFontConfig cfg;
    cfg.SizePixels = std::round(13.f * ui_scale_);
    io.Fonts->AddFontDefault(&cfg);
    if (capture_) {  // titles and labels that stay readable in small GIFs
      ImFontConfig big;
      big.SizePixels = 30.f;
      big_font_ = io.Fonts->AddFontDefault(&big);
    }
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

  void NewFrame() {
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
  }

  void EndFrame(const std::string& screenshot = "") {
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    if (!screenshot.empty() && !selftest::Screenshot(screenshot)) { std::printf("  cannot write %s\n", screenshot.c_str()); }
    glfwSwapBuffers(window_);
  }

  void Shutdown() {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

  // ------------------------------------------------------------------ input

  bool KeyDown(int key) const { return glfwGetKey(window_, key) == GLFW_PRESS; }
  bool KeyPressed(int key) const { return KeyDown(key) && !keys_prev_[key]; }
  void UpdateKeyHistory() {
    for (int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k) { keys_prev_[k] = glfwGetKey(window_, k) == GLFW_PRESS; }
  }

  void HandleInput() {
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantTextInput && !io.WantCaptureKeyboard) {
      const int step = io.KeyShift ? 10 : 1;
      if (KeyPressed(GLFW_KEY_SPACE)) { playing_ = !playing_; }
      if (KeyPressed(GLFW_KEY_RIGHT) || KeyPressed(GLFW_KEY_PERIOD)) { StepFrames(step); }
      if (KeyPressed(GLFW_KEY_LEFT) || KeyPressed(GLFW_KEY_COMMA)) { StepFrames(-step); }
      if (KeyPressed(GLFW_KEY_HOME)) { time_ = 0.0; }
      if (KeyPressed(GLFW_KEY_END)) { time_ = Duration(); playing_ = false; }
      if (KeyPressed(GLFW_KEY_UP)) { speed_ = std::min(4.f, speed_ * 2.f); }
      if (KeyPressed(GLFW_KEY_DOWN)) { speed_ = std::max(0.125f, speed_ * 0.5f); }
      if (KeyPressed(GLFW_KEY_T)) { in_place_ = !in_place_; }
      if (KeyPressed(GLFW_KEY_C) || KeyPressed(GLFW_KEY_TAB)) { follow_ = !follow_; ResetCamera(); }
      if (KeyPressed(GLFW_KEY_R)) { ResetCamera(); }
      if (KeyPressed(GLFW_KEY_H) || KeyPressed(GLFW_KEY_F1)) { show_help_ = !show_help_; }
      if (KeyPressed(GLFW_KEY_F2)) { show_ui_ = !show_ui_; }
      if (KeyPressed(GLFW_KEY_ESCAPE)) { glfwSetWindowShouldClose(window_, GLFW_TRUE); }
    }
    // mouse: right / left drag orbit, middle drag pan, wheel zoom
    const bool over_ui = io.WantCaptureMouse;
    const ImVec2 md = io.MouseDelta;
    if (!over_ui && io.MouseWheel != 0.f) { cam_.Zoom(std::pow(0.88f, io.MouseWheel)); }
    if ((io.MouseClicked[0] || io.MouseClicked[1]) && !over_ui) { lmb_ = io.MouseClicked[0]; rmb_ = io.MouseClicked[1]; }
    if ((lmb_ && io.MouseDown[0]) || (rmb_ && io.MouseDown[1])) {
      if (io.KeyShift && cam_.mode == OrbitCamera::Mode::Free) {
        cam_.PanByDrag(io.MousePos.x - md.x, io.MousePos.y - md.y, io.MousePos.x, io.MousePos.y, io.DisplaySize.x,
                       io.DisplaySize.y);
      } else {
        cam_.Orbit(-md.x * cam_.mouse_sensitivity, md.y * cam_.mouse_sensitivity);
      }
    }
    if (!io.MouseDown[0]) { lmb_ = false; }
    if (!io.MouseDown[1]) { rmb_ = false; }
    if (io.MouseClicked[2] && !over_ui) { mmb_ = true; }
    if (mmb_ && io.MouseDown[2]) {
      if (cam_.mode == OrbitCamera::Mode::Follow) { follow_ = false; cam_.SetMode(OrbitCamera::Mode::Free); }
      cam_.PanByDrag(io.MousePos.x - md.x, io.MousePos.y - md.y, io.MousePos.x, io.MousePos.y, io.DisplaySize.x,
                     io.DisplaySize.y);
    }
    if (!io.MouseDown[2]) { mmb_ = false; }
  }

  // the follow camera looks at the characters (their middle when two are shown); snap: no smoothing
  void UpdateCamera(float dt, bool snap = false) {
    Layout();
    V3 target(0, 0, 0);
    float yaw = 0.f;
    bool moving = false;
    for (int i = 0; i < nshown_; ++i) {
      const Shown& s = shown_[i];
      if (!s.clip->human) { continue; }
      double x, z, y, w;
      s.clip->Root(s.time, x, z, y, w);
      const Vec3 p = s.clip->ToView(Vec3(x, 0, z), s.place);
      target = target + V3(static_cast<float>(p.x), 0.f, static_cast<float>(p.z)) * (1.f / nshown_);
      yaw = static_cast<float>(s.place.Yaw(y));
      moving = true;
    }
    if (snap) { cam_.follow.target = target + V3(0.f, cam_.follow_height, 0.f); }
    cam_.Update(dt, target, yaw, moving);
    if (snap) { cam_.SnapToActive(); }
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
    cam_.ProjectionMatrix(m, static_cast<float>(fbw) / static_cast<float>(fbh));
    ::glLoadMatrixf(m);
    ::glMatrixMode(GL_MODELVIEW);
    cam_.ViewMatrix(m);
    ::glLoadMatrixf(m);
    const GLfloat l0[4] = {0.35f, 1.0f, 0.45f, 0.f};
    const GLfloat l1[4] = {-0.6f, 0.5f, -0.5f, 0.f};
    ::glLightfv(GL_LIGHT0, GL_POSITION, l0);
    ::glLightfv(GL_LIGHT1, GL_POSITION, l1);
    ::glEnable(GL_FOG);
    ::glFogi(GL_FOG_MODE, GL_LINEAR);
    ::glFogfv(GL_FOG_COLOR, sky);
    ::glFogf(GL_FOG_START, 2500.f + cam_.current.distance);
    ::glFogf(GL_FOG_END, 7500.f + cam_.current.distance);
    const V3 c = cam_.current.target;
    gl_util::CheckerGround(c.x, c.z);
    ::glDisable(GL_FOG);

    Layout();
    if (nshown_ == 0) { return; }
    for (int i = 0; i < nshown_; ++i) {
      Shown& s = shown_[i];
      s.clip->Pose(s.time, pose_[i]);
      s.clip->Place(pose_[i], s.place);
      s.head = Vec3(0, -1e9, 0);
      for (const auto& b : pose_[i]) {
        const Vec3 p = viewer::BonePos(b);
        if (p.y > s.head.y) { s.head = p; }
      }
    }
    if (RobotCell* cell = Cell(current_)) {
      RenderRobot(*cell);
    } else {
      RenderPeople(c);
    }
  }

  void RenderRobot(const RobotCell& cell) {
    const auto& bones = pose_[0];
    const int frame = shown_[0].clip->FrameAt(time_);
    cell.DrawStatic();
    if (shadow_) {
      ::glClear(GL_STENCIL_BUFFER_BIT);  // only the work surface receives the shadow, not the floor under it
      cell.DrawReceiver();
      gl_util::PlanarShadow(static_cast<float>(cell.surface), 0.f, 0.f, 0.30f, [&] {
        cell.DrawRobot(bones);
        cell.DrawProps(bones);
      });
    }
    cell.DrawDirt(frame);
    cell.DrawRobot(bones);
    cell.DrawProps(bones);
    if (tool_path_) { cell.DrawTcpPath(frame); }
  }

  void RenderPeople(const V3& center) {
    // the floor under each character: a treadmill (in place) or its footprints
    for (int i = 0; i < nshown_; ++i) {
      const Shown& s = shown_[i];
      if (!s.clip->human) { continue; }
      if (in_place_) {
        double x, z, yaw, walked;
        s.clip->Root(s.time, x, z, yaw, walked);
        viewer::DrawTreadmill(s.place.tx, s.place.tz, walked);
      } else if (footprints_) {
        DrawFootprints(s);
      }
    }
    if (shadow_) {
      gl_util::PlanarShadow(0.f, center.x, center.z, 0.35f, [&] {
        for (int i = 0; i < nshown_; ++i) { DrawPerson(i); }
      });
    }
    if (trail_ && !in_place_) {
      for (int i = 0; i < nshown_; ++i) { DrawTrail(shown_[i]); }
    }
    for (int i = 0; i < nshown_; ++i) {
      if (i == 0) {
        ::glColor3f(0.86f, 0.78f, 0.68f);
      } else {
        ::glColor3f(0.55f, 0.72f, 0.80f);
      }
      DrawPerson(i);
    }
  }

  void DrawPerson(int i) {
    const Shown& s = shown_[i];
    if (mannequin_ && s.clip->human) {
      viewer::DrawMannequin(*s.clip, pose_[i]);
    } else {
      ::glEnable(GL_LIGHTING);
      GLfloat col[4];
      ::glGetFloatv(GL_CURRENT_COLOR, col);
      ::glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, col);
      dfm2::opengl::DrawBone_Octahedron(pose_[i], UINT_MAX, UINT_MAX, 0.1, 1.0);
      ::glDisable(GL_LIGHTING);
    }
    if (cane_ && s.clip->cane_hand >= 0) {
      const MotionClip& c = *s.clip;
      const int end = c.EndSiteOf(c.cane_hand);
      const Vec3 w = viewer::BonePos(pose_[i][c.cane_hand]);
      const Vec3 grip = end >= 0 ? w + (viewer::BonePos(pose_[i][end]) - w) * 0.3 : w;
      const double ft = std::clamp(s.time / c.frame_time, 0.0, c.nframes - 1.0);
      const int f0 = static_cast<int>(ft), f1 = std::min(f0 + 1, c.nframes - 1);
      const Vec3 tip = c.cane_tip[f0] + (c.cane_tip[f1] - c.cane_tip[f0]) * (ft - f0);
      double x, z, yaw, walked;
      c.Root(s.time, x, z, yaw, walked);
      viewer::DrawCane(grip, c.ToView(tip, s.place), s.place.Yaw(yaw));
    }
  }

  void DrawFootprints(const Shown& s) {
    const MotionClip& c = *s.clip;
    const int frame = c.FrameAt(s.time);
    const int keep = static_cast<int>(5.0 / c.frame_time);
    ::glDisable(GL_LIGHTING);
    ::glEnable(GL_POLYGON_OFFSET_FILL);
    ::glPolygonOffset(-1.f, -2.f);
    for (const viewer::FootStep& st : c.steps) {
      if (st.start > frame || frame - st.start > keep) { continue; }
      const float age = static_cast<float>(frame - st.start) / static_cast<float>(keep);
      viewer::DrawFootprint(c.ToView(st.pos, s.place), s.place.Yaw(st.yaw), st.foot, 0.75f * (1.f - age));
    }
    ::glDisable(GL_POLYGON_OFFSET_FILL);
  }

  void DrawTrail(const Shown& s) {
    const MotionClip& c = *s.clip;
    if (!c.human) { return; }
    const int frame = c.FrameAt(s.time);
    const int keep = static_cast<int>(8.0 / c.frame_time);
    ::glDisable(GL_LIGHTING);
    ::glLineWidth(2.f);
    ::glBegin(GL_LINE_STRIP);
    for (int f = std::max(0, frame - keep); f <= frame; f += 2) {
      const Vec3 p = c.ToView(Vec3(c.root_x[f], c.floor, c.root_z[f]), s.place);
      ::glColor4f(0.25f, 0.25f, 0.3f, 0.1f + 0.6f * static_cast<float>(f - frame + keep) / static_cast<float>(keep));
      ::glVertex3d(p.x, 0.6, p.z);
    }
    ::glEnd();
  }

  // ------------------------------------------------------------------ UI

  void DrawUI() {
    const float s = ui_scale_;
    const ImGuiIO& io = ImGui::GetIO();
    if (show_badge_) { DrawBadge(); }
    if (labels_ && nshown_ > 1) { DrawLabels(); }
    if (!show_ui_) { return; }
    const float panel_w = 390 * s;
    ImGui::SetNextWindowPos(ImVec2(10 * s, 10 * s), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panel_w, io.DisplaySize.y - 20 * s), ImGuiCond_Always);
    ImGui::Begin("Motion Viewer", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    DrawLibrary();
    ImGui::Separator();
    DrawClipPanel();
    ImGui::End();
    DrawTimeline(panel_w + 20 * s);
    if (show_help_) { DrawHelp(panel_w + 20 * s); }
  }

  void DrawLibrary() {
    const float s = ui_scale_;
    if (ImGui::CollapsingHeader("Elderly & assisted walking (100STYLE)", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::BeginTable("##walk", 4, ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 118 * s);
        for (const Kind& k : kKinds) { ImGui::TableSetupColumn(k.label, ImGuiTableColumnFlags_WidthFixed, 74 * s); }
        ImGui::TableHeadersRow();
        for (const Style& st : kStyles) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(st.label);
          for (const Kind& k : kKinds) {
            ImGui::TableNextColumn();
            const int i = Find((std::string(st.key) + "_" + k.key).c_str());
            if (i < 0) { continue; }
            ImGui::PushID(i);
            if (ImGui::Selectable(k.label, i == current_, 0, ImVec2(70 * s, 0))) { Select(i); }
            ImGui::PopID();
          }
        }
        ImGui::EndTable();
      }
    }
    if (ImGui::CollapsingHeader("Robot arm: cleaning & assembly", ImGuiTreeNodeFlags_DefaultOpen)) {
      for (const Kind& t : kTasks) {
        const int i = Find(t.key);
        if (i >= 0 && ImGui::Selectable(t.label, i == current_)) { Select(i); }
      }
    }
    bool any_file = false;
    for (const Entry& e : entries_) { any_file |= e.group == "file"; }
    if (any_file && ImGui::CollapsingHeader("Files", ImGuiTreeNodeFlags_DefaultOpen)) {
      for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        if (entries_[i].group != "file") { continue; }
        ImGui::PushID(i);
        if (ImGui::Selectable(entries_[i].key.c_str(), i == current_)) { Select(i); }
        ImGui::PopID();
      }
    }
    if (entries_.empty() || !log_.empty()) { ImGui::TextWrapped("%s", log_.c_str()); }
  }

  void DrawClipPanel() {
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    const Entry& e = entries_[current_];
    ImGui::TextColored(ImVec4(1.f, 0.85f, 0.45f, 1.f), "%s", e.title.c_str());
    ImGui::TextWrapped("%s", e.about.c_str());
    char info[256];
    std::snprintf(info, sizeof(info), "%s.bvh: %d frames, %.0f fps, %.1f s, %d bones", e.key.c_str(), c->nframes,
                  1.0 / c->frame_time, c->Duration(), c->NumBones());
    Note(info);
    if (RobotCell* cell = Cell(current_)) {
      DrawRobotPanel(*c, *cell);
      return;
    }
    ImGui::Spacing();
    // compare with another clip of the same skeleton
    const char* preview = compare_ >= 0 ? entries_[compare_].title.c_str() : "(none)";
    ImGui::SetNextItemWidth(250 * ui_scale_);
    if (ImGui::BeginCombo("Compare with", preview)) {
      if (ImGui::Selectable("(none)", compare_ < 0)) { compare_ = -1; }
      for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        if (i == current_ || entries_[i].group == "robot") { continue; }
        if (ImGui::Selectable(entries_[i].title.c_str(), i == compare_)) {
          MotionClip* o = Clip(i);
          compare_ = o && o->human && o->NumBones() == c->NumBones() ? i : -1;
        }
      }
      ImGui::EndCombo();
    }
    if (ImGui::RadioButton("Own path", !in_place_)) { in_place_ = false; }
    ImGui::SameLine();
    if (ImGui::RadioButton("Treadmill (in place)", in_place_)) { in_place_ = true; }
    ImGui::SameLine();
    ImGui::TextDisabled("(T)");
    if (ImGui::Checkbox("Follow camera (C)", &follow_)) { ResetCamera(); }
    ImGui::SameLine();
    ImGui::Checkbox("Mannequin", &mannequin_);
    ImGui::Checkbox("Footprints", &footprints_);
    ImGui::SameLine();
    ImGui::Checkbox("Trail", &trail_);
    ImGui::SameLine();
    ImGui::Checkbox("Shadow", &shadow_);
    if (c->cane_hand >= 0) {
      ImGui::Checkbox("Walking stick", &cane_);
      ImGui::SameLine();
    }
    ImGui::Checkbox("Labels", &labels_);

    ImGui::Spacing();
    ImGui::TextDisabled("Gait (whole clip)");
    MotionClip* o = compare_ >= 0 ? Clip(compare_) : nullptr;
    if (!c->human) {
      ImGui::TextDisabled("not a human skeleton");
      return;
    }
    if (ImGui::BeginTable("##gait", o ? 3 : 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 120 * ui_scale_);
      ImGui::TableSetupColumn(ShortTitle(current_).c_str(), ImGuiTableColumnFlags_WidthFixed, 110 * ui_scale_);
      if (o) { ImGui::TableSetupColumn(ShortTitle(compare_).c_str(), ImGuiTableColumnFlags_WidthFixed, 110 * ui_scale_); }
      ImGui::TableHeadersRow();
      const char* rows[] = {"Walking speed", "Cadence", "Step length", "Step width", "Double support"};
      for (int r = 0; r < 5; ++r) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(rows[r]);
        for (const MotionClip* g : {c, o}) {
          if (!g) { continue; }
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(GaitValue(g->gait, r).c_str());
        }
      }
      ImGui::EndTable();
    }
    Note("Steps are foot contacts (heel or toes on the floor). Speed is the hips while walking.");
  }

  [[nodiscard]] std::string ShortTitle(int i) const {
    const std::string& t = entries_[i].title;
    const size_t dash = t.find(" - ");
    return dash == std::string::npos ? t : t.substr(0, dash);
  }

  static std::string GaitValue(const viewer::GaitStats& g, int row) {
    if (!g.walking) { return row == 0 ? "standing" : "-"; }
    char buf[64];
    switch (row) {
      case 0: std::snprintf(buf, sizeof(buf), "%.2f m/s", g.speed / 100.0); break;
      case 1: std::snprintf(buf, sizeof(buf), "%.0f steps/min", g.cadence); break;
      case 2: std::snprintf(buf, sizeof(buf), "%.0f cm", g.step_length); break;
      case 3: std::snprintf(buf, sizeof(buf), "%.0f cm", g.step_width); break;
      default: std::snprintf(buf, sizeof(buf), "%.0f %%", g.double_support); break;
    }
    return buf;
  }

  void DrawRobotPanel(const MotionClip& c, const RobotCell& cell) {
    const int frame = c.FrameAt(time_);
    ImGui::Spacing();
    ImGui::Text("Now: %s", cell.StepAt(frame));
    const int f0 = std::max(0, frame - 1), f1 = std::min(c.nframes - 1, frame + 1);
    const double v = (cell.tcp[f1] - cell.tcp[f0]).Length() / (std::max(1, f1 - f0) * c.frame_time);
    ImGui::Text("Tool speed %.0f cm/s", v);
    if (!cell.dirt.empty()) {
      ImGui::SameLine();
      ImGui::Text("   dirt removed %.0f %%", 100.0 * cell.Cleaned(frame));
    }
    ImGui::Checkbox("Tool path", &tool_path_);
    ImGui::SameLine();
    ImGui::Checkbox("Shadow", &shadow_);
    ImGui::SameLine();
    if (ImGui::Button("Reset view (R)")) { ResetCamera(); }
    ImGui::Spacing();
    ImGui::TextDisabled("Program (click to jump)");
    for (size_t k = 0; k < cell.steps.size(); ++k) {
      const auto& st = cell.steps[k];
      const bool active = st.first <= frame && (k + 1 == cell.steps.size() || cell.steps[k + 1].first > frame);
      char label[160];
      std::snprintf(label, sizeof(label), "%5.1f s  %s##%zu", st.first * c.frame_time, st.second.c_str(), k);
      if (ImGui::Selectable(label, active)) { time_ = st.first * c.frame_time; }
    }
  }

  void DrawTimeline(float left) {
    const float s = ui_scale_;
    const ImGuiIO& io = ImGui::GetIO();
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    const float w = io.DisplaySize.x - left - 10 * s;
    ImGui::SetNextWindowPos(ImVec2(left, io.DisplaySize.y - 10 * s), ImGuiCond_Always, ImVec2(0.f, 1.f));
    ImGui::SetNextWindowSize(ImVec2(w, 0), ImGuiCond_Always);
    ImGui::Begin("##timeline", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);
    if (ImGui::Button("|<")) { time_ = 0.0; }
    ImGui::SameLine();
    if (ImGui::Button("<")) { StepFrames(-1); }
    ImGui::SameLine();
    if (ImGui::Button(playing_ ? "Pause" : "Play", ImVec2(60 * s, 0))) { playing_ = !playing_; }
    ImGui::SameLine();
    if (ImGui::Button(">")) { StepFrames(1); }
    ImGui::SameLine();
    float t = static_cast<float>(time_);
    ImGui::SetNextItemWidth(w - 420 * s);
    char fmt[64];
    std::snprintf(fmt, sizeof(fmt), "%%.2f / %.1f s  (frame %d)", c->Duration(), c->FrameAt(time_));
    if (ImGui::SliderFloat("##time", &t, 0.f, static_cast<float>(c->Duration()), fmt)) { time_ = t; }
    if (RobotCell* cell = Cell(current_)) {  // program steps as ticks on the timeline
      const ImVec2 a = ImGui::GetItemRectMin(), b = ImGui::GetItemRectMax();
      ImDrawList* dl = ImGui::GetWindowDrawList();
      for (const auto& st : cell->steps) {
        const float x = a.x + (b.x - a.x) * static_cast<float>(st.first * c->frame_time / std::max(1e-6, c->Duration()));
        dl->AddLine(ImVec2(x, a.y), ImVec2(x, a.y + 5 * s), IM_COL32(255, 200, 90, 220), 2.f);
      }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110 * s);
    ImGui::SliderFloat("##speed", &speed_, 0.125f, 2.f, "speed %.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &loop_);
    ImGui::SameLine();
    ImGui::TextDisabled("H: help");
    ImGui::End();
  }

  void DrawBadge() {
    MotionClip* c = Clip(current_);
    if (!c) { return; }
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    std::string text = entries_[current_].title;
    if (compare_ >= 0) { text += "  vs  " + ShortTitle(compare_); }
    if (RobotCell* cell = Cell(current_)) {
      const char* step = cell->StepAt(c->FrameAt(time_));
      if (*step) { text += "  |  " + std::string(step); }
    } else if (in_place_) {
      text += "  |  treadmill";
    }
    if (!playing_) { text += "  |  PAUSED"; }
    ImFont* font = overlay_font_ ? overlay_font_ : ImGui::GetFont();
    const float size = font->FontSize, pad = size * 0.75f;
    const ImVec2 ts = font->CalcTextSizeA(size, FLT_MAX, 0.f, text.c_str());
    const float left = show_ui_ ? 400 * ui_scale_ : 0.f;
    const ImVec2 p(left + (io.DisplaySize.x - left - ts.x) * 0.5f, pad);
    dl->AddRectFilled(ImVec2(p.x - pad, p.y - pad * 0.4f), ImVec2(p.x + ts.x + pad, p.y + ts.y + pad * 0.4f),
                      IM_COL32(20, 20, 25, 190), pad * 0.5f);
    dl->AddText(font, size, p, playing_ ? IM_COL32(255, 255, 255, 255) : IM_COL32(255, 200, 80, 255), text.c_str());
  }

  // name and walking speed above each character when two are compared
  void DrawLabels() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImFont* font = overlay_font_ ? overlay_font_ : ImGui::GetFont();
    const float size = font->FontSize, pad = size * 0.6f;
    const int idx[2] = {current_, compare_};
    std::string text[2];
    ImVec2 pos[2], ts[2];
    bool visible[2] = {false, false};
    for (int i = 0; i < nshown_; ++i) {
      const Shown& s = shown_[i];
      float sx, sy;
      visible[i] = cam_.WorldToScreen(V3(static_cast<float>(s.head.x), static_cast<float>(s.head.y + 22), static_cast<float>(s.head.z)),
                                      io.DisplaySize.x, io.DisplaySize.y, sx, sy);
      text[i] = ShortTitle(idx[i]);
      if (s.clip->gait.walking) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "  %.2f m/s, %.0f steps/min", s.clip->gait.speed / 100.0, s.clip->gait.cadence);
        text[i] += buf;
      }
      ts[i] = font->CalcTextSizeA(size, FLT_MAX, 0.f, text[i].c_str());
      pos[i] = ImVec2(sx - ts[i].x * 0.5f, sy - ts[i].y);
    }
    // keep two labels at the same height apart
    if (nshown_ == 2 && visible[0] && visible[1] && std::fabs(pos[0].y - pos[1].y) < ts[0].y + pad) {
      const int l = pos[0].x <= pos[1].x ? 0 : 1, r = 1 - l;
      const float overlap = pos[l].x + ts[l].x + 3 * pad - pos[r].x;
      if (overlap > 0) {
        pos[l].x -= overlap * 0.5f;
        pos[r].x += overlap * 0.5f;
      }
    }
    for (int i = 0; i < nshown_; ++i) {
      if (!visible[i]) { continue; }
      const ImVec2 p = pos[i];
      const ImU32 bg = i == 0 ? IM_COL32(110, 90, 60, 210) : IM_COL32(50, 90, 110, 210);
      dl->AddRectFilled(ImVec2(p.x - pad, p.y - pad * 0.4f), ImVec2(p.x + ts[i].x + pad, p.y + ts[i].y + pad * 0.4f), bg,
                        pad * 0.5f);
      dl->AddText(font, size, p, IM_COL32(255, 255, 255, 255), text[i].c_str());
    }
  }

  void DrawHelp(float left) {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(left, io.DisplaySize.y - 70 * ui_scale_), ImGuiCond_Always, ImVec2(0.f, 1.f));
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::Begin("##help", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs);
    auto row = [](const char* k, const char* v) {
      ImGui::TextColored(ImVec4(1.f, 0.85f, 0.45f, 1.f), "%s", k);
      ImGui::SameLine(150 * ImGui::GetFontSize() / 13.f);
      ImGui::TextUnformatted(v);
    };
    row("Space", "play / pause");
    row("Left / Right", "step a frame (Shift: 10)");
    row("Home / End", "first / last frame");
    row("Up / Down", "faster / slower");
    row("T", "treadmill (walk in place)");
    row("C / Tab", "follow camera on / off");
    row("R", "reset the view");
    row("Drag, wheel", "orbit, zoom");
    row("Shift+drag, MMB", "pan (free camera)");
    row("F2 / Esc", "hide panels / quit");
    ImGui::End();
  }

  // ------------------------------------------------------------------ capture

  int Capture(const std::string& dir) {
    std::error_code ec;
    fs::create_directories(dir, ec);
    std::printf("gait        speed  cadence  step length  step width  double support\n");
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
      const MotionClip* c = entries_[i].group == "walk" ? Clip(i) : nullptr;
      if (!c || !c->gait.walking) { continue; }
      const viewer::GaitStats& g = c->gait;
      std::printf("  %-22s %4.2f m/s  %3.0f /min  %4.0f cm  %4.0f cm  %4.0f %%\n", entries_[i].key.c_str(), g.speed / 100.0,
                  g.cadence, g.step_length, g.step_width, g.double_support);
    }
    int failures = 0;
    for (const Shot& shot : CaptureScript()) {
      const int i = Find(shot.clip);
      if (i < 0) {
        std::printf("  [skip] %s: no clip %s\n", shot.name, shot.clip);
        ++failures;
        continue;
      }
      in_place_ = shot.in_place;
      follow_ = true;
      show_ui_ = shot.ui;
      overlay_font_ = shot.gif_seconds > 0.0 ? big_font_ : nullptr;
      Select(i);
      compare_ = Find(shot.compare);
      ResetCamera();
      if (shot.distance > 0.f) {
        OrbitCamera::Rig& r = cam_.Active();
        r.yaw = shot.yaw;
        r.pitch = shot.pitch;
        r.distance = shot.distance;
      }
      time_ = shot.time;
      playing_ = true;  // the capture sets the time of every frame itself
      UpdateCamera(0.f, true);
      for (int k = 0; k < 3; ++k) { RenderFrame(""); }  // let the UI lay itself out
      if (shot.gif_seconds <= 0.0) {
        RenderFrame((fs::path(dir) / (std::string(shot.name) + ".png")).string());
        std::printf("  %s.png\n", shot.name);
        continue;
      }
      const fs::path sub = fs::path(dir) / shot.name;
      fs::create_directories(sub, ec);
      const double fps = 20.0;
      const int n = static_cast<int>(shot.gif_seconds * fps);
      for (int k = 0; k < n; ++k) {
        char file[32];
        std::snprintf(file, sizeof(file), "%04d.png", k);
        RenderFrame((sub / file).string());
        time_ += 1.0 / fps;
        UpdateCamera(static_cast<float>(1.0 / fps));
      }
      std::printf("  %s/ (%d frames at %.0f fps)\n", shot.name, n, fps);
    }
    Shutdown();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  void RenderFrame(const std::string& file) {
    glfwPollEvents();
    NewFrame();
    Render();
    DrawUI();
    EndFrame(file);
  }
};

int main(int argc, char* argv[]) {
  Viewer app;
  return app.Run(argc, argv);
}
