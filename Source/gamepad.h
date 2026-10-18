/*
 *  Yifei Chen
 *  Motion Matching Demo - gamepad input
 *
 *  Thin wrapper over the GLFW (>= 3.3) gamepad API.
 *  Any controller that SDL's game controller database knows about (Xbox, PlayStation,
 *  Switch Pro, most generic pads) is exposed with the Xbox button layout.
 *  Joysticks without a mapping fall back to the raw XInput axis/button order.
 */
#ifndef MM_GAMEPAD_H
#define MM_GAMEPAD_H

#include <cmath>
#include <cstring>
#include <string>
#include <GLFW/glfw3.h>

class Gamepad {
 public:
  enum Button {
    A = GLFW_GAMEPAD_BUTTON_A,
    B = GLFW_GAMEPAD_BUTTON_B,
    X = GLFW_GAMEPAD_BUTTON_X,
    Y = GLFW_GAMEPAD_BUTTON_Y,
    LB = GLFW_GAMEPAD_BUTTON_LEFT_BUMPER,
    RB = GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER,
    Back = GLFW_GAMEPAD_BUTTON_BACK,
    Start = GLFW_GAMEPAD_BUTTON_START,
    Guide = GLFW_GAMEPAD_BUTTON_GUIDE,
    LStick = GLFW_GAMEPAD_BUTTON_LEFT_THUMB,
    RStick = GLFW_GAMEPAD_BUTTON_RIGHT_THUMB,
    DpadUp = GLFW_GAMEPAD_BUTTON_DPAD_UP,
    DpadRight = GLFW_GAMEPAD_BUTTON_DPAD_RIGHT,
    DpadDown = GLFW_GAMEPAD_BUTTON_DPAD_DOWN,
    DpadLeft = GLFW_GAMEPAD_BUTTON_DPAD_LEFT,
    Count = GLFW_GAMEPAD_BUTTON_LAST + 1
  };

  // read every frame
  bool connected = false;
  bool mapped = false;       // true if GLFW knows the layout of this controller
  std::string name;
  float lx = 0.f, ly = 0.f;  // left stick, deadzone applied, |v| <= 1, +y is stick UP
  float rx = 0.f, ry = 0.f;  // right stick, deadzone applied, +y is stick UP
  float lt = 0.f, rt = 0.f;  // triggers in [0,1]
  float raw_lx = 0.f, raw_ly = 0.f, raw_rx = 0.f, raw_ry = 0.f;

  // settings
  float deadzone = 0.15f;
  float trigger_threshold = 0.5f;

  void Update() {
    const bool was_connected = connected;
    std::memcpy(prev_, down_, sizeof(down_));
    std::memset(down_, 0, sizeof(down_));
    connected = false;
    mapped = false;

    if (jid_ < 0 || !glfwJoystickPresent(jid_)) { jid_ = FindJoystick(); }
    if (jid_ < 0) {
      lx = ly = rx = ry = lt = rt = 0.f;
      raw_lx = raw_ly = raw_rx = raw_ry = 0.f;
      if (was_connected) { name.clear(); }
      return;
    }
    connected = true;

    float axes[6] = {0, 0, 0, 0, -1, -1};
    GLFWgamepadstate state;
    if (glfwJoystickIsGamepad(jid_) && glfwGetGamepadState(jid_, &state)) {
      mapped = true;
      const char* gname = glfwGetGamepadName(jid_);
      name = gname ? gname : "Gamepad";
      for (int i = 0; i < 6; ++i) { axes[i] = state.axes[i]; }
      for (int i = 0; i < Count; ++i) { down_[i] = state.buttons[i] == GLFW_PRESS; }
    } else {
      // unknown controller: assume XInput ordering
      const char* jname = glfwGetJoystickName(jid_);
      name = jname ? jname : "Joystick";
      int naxes = 0, nbuttons = 0;
      const float* a = glfwGetJoystickAxes(jid_, &naxes);
      const unsigned char* b = glfwGetJoystickButtons(jid_, &nbuttons);
      for (int i = 0; i < 6 && i < naxes; ++i) { axes[i] = a[i]; }
      static const int kRawToButton[14] = {A, B, X, Y, LB, RB, Back, Start, LStick, RStick,
                                           DpadUp, DpadRight, DpadDown, DpadLeft};
      for (int i = 0; i < 14 && i < nbuttons; ++i) { down_[kRawToButton[i]] = b[i] == GLFW_PRESS; }
    }
    raw_lx = axes[GLFW_GAMEPAD_AXIS_LEFT_X];
    raw_ly = -axes[GLFW_GAMEPAD_AXIS_LEFT_Y];   // GLFW: up is -1
    raw_rx = axes[GLFW_GAMEPAD_AXIS_RIGHT_X];
    raw_ry = -axes[GLFW_GAMEPAD_AXIS_RIGHT_Y];
    RadialDeadzone(raw_lx, raw_ly, lx, ly);
    RadialDeadzone(raw_rx, raw_ry, rx, ry);
    lt = Trigger(axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]);
    rt = Trigger(axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER]);
  }

  [[nodiscard]] bool Down(Button b) const { return connected && down_[b]; }
  [[nodiscard]] bool Pressed(Button b) const { return connected && down_[b] && !prev_[b]; }
  [[nodiscard]] bool LeftTriggerDown() const { return lt > trigger_threshold; }
  [[nodiscard]] bool RightTriggerDown() const { return rt > trigger_threshold; }
  [[nodiscard]] float LeftMagnitude() const { return std::sqrt(lx * lx + ly * ly); }

 private:
  static int FindJoystick() {
    // prefer a controller with a known mapping
    for (int j = GLFW_JOYSTICK_1; j <= GLFW_JOYSTICK_LAST; ++j) {
      if (glfwJoystickPresent(j) && glfwJoystickIsGamepad(j)) { return j; }
    }
    for (int j = GLFW_JOYSTICK_1; j <= GLFW_JOYSTICK_LAST; ++j) {
      if (glfwJoystickPresent(j)) { return j; }
    }
    return -1;
  }

  void RadialDeadzone(float x, float y, float& ox, float& oy) const {
    const float mag = std::sqrt(x * x + y * y);
    if (mag <= deadzone || mag < 1e-6f) {
      ox = oy = 0.f;
      return;
    }
    const float scaled = std::fmin((mag - deadzone) / (1.f - deadzone), 1.f);
    ox = x / mag * scaled;
    oy = y / mag * scaled;
  }

  // triggers rest at -1 and go to +1
  static float Trigger(float v) {
    const float t = (v + 1.f) * 0.5f;
    return t < 0.02f ? 0.f : (t > 1.f ? 1.f : t);
  }

  int jid_ = -1;
  bool down_[Count] = {};
  bool prev_[Count] = {};
};

#endif
