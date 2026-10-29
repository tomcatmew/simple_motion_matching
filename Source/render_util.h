/*
 *  Yifei Chen
 *  Motion Matching Demo - small legacy OpenGL drawing helpers
 */
#ifndef MM_RENDER_UTIL_H
#define MM_RENDER_UTIL_H

#include <cmath>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

namespace gl_util {

// unit sphere with normals, compiled once into a display list
inline void UnitSphere() {
  static GLuint list = 0;
  if (list == 0) {
    list = ::glGenLists(1);
    ::glNewList(list, GL_COMPILE);
    const int nla = 12, nlo = 18;
    const double pi = 3.14159265358979;
    for (int i = 0; i < nla; ++i) {
      const double a0 = pi * (-0.5 + double(i) / nla), a1 = pi * (-0.5 + double(i + 1) / nla);
      ::glBegin(GL_QUAD_STRIP);
      for (int j = 0; j <= nlo; ++j) {
        const double b = 2 * pi * double(j) / nlo;
        const double x0 = std::cos(a0) * std::cos(b), y0 = std::sin(a0), z0 = std::cos(a0) * std::sin(b);
        const double x1 = std::cos(a1) * std::cos(b), y1 = std::sin(a1), z1 = std::cos(a1) * std::sin(b);
        ::glNormal3d(x1, y1, z1);
        ::glVertex3d(x1, y1, z1);
        ::glNormal3d(x0, y0, z0);
        ::glVertex3d(x0, y0, z0);
      }
      ::glEnd();
    }
    ::glEndList();
  }
  ::glCallList(list);
}

// shaded sphere in the current color
inline void Sphere(double x, double y, double z, double r) {
  const GLboolean lighting = ::glIsEnabled(GL_LIGHTING);
  ::glEnable(GL_LIGHTING);
  ::glEnable(GL_COLOR_MATERIAL);
  ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  ::glPushMatrix();
  ::glTranslated(x, y, z);
  ::glScaled(r, r, r);
  UnitSphere();
  ::glPopMatrix();
  ::glDisable(GL_COLOR_MATERIAL);
  if (!lighting) { ::glDisable(GL_LIGHTING); }
}

inline void Sphere(double x, double y, double z, double r, float cr, float cg, float cb) {
  ::glColor3f(cr, cg, cb);
  Sphere(x, y, z, r);
}

// flat ring on the ground
inline void Ring(double x, double y, double z, double r0, double r1, int n = 32) {
  ::glBegin(GL_QUAD_STRIP);
  for (int i = 0; i <= n; ++i) {
    const double a = 2 * 3.14159265358979 * i / n;
    ::glVertex3d(x + r1 * std::cos(a), y, z + r1 * std::sin(a));
    ::glVertex3d(x + r0 * std::cos(a), y, z + r0 * std::sin(a));
  }
  ::glEnd();
}

// arrow on the ground from (x,z) along (dx,dz)
inline void GroundArrow(double x, double z, double dx, double dz, double y, double head) {
  const double len = std::sqrt(dx * dx + dz * dz);
  if (len < 1e-6) { return; }
  const double ux = dx / len, uz = dz / len;
  const double ex = x + dx, ez = z + dz;
  ::glBegin(GL_LINES);
  ::glVertex3d(x, y, z);
  ::glVertex3d(ex, y, ez);
  ::glEnd();
  ::glBegin(GL_TRIANGLES);
  ::glVertex3d(ex + ux * head, y, ez + uz * head);
  ::glVertex3d(ex - uz * head * 0.6, y, ez + ux * head * 0.6);
  ::glVertex3d(ex + uz * head * 0.6, y, ez - ux * head * 0.6);
  ::glEnd();
}

// filled disc on a horizontal plane
inline void Disc(double x, double y, double z, double r, int n = 20) {
  ::glBegin(GL_TRIANGLE_FAN);
  ::glNormal3d(0, 1, 0);
  ::glVertex3d(x, y, z);
  for (int i = 0; i <= n; ++i) {
    const double a = -2 * 3.14159265358979 * i / n;
    ::glVertex3d(x + r * std::cos(a), y, z + r * std::sin(a));
  }
  ::glEnd();
}

// shaded cylinder from a to b in the current color, with flat caps
inline void Cylinder(double ax, double ay, double az, double bx, double by, double bz, double r, int n = 20) {
  const double dx = bx - ax, dy = by - ay, dz = bz - az;
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 1e-9 || r <= 0) { return; }
  const double w[3] = {dx / len, dy / len, dz / len};
  // any vector not parallel to the axis, then two directions around it
  const double t[3] = {std::fabs(w[0]) < 0.9 ? 1.0 : 0.0, std::fabs(w[0]) < 0.9 ? 0.0 : 1.0, 0.0};
  double u[3] = {w[1] * t[2] - w[2] * t[1], w[2] * t[0] - w[0] * t[2], w[0] * t[1] - w[1] * t[0]};
  const double lu = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  for (double& c : u) { c /= lu; }
  const double v[3] = {w[1] * u[2] - w[2] * u[1], w[2] * u[0] - w[0] * u[2], w[0] * u[1] - w[1] * u[0]};
  const GLboolean lighting = ::glIsEnabled(GL_LIGHTING);
  ::glEnable(GL_LIGHTING);
  ::glEnable(GL_COLOR_MATERIAL);
  ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  ::glBegin(GL_QUAD_STRIP);
  for (int i = 0; i <= n; ++i) {
    const double a = 2 * 3.14159265358979 * i / n;
    const double c = std::cos(a), s = std::sin(a);
    const double nx = u[0] * c + v[0] * s, ny = u[1] * c + v[1] * s, nz = u[2] * c + v[2] * s;
    ::glNormal3d(nx, ny, nz);
    ::glVertex3d(bx + nx * r, by + ny * r, bz + nz * r);
    ::glVertex3d(ax + nx * r, ay + ny * r, az + nz * r);
  }
  ::glEnd();
  for (int cap = 0; cap < 2; ++cap) {
    const double sgn = cap == 0 ? -1.0 : 1.0;
    const double ox = cap == 0 ? ax : bx, oy = cap == 0 ? ay : by, oz = cap == 0 ? az : bz;
    ::glBegin(GL_TRIANGLE_FAN);
    ::glNormal3d(w[0] * sgn, w[1] * sgn, w[2] * sgn);
    ::glVertex3d(ox, oy, oz);
    for (int i = 0; i <= n; ++i) {
      const double a = 2 * 3.14159265358979 * (cap == 0 ? n - i : i) / n;
      const double c = std::cos(a), s = std::sin(a);
      ::glVertex3d(ox + (u[0] * c + v[0] * s) * r, oy + (u[1] * c + v[1] * s) * r, oz + (u[2] * c + v[2] * s) * r);
    }
    ::glEnd();
  }
  ::glDisable(GL_COLOR_MATERIAL);
  if (!lighting) { ::glDisable(GL_LIGHTING); }
}

// shaded box in the current color: center and half sizes, axis aligned in the current frame
inline void Box(double cx, double cy, double cz, double hx, double hy, double hz) {
  static const int faces[6][4] = {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3}, {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
  static const double normals[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  const GLboolean lighting = ::glIsEnabled(GL_LIGHTING);
  ::glEnable(GL_LIGHTING);
  ::glEnable(GL_COLOR_MATERIAL);
  ::glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  ::glBegin(GL_QUADS);
  for (int f = 0; f < 6; ++f) {
    ::glNormal3dv(normals[f]);
    for (int k : faces[f]) {
      ::glVertex3d(cx + ((k & 1) ? hx : -hx), cy + ((k & 2) ? hy : -hy), cz + ((k & 4) ? hz : -hz));
    }
  }
  ::glEnd();
  ::glDisable(GL_COLOR_MATERIAL);
  if (!lighting) { ::glDisable(GL_LIGHTING); }
}

// checkerboard floor around (cx, cz); it also writes 1 to the stencil buffer for PlanarShadow
inline void CheckerGround(float cx, float cz) {
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
}

/**
 * planar projected shadow from a directional light onto the plane y = h, darkening only the pixels
 * where the stencil buffer is 1 (the receiver, see CheckerGround). `draw_casters` renders the geometry.
 */
template <class F>
inline void PlanarShadow(float h, float cx, float cz, float alpha, F draw_casters) {
  const float plane[4] = {0.f, 1.f, 0.f, -(h + 0.3f)};
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
  draw_casters();
  ::glPopMatrix();
  ::glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  ::glDepthMask(GL_TRUE);
  ::glStencilFunc(GL_EQUAL, 2, ~0u);
  ::glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  ::glDisable(GL_DEPTH_TEST);
  ::glDisable(GL_LIGHTING);
  ::glColor4f(0.05f, 0.05f, 0.08f, alpha);
  ::glBegin(GL_QUADS);
  const double r = 4000.0;
  ::glVertex3d(cx - r, h + 0.2, cz - r);
  ::glVertex3d(cx - r, h + 0.2, cz + r);
  ::glVertex3d(cx + r, h + 0.2, cz + r);
  ::glVertex3d(cx + r, h + 0.2, cz - r);
  ::glEnd();
  ::glEnable(GL_DEPTH_TEST);
  ::glDisable(GL_STENCIL_TEST);
}

}  // namespace gl_util

#endif
