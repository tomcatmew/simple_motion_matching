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

}  // namespace gl_util

#endif
