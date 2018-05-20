#pragma once
#include "math.h"


template <typename type_t>
struct rect2_t {
  type_t x0, y0, x1, y1;
};

using rect2f_t = rect2_t<float>;

struct clip_t {

  // return true if a triangle is backfacing
  template <typename vec_t>
  bool is_backface(const vec_t &a, const vec_t &b, const vec_t &c) {
    const float v1 = a.x - b.x;
    const float v2 = a.y - b.y;
    const float w1 = c.x - b.x;
    const float w2 = c.y - b.y;
    return 0.f > (v1 * w2 - v2 * w1);
  }

  template <typename vec_t>
  int classify(const rect2f_t &r, const vec_t &p) {
    return (p.x < r.x0 ? 1 : 0) |
           (p.x > r.x1 ? 2 : 0) |
           (p.y < r.y0 ? 4 : 0) |
           (p.y > r.y1 ? 8 : 0);
  };

  // return true if a triangle intersect unit square [0,0] -> [1,1]
  template <typename vec_t>
  bool tri_vis(const vec_t &a, const vec_t &b, const vec_t &c, const rect2f_t & r) {

    // cohen-sutherland style trivial box clipping
    {
      const int ca = classify(r, a), cb = classify(r, b), cc = classify(r, c);

      if (0 == (ca | cb | cc)) {
        // all in center, no clipping
        return true;
      }

      const int code = ca & cb & cc;
      if ((code & 1) || (code & 2) || (code & 4) || (code & 8)) {
        // all outside one plane
        return false;
      }
    }

    // reject backfaces
    // note: they mess up the plane equation test below with inverted normals
    if (is_backface(a, b, c)) {
      return false;
    }

    // triangle edge distance rejection
    {
      struct plane_t {

        plane_t(const vec_t &a, const vec_t &b)
          : nx(b.y - a.y), ny(a.x - b.x), d(a.x * nx + a.y * ny) {
          // fixme: invert to correct normals
        }

        bool test(const vec_t &p) const {
          return d > (p.x * nx + p.y * ny);
        }

        // trivial rejection based on edge normal and closest box vertex
        bool trivial(const rect2f_t & r) const {
          switch ((nx > 0.f) | ((ny > 0.f) << 1)) {
          case 3: return d > (r.x1 * nx + r.y1 * ny);  // - nx  - ny
          case 2: return d > (r.x0 * nx + r.y1 * ny);  // + nx  - ny
          case 1: return d > (r.x1 * nx + r.y0 * ny);  // - nx  + ny
          case 0: return d > (r.x0 * nx + r.y0 * ny);  // + nx  + ny
          }
          return false;  // keep compiler happy
        }

        float nx, ny, d;
      };

      const plane_t pab{a, b}, pbc{b, c}, pca{c, a};

      //TODO: classif, outside, on edge, inside

      // perform trivial reject tests
      if (pab.trivial(r) || pbc.trivial(r) || pca.trivial(r)) {
        return false;
      }
    }

    // in, but needs clipping
    return true;
  }

}; // clip_t
