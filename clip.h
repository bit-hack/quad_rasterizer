#pragma once
#include "math.h"


template <typename type_t>
struct rect2_t {
  type_t x0, y0, x1, y1;
};

using rect2f_t = rect2_t<float>;

template <typename vec_t>
struct clip_t {

  struct plane_t {

    plane_t(const vec_t &a, const vec_t &b)
      : _nx(b.y - a.y)
      , _ny(a.x - b.x)
      , _d(a.x * _nx + a.y * _ny) {
      // fixme: invert to correct normals
    }

    bool test(const vec_t &p) const {
      return _d > (p.x * _nx + p.y * _ny);
    }

    // trivial rejection based on edge normal and closest box vertex
    bool trivial_out(const rect2f_t &r) const {
      switch ((_nx > 0.f) | ((_ny > 0.f) << 1)) {
      case 3: return _d > (r.x1 * _nx + r.y1 * _ny); // - nx  - ny
      case 2: return _d > (r.x0 * _nx + r.y1 * _ny); // + nx  - ny
      case 1: return _d > (r.x1 * _nx + r.y0 * _ny); // - nx  + ny
      case 0: return _d > (r.x0 * _nx + r.y0 * _ny); // + nx  + ny
      default: __assume(false);
      }
    }

    // trivial inclusion based on edge normal and closest box vertex
    bool trivial_in(const rect2f_t &r) const {
      switch ((_nx > 0.f) | ((_ny > 0.f) << 1)) {
      case 3: return _d < (r.x0 * _nx + r.y0 * _ny); // - nx  - ny
      case 2: return _d < (r.x1 * _nx + r.y0 * _ny); // + nx  - ny
      case 1: return _d < (r.x0 * _nx + r.y1 * _ny); // - nx  + ny
      case 0: return _d < (r.x1 * _nx + r.y1 * _ny); // + nx  + ny
      default: __assume(false);
      }
    }

    float _nx, _ny, _d;
  };

  const plane_t _e01, _e12, _e20;
  const float2 _v0, _v1, _v2;

  // constructor
  clip_t(const vec_t &v0, const vec_t &v2, const vec_t &v1)
    : _e01{v0, v1}, _e12{v1, v2}, _e20{v2, v0}
    , _v0(v0), _v1(v1), _v2(v2)
  {}

  // return true if a triangle is backfacing
  bool is_backface() const {
    const float v1 = _v0.x - _v1.x;
    const float v2 = _v0.y - _v1.y;
    const float w1 = _v2.x - _v1.x;
    const float w2 = _v2.y - _v1.y;
    return 0.f > (v1 * w2 - v2 * w1);
  }

  static int classify(const rect2f_t &r, const vec_t &p) {
    return (p.x < r.x0 ? 1 : 0) |
           (p.x > r.x1 ? 2 : 0) |
           (p.y < r.y0 ? 4 : 0) |
           (p.y > r.y1 ? 8 : 0);
  };

  bool trivial_out(const rect2f_t &r) const {
      return _e01.trivial_out(r) ||
             _e12.trivial_out(r) ||
             _e20.trivial_out(r);
  }

  bool trivial_in(const rect2f_t &r) const {
      return _e01.trivial_in(r) &&
             _e12.trivial_in(r) &&
             _e20.trivial_in(r);
  }

  // return true if triangle overlaps aabb
  // return false if fully outside aabb
  bool overlap_aabb(const rect2f_t &r) const {

    // classify all vertices
    const int ca = classify(r, _v0),
              cb = classify(r, _v1),
              cc = classify(r, _v2);

    if (0 == (ca | cb | cc)) {
      // all in center, no clipping
      return true;
    }

    const int code = ca & cb & cc;
    if ((code & 1) || (code & 2) || (code & 4) || (code & 8)) {
      // all outside one plane
      return false;
    }

    // overlap but clipping
    return true;
  }

  // return true if a triangle intersects rectangle
  bool tri_vis(const rect2f_t & r) {

    // cohen-sutherland style trivial box clipping
    if (!overlap_aabb(r)) {
      return false;
    }

    // reject backfaces
    // note: they mess up the plane equation test below with inverted normals
    if (is_backface()) {
      return false;
    }

    // in, but needs clipping
    if (trivial_out(r)) {
      return false;
    }

    return true;
  }

}; // clip_t
