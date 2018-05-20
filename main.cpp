#define _SDL_main_h
#include <SDL/SDL.h>

#include <assert.h>
#include <stdint.h>

#include <array>

#include "math.h"
#include "clip.h"


// squared length of two 2d vectors
constexpr float len_sqr(const float2 &a, const float2 &b) {
  return (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
}

constexpr int32_t minv(int32_t a, int32_t b) {
  return a < b ? a : b;
}

constexpr int32_t maxv(int32_t a, int32_t b) {
  return a > b ? a : b;
}

constexpr int32_t clampv(int32_t lo, int32_t v, int32_t hi) {
  return minv(hi, maxv(lo, v));
}

// return true if a triangle is backfacing
bool is_backface(const float2 &a, const float2 &b, const float2 &c) {
  const float v1 = a.x - b.x;
  const float v2 = a.y - b.y;
  const float w1 = c.x - b.x;
  const float w2 = c.y - b.y;
  return 0.f > (v1 * w2 - v2 * w1);
}

// return true if 'p' falls inside triangle 'a, b, c'
bool point_in_tri(const float2 &a, const float2 &b, const float2 &c,
                  const float2 &p) {
  // Compute vectors
  const auto v0 = float2{c.x - a.x, c.y - a.y}; // C - A;
  const auto v1 = float2{b.x - a.x, b.y - a.y}; // B - A;
  const auto v2 = float2{p.x - a.x, p.y - a.y}; // P - A;

  // Compute dot products
  const float dot00 = float2::dot(v0, v0);
  const float dot01 = float2::dot(v0, v1);
  const float dot02 = float2::dot(v0, v2);
  const float dot11 = float2::dot(v1, v1);
  const float dot12 = float2::dot(v1, v2);

  // Compute barycentric coordinates
  const float denom = (dot00 * dot11 - dot01 * dot01);
  const float u = (dot11 * dot02 - dot01 * dot12) / denom;
  const float v = (dot00 * dot12 - dot01 * dot02) / denom;

  // Check if point is in triangle
  return (u >= 0) && (v >= 0) && (u + v < 1);
}

// plot a pixel to the screen
void plot(SDL_Surface *surf, int32_t x, int32_t y, uint32_t rgb = 0xdadada) {
  assert(surf);
  if (x < 0 || y < 0 || x >= surf->w || y >= surf->h) {
    return;
  }
  uint32_t *pix = (uint32_t *)surf->pixels;
  pix[x + y * surf->w] = rgb;
}

// calculate scaled edge function
//
// - scale the normals so points on the edge would be 0.f distance
//   and the prime vertex would be at 1.f
//
std::array<float2, 3>
tri_calc(const std::array<float2, 3> &t) {
  // find edges
  const float2 e01 = t[0] - t[1];
  const float2 e12 = t[1] - t[2];
  const float2 e20 = t[2] - t[0];
  // edge normals
  float2 n01 = float2::cross(e01);
  float2 n12 = float2::cross(e12);
  float2 n20 = float2::cross(e20);
  // distance between edge and opposite vertex
  const float d01 = float2::dot(n01, t[2] - t[0]);
  const float d12 = float2::dot(n12, t[0] - t[1]);
  const float d20 = float2::dot(n20, t[1] - t[2]);
  // normalize to get interpolants
  n01 /= d01;
  n12 /= d12;
  n20 /= d20;
  // return interpolants for each vertex
  return std::array<float2, 3>{n12, n20, n01};
}

// evaluate edge functions at a point
//
std::array<float, 3> tri_eval(const std::array<float2, 3> &t,
                              const std::array<float2, 3> &i,
                              const float2 &p) {
  // vector to point on edge
  const float2 d0 = p - t[1];
  const float2 d1 = p - t[2];
  const float2 d2 = p - t[0];
  // distance from edge
  const float e0 = float2::dot(d0, i[0]);
  const float e1 = float2::dot(d1, i[1]);
  const float e2 = float2::dot(d2, i[2]);
  // return
  return std::array<float, 3>{e0, e1, e2};
}

void raster(SDL_Surface *surf, const std::array<float2, 3> &v) {
  const std::array<float2, 3> i = tri_calc(v);
  std::array<float, 3> jy = tri_eval(v, i, float2{0, 0});
  uint32_t *p = (uint32_t *)surf->pixels;
  for (uint32_t y = 0; y < uint32_t(surf->h); ++y) {
    std::array<float, 3> jx = jy;
    for (uint32_t x = 0; x < uint32_t(surf->w); ++x) {
      p[x] = 0x101010;
      if (jx[0] > 0.f && jx[1] > 0.f && jx[2] > 0.f) {
        const int cr = 0xff & int(jx[0] * 255.f);
        const int cg = 0xff & int(jx[1] * 255.f);
        const int cb = 0xff & int(jx[2] * 255.f);
        p[x] = (cr << 16) | (cg << 8) | cb;
      }
      jx[0] += i[0].x;
      jx[1] += i[1].x;
      jx[2] += i[2].x;
    }
    p += surf->pitch / 4;
    jy[0] += i[0].y;
    jy[1] += i[1].y;
    jy[2] += i[2].y;
  }
}

void draw_rect(SDL_Surface *surf, const rect2f_t &r, const uint32_t rgb) {

  uint32_t *p = (uint32_t*)surf->pixels;

  int x0 = int(r.x0), x1 = int(r.x1-1);
  int y0 = int(r.y0), y1 = int(r.y1-1);

  for (int x = x0; x <= x1; ++x) {
    p[y0 * 512 + x] = rgb;
    p[y1 * 512 + x] = rgb;
  }

  for (int y = y0; y <= y1; ++y) {
    p[y * 512 + x0] = rgb;
    p[y * 512 + x1] = rgb;
  }
}

void quad_tree(SDL_Surface *surf, clip_t<float2> &clip, const rect2f_t r) {

  draw_rect(surf, r, 0x303030);

  // if this triangle is clipped
  if (!clip.tri_vis(r)) {
    return;
  }

  // if we can clip further
  if ((r.x1 - r.x0) > 32 && (r.y1 - r.y0) > 32) {

    // mid point
    const float mx = (r.x0 + r.x1) * .5f;
    const float my = (r.y0 + r.y1) * .5f;

    quad_tree(surf, clip, rect2f_t{r.x0, r.y0, mx,   my  });
    quad_tree(surf, clip, rect2f_t{mx,   r.y0, r.x1, my  });
    quad_tree(surf, clip, rect2f_t{r.x0, my,   mx,   r.y1});
    quad_tree(surf, clip, rect2f_t{mx,   my,   r.x1, r.y1});
  }
  else {
    if (clip.trivial_in(r)) {
      draw_rect(surf, r, 0x107010);
    }
    else {
      draw_rect(surf, r, 0x3030a0);
    }
  }
}

// find the nearest vertex to point p
float2 *nearest(std::array<float2, 3> &tri, const float2 &p) {
  float2 *out = nullptr;
  float best = 100000.f;
  for (auto &c : tri) {
    const float dx = p.x - c.x;
    const float dy = p.y - c.y;
    const float ds = dx * dx + dy * dy;
    if (ds < best) {
      best = ds;
      out = &c;
    }
  }
  return best < 256.f ? out : nullptr;
}

// program entry
int main(const int argc, const char **args) {

  if (SDL_Init(SDL_INIT_VIDEO)) {
    return 1;
  }

  SDL_Surface *surf = SDL_SetVideoMode(512, 512, 32, 0);
  if (!surf) {
    return 2;
  }

  std::array<float2, 3> tri = {
    float2{256, 64},
    float2{512 - 64, 512 - 64},
    float2{64, 512 - 64}};

  float2 *drag = nullptr;
  bool dragall = false;

  bool active = true;
  while (active) {

    SDL_FillRect(surf, nullptr, 0x101010);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
      case SDL_QUIT:
        active = false;
        break;
      case SDL_MOUSEBUTTONUP:
        drag = nullptr;
        dragall = false;
        break;
      case SDL_MOUSEBUTTONDOWN:
        if (!drag) {
          const float2 p =
              float2{float(event.button.x), float(event.button.y)};
          // check for drags
          drag = nearest(tri, p);
          if (!drag) {
            dragall = point_in_tri(tri[0], tri[1], tri[2], p);
          }
          // clear relative mouse state
          SDL_GetRelativeMouseState(nullptr, nullptr);
        }
        break;
      }
    }

    if (drag || dragall) {
      int rx, ry;
      SDL_GetRelativeMouseState(&rx, &ry);
      if (drag) {
        drag->x += rx;
        drag->y += ry;
      }
      if (dragall) {
        for (auto &c : tri) {
          c.x += rx;
          c.y += ry;
        }
      }
    }

    uint32_t tri_rgb = 0xff6040;

    // draw screen
    if (is_backface(tri[0], tri[1], tri[2])) {
      tri_rgb = 0xdadada;
    }

    // draw triangle
    raster(surf, tri);
    clip_t<float2> clip{tri[0], tri[1], tri[2]};
    quad_tree(surf, clip, rect2f_t {0, 0, 512, 512});

    // draw vertex nodes
    for (const auto &p : tri) {
      SDL_Rect r = {int16_t(p.x - 3), int16_t(p.y - 3), 7, 7};
      SDL_FillRect(surf, &r, 0xdadada);
    }

    SDL_Flip(surf);
    SDL_Delay(1);
  }

  return 0;
}
