#pragma once
#include <cmath>
#include <complex>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <vector>
namespace a3_math {
typedef struct {
  float x, y;
} vec2;

vec2 left(vec2 v) { return {-v.y, v.x}; }
vec2 right(vec2 v) { return {v.y, -v.x}; }

const float dot(const vec2 a, const vec2 b) { return a.x * b.x + a.y * b.y; }
const float cross(const vec2 a, const vec2 b) { return a.x * b.y - a.y * b.x; }
const float len_sq(const vec2 v) { return dot(v, v); }

const float len(const vec2 v) { return std::sqrtf(len_sq(v)); }

const vec2 mul(vec2 a, vec2 b) { return {a.x * b.x, a.y * b.y}; }
const vec2 mul(vec2 a, float b) { return mul(a, {b, b}); }
const vec2 mul(float a, vec2 b) { return mul({a, a}, b); }
const vec2 div(vec2 a, vec2 b) { return {a.x / b.x, a.y / b.y}; }

const vec2 div(vec2 a, float b) { return mul(a, 1.0f / b); }

const vec2 normalize(const vec2 v) { return div(v, len(v)); }

const vec2 neg(const vec2 v) { return {-v.x, -v.y}; }
const vec2 sub(const vec2 lhs, const vec2 rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}
const vec2 add(const vec2 lhs, const vec2 rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

const vec2 lerp(const vec2 a, const vec2 b, const float t) {
  return add(mul(sub(b, a), t), a);
}
const float dist(const vec2 a, const vec2 b) { return len(sub(a, b)); }
typedef struct {
  vec2 x, y;
} mat2;
vec2 mul(const mat2 &mat, const vec2 v) {
  return add(mul(mat.x, v.x), mul(mat.y, v.y));
}
typedef struct {
  vec2 pos, front;
} pose;

const mat2 orientation(const pose &pose) {
  return {pose.front, left(pose.front)};
}
const vec2 dir_to_local(const pose &frame, const vec2 dir) {
  mat2 m = orientation(frame);
  return {dot(m.x, dir), dot(m.y, dir)};
}
const vec2 p_to_local(const pose &frame, const vec2 p) {
  return dir_to_local(frame, sub(p, frame.pos));
}

const pose to_local(const pose &frame, const pose &pose) {
  return {p_to_local(frame, pose.pos), dir_to_local(frame, pose.front)};
}
const vec2 dir_to_world(const pose &frame, const vec2 dir) {
  mat2 m = orientation(frame);
  return mul(m, dir);
}

const vec2 p_to_world(const pose &frame, const vec2 p) {
  return add(frame.pos, dir_to_world(frame, p));
}

const pose to_world(const pose &frame, const pose &pose) {
  return {p_to_world(frame, pose.pos), dir_to_world(frame, pose.front)};
}

const float wrap(const float val, const float min, const float max) {
  const float range = max - min;
  if (range < 0.0f)
    return min;
  return std::fmodf(std::fmodf(val - min, range) + range, range) + min;
}

const float wrap_rad(const float val) {
  return std::fmodf(std::fmodf(val, M_PI_2f) + M_PI_2f, M_PI_2f);
}
#define DEG_TO_RAD M_1_PIf * 180.0f
#define RAD_TO_DEG M_PIf / 180.0f
class angle {
  float val;

public:
  angle(const float rad) { val = rad; }

  const float rad() const { return val; }

  const float deg() const { return val * (DEG_TO_RAD); }

  const angle clean() const { return angle(wrap_rad(val)); }
};

typedef struct {
  float len;
  angle orientation;
} polar;

angle deg_to_angle(const float deg) { return angle(deg * RAD_TO_DEG); }

angle angle_diff(const angle a, const angle b) {
  float delta = a.rad() - b.rad();
  const bool neg = delta < 0;
  if (neg)
    delta = -delta;

  delta -= M_PIf * std::fmodf(delta, M_PIf);
  if (neg)
    delta = -delta;
  return delta;
}

#undef DEG_TO_RAD
#undef RAD_TO_DEG
const angle orientation(const vec2 v) {
  vec2 v_norm = normalize(v);
  return angle(std::atan2f(v_norm.y, v_norm.x));
}
const angle signed_angle_between(const vec2 a, const vec2 b) {
  vec2 a_norm = normalize(a);
  vec2 b_norm = normalize(b);
  return std::atan2(cross(a_norm, b_norm), dot(a_norm, b_norm));
}
const polar to_polar(const vec2 v) { return {len(v), orientation(v)}; }

const void total_diff(const pose &world, const pose &target, vec2 &movement_vec,
                      angle &orientation_delta) {
  const pose rel = to_local(world, target);
  movement_vec = rel.pos;
  orientation_delta = orientation(rel.front);
}

std::optional<vec2> tdoa(const vec2 *anchors, const float *pings,
                         const int count, const vec2 guess, const int max_iter,
                         const float eps, const float max_step,
                         const float accept) {
  if (count < 3)
    return std::nullopt;

  const float eps_sq = eps * eps;

  std::vector<float> tdoa_dist(count - 1);
  std::vector<float> r(count);

  const float ping_0 = pings[0];

  for (int i = 1; i < count; ++i)
    tdoa_dist[i - 1] = pings[i] - ping_0;

  vec2 p = guess;

  const float max_step_sq = max_step * max_step;

  constexpr float lambda = 1e-3f;
  float step_sq;

  for (int iter = 0; iter < max_iter; ++iter) {
    for (int i = 0; i < count; ++i)
      r[i] = std::max(dist(p, anchors[i]), eps);

    const float r0 = r[0];

    float a11 = 0.0f;
    float a12 = 0.0f;
    float a22 = 0.0f;

    float b1 = 0.0f;
    float b2 = 0.0f;
    for (int i = 1; i < count; ++i) {
      const float ri = r[i];

      const float fi = (ri - r0) - tdoa_dist[i - 1];

      const vec2 j =
          sub(div(sub(p, anchors[i]), ri), div(sub(p, anchors[0]), r0));

      a11 += j.x * j.x;
      a12 += j.x * j.y;
      a22 += j.y * j.y;

      b1 += j.x * fi;
      b2 += j.y * fi;
    }

    a11 += lambda;
    a22 += lambda;

    const float det = a11 * a22 - a12 * a12;

    if (fabsf(det) < 1e-8f)
      continue;

    vec2 delta;

    delta.x = (-b1 * a22 + b2 * a12) / det;

    delta.y = (b1 * a12 - b2 * a11) / det;

    step_sq = len_sq(delta);

    if (step_sq > max_step_sq) {
      const float step = std::sqrt(step_sq);

      delta = mul(div(delta, step), max_step);
    }
    float step = len(delta);

    float scale2 = 1.0f;

    if (step > max_step)
      scale2 = max_step / step;

    p = add(p, mul(delta, 0.5f * scale2));

    if (step_sq < eps_sq)
      return p;
  }
  if (step_sq < accept)
    return p;
  return std::nullopt;
}
const std::optional<vec2> tdoa(const vec2 *anchors, const float *pings,
                               const int count, const int max_iter,
                               const float eps, const float max_step,
                               const float accept) {
  if (count <= 0)
    return std::nullopt;
  vec2 guess = anchors[0];
  for (int i = 1; i < count; i++)
    guess = add(guess, anchors[i]);
  guess = div(guess, count);
  return tdoa(anchors, pings, count, guess, max_iter, eps, max_step, accept);
}
} // namespace a3_math
