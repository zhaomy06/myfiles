#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "google/protobuf/repeated_field.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

namespace npp {

namespace pnc {
namespace math_utils {

template <typename T>
T clamp(const T& low, const T& high, const T& x)
{
    return std::max(low, std::min(high, x));
}

template <class T>
static inline void ResizeAndResetEigen(T &x, size_t n) {
  x.resize(n);
  x.setZero();
}

template <class T>
static inline void ResizeAndResetEigen(T &x, size_t m, size_t n) {
  x.resize(m, n);
  x.setZero();
}

template <class T>
static inline void ResizeEigenVec(std::vector<T> &x_vec, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(n);
  }
}

template <class T>
static inline void ResetEigenVec(std::vector<T> &x_vec) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].setZero();
  }
}

template <class T>
static inline void ResizeAndResetEigenVec(std::vector<T> &x_vec, size_t N,
                                          size_t n) {
  x_vec.resize(N);
  ResizeEigenVec(x_vec, n);
  ResetEigenVec(x_vec);
}

template <class T>
static inline void ResizeEigenMat(std::vector<T> &x_vec, size_t m, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(m, n);
  }
}

template <class T>
static inline void ResizeAndResetEigenMat(std::vector<T> &x_vec, size_t N,
                                          size_t m, size_t n) {
  x_vec.resize(N);
  ResizeEigenMat(x_vec, m, n);
  ResetEigenVec(x_vec);
}

template <typename T>
static inline T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

template <class T>
T Clip(T x, T max_x, T min_x) {
  if (x > max_x) return max_x;
  if (x < min_x) return min_x;
  return x;
}

static inline bool IsDoubleEqual(double a, double b) {
  if (fabs(a - b) < 1E-8) {
    return true;
  } else {
    return false;
  }
}

static inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

class LinearInterp {
 public:
  struct InterpolatedIndex {
    int id{0};
    double ratio{0.0};
    InterpolatedIndex(int id, double ratio) : id(id), ratio(ratio) {}
  };

 public:
  LinearInterp() = default;
  ~LinearInterp() = default;

 public:
  static inline double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
      a += (2.0 * M_PI);
    }
    return a - M_PI;
  }

  static inline double InterpolateAngle(double y1, double y2, double weight) {
    y1 = NormalizeAngle(y1);
    y2 = NormalizeAngle(y2);

    if (y1 - y2 > M_PI) {
      y2 += M_PI * 2;
    } else if (y2 - y1 > M_PI) {
      y1 += M_PI * 2;
    }

    auto y = (1 - weight) * y1 + weight * y2;
    y = NormalizeAngle(y);

    return y;
  }
  static InterpolatedIndex GetIndexFromS(const std::vector<double> &s_list,
                                         const double s) {
    if (s_list.empty()) {
      return {0, 0.0};
    }
    if (s <= s_list.front()) {
      return {0, 0.0};
    }
    if (s >= s_list.back()) {
      return {static_cast<int>(s_list.size() - 2), 1.0};
    }
    auto it = std::upper_bound(s_list.begin(), s_list.end(), s);
    if (it == s_list.end()) {
      return {static_cast<int>(s_list.size() - 2), 1.0};
    }
    const int id = static_cast<int>(std::distance(s_list.begin(), it)) - 1;
    const double ratio = (s - s_list[id]) / (s_list[id + 1] - s_list[id]);
    return {id, ratio};
  }
  template <typename T>
  static std::pair<bool, T> Interp(const std::vector<T> &data,
                                   const std::vector<double> &s_list,
                                   const double s) {
    if (data.empty()) {
      return {false, T()};
    }
    if (data.size() != s_list.size()) {
      return {false, T()};
    }
    if (data.size() == 1) {
      return {true, data.front()};
    }
    const auto index = GetIndexFromS(s_list, s);
    return {true, GetInterpBetween(data, index)};
  }
  /*
   * This function requires xp to be sorted in nondecreasing order
   */
  template <typename T>
  static inline T Interp(T x, const std::vector<T> &xp,
                         const std::vector<T> &fp) {
    const size_t N = xp.size();
    size_t hi;
    for (hi = 0; hi < N && x > xp[hi]; hi++) {
    }

    if (hi == 0) return fp[0];

    if (hi == N && x > xp[N - 1]) return fp[N - 1];

    const size_t low = hi - 1;
    const T xp_diff = xp[hi] - xp[low];
    if (xp_diff < static_cast<T>(1e-5f) && xp_diff > static_cast<T>(-1e-5f))
      return fp[low];

    return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
  }
  static double GetInterpBetween(const std::vector<double> &data,
                                 const InterpolatedIndex &index) {
    const auto &p0 = data[index.id];
    const auto &p1 = data[index.id + 1];
    double ret{};
    const auto ratio = index.ratio;
    ret = p0 + (p1 - p0) * ratio;
    return ret;
  }
};

class ProtoUtil {
 public:
  template <typename T>
  static void EnsureSize(
      google::protobuf::RepeatedPtrField<T> *repeated_ptr_field,
      int target_size) {
    if (repeated_ptr_field->size() == target_size) {
      // 如果size已经是预期了，无需操作
      return;
    } else if (repeated_ptr_field->size() < target_size) {
      // 如果当前的size小于target size，那么就reserver并且add
      repeated_ptr_field->Reserve(target_size);
      do {
        repeated_ptr_field->Add();
      } while (repeated_ptr_field->size() < target_size);
    } else {
      // 如果当前的size大于target size，就clear，并且add
      do {
        repeated_ptr_field->RemoveLast();
      } while (repeated_ptr_field->size() > target_size);
    }
    // 最终，得到的数组size和目标size是相同的
  }

  template <typename T>
  static void EnsureSize(google::protobuf::RepeatedField<T> *repeated_field,
                         int target_size, T value) {
    if (repeated_field->size() == target_size) {
      // 如果size已经是预期了，无需操作
      return;
    } else if (repeated_field->size() < target_size) {
      // 如果当前的size小于target size，那么就reserver并且add
      repeated_field->Reserve(target_size);
      do {
        repeated_field->Add(value);
      } while (repeated_field->size() < target_size);
    } else {
      // 如果当前的size大于target size，就clear，并且add
      do {
        repeated_field->RemoveLast();
      } while (repeated_field->size() > target_size);
    }
    // 最终，得到的数组size和目标size是相同的
  }
};

}  // namespace math_utils
}  // namespace pnc
}  // namespace npp
