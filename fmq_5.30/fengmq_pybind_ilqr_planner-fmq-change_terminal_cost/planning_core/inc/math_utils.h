#include<cmath>


inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

inline double clip(double x,double low,double high){
    x=(x>high)?high:x;
    x=(x<low)?low:x;
    return x;
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