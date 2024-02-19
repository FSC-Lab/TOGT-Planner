#pragma once

#include <limits>
#include <Eigen/Eigen>

namespace drolib {

template <typename T> 
class MinMaxRecorder {
 public:
  MinMaxRecorder() = default;
  MinMaxRecorder(const T& initVal)
    : min_(initVal), max_(initVal) {}
    
  inline void reset() {
    min_ = std::numeric_limits<T>::max();
    max_ = std::numeric_limits<T>::min();
  }

  inline bool valid() const {
    if (min_ == std::numeric_limits<T>::max() || 
        max_ == std::numeric_limits<T>::min()) {
      return false;
    }
    return true;
  }

  void add(const T &val) {
    if (val < min_) {
      min_ = val;
    }
    if (val > max_) {
      max_ = val;
    }
  }

  inline T min() const { return min_; }
  inline T max() const { return max_; }

private:
  T min_{std::numeric_limits<T>::max()};
  T max_{std::numeric_limits<T>::min()};
};

template <typename T, int N> 
class MinMaxVecRecorder {
 public:
  MinMaxVecRecorder() {
    reset();
  }

  inline void reset() {
    min_.setConstant(std::numeric_limits<T>::max());
    max_.setConstant(std::numeric_limits<T>::min());
  }

  inline bool valid() const {
    for (size_t i{0}; i < N; ++i) {
      if (min_(i) == std::numeric_limits<T>::max() || 
          max_(i) == std::numeric_limits<T>::min()) {
        return false;
      }
    }

    return true;
  }

  void add(const Eigen::Matrix<T, N, 1> &vec) {
    for (size_t i{0}; i < N; ++i) {
      if (vec(i) < min_(i)) {
        min_(i) = vec(i);
      }
      if (vec(i) > max_(i)) {
        max_(i) = vec(i);
      }
    }
  }

  inline Eigen::Matrix<T, N, 1> min() const { return min_; }
  inline Eigen::Matrix<T, N, 1> max() const { return max_; }

private:
  Eigen::Matrix<T, N, 1> min_;
  Eigen::Matrix<T, N, 1> max_;
};

} // namespace drolib