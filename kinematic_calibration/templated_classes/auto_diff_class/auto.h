#ifndef AUTO
#define AUTO

#include "cmath"
template <int N, typename T> class Auto {
public:
  T val;
  T epsilon[N];
  Auto(T _val) : val(_val) {
    for (int i = 0; i < N; i++) {
      epsilon[i] = 0;
    }
  }
  Auto() {
    val = 0;

    for (int i = 0; i < N; i++) {
      epsilon[i] = 0;
    }
  }
  // new constructor
  Auto(const Auto<N, T> &a) {
    val = a.val;
    for (int i = 0; i < N; i++) {
      epsilon[i] = a.epsilon[i];
    }
  }
  Auto operator+(const Auto &a) const {
    Auto<N, T> ret;
    ret.val = this->val + a.val;
    for (int i = 0; i < N; i++) {
      ret.epsilon[i] = this->epsilon[i] + a.epsilon[i];
    }
    return ret;
  }

  Auto &operator+=(const Auto &a) {
    this->val = this->val + a.val;
    for (int i = 0; i < N; i++) {
      this->epsilon[i] = this->epsilon[i] + a.epsilon[i];
    }
    return *this;
  }
  Auto &operator-=(const Auto &a) {
    this->val = this->val - a.val;
    for (int i = 0; i < N; i++) {
      this->epsilon[i] = this->epsilon[i] - a.epsilon[i];
    }
    return *this;
  }

  bool operator<(const Auto &b) const { return this->val < b.val; }
  bool operator>(const Auto &b) const { return this->val > b.val; }
  Auto operator*(const Auto &a) const {
    Auto<N, T> ret;
    ret.val = this->val * a.val;
    for (int i = 0; i < N; i++) {
      ret.epsilon[i] = this->epsilon[i] * a.val + a.epsilon[i] * this->val;
    }
    return ret;
  }
  Auto operator-(const Auto &a) const {
    Auto<N, T> ret;
    ret.val = this->val - a.val;
    for (int i = 0; i < N; i++) {
      ret.epsilon[i] = this->epsilon[i] - a.epsilon[i];
    }
    return ret;
  }

  Auto operator-() const {
    Auto out;
    out.val = -val;
    for (int i = 0; i < N; ++i)
      out.epsilon[i] = -epsilon[i];
    return out;
  }
  Auto operator/(const Auto &a) const {
    Auto<N, T> ret;
    ret.val = this->val / a.val;
    T c_squared = a.val * a.val;
    for (int i = 0; i < N; i++) {
      ret.epsilon[i] =
          (this->epsilon[i] * a.val - a.epsilon[i] * this->val) / (c_squared);
    }
    return ret;
  }
};

template <int N, typename T> Auto<N, T> sin(const Auto<N, T> &u) {
  Auto<N, T> out(std::sin(u.val));
  for (int i = 0; i < N; ++i)
    out.epsilon[i] = std::cos(u.val) * u.epsilon[i];
  return out;
}

template <int N, typename T> Auto<N, T> cos(const Auto<N, T> &u) {
  Auto<N, T> out(std::cos(u.val));
  for (std::size_t i = 0; i < N; ++i)
    out.epsilon[i] = -std::sin(u.val) * u.epsilon[i];
  return out;
}

template <int N, typename T> Auto<N, T> sqrt(const Auto<N, T> &u) {
  T r_sqrt = std::sqrt(u.val);
  Auto<N, T> out(r_sqrt);
  T coeff = 0.5 / r_sqrt;
  for (int i = 0; i < N; ++i)
    out.epsilon[i] = coeff * u.epsilon[i];
  return out;
}
template <int N, typename T> void initialize(const T *params) {
  for (int i = 0; i < N; i++) {
    Auto<N, T> p(params[i]);
  }
}

template <int N, typename T> Auto<N, T> acos(const Auto<N, T> &a) {
  Auto<N, T> ret;
  ret.val = acos(a.val);
  T d = -1.0 / sqrt(1.0 - a.val * a.val); // derivative wrt a.val
  for (int i = 0; i < N; i++) {
    ret.epsilon[i] = d * a.epsilon[i];
  }
  return ret;
}

#endif // !AUTO
