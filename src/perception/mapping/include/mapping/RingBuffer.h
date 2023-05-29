#pragma once

template <typename _Datatype>
struct RingBuffer {
 public:
  int size_x, size_y, size_z;
  std::vector<_Datatype> data;

  inline void setup(int _size_x, int _size_y, int _size_z) {
    size_x = _size_x;
    size_y = _size_y;
    size_z = _size_z;
    data.resize(size_x * size_y * size_z);
  }
  inline const int idx2add(int x, int N) const {
    // return x % N >= 0 ? x % N : x % N + N;
    // NOTE this is much faster than before!!
    return (x & N) >= 0 ? (x & N) : (x & N) + N;
  }
  inline const Eigen::Vector3i idx2add(const Eigen::Vector3i& id) const {
    return Eigen::Vector3i(idx2add(id.x(), size_x - 1),
                           idx2add(id.y(), size_y - 1),
                           idx2add(id.z(), size_z - 1));
  }
  // NOTE dangerous!! ad should be the address in the data
  inline const _Datatype& at(const Eigen::Vector3i& ad) const {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype& at(const Eigen::Vector3i& ad) {
    return data[(ad.z() * size_y + ad.y()) * size_x + ad.x()];
  }
  inline _Datatype* atPtr(const Eigen::Vector3i& ad) {
    return &(data[(ad.z() * size_y + ad.y()) * size_x + ad.x()]);
  }
  inline const _Datatype& atId(const Eigen::Vector3i& id) const {
    return at(idx2add(id));
  }
  inline _Datatype& atId(const Eigen::Vector3i& id) {
    return at(idx2add(id));
  }
  inline _Datatype* atIdPtr(const Eigen::Vector3i& id) {
    return atPtr(idx2add(id));
  }
  inline void fillData(const _Datatype& val) {
    std::fill(data.begin(), data.end(), val);
  }
};
