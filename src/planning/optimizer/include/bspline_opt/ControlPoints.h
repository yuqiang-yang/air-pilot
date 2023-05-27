#ifndef ControlPoints_H_
#define ControlPoints_H_

namespace air_pilot
{
  class ControlPoints // Control point on trajectory
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> temp_flag;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      temp_flag.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      temp_flag.resize(size);
    }
  };
} // namespace air_pilot
#endif