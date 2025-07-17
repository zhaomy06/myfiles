

#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

constexpr double kMathEpsilon=0.000000001;

inline double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}



inline double PtSegDistance(double query_x, double query_y, double start_x,
                     double start_y, double end_x, double end_y,
                     double length) {
  const double x0 = query_x - start_x;
  const double y0 = query_y - start_y;
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double proj = x0 * dx + y0 * dy;
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length * length) {
    return hypot(x0 - dx, y0 - dy);
  }
  return std::abs(x0 * dy - y0 * dx) / length;
}

inline bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

class Vec2d {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  double Angle() const { return std::atan2(y_, x_); }

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }
  Vec2d operator+(const Vec2d &other) const {
  return Vec2d(x_ + other.x(), y_ + other.y());
}

  Vec2d operator-(const Vec2d &other) const {
  return Vec2d(x_ - other.x(), y_ - other.y());
}

  Vec2d operator*(const double ratio) const {
  return Vec2d(x_ * ratio, y_ * ratio);
}

  Vec2d operator/(const double ratio) const {
  assert(std::abs(ratio) > kMathEpsilon);
  return Vec2d(x_ / ratio, y_ / ratio);
}
  double CrossProd(const Vec2d &other) const {
  return x_ * other.y() - y_ * other.x();
}

double InnerProd(const Vec2d &other) const {
  return x_ * other.x() + y_ * other.y();
}

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

inline double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}


class LineSegment2d {
 public:

  LineSegment2d()=default;

  LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
    const double dx = end_.x() - start_.x();
    const double dy = end_.y() - start_.y();
    length_ = std::pow(dx*dx+dy*dy,0.5);
    unit_direction_ =
        (length_ <= kMathEpsilon ? Vec2d(0, 0)
                                : Vec2d(dx / length_, dy / length_));
    heading_ = unit_direction_.Angle();

    InitMaxMin();
    }
  void InitMaxMin() {
  min_x_ = std::min(start_.x(), end_.x());
  max_x_ = std::max(start_.x(), end_.x());
  min_y_ = std::min(start_.y(), end_.y());
  max_y_ = std::max(start_.y(), end_.y());
  }   
  
void Set(const double start_x, const double start_y,
                        const double end_x, const double end_y) {
  start_.set_x(start_x);
  start_.set_y(start_y);
  end_.set_x(end_x);
  end_.set_y(end_y);
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  length_ = std::pow(dx*dx+dy*dy,0.5);
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();

  InitMaxMin();
}

  const Vec2d &start() const { return start_; }


  const Vec2d &end() const { return end_; }

  
  double length() const { return length_; }

    bool IsPointIn(const Vec2d &point) const {
        if (length_ <= kMathEpsilon) {
            return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
                std::abs(point.y() - start_.y()) <= kMathEpsilon;
        }
        const double prod = CrossProd(point, start_, end_);

        if (std::abs(prod) > kMathEpsilon) {
            return false;
        }
        return IsWithin(point.x(), start_.x(), end_.x()) &&
                IsWithin(point.y(), start_.y(), end_.y());
    }

    bool GetIntersect(const LineSegment2d &other_segment,
                                 Vec2d *const point) const {
    assert(point != nullptr);
    if (IsPointIn(other_segment.start())) {
        *point = other_segment.start();
        return true;
    }
    if (IsPointIn(other_segment.end())) {
        *point = other_segment.end();
        return true;
    }
    if (other_segment.IsPointIn(start_)) {
        *point = start_;
        return true;
    }
    if (other_segment.IsPointIn(end_)) {
        *point = end_;
        return true;
    }
    if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
        return false;
    }
    const double cc1 = CrossProd(start_, end_, other_segment.start());
    const double cc2 = CrossProd(start_, end_, other_segment.end());
    if (cc1 * cc2 >= -kMathEpsilon) {
        return false;
    }
    const double cc3 =
        CrossProd(other_segment.start(), other_segment.end(), start_);
    const double cc4 =
        CrossProd(other_segment.start(), other_segment.end(), end_);
    if (cc3 * cc4 >= -kMathEpsilon) {
        return false;
    }
    const double ratio = cc4 / (cc4 - cc3);
    *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                    start_.y() * ratio + end_.y() * (1.0 - ratio));
    return true;
}

    bool HasIntersect(const LineSegment2d &other_segment) const {
    Vec2d point;
    return GetIntersect(other_segment, &point);
    }


  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }
 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;

  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
};




class Box2d {
 public:
  Box2d() = default;
 
  Box2d(const Vec2d &center, const double heading, const double length,
        const double width): center_(center),
      length_(length),
      width_(width),
      half_length_(length / 2.0),
      half_width_(width / 2.0),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {

    InitCorners();
}

  const Vec2d &center() const { return center_; }

  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  double length() const { return length_; }
  double width() const { return width_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double heading() const { return heading_; }
  double cos_heading() const { return cos_heading_; }
  double sin_heading() const { return sin_heading_; }

void InitCorners() {
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(center_.x() + dx1 + dx2, center_.y() + dy1 + dy2);
  corners_.emplace_back(center_.x() + dx1 - dx2, center_.y() + dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 - dx2, center_.y() - dy1 - dy2);
  corners_.emplace_back(center_.x() - dx1 + dx2, center_.y() - dy1 + dy2);

  for (auto &corner : corners_) {
    max_x_ = std::fmax(corner.x(), max_x_);
    min_x_ = std::fmin(corner.x(), min_x_);
    max_y_ = std::fmax(corner.y(), max_y_);
    min_y_ = std::fmin(corner.y(), min_y_);
  }
}

bool IsPointIn(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
  return dx <= half_length_ + kMathEpsilon && dy <= half_width_ + kMathEpsilon;
}

double DistanceTo(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return std::pow(dx*dx+dy*dy,0.5);
}

double DistanceTo(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return DistanceTo(line_segment.start());
  }
  const double ref_x1 = line_segment.start().x() - center_.x();
  const double ref_y1 = line_segment.start().y() - center_.y();
  double x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
  double y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
  double box_x = half_length_;
  double box_y = half_width_;
  int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
  int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
  if (gx1 == 0 && gy1 == 0) {
    return 0.0;
  }
  const double ref_x2 = line_segment.end().x() - center_.x();
  const double ref_y2 = line_segment.end().y() - center_.y();
  double x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
  double y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
  int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
  int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
  if (gx2 == 0 && gy2 == 0) {
    return 0.0;
  }
  if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
    x1 = -x1;
    gx1 = -gx1;
    x2 = -x2;
    gx2 = -gx2;
  }
  if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
    y1 = -y1;
    gy1 = -gy1;
    y2 = -y2;
    gy2 = -gy2;
  }
  if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
    std::swap(x1, y1);
    std::swap(gx1, gy1);
    std::swap(x2, y2);
    std::swap(gx2, gy2);
    std::swap(box_x, box_y);
  }
  if (gx1 == 1 && gy1 == 1) {
    switch (gx2 * 3 + gy2) {
      case 4:
        return PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                             line_segment.length());
      case 3:
        return (x1 > x2) ? (x2 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 2:
        return (x1 > x2) ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                         line_segment.length())
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case -1:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -4:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.0
                   ? PtSegDistance(box_x, -box_y, x1, y1, x2, y2,
                                   line_segment.length())
                   : (CrossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.0
                          ? 0.0
                          : PtSegDistance(-box_x, box_y, x1, y1, x2, y2,
                                          line_segment.length()));
    }
  } else {
    switch (gx2 * 3 + gy2) {
      case 4:
        return (x1 < x2) ? (x1 - box_x)
                         : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                         line_segment.length());
      case 3:
        return std::min(x1, x2) - box_x;
      case 1:
      case -2:
        return CrossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.0
                   ? 0.0
                   : PtSegDistance(box_x, box_y, x1, y1, x2, y2,
                                   line_segment.length());
      case -3:
        return 0.0;
    }
  }
  return 0.0;
}

bool HasOverlap(const LineSegment2d &line_segment) const {
  if (line_segment.length() <= kMathEpsilon) {
    return IsPointIn(line_segment.start());
  }
  if (std::fmax(line_segment.start().x(), line_segment.end().x()) < min_x() ||
      std::fmin(line_segment.start().x(), line_segment.end().x()) > max_x() ||
      std::fmax(line_segment.start().y(), line_segment.end().y()) < min_y() ||
      std::fmin(line_segment.start().y(), line_segment.end().y()) > max_y()) {
    return false;
  }
  return DistanceTo(line_segment) <= kMathEpsilon;
}

bool HasOverlap(const Box2d &box) const {
  if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
      box.min_y() > max_y()) {
    return false;
  }

  const double shift_x = box.center_x() - center_.x();
  const double shift_y = box.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}


  /**
   * @brief Constructor which takes the heading-axis and the width of the box
   * @param axis The heading-axis
   * @param width The width of the box, which is taken perpendicularly
   * to the heading direction.
   */
  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }

 private:
  Vec2d center_;
  double length_ = 0.0;
  double width_ = 0.0;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;

  std::vector<Vec2d> corners_;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

class Obbox {
 public:
  Obbox() = default;
  explicit Obbox(const std::vector<Eigen::Vector2d> &points) {
    points_ = points;
    SaveLines(points);
    ComputeOBBUsingPCA(points);
  }


  bool IsCollision(const LineSegment2d &line) const {
    const auto &lines = GetLines();
    if (lines.size() == 0) {
      return false;
    } else if (lines.size() == 1) {
      return lines[0].HasIntersect(line);
    }
    // skip if the line segment is outside of bounding box
    if (!GetObbox().HasOverlap(line)) {
      return false;
    }

    for (const auto &obb_line : lines) {
      if (obb_line.max_x() < line.min_x() || obb_line.min_x() > line.max_x() ||
          obb_line.max_y() < line.min_y() || obb_line.min_y() > line.max_y()) {
        continue;
      }

      if (obb_line.HasIntersect(line)) {
        return true;
      }
    }
    return false;
  }

  bool IsCollision(const Box2d &input_box) const {
    if (!GetObbox().HasOverlap(input_box)) {
      return false;
    }
    const auto &lines = GetLines();
    for (const auto &obb_line : lines) {
      if (input_box.HasOverlap(obb_line)) {
        return true;
      }
    }
    return false;
  }

  void SaveLines(const std::vector<Eigen::Vector2d> &points) {
    if (points.size() < 2) {
      return;
    }
    lines_.resize(points.size() - 1);
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const auto &pt1 = points[i];
      const auto &pt2 = points[i + 1];
      lines_[i].Set(pt1.x(), pt1.y(), pt2.x(), pt2.y());
    }
  }


  Eigen::Vector2d ComputeMeanPoint(const std::vector<Eigen::Vector2d> &points) {
    Eigen::Vector2d mean(0, 0);
    for (const auto &point : points) {
      mean += point;
    }
    mean /= points.size();
    return mean;
  }

  Eigen::Matrix2d ComputeCovarianceMatrix(
      const std::vector<Eigen::Vector2d> &points, const Eigen::Vector2d &mean) {
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto &point : points) {
      const Eigen::Vector2d centered = point - mean;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size());
    return covariance;
  }

  void ComputeOBBUsingPCA(const std::vector<Eigen::Vector2d> &points) {
    const auto mean = ComputeMeanPoint(points);
    const Eigen::Matrix2d covariance = ComputeCovarianceMatrix(points, mean);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);

    const Eigen::Vector2d &axis1 = solver.eigenvectors().col(0).normalized();
    const Eigen::Vector2d &axis2 = solver.eigenvectors().col(1).normalized();
    const double heading = std::atan2(axis1.y(), axis1.x());  //
    double minDot[2] = {std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max()};
    double maxDot[2] = {std::numeric_limits<double>::lowest(),
                        std::numeric_limits<double>::lowest()};
    for (const auto &point : points) {
      const Eigen::Vector2d centered = point - mean;
      const double dot1 = axis1.dot(centered);
      const double dot2 = axis2.dot(centered);

      minDot[0] = std::fmin(minDot[0], dot1);
      maxDot[0] = std::fmax(maxDot[0], dot1);

      minDot[1] = std::fmin(minDot[1], dot2);
      maxDot[1] = std::fmax(maxDot[1], dot2);
    }
    const Eigen::Vector2d obb_center = mean +
                                       (axis1 * (minDot[0] + maxDot[0]) / 2) +
                                       (axis2 * (minDot[1] + maxDot[1]) / 2);
    const double length = maxDot[0] - minDot[0];
    const double width = maxDot[1] - minDot[1];
    obbox_ = Box2d({obb_center.x(), obb_center.y()}, heading, length, width);
  }
  const Box2d &GetObbox() const { return obbox_; }
  const std::vector<LineSegment2d> &GetLines() const { return lines_; }
  const std::vector<Eigen::Vector2d> &GetPoints() const { return points_; }

 private:
  Box2d obbox_;
  std::vector<LineSegment2d> lines_;
  std::vector<Eigen::Vector2d> points_{};
};