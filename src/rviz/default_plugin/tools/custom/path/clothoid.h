#ifndef TRAJ_TOOLS_PATH_CLOTHOID_H_
#define TRAJ_TOOLS_PATH_CLOTHOID_H_

#include "path/path_2d.h"

namespace traj_tools {

class Clothoid : public Path2d
{
public:
  Clothoid();

  Clothoid(const Vec2& pos0,
           Float theta0,
           Float m_start_k,
           Float c,
           Float s);

  void Copy(const Clothoid& clothoid);
  /**
   * @brief Copy from another clothoid, from the start to the given length
   * @param clothoid copy source
   * @param length end of the copy range
   */
  void Copy(const Clothoid& clothoid, Float length);

  void Reset(const Vec2& pos0,
             Float theta0,
             Float m_start_k,
             Float c,
             Float s);

  void Reset(const Vec2& pos0,
             Float theta0,
             Float k0);

  virtual Float GetPos(Float s,
                       Float& x,
                       Float& y,
                       Float& k,
                       Float& theta) const;

  Clothoid* CutAtDistance(Float s);

  inline Float delta_y() const {
    return end_pos_[1] - start_pos_[1];
  }

  void Translate(const Vec2& delta_pos);

  static inline Float calc_fresnel_cos(Float s);

  static inline Float calc_fresnel_sin(Float s);

  static inline void calc_fresnel(Float s, Float& fresnel_cos, Float& fresnel_sin);

private:
  void CopyFromPos(const Clothoid& clothoid,
                   Float x0,
                   Float y0,
                   Float theta0,
                   Float k0,
                   Float distance);

  inline void calc_pos(Float pos,
                       Float& x,
                       Float& y,
                       Float& k,
                       Float& theta) const;
  friend class SShapeClothoid;
  Float c_;
  Float m_;             ///temporary variable：sqrt(c / 2)
  Float p_;             ///temporary variable：k0 / c
  Float e_;             ///temporary variable：sqrt(2 / c) * cos(theta0 - k0^2 / (2 * c))
  Float f_;             ///temporary variable：sqrt(2 / c) * sin(theta0 - k0^2 / (2 * c))
  Float g_;             ///temporary variable：sqrt(2 / c)[sin(theta0 - k0^2 / (2 * c)) * integral(sin(t^2))(0, sqrt(c / 2) * k0)
  // - cos(theta0 - k0^2 / (2 * c)) * integral(cos(t^2))(0, sqrt(c / 2) * k0)]
  Float h_;             ///temporary variable：sqrt(2 / c)[cos(theta0 - k0^2 / (2 * c)) * integral(sin(t^2))(0, sqrt(c / 2) * k0)
  // - sin(theta0 - k0^2 / (2 * c)) * integral(cos(t^2))(0, sqrt(c / 2) * k0)]
};

}       //namespace traj_tools

#endif // TRAJ_TOOLS_PATH_CLOTHOID_H_
