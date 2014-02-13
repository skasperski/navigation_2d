/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef __OpenKarto__PoseTransform_h__
#define __OpenKarto__PoseTransform_h__

#include <OpenKarto/Geometry.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Implementation of a Pose2 transform
   */
  class KARTO_EXPORT Transform
  {
  public:
    /**
     * Identity transformation
     */
    Transform();

    /**
     * Transformation from the origin to the given pose
     * @param rPose pose
     */
    Transform(const Pose2& rPose);

    /**
     * Transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    Transform(const Pose2& rPose1, const Pose2& rPose2);

  public:
    /**
     * Transforms the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 TransformPose(const Pose2& rSourcePose) const
    {
      Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

      return Pose2(newPosition.GetPosition(), angle);
    }

    /**
     * Inverse transformation of the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 InverseTransformPose(const Pose2& rSourcePose) const
    {
      Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

      // components of transform
      return Pose2(newPosition.GetPosition(), angle);
    }

  private:
    /**
     * Sets this to be the transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    void SetTransform(const Pose2& rPose1, const Pose2& rPose2);

  private:
    // pose transformation
    Pose2 m_Transform;

    Matrix3 m_Rotation;
    Matrix3 m_InverseRotation;
  }; // Transform

  //@}

}

#endif // __OpenKarto__PoseTransform_h__
