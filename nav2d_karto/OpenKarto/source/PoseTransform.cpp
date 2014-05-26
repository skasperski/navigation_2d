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

#include <OpenKarto/PoseTransform.h>

namespace karto
{

  Transform::Transform()
  {
    SetTransform(Pose2(), Pose2());
  }

  Transform::Transform(const Pose2& rPose)
  {
    SetTransform(Pose2(), rPose);
  }

  Transform::Transform(const Pose2& rPose1, const Pose2& rPose2)
  {
    SetTransform(rPose1, rPose2);
  }

  void Transform::SetTransform(const Pose2& rPose1, const Pose2& rPose2)
  {
    if (rPose1 == rPose2)
    {
      m_Rotation.SetToIdentity();
      m_InverseRotation.SetToIdentity();
      m_Transform = Pose2();
      return;
    }

    // heading transformation
    m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
    m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

    // position transformation
    Pose2 newPosition;
    if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
    {
      newPosition = rPose2 - m_Rotation * rPose1;
    }
    else
    {
      newPosition = rPose2;
    }

    m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
  }

}