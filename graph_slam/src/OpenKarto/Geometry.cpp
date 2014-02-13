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

#include <OpenKarto/Geometry.h>
#include <OpenKarto/String.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Pose2::Pose2()
    : m_Heading(0.0)
  {
  }

  Pose2::Pose2(const Vector2d& rPosition, kt_double heading)
    : m_Position(rPosition)
    , m_Heading(heading)
  {
  }

  Pose2::Pose2(kt_double x, kt_double y, kt_double heading)
    : m_Position(x, y)
    , m_Heading(heading)
  {
  }

  Pose2::Pose2(const Pose3& rPose)
    : m_Position(rPose.GetPosition().GetX(), rPose.GetPosition().GetY())
  {
    kt_double t1, t2;

    // calculates heading from orientation
    rPose.GetOrientation().ToEulerAngles(m_Heading, t1, t2);
  }

  Pose2::Pose2(const Pose2& rOther)
    : m_Position(rOther.m_Position)
    , m_Heading(rOther.m_Heading)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  BoundingBox2::BoundingBox2()
    : m_Minimum(DBL_MAX, DBL_MAX)
    , m_Maximum(-DBL_MAX, -DBL_MAX)
  {
  }

  BoundingBox2::BoundingBox2(const Vector2d& rMinimum, const Vector2d& rMaximum)
    : m_Minimum(rMinimum)
    , m_Maximum(rMaximum)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  BoundingBox3::BoundingBox3()
  : m_Minimum(DBL_MAX, DBL_MAX, DBL_MAX)
  , m_Maximum(-DBL_MAX, -DBL_MAX, -DBL_MAX)
  {
  }

  BoundingBox3::~BoundingBox3()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void Quaternion::ToAngleAxis(kt_double& rAngle, karto::Vector3d& rAxis) const
  {
    // The quaternion representing the rotation is
    //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

    kt_double fSqrLength = m_Values[0] * m_Values[0] + m_Values[1] * m_Values[1] + m_Values[2] * m_Values[2];
    if ( fSqrLength > 0.0 )
    {
      rAngle = 2.0 * acos(m_Values[3]);
      kt_double fInvLength = 1.0 / sqrt(fSqrLength);
      rAxis.SetX(m_Values[0] * fInvLength);
      rAxis.SetY(m_Values[1] * fInvLength);
      rAxis.SetZ(m_Values[2] * fInvLength);
    }
    else
    {
      // angle is 0 (mod 2*pi), so any axis will do
      rAngle = 0.0;
      rAxis.SetX(1.0);
      rAxis.SetY(0.0);
      rAxis.SetZ(0.0);
    }
  }

  void Quaternion::FromAngleAxis(kt_double angleInRadians, const karto::Vector3d& rAxis)
  {
    kt_double axisLength = rAxis.Length();
    if (axisLength < KT_TOLERANCE)
    {
      assert(false);
      // special case for zero length
      *this = Quaternion();
      return;
    }
    kt_double halfAngle = 0.5*angleInRadians;

    kt_double inverseLength  = 1.0;// / axisLength;
    kt_double sinHalfAngle = sin(halfAngle);
    kt_double cosHalfAngle = cos(halfAngle);

    m_Values[0] = inverseLength * sinHalfAngle * rAxis.GetX();
    m_Values[1] = inverseLength * sinHalfAngle * rAxis.GetY();
    m_Values[2] = inverseLength * sinHalfAngle * rAxis.GetZ();
    m_Values[3] = cosHalfAngle;
  }

  void Quaternion::ToEulerAngles(kt_double& rYaw, kt_double& rPitch, kt_double& rRoll) const 
  {
    kt_double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

    if (test > 0.499)
    { 
      // singularity at north pole
      rYaw = 2 * atan2(m_Values[0], m_Values[3]);;
      rPitch = KT_PI_2;
      rRoll = 0;
    }
    else if (test < -0.499)
    { 
      // singularity at south pole
      rYaw = -2 * atan2(m_Values[0], m_Values[3]);
      rPitch = -KT_PI_2;
      rRoll = 0;
    }
    else
    {
      kt_double sqx = m_Values[0] * m_Values[0];
      kt_double sqy = m_Values[1] * m_Values[1];
      kt_double sqz = m_Values[2] * m_Values[2];

      rYaw = atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2], 1 - 2 * sqy - 2 * sqz);
      rPitch = asin(2 * test);
      rRoll = atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2], 1 - 2 * sqx - 2 * sqz);
    }
  }

  void Quaternion::FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll)
  {
    kt_double angle;

    angle = yaw * 0.5; 
    kt_double cYaw = cos(angle);
    kt_double sYaw = sin(angle);

    angle = pitch * 0.5; 
    kt_double cPitch = cos(angle);
    kt_double sPitch = sin(angle);

    angle = roll * 0.5; 
    kt_double cRoll = cos(angle);
    kt_double sRoll = sin(angle);

    m_Values[0] = sYaw * sPitch * cRoll + cYaw * cPitch * sRoll;
    m_Values[1] = sYaw * cPitch * cRoll + cYaw * sPitch * sRoll;
    m_Values[2] = cYaw * sPitch * cRoll - sYaw * cPitch * sRoll;
    m_Values[3] = cYaw * cPitch * cRoll - sYaw * sPitch * sRoll;
  }

  const String Quaternion::ToString() const
  {
    String valueString;
    valueString.Append(StringHelper::ToString(GetX()));
    valueString.Append(" ");
    valueString.Append(StringHelper::ToString(GetY()));
    valueString.Append(" ");
    valueString.Append(StringHelper::ToString(GetZ()));
    valueString.Append(" ");
    valueString.Append(StringHelper::ToString(GetW()));
    return valueString;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

}