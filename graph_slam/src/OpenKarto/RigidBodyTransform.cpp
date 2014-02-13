/*
 * Karto (tm) Robot Navigation Software - Software Development Kit
 * Release 2.1
 *
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Karto (tm) is a Trademark of SRI International (R).
 *
 * Author(s): Benson Limketkai (bensonl@ai.sri.com)
  */

#include <OpenKarto/Geometry.h>
#include <OpenKarto/RigidBodyTransform.h>

using namespace karto;

RigidBodyTransform::RigidBodyTransform(const karto::Pose2& rPose)
{
	SetTransform(karto::Pose2(), rPose);
}

RigidBodyTransform::RigidBodyTransform(const karto::Pose2& rPose1, const karto::Pose2& rPose2)
{
  SetTransform(rPose1, rPose2);
}

void RigidBodyTransform::SetTransform(const karto::Pose2& rPose1, const karto::Pose2& rPose2)
{
  m_Transform = rPose2;
  m_Transform.SetX(m_Transform.GetX() - rPose1.GetX());
  m_Transform.SetY(m_Transform.GetY() - rPose1.GetY());

  karto::Matrix3 rotation;
  rotation.FromAxisAngle(0, 0, 1, -rPose1.GetHeading());
  m_Transform = rotation * m_Transform;
  m_Transform.SetHeading(m_Transform.GetHeading() - rPose1.GetHeading());

  m_InvTransform = rPose1;
  m_InvTransform.SetX(m_InvTransform.GetX() - rPose2.GetX());
  m_InvTransform.SetY(m_InvTransform.GetY() - rPose2.GetY());

  karto::Matrix3 invRotation;
  invRotation.FromAxisAngle(0, 0, 1, -rPose2.GetHeading());
  m_InvTransform = invRotation * m_InvTransform;
  m_InvTransform.SetHeading(m_InvTransform.GetHeading() - rPose2.GetHeading());
}

karto::Pose2 RigidBodyTransform::TransformPose(const karto::Pose2& rSourcePose)
{
  karto::Matrix3 rotation;
  rotation.FromAxisAngle(0, 0, 1, rSourcePose.GetHeading());
  karto::Pose2 newPose = rotation * m_Transform;
  newPose += rSourcePose;

  return newPose;
}

karto::Pose2 RigidBodyTransform::InverseTransformPose(const karto::Pose2& rSourcePose)
{
  karto::Matrix3 rotation;
  rotation.FromAxisAngle(0, 0, 1, rSourcePose.GetHeading());
  karto::Pose2 newPose = rotation * m_InvTransform;
  newPose += rSourcePose;

  return newPose;
}
