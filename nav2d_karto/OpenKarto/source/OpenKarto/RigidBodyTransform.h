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

#pragma once

#ifndef __KARTO_RIGIDBODYTRANSFORM__
#define __KARTO_RIGIDBODYTRANSFORM__

#include <OpenKarto/Types.h>
#include <OpenKarto/Macros.h>

namespace karto
{
  /**
   * \ingroup Karto
   * Implementation of a Pose2 rigid body transform
   */
  class KARTO_EXPORT RigidBodyTransform
  {
	private:
		karto::Pose2 m_Transform;
		karto::Pose2 m_InvTransform;

	public:
    /**
     * Transformation from the origin to the given pose
     * @param rPose pose
     */
    RigidBodyTransform(const karto::Pose2& rPose);

    /**
     * Transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    RigidBodyTransform(const karto::Pose2& rPose1, const karto::Pose2& rPose2);

	public:
		/**
     * Transforms the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    karto::Pose2 TransformPose(const karto::Pose2& rSourcePose);

    /**
     * Inverse transformation of the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    karto::Pose2 InverseTransformPose(const karto::Pose2& rSourcePose);

	private:
		/**
		* Sets this to be the transformation from the first pose to the second pose
		* @param rPose1 first pose
		* @param rPose2 second pose
		*/
		void SetTransform(const karto::Pose2& rPose1, const karto::Pose2& rPose2);
	}; // class RigidBodyTransform

} // namespace karto

#endif // __KARTO_RIGIDBODYTRANSFORM__


