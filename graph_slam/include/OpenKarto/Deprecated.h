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

#ifndef __OpenKarto_Deprecated_h__
#define __OpenKarto_Deprecated_h__

#include <vector>

#include <OpenKarto/Objects.h>
#include <OpenKarto/SensorData.h>

namespace karto
{

  /**
   * Type declaration of Pose2 vector.  Please use Pose2List instead.
   */
  typedef KARTO_DEPRECATED std::vector<Pose2> Pose2Vector;

  /**
   * Type declaration of Vector2d vector.  Please use Vector2dList instead.
   */
  typedef KARTO_DEPRECATED std::vector<Vector2d> Vector2dVector;

  /**
   * Type declaration of range readings vector.  Please use RangeReadingsList instead.
   */
  typedef KARTO_DEPRECATED std::vector<kt_double> RangeReadingsVector;

  /**
   * Type declaration of LocalizedRangeScan vector.  Please use more general class LocalizedLaserScan instead along with LocalizedLaserScanList.
   */
  typedef KARTO_DEPRECATED std::vector<SmartPointer<LocalizedRangeScan> > LocalizedRangeScanVector;

  /**
   * Type declaration of ObjectVector.  Please use ObjectList instead.
   */
  typedef KARTO_DEPRECATED karto::List<ObjectPtr> ObjectVector;

}

#endif // __OpenKarto_Deprecated_h__
