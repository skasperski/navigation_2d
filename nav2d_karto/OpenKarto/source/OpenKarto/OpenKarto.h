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

#ifndef __OpenKarto_h__
#define __OpenKarto_h__

#include <iostream>

#include <OpenKarto/AbstractGpsEstimationManager.h>
#include <OpenKarto/CoordinateConverter.h>
#include <OpenKarto/Deprecated.h>
#include <OpenKarto/Event.h>
#include <OpenKarto/Exception.h>
#include <OpenKarto/Geometry.h>
#include <OpenKarto/Grid.h>
#include <OpenKarto/GridIndexLookup.h>
#include <OpenKarto/Identifier.h>
#include <OpenKarto/List.h>
#include <OpenKarto/Logger.h>
#include <OpenKarto/Macros.h>
#include <OpenKarto/Math.h>
#include <OpenKarto/Meta.h>
#include <OpenKarto/MetaClassHelper.h>
#include <OpenKarto/MetaEnumHelper.h>
#include <OpenKarto/Module.h>
#include <OpenKarto/Object.h>
#include <OpenKarto/Objects.h>
#include <OpenKarto/OccupancyGrid.h>
#include <OpenKarto/OpenMapper.h>
#include <OpenKarto/Pair.h>
#include <OpenKarto/Parameter.h>
#include <OpenKarto/PoseTransform.h>
#include <OpenKarto/RangeTransform.h>
#include <OpenKarto/Referenced.h>
#include <OpenKarto/RigidBodyTransform.h>
#include <OpenKarto/Sensor.h>
#include <OpenKarto/SensorData.h>
#include <OpenKarto/SensorRegistry.h>
#include <OpenKarto/SmartPointer.h>
#include <OpenKarto/String.h>
#include <OpenKarto/StringHelper.h>
#include <OpenKarto/TypeCasts.h>
#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE
  /**
   * Internal function please don't call. 
   * Initialize and register OpenKarto classes with MetaClassManager
   * @note Please don't call. Called by Environment::Initialize()
   */
  void InitializeOpenKartoMetaClasses();
  //@endcond

  //@}

};

#endif // __OpenKarto_h__
