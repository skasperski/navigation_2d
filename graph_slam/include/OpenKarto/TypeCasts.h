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

#ifndef __OpenKarto_TypeCasts_h__
#define __OpenKarto_TypeCasts_h__

#include <OpenKarto/Sensor.h>
#include <OpenKarto/SensorData.h>
#include <OpenKarto/Objects.h>
#include <OpenKarto/OccupancyGrid.h>

namespace karto
{
  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  KARTO_TYPECHECKCAST(Sensor)
  KARTO_TYPECHECKCAST(SensorData)
  KARTO_TYPECHECKCAST(LaserRangeFinder)
  KARTO_TYPECHECKCAST(Drive)
  KARTO_TYPECHECKCAST(DrivePose)
  KARTO_TYPECHECKCAST(LocalizedRangeScan)
  KARTO_TYPECHECKCAST(LocalizedPointScan)
  KARTO_TYPECHECKCAST(ModuleParameters)
  KARTO_TYPECHECKCAST(DatasetInfo)
  KARTO_TYPECHECKCAST(OccupancyGrid)

  /**
   * Determines whether a given object is a laser scan object
   * @param pObject object in question
   * @return whether the given object is a laser scan object
   */
  inline kt_bool IsLocalizedLaserScan(Object* pObject)
  {
    return IsLocalizedRangeScan(pObject) || IsLocalizedPointScan(pObject);
  }

#ifdef WIN32
#define EXPORT_KARTO_LIST(declspec, T) \
  template class declspec karto::List<T>;

  EXPORT_KARTO_LIST(KARTO_EXPORT, kt_double)
  EXPORT_KARTO_LIST(KARTO_EXPORT, Pose2)
  EXPORT_KARTO_LIST(KARTO_EXPORT, Vector2d)
  EXPORT_KARTO_LIST(KARTO_EXPORT, SmartPointer<LocalizedLaserScan>)
  EXPORT_KARTO_LIST(KARTO_EXPORT, SmartPointer<CustomItem>)
  EXPORT_KARTO_LIST(KARTO_EXPORT, SmartPointer<Sensor>)
  EXPORT_KARTO_LIST(KARTO_EXPORT, SmartPointer<AbstractParameter>)
  EXPORT_KARTO_LIST(KARTO_EXPORT, SmartPointer<Object>)
#endif

  //@}
}

#endif // __OpenKarto_TypeCasts_h__
