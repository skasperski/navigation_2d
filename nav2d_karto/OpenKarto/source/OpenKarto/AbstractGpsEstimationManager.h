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

#ifndef __OpenKarto_AbstractGpsEstimationManager_h__
#define __OpenKarto_AbstractGpsEstimationManager_h__

#include <OpenKarto/Referenced.h>
#include <OpenKarto/Geometry.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  class LocalizedObject;

  /**
   * Manages estimations of GPS readings.
   * @cond EXCLUDE
   * Mainly so mapper module has a way to update points lazily
   * @endcond
   */
  class AbstractGpsEstimationManager : public Referenced
  {
  public:
    /**
     * Gets the GPS estimate of the given scan (return value is
     * meaningless if IsGpsEstimateValid(pScan) returns false).
     * @param pScan scan
     * @return GPS estimate of the given scan
     */
    virtual gps::PointGps GetGpsEstimate(const LocalizedObject* pScan) const = 0;
    
    /**
     * Sets the GPS estimate of the given scan
     * @param pScan scan
     * @param rGpsEstimate GPS estimate of the given scan
     */
    virtual void SetGpsEstimate(const LocalizedObject* pScan, const gps::PointGps& rGpsEstimate) = 0;
    
    /**
     * Whether the GPS estimate of the given scan is valid
     * @param pScan scan
     * @return whether the GPS estimate of the given scan is valid
     */
    virtual kt_bool IsGpsEstimateValid(const LocalizedObject* pScan) const = 0;
  };

  //@}
}


#endif // __OpenKarto_AbstractGpsEstimationManager_h__
