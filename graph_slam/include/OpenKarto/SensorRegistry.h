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

#ifndef __OpenKarto_SensorRegistry_h__
#define __OpenKarto_SensorRegistry_h__

#include <OpenKarto/Referenced.h>
#include <OpenKarto/Identifier.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  class Sensor;

  struct SensorRegistryPrivate;

  /**
   * Sensors registry. Manages all sensors in Karto and is used internally by Karto. 
   * This class is subject to change and it is not recommend to call any of the SensorRegistry's methods.
   */
  class KARTO_EXPORT SensorRegistry : public Referenced
  {
    friend class Sensor;
    
  public:
    /**
     * Default constructor
     */
    SensorRegistry();

    /**
     * Destructor
     */
    virtual ~SensorRegistry();

  public:
    /**
     * Gets singleton instance of SensorRegistry
     */
    static SensorRegistry* GetInstance();

  public:
    /**
     * Gets the sensor with the given name
     * @param rName name of sensor
     * @return sensor
     */
    Sensor* GetSensorByName(const Identifier& rName);
    
    /**
     * Gets the sensor with the given name
     * @param rName name of sensor
     * @return sensor
     */
    template<class C>
    C* GetSensorByName(const Identifier& rName)
    {
      Sensor* pSensor = GetSensorByName(rName);
      
      return dynamic_cast<C*>(pSensor);
    }
    
    /**
     * Checks that given sensor is not NULL and has a non-empty name
     * @param pSensor sensor to validate
     */
    static void Validate(Sensor* pSensor);
    
    /**
     * Clears the registry
     */
    void Clear();

  private:
    /**
     * Registers a sensor.  The sensor name must be unique; if not, an exception
     * will be thrown
     * @param pSensor sensor to register
     */
    void RegisterSensor(Sensor* pSensor);
    
    /**
     * Unregisters the given sensor
     * @param pSensor sensor to unregister
     */
    void UnregisterSensor(Sensor* pSensor);
    
  private:
    SensorRegistryPrivate* m_pSensorRegistryPrivate;
  };

  //@}

}

#endif // __OpenKarto_SensorRegistry_h__
