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

#include <map>
#include <iostream>

#ifdef USE_TBB
#include <tbb/mutex.h>
#endif

#include <OpenKarto/SensorRegistry.h>
#include <OpenKarto/Exception.h>
#include <OpenKarto/Sensor.h>
#include <OpenKarto/Logger.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // Note to not change the Sensor* to SensorPtr or SmartPointer<Sensor>. 
  struct SensorRegistryPrivate
  {
    List<Sensor*> m_Sensors;

    typedef std::map<Identifier, Sensor*> SensorManagerMap;
    SensorManagerMap m_SensorMap;
  };

  SensorRegistry::SensorRegistry()
    : m_pSensorRegistryPrivate(new SensorRegistryPrivate())
  {
  }

  SensorRegistry::~SensorRegistry()
  {
    m_pSensorRegistryPrivate->m_Sensors.Clear();
    delete m_pSensorRegistryPrivate;
  }

  SensorRegistry* SensorRegistry::GetInstance()
  {
#ifdef USE_TBB
    static tbb::mutex myMutex;
    tbb::mutex::scoped_lock lock(myMutex);
#endif

    static SmartPointer<SensorRegistry> sInstance = new SensorRegistry();
    return sInstance;
  }

  void SensorRegistry::RegisterSensor(Sensor* pSensor)
  {
    if (pSensor != NULL)
    {
      if (pSensor->GetIdentifier().GetScope() != "Karto/System")
      {
        Validate(pSensor);

        Log(LOG_DEBUG, String("Registering sensor: [") + pSensor->GetIdentifier().ToString() + "]");
      }

      if ((m_pSensorRegistryPrivate->m_SensorMap.find(karto::Identifier(pSensor->GetIdentifier())) != m_pSensorRegistryPrivate->m_SensorMap.end()))
      {
        String errorMessage;
        errorMessage.Append("Cannot register sensor: already registered: [");
        errorMessage.Append(pSensor->GetIdentifier().ToString());
        errorMessage.Append("]");

        throw Exception(errorMessage);
      }

      m_pSensorRegistryPrivate->m_SensorMap[karto::Identifier(pSensor->GetIdentifier())] = pSensor;
      m_pSensorRegistryPrivate->m_Sensors.Add(pSensor);
    }
  }

  void SensorRegistry::UnregisterSensor(Sensor* pSensor)
  {
    if (pSensor != NULL)
    {
      if (pSensor->GetIdentifier().GetScope() != "Karto/System")
      {
        Log(LOG_DEBUG, String("Unregistering sensor: [") + pSensor->GetIdentifier().ToString() + "]");
      }

      if (m_pSensorRegistryPrivate->m_SensorMap.find(pSensor->GetIdentifier()) != m_pSensorRegistryPrivate->m_SensorMap.end())
      {
        m_pSensorRegistryPrivate->m_SensorMap.erase(pSensor->GetIdentifier());

        m_pSensorRegistryPrivate->m_Sensors.Remove(pSensor);
      }
      else
      {
        String errorMessage;
        errorMessage.Append("Cannot unregister sensor: not registered: [");
        errorMessage.Append(pSensor->GetIdentifier().ToString());
        errorMessage.Append("]");

        throw Exception(errorMessage);
      }
    }
  }

  Sensor* SensorRegistry::GetSensorByName(const Identifier& rName)
  {
    if (m_pSensorRegistryPrivate->m_SensorMap.find(rName) != m_pSensorRegistryPrivate->m_SensorMap.end())
    {
      Sensor* pSensor = m_pSensorRegistryPrivate->m_SensorMap[rName];

      assert(pSensor != NULL);

      return pSensor;
    }

    String errorMessage;
    errorMessage.Append("Sensor not registered: [");
    errorMessage.Append(rName.ToString());
    errorMessage.Append("]");
    throw Exception(errorMessage);
  }

  void SensorRegistry::Clear()
  {
    m_pSensorRegistryPrivate->m_Sensors.Clear();
    m_pSensorRegistryPrivate->m_SensorMap.clear();
  }

  void SensorRegistry::Validate(Sensor* pSensor)
  {
    if (pSensor == NULL)
    {
      throw Exception("Invalid sensor: NULL");
    }
    else if (pSensor->GetIdentifier().Size() == 0)
    {
      throw Exception("Invalid sensor: Nameless");        
    }
  }

}

