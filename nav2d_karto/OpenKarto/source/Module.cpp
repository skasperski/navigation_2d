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

#include <OpenKarto/Module.h>
#include <OpenKarto/TypeCasts.h>
#include <OpenKarto/Logger.h>

#include <iostream>

namespace karto
{

  Module::Module(const Identifier& rName)
    : Object(rName)
  {
  }

  Module::~Module()
  {
  }

  kt_bool Module::Process(karto::Object* pObject)
  {
    kt_bool isObjectProcessed = false;
    
    if (IsSensor(pObject))
    {
      m_Sensors.Add(dynamic_cast<karto::Sensor*>(pObject));
      isObjectProcessed = true;
    }
    else if (IsModuleParameters(pObject))
    {
      karto::ModuleParameters* pParameters = dynamic_cast<karto::ModuleParameters*>(pObject);
      
      if (pParameters != NULL && pParameters->GetIdentifier() == GetIdentifier())
      {
        // copy parameters
        const karto::ParameterList& rParameters = pParameters->GetParameters();
        karto_const_forEach(karto::ParameterList, &rParameters)
        {
          karto::AbstractParameter* pParameterFrom = *iter;
          karto::AbstractParameter* pParameterTo = GetParameter(pParameterFrom->GetName());
          if (pParameterTo != NULL)
          {
            pParameterTo->SetValueFromString(pParameterFrom->GetValueAsString());
          }
          else
          {
            Log(LOG_WARNING, String("Invalid ") + GetIdentifier().ToString() + " parameter: " + pParameterFrom->GetName() + " parameter is ignored!");
          }
        }
      }
      
      isObjectProcessed = true;
    }

    return isObjectProcessed;
  }

  void Module::Reset()
  {
    m_Sensors.Clear();
  }

}

