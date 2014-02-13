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

#include <iostream>
#include <algorithm>
#include <map>
#include <vector>

#include <OpenKarto/Parameter.h>
#include <OpenKarto/Logger.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ParameterDescription::ParameterDescription(const karto::String& rName, const karto::String& rDisplayName, const karto::String& rDescription)
    : m_Name(rName)
    , m_DisplayName(rDisplayName)
    , m_Description(rDescription)
    , m_Flags(ParameterFlag_Write | ParameterFlag_Read)
    , m_nDecimalPlaces(4)
  {
    assert(m_Name.Size() != 0);

    m_FieldNames[0] = "X";
    m_FieldNames[1] = "Y";
    m_FieldNames[2] = "Z";
    m_FieldNames[3] = "W";
  }

  ParameterDescription::~ParameterDescription()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  AbstractParameter::AbstractParameter(ParameterDescription* pDescription, ParameterSet* pParameterSet)
    : m_pDescription(pDescription)
    , m_pParameterSet(pParameterSet)
  {
    InitializeParameters();
  }

  AbstractParameter::~AbstractParameter()
  {
  }

  void AbstractParameter::InitializeParameters()
  {
    if (m_pParameterSet != NULL)
    {
      m_pParameterSet->AddParameter(this);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ParameterSetPrivate
  {
    typedef std::map<karto::String, SmartPointer<AbstractParameter> > ParameterMap;

    ParameterList m_Parameters;
    ParameterMap m_ParametersMap;
  };

  ParameterSet::ParameterSet()
    : m_pPrivate(new ParameterSetPrivate())
  {
  }

  ParameterSet::~ParameterSet()
  {
    delete m_pPrivate;
  }

  void ParameterSet::AddParameter(AbstractParameter* pParameter)
  {
    if (pParameter != NULL)
    {
      if (m_pPrivate->m_ParametersMap.find(pParameter->GetName()) == m_pPrivate->m_ParametersMap.end())
      {
        m_pPrivate->m_ParametersMap[pParameter->GetName()] = pParameter;
        m_pPrivate->m_Parameters.Add(pParameter);
      }
      else
      {
        throw Exception("ParameterSet::AddParameter - Paramter with name: '" + pParameter->GetName() + "' already exists. Please rename property before adding again.");
      }
    }
  }

  void ParameterSet::RemoveParameter(AbstractParameter* pParameter)
  {
    if (pParameter != NULL)
    {
      ParameterSetPrivate::ParameterMap::iterator iter = m_pPrivate->m_ParametersMap.find(pParameter->GetName());
      if (iter != m_pPrivate->m_ParametersMap.end())
      {
        m_pPrivate->m_ParametersMap.erase(iter);

        m_pPrivate->m_Parameters.Remove(pParameter);
      }
    }
  }

  void ParameterSet::Clear()
  {
    m_pPrivate->m_ParametersMap.clear();
    m_pPrivate->m_Parameters.Clear();
  }

  const ParameterList& ParameterSet::GetParameters() const
  {
    return m_pPrivate->m_Parameters;
  }

  ParameterList& ParameterSet::GetParameters()
  {
    return m_pPrivate->m_Parameters;
  }

  AbstractParameter* ParameterSet::GetParameter(const karto::String& rParameterName) const
  {
    ParameterSetPrivate::ParameterMap::const_iterator iter = m_pPrivate->m_ParametersMap.find(rParameterName);
    if (iter != m_pPrivate->m_ParametersMap.end())
    {
      return iter->second;
    }

    return NULL;
  }

  AbstractParameter* ParameterSet::GetParameter(const karto::String& rParameterName)
  {
    ParameterSetPrivate::ParameterMap::iterator iter = m_pPrivate->m_ParametersMap.find(rParameterName);
    if (iter != m_pPrivate->m_ParametersMap.end())
    {
      return iter->second;
    }

    return NULL;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ParameterEnumPrivate
  {
    typedef std::vector<EnumPair> EnumPairVector;
    EnumPairVector m_EnumPairs;
  };

  ParameterEnum::ParameterEnum(ParameterSet* pParent, const karto::String& rName, const karto::String& rDisplayName, const karto::String& rDescription, kt_int64s value)
    : Parameter<kt_int64s>(pParent, rName, rDisplayName, rDescription, value)
    , m_pPrivate(new ParameterEnumPrivate)
  {
  }

  ParameterEnum::~ParameterEnum()
  {
    delete m_pPrivate;
  }

  void ParameterEnum::SetValueFromString(const String& rStringValue)
  {
    ParameterEnumPrivate::EnumPairVector::const_iterator iter = std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByName(rStringValue));
    if (iter == m_pPrivate->m_EnumPairs.end())
    {
      StringBuilder validValues;

      forEach(ParameterEnumPrivate::EnumPairVector, &(m_pPrivate->m_EnumPairs))
      {
        validValues << iter->name << ", ";
      }

      throw Exception("ParameterEnum::SetValueFromString - Unable to set enum: '" + rStringValue + "'. Valid values are: " + validValues.ToString());
    }
    else
    {
      SetValue(iter->value);
    }
  }

  const String ParameterEnum::GetValueAsString() const
  {
    ParameterEnumPrivate::EnumPairVector::const_iterator iter = std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByValue(m_Value));
    if (iter == m_pPrivate->m_EnumPairs.end())
    {
      throw Exception("ParameterEnum::GetValueAsString - Unable to lookup enum");
    }
    else
    {
      return iter->name;
    }
  }

  void ParameterEnum::DefineEnumValue(const String& rName, kt_int64s value)
  {
    ParameterEnumPrivate::EnumPairVector::iterator iter = std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByName(rName));
    if (iter == m_pPrivate->m_EnumPairs.end())
    {
      EnumPair enumPair;
      enumPair.name = rName;
      enumPair.value = value;
      m_pPrivate->m_EnumPairs.push_back(enumPair);
    }
    else
    {
      karto::Log(karto::LOG_WARNING, "ParameterEnum::DefineEnumValue - Overriding enum value: " + rName + " with " + StringHelper::ToString(value));

      iter->value = value;
    }
  }

  const EnumPairList ParameterEnum::GetEnumValues() const
  {
    EnumPairList enumPairList;

    forEach(ParameterEnumPrivate::EnumPairVector, &(m_pPrivate->m_EnumPairs))
    {
      enumPairList.Add(*iter);
    }

    return enumPairList;
  }

}
