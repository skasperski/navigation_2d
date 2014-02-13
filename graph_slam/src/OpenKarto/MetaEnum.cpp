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

#include <vector>
#include <algorithm>

#include <OpenKarto/MetaEnum.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaEnumPrivate
  {
    karto::String m_Name;

    typedef std::vector<EnumPair> EnumPairVector;
    EnumPairVector m_EnumPairs;
  };
 
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MetaEnum::MetaEnum(const karto::String& rName)
    : m_pPrivate(new MetaEnumPrivate)
  {
    m_pPrivate->m_Name = rName;
  }

  MetaEnum::~MetaEnum()
  {
    delete m_pPrivate;
  }

  kt_bool MetaEnum::HasName(const karto::String& rName) const
  {
    return std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByName(rName)) != m_pPrivate->m_EnumPairs.end();
  }

  kt_bool MetaEnum::HasValue(kt_int64s value) const
  {
    return std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByValue(value)) != m_pPrivate->m_EnumPairs.end();
  }

  kt_int64s MetaEnum::GetValue(const karto::String& rName) const
  {
    MetaEnumPrivate::EnumPairVector::const_iterator iter = std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByName(rName));
    if (iter == m_pPrivate->m_EnumPairs.end())
    {
      assert(false);
      throw karto::Exception("No EnumPair with name: " +rName);
    }

    return iter->value;
  }

  const karto::String& MetaEnum::GetName() const
  {
    return m_pPrivate->m_Name;
  }

  const karto::String& MetaEnum::GetName(kt_int64s value) const
  {
    MetaEnumPrivate::EnumPairVector::const_iterator iter = std::find_if(m_pPrivate->m_EnumPairs.begin(), m_pPrivate->m_EnumPairs.end(), FindByValue(value));
    if (iter == m_pPrivate->m_EnumPairs.end())
    {
      assert(false);
      throw karto::Exception("No EnumPair with value: " + karto::StringHelper::ToString(value));
    }

    return iter->name;
  }

  kt_size_t MetaEnum::GetSize() const
  {
    return m_pPrivate->m_EnumPairs.size();
  }

  kt_bool MetaEnum::operator==(const MetaEnum& rOther) const
  {
    return m_pPrivate->m_Name == rOther.m_pPrivate->m_Name;
  }

  kt_bool MetaEnum::operator!=(const MetaEnum& rOther) const
  {
    return m_pPrivate->m_Name != rOther.m_pPrivate->m_Name;
  }

  const EnumPair& MetaEnum::GetPair(kt_size_t index) const
  {
    if (index >= m_pPrivate->m_EnumPairs.size())
    {
      assert(false);
      throw karto::Exception("MetaEnum::GetPair() - Index out of range");
    }

    return m_pPrivate->m_EnumPairs[index];
  }

  void MetaEnum::AddEnumPair(const EnumPair& rPair)
  {
    m_pPrivate->m_EnumPairs.push_back(rPair);
  }

}