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

#include <OpenKarto/MetaEnum.h>
#include <OpenKarto/MetaEnumManager.h>
#include <OpenKarto/SmartPointer.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaEnumManagerPrivate
  {
    typedef karto::SmartPointer<MetaEnum> MetaEnumPtr;
    typedef std::map<karto::String, MetaEnumPtr> EnumByNameTable;
    typedef std::map<karto::String, MetaEnumPtr> EnumByIdTable;

    EnumByNameTable m_MetaEnumByName;
    EnumByIdTable m_MetaEnumById;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MetaEnumManager::MetaEnumManager()
    : m_pPrivate(new MetaEnumManagerPrivate)
  {
  }

  MetaEnumManager::~MetaEnumManager()
  {
    Clear();

    delete m_pPrivate;
    m_pPrivate = NULL;
  }

  MetaEnumManager& MetaEnumManager::GetInstance()
  {
    static MetaEnumManager manager;
    return manager;
  }

  MetaEnum& MetaEnumManager::RegisterNew(const karto::String& rName, const karto::String& rId)
  {
    if ((m_pPrivate->m_MetaEnumByName.find(rName) != m_pPrivate->m_MetaEnumByName.end()) || (m_pPrivate->m_MetaEnumById.find(rId) != m_pPrivate->m_MetaEnumById.end()))
    {
      assert(false);
    }

    MetaEnumManagerPrivate::MetaEnumPtr newEnum = new MetaEnum(rName);
    m_pPrivate->m_MetaEnumByName[rName] = newEnum;
    m_pPrivate->m_MetaEnumById[rId] = newEnum;

    return *newEnum;
  }

  const MetaEnum& MetaEnumManager::GetByName(const karto::String& rName) const
  {
    MetaEnumManagerPrivate::EnumByNameTable::const_iterator iter = m_pPrivate->m_MetaEnumByName.find(rName);
    if (iter == m_pPrivate->m_MetaEnumByName.end())
    {
      throw karto::Exception("No MetaEnum for enum with name: " + rName);
    }

    return *iter->second;
  }

  const MetaEnum& MetaEnumManager::GetById(const karto::String& rId) const
  {
    MetaEnumManagerPrivate::EnumByIdTable::const_iterator iter = m_pPrivate->m_MetaEnumById.find(rId);
    if (iter == m_pPrivate->m_MetaEnumById.end())
    {
      throw karto::Exception("No MetaEnum for enum with id: " + rId);
    }

    return *iter->second;
  }

  kt_bool MetaEnumManager::EnumExists(const karto::String& rId) const
  {
    return m_pPrivate->m_MetaEnumById.find(rId) != m_pPrivate->m_MetaEnumById.end();
  }

  kt_size_t MetaEnumManager::GetSize() const
  {
    return m_pPrivate->m_MetaEnumByName.size();
  }

  const MetaEnum& MetaEnumManager::operator[]( kt_size_t index ) const
  {
    if (index >= m_pPrivate->m_MetaEnumByName.size())
    {
      assert(false);
    }

    MetaEnumManagerPrivate::EnumByNameTable::const_iterator iter = m_pPrivate->m_MetaEnumByName.begin();
    std::advance(iter, index);

    return *iter->second;
  }

  void MetaEnumManager::Clear()
  {
    m_pPrivate->m_MetaEnumById.clear();
    m_pPrivate->m_MetaEnumByName.clear();
  }

}