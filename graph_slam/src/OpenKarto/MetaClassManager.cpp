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

#include <OpenKarto/MetaClass.h>
#include <OpenKarto/MetaClassManager.h>
#include <OpenKarto/SmartPointer.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaClassManagerPrivate
  {
    typedef karto::SmartPointer<MetaClass> ClassPtr;
    typedef std::map<karto::String, ClassPtr> ClassByNameTable;
    typedef std::map<karto::String, ClassPtr> ClassByIdTable;

    ClassByNameTable m_MetaClassByName; 
    ClassByIdTable m_MetaClassById; 
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MetaClassManager::MetaClassManager()
    : m_pPrivate(new MetaClassManagerPrivate)
  {
  }

  MetaClassManager::~MetaClassManager()
  {
    Clear();

    delete m_pPrivate;
    m_pPrivate = NULL;
  }

  MetaClassManager& MetaClassManager::GetInstance()
  {
    static MetaClassManager manager;
    return manager;
  }

  MetaClass& MetaClassManager::RegisterNew(const karto::String& rName, const karto::String& rId)
  {
    if ((m_pPrivate->m_MetaClassByName.find(rName) != m_pPrivate->m_MetaClassByName.end()) || (m_pPrivate->m_MetaClassById.find(rId) != m_pPrivate->m_MetaClassById.end()))
    {
      throw karto::Exception("MetaClass already exists for class with name: " + rName);
    }

    MetaClassManagerPrivate::ClassPtr newClass = new MetaClass(rName);
    m_pPrivate->m_MetaClassByName[rName] = newClass;
    m_pPrivate->m_MetaClassById[rId] = newClass;

    return *newClass;
  }

  const MetaClass& MetaClassManager::GetByName(const karto::String& rName) const
  {
    MetaClassManagerPrivate::ClassByNameTable::const_iterator iter = m_pPrivate->m_MetaClassByName.find(rName);
    if (iter == m_pPrivate->m_MetaClassByName.end())
    {
      throw karto::Exception("No MetaClass for class with name: " + rName);
    }

    return *iter->second;
  }

  const MetaClass& MetaClassManager::GetById(const karto::String& rId) const
  {
    MetaClassManagerPrivate::ClassByIdTable::const_iterator iter = m_pPrivate->m_MetaClassById.find(rId);
    if (iter == m_pPrivate->m_MetaClassById.end())
    {
      throw karto::Exception("No MetaClass for class with id: " + rId);
    }

    return *iter->second;
  }

  kt_bool MetaClassManager::ClassExists(const karto::String& rId) const
  {
    return m_pPrivate->m_MetaClassById.find(rId) != m_pPrivate->m_MetaClassById.end();
  }

  kt_size_t MetaClassManager::GetSize() const
  {
    return m_pPrivate->m_MetaClassByName.size();
  }

  const MetaClass& MetaClassManager::operator[](kt_size_t index) const
  {
    if (index >= m_pPrivate->m_MetaClassByName.size())
    {
      throw karto::Exception("No MetaClass for index: " + index);
    }

    MetaClassManagerPrivate::ClassByNameTable::const_iterator iter = m_pPrivate->m_MetaClassByName.begin();
    std::advance(iter, index);

    return *iter->second;
  }

  void MetaClassManager::Clear()
  {
    m_pPrivate->m_MetaClassById.clear();
    m_pPrivate->m_MetaClassByName.clear();
  }

}