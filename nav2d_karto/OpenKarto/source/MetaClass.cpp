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

#include <OpenKarto/MetaClass.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaArgsPrivate
  {
    std::vector<Any> m_Arguments;
  };

  MetaArguments::MetaArguments()
    : m_pPrivate(new MetaArgsPrivate)
  {
  }

  MetaArguments::MetaArguments(const Any& a0)
    : m_pPrivate(new MetaArgsPrivate)
  {
    m_pPrivate->m_Arguments.push_back(a0);
  }

  MetaArguments::MetaArguments(const Any& a0, const Any& a1)
    : m_pPrivate(new MetaArgsPrivate)
  {
    m_pPrivate->m_Arguments.push_back(a0);
    m_pPrivate->m_Arguments.push_back(a1);
  }

  MetaArguments::MetaArguments(const Any& a0, const Any& a1, const Any& a2)
    : m_pPrivate(new MetaArgsPrivate)
  {
    m_pPrivate->m_Arguments.push_back(a0);
    m_pPrivate->m_Arguments.push_back(a1);
    m_pPrivate->m_Arguments.push_back(a2);
  }

  MetaArguments::MetaArguments(const Any& a0, const Any& a1, const Any& a2, const Any& a3)
    : m_pPrivate(new MetaArgsPrivate)
  {
    m_pPrivate->m_Arguments.push_back(a0);
    m_pPrivate->m_Arguments.push_back(a1);
    m_pPrivate->m_Arguments.push_back(a2);
    m_pPrivate->m_Arguments.push_back(a3);
  }

  MetaArguments::MetaArguments(const Any& a0, const Any& a1, const Any& a2, const Any& a3, const Any& a4)
    : m_pPrivate(new MetaArgsPrivate)
  {
    m_pPrivate->m_Arguments.push_back(a0);
    m_pPrivate->m_Arguments.push_back(a1);
    m_pPrivate->m_Arguments.push_back(a2);
    m_pPrivate->m_Arguments.push_back(a3);
    m_pPrivate->m_Arguments.push_back(a4);
  }

  MetaArguments::~MetaArguments()
  {
    delete m_pPrivate;
  }

  kt_size_t MetaArguments::GetCount() const
  {
    return m_pPrivate->m_Arguments.size();
  }

  const Any& MetaArguments::operator[](kt_size_t index)  const
  {
    assert(index >= 0 && index < GetCount());
    return m_pPrivate->m_Arguments[index];
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MetaClass::MetaClass(const karto::String& rName)
    : m_Name(rName)
  {
  }

  MetaClass::~MetaClass()
  {
  }

  const karto::String& MetaClass::GetName() const
  {
    return m_Name;
  }

  kt_size_t MetaClass::GetBaseSize() const
  {
    return m_BaseClasses.Size();
  }

  const MetaClass& MetaClass::GetBase(kt_size_t index) const
  {
    if (index >= m_BaseClasses.Size())
    {
      assert(false);
      throw karto::Exception("MetaClass::GetBase() - Index out of range");
    }

    return *m_BaseClasses[index];
  }

  bool MetaClass::operator==(const MetaClass& rOther) const
  {
    return m_Name == rOther.m_Name;
  }

  bool MetaClass::operator!=(const MetaClass& rOther) const
  {
    return m_Name != rOther.m_Name;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

}