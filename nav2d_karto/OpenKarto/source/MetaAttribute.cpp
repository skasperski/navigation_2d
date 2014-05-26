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

#include <OpenKarto/MetaAttribute.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaAttributePrivate
  {
    typedef std::map< karto::String, Any > Attributes;
    Attributes m_Attributes;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  MetaAttribute::MetaAttribute()
    : m_pPrivate(new MetaAttributePrivate())
  {
  }

  MetaAttribute::~MetaAttribute()
  {
    delete m_pPrivate;
  }

  kt_bool MetaAttribute::HasAttribute(const karto::String& rId) const
  {
    return m_pPrivate->m_Attributes.find(rId) != m_pPrivate->m_Attributes.end();
  }

  const Any& MetaAttribute::GetAttribute(const karto::String& rId) const
  {
    MetaAttributePrivate::Attributes::const_iterator iter = m_pPrivate->m_Attributes.find(rId);
    if (iter != m_pPrivate->m_Attributes.end())
    {
      return iter->second;
    }

    return Any::Empty;
  }

  void MetaAttribute::AddAttribute(const karto::String& rId, const Any& rValue) const
  {
    m_pPrivate->m_Attributes[rId] = rValue;
  }

}