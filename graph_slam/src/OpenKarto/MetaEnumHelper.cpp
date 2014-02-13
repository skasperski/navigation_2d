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

#include <OpenKarto/MetaEnumHelper.h>
#include <OpenKarto/MetaEnum.h>

namespace karto
{

  MetaEnumHelper::MetaEnumHelper(MetaEnum& rMetaEnum)
    : m_pMetaEnum(&rMetaEnum)
  {
  }

  MetaEnumHelper& MetaEnumHelper::Value(const karto::String& rName, kt_int64s value)
  {
    assert(!m_pMetaEnum->HasName(rName));
    assert(!m_pMetaEnum->HasValue(value));

    EnumPair pair;
    pair.name = rName;
    pair.value = value;
    m_pMetaEnum->AddEnumPair(pair);

    return *this;
  }

}