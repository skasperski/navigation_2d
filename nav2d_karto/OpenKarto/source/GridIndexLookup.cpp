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

#include <OpenKarto/GridIndexLookup.h>

namespace karto
{

  LookupArray::LookupArray()
    : m_pArray(NULL)
    , m_Capacity(0)
    , m_Size(0)
  {
  }

  LookupArray::~LookupArray()
  {
    assert(m_pArray != NULL);

    delete[] m_pArray;
    m_pArray = NULL;
  }

  void LookupArray::Clear()
  {
    memset(m_pArray, 0, sizeof(kt_int32s) * m_Capacity);
  }

  kt_int32u LookupArray::GetSize() const
  {
    return m_Size;
  }

  void LookupArray::SetSize(kt_int32u size)
  {
    assert(size != 0);

    if (size > m_Capacity)
    {
      if (m_pArray != NULL)
      {
        delete [] m_pArray;
      }
      m_Capacity = size;
      m_pArray = new kt_int32s[m_Capacity];
    }

    m_Size = size;
  }

}