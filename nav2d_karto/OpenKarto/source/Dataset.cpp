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

#include <OpenKarto/Dataset.h>
#include <OpenKarto/TypeCasts.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct DatasetPrivate
  {
    ObjectList m_Objects;
    SmartPointer<DatasetInfo> m_pDatasetInfo;
  };

  Dataset::Dataset()
    : m_pDatasetPrivate(new DatasetPrivate())
  {
    m_pDatasetPrivate->m_pDatasetInfo = NULL;
  }

  Dataset::~Dataset()
  {
    Clear();

    delete m_pDatasetPrivate;
  }

  void Dataset::Add(Object* pObject)
  {
    if (pObject != NULL)
    {
      if (IsDatasetInfo(pObject))
      {
        m_pDatasetPrivate->m_pDatasetInfo = dynamic_cast<DatasetInfo*>(pObject);
      }
      else
      {
        m_pDatasetPrivate->m_Objects.Add(pObject);
      }
    }
  }

  DatasetInfo* Dataset::GetDatasetInfo()
  {
    return m_pDatasetPrivate->m_pDatasetInfo;
  }

  void Dataset::Clear()
  {
    m_pDatasetPrivate->m_Objects.Clear();
    m_pDatasetPrivate->m_pDatasetInfo = NULL;
  }

  const ObjectList& Dataset::GetObjects() const
  {
    return m_pDatasetPrivate->m_Objects;
  }

  Object* Dataset::operator[](kt_int32u index) const
  {
    assert(index < m_pDatasetPrivate->m_Objects.Size());
    return m_pDatasetPrivate->m_Objects[index];
  }

}