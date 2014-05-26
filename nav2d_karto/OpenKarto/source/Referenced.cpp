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

#ifdef USE_TBB
#include <tbb/mutex.h>
#endif

#include <OpenKarto/Referenced.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ReferencedPrivate
  {
#ifdef USE_TBB
    tbb::mutex m_Mutex;
#endif

    kt_int32s m_Counter;

    ReferencedPrivate()
      : m_Counter(0)
    {
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Referenced::Referenced()
    : m_pReferencedPrivate(new ReferencedPrivate())
  {
  }

  Referenced::~Referenced()
  {
    if (m_pReferencedPrivate->m_Counter > 0)
    {
      std::cerr << "Deleting referenced object!!" << std::endl;
      assert(false);
    }

    delete m_pReferencedPrivate;
    m_pReferencedPrivate = NULL;
  }

  kt_int32s Referenced::Reference() const
  {
#ifdef USE_TBB
    tbb::mutex::scoped_lock lock(m_pReferencedPrivate->m_Mutex);
#endif

    return ++m_pReferencedPrivate->m_Counter;
  }

  kt_int32s Referenced::Unreference() const
  {
    kt_int32s count = 0;
    bool deleteMe = false;

    {
#ifdef USE_TBB
      tbb::mutex::scoped_lock lock(m_pReferencedPrivate->m_Mutex);
#endif

      count = --m_pReferencedPrivate->m_Counter;
      deleteMe = count <= 0;
    }

    if (deleteMe)
    {
      delete this;
    }

    return count;
  }

  kt_int32s Referenced::UnreferenceNoDelete() const
  {
    {
#ifdef USE_TBB
      tbb::mutex::scoped_lock lock(m_pReferencedPrivate->m_Mutex);
#endif

      return --m_pReferencedPrivate->m_Counter;
    }
  }

  kt_int32s Referenced::GetReferenceCount()
  {
    return m_pReferencedPrivate->m_Counter;
  }

}
