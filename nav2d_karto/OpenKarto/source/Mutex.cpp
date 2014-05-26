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

#ifdef USE_TBB
#include <tbb/mutex.h>
#else
#include <iostream>
#endif

#include <OpenKarto/Mutex.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MutexPrivate
  {
#ifdef USE_TBB
    tbb::mutex m_Mutex;
#endif
  };

  struct ScopedLockPrivate
  {
#ifdef USE_TBB
    tbb::mutex::scoped_lock m_Lock;
#endif
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Mutex::Mutex()
    : m_pMutexPrivate(new MutexPrivate())
  {
#ifndef USE_TBB
    std::cout << "Warning 'NULL Mutex' is used, so no thread safety! Compile with TBB and define USE_TBB to enable TBB mutex." << std::endl;
#endif
  }

  Mutex::~Mutex()
  {
    delete m_pMutexPrivate;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Mutex::ScopedLock::ScopedLock()
    : m_pScopedLockPrivate(new ScopedLockPrivate())
  {
  }

  Mutex::ScopedLock::ScopedLock(Mutex& rMutex)
    : m_pScopedLockPrivate(new ScopedLockPrivate())
  {
    Acquire(rMutex);
  }

  Mutex::ScopedLock::~ScopedLock()
  {
    Release();

    delete m_pScopedLockPrivate;
  }

  void Mutex::ScopedLock::Acquire(Mutex& rMutex)
  {
#ifdef USE_TBB
    m_pScopedLockPrivate->m_Lock.acquire(rMutex.m_pMutexPrivate->m_Mutex);
#endif
  }

  bool Mutex::ScopedLock::TryAcquire(Mutex& rMutex)
  {
#ifdef USE_TBB
    return m_pScopedLockPrivate->m_Lock.try_acquire(rMutex.m_pMutexPrivate->m_Mutex);
#else
    return true;
#endif
  }

  void Mutex::ScopedLock::Release()
  {
#ifdef USE_TBB
    m_pScopedLockPrivate->m_Lock.release();
#endif
  }
}
