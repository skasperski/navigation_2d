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

#pragma once

#ifndef __OpenKarto_Mutex_h__
#define __OpenKarto_Mutex_h__

#include <OpenKarto/Macros.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MutexPrivate;
  struct ScopedLockPrivate;

  /** 
   * Wrapper for TBB mutex.  Check if OpenKarto is compiled with TBB.
   * @note All releases of Karto from SRI International are compiled with TBB with USE_POCO=1. If you 
   * are manually compiling OpenKarto and don't include TBB, thread safety is NOT guaranteed!
   */
  class KARTO_EXPORT Mutex
  {
  public:
    /** 
     * Mutex
     */
    Mutex();

    /** 
     * Destructor
     */
    ~Mutex();

  public:   
    /**
     * Wrapper for TBB mutex::scopedlock
     */
    class KARTO_EXPORT ScopedLock
    {   
    public:   
      ScopedLock();
      
      /**
       * Locks with the given mutex after construction
       * @param rMutex mutex to acquire
       */
      ScopedLock(Mutex& rMutex);
      ~ScopedLock();

    public:
      /**
       * Locks with the given mutex
       * @param rMutex mutex to acquire
       */
      void Acquire(Mutex& rMutex);
      
      /**
       * Tries to lock with the given mutex
       * @param rMutex mutex to acquire
       * @return true if the mutex has been acquired, false otherwise
       */
      bool TryAcquire( Mutex& rMutex);
      
      /**
       * Releases the lock
       */
      void Release();

    private:
      ScopedLockPrivate* m_pScopedLockPrivate;
    };

  private:
    // restrict the following functions
    Mutex(const Mutex&);   
    void operator=(const Mutex&);   

  private:
    MutexPrivate* m_pMutexPrivate;
  };

  //@}

}

#endif // __OpenKarto_Mutex_h__
