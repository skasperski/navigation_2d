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

#ifndef __OpenKarto_SmartPointer_h__
#define __OpenKarto_SmartPointer_h__

#include <OpenKarto/Referenced.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages reference-counted shared objects
   */
  template<class T>
  class SmartPointer
  {
  public:
    /**
     * Default constructor
     */
    SmartPointer()
      : m_pPointer(NULL)
    {
    }

    /**
     * Copy constructor
     */
    SmartPointer(const SmartPointer& rOther)
      : m_pPointer(rOther.m_pPointer)
    {
      if (m_pPointer != NULL)
      {
        m_pPointer->Reference();
      }
    }

    /**
     * Copy constructor
     */
    template<class Other> SmartPointer(const SmartPointer<Other>& rOther) 
      : m_pPointer(rOther.m_pPointer) 
    {
      if (m_pPointer != NULL)
      {
        m_pPointer->Reference();
      }
    }

    /**
     * Assignment operator
     */
    SmartPointer(T* pPointer)
      : m_pPointer(pPointer)
    {
      if (m_pPointer != NULL)
      {
        m_pPointer->Reference();
      }
    }

    /**
     * Destructor
     */
    virtual ~SmartPointer()
    {
      Release();
    }

  public:
    /**
     * Sets pointer to NULL
     */
    inline void Release()
    {
      if (m_pPointer)
      {
        m_pPointer->Unreference();
      }

      m_pPointer = NULL;
    }

    /**
     * Gets the contained pointer
     * @return contained pointer
     */
    T* Get() const
    {
      return m_pPointer;
    }

    /**
     * Whether the contained pointer is valid
     * @return true if the contained pointer is valid
     */
    kt_bool IsValid() const
    {
      return m_pPointer != NULL; 
    }

  public:
    /**
     * Gets the contained pointer
     * @return contained pointer
     */
    operator T*() const 
    {
      return m_pPointer; 
    }

    /**
     * Gets a reference to the contained pointer
     * @return reference to the contained pointer
     */
    T& operator*() const
    {
      return *m_pPointer;
    }

    /**
     * Gets the contained pointer
     * @return contained pointer
     */
    T* operator->() const
    {
      return m_pPointer;
    }

    /**
     * Assignment operator
     */
    inline SmartPointer& operator=(const SmartPointer& rOther)
    {
      if (m_pPointer==rOther.m_pPointer)
      {
        return *this;
      }

      T* pTempPointer = m_pPointer;
      m_pPointer = rOther.m_pPointer;

      if (m_pPointer)
      {
        m_pPointer->Reference();
      }

      if (pTempPointer)
      {
        pTempPointer->Unreference();
      }

      return *this;
    }

    /**
     * Assignment operator
     */
    inline SmartPointer& operator=(T* pPointer)
    {
      if (m_pPointer == pPointer)
      {
        return *this;
      }

      T* pTempPointer = m_pPointer;
      m_pPointer = pPointer;

      if (m_pPointer)
      {
        m_pPointer->Reference();
      }

      if (pTempPointer)
      {
        pTempPointer->Unreference();
      }

      return *this;
    }

    /**
     * Assignment operator
     */
    template<class Other> SmartPointer& operator=(const SmartPointer<Other>& rOther)
    {
      if (m_pPointer == rOther.m_pPointer) 
      {
        return *this;
      }

      T* pTempPointer = m_pPointer;
      m_pPointer = rOther.m_pPointer;
      
      if (m_pPointer)
      {
        m_pPointer->Reference();
      }

      if (pTempPointer)
      {
        pTempPointer->Unreference();
      }

      return *this;
    }

  private:
    T* m_pPointer;

    template<class Other> friend class SmartPointer;
  };

  //@{}

}

#endif // __OpenKarto_SmartPointer_h__
