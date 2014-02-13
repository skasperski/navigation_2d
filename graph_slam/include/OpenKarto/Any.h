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

#ifndef __OpenKarto_Any_h__
#define __OpenKarto_Any_h__

#include <algorithm>
#include <typeinfo>

#include <OpenKarto/Exception.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // The following code is from the boost library with the following license 
  // and modified to fit into the KartoSDK
  
  // Copyright Kevlin Henney, 2000, 2001, 2002. All rights reserved.
  //
  // Distributed under the Boost Software License, Version 1.0. (See
  // accompAnying file LICENSE_1_0.txt or copy at
  // http://www.boost.org/LICENSE_1_0.txt)
  // -- End original copyright --

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * The Any class can hold instances of any type that satisfies ValueType requirements.
   */
  class Any
  {
  public: 
    /** 
     * Default constructor
     */
    Any()
      : m_pContent(NULL)
    {
    }

    /** 
     * Makes a copy of value, so that the initial content of the new instance 
     * is equivalent in both type and value to value.
     * @param rValue
     */
    template<typename T>
    Any(const T& rValue)
      : m_pContent(new Holder<T>(rValue))
    {
    }

    /**
     * Copy constructor that copies content of other into new instance, so 
     * that any content is equivalent in both type and value to the content 
     * of other, or empty if other is empty. 
     * @param rOther
     */
    Any(const Any& rOther)
      : m_pContent(rOther.m_pContent ? rOther.m_pContent->Clone() : NULL)
    {
    }

    /**
     * Releases any and all resources used in management of instance.
     */
    ~Any()
    {
      delete m_pContent;
    }

  public:
    /**
     * Exchange of the contents of *this and rOther.
     * @param rOther other Any
     * @return this Any swapped with rOther
     */
    Any& Swap(Any& rOther)
    {
      std::swap(m_pContent, rOther.m_pContent);
      return *this;
    }

    /**
     * Assignment operator.
     */
    template<typename T>
    Any & operator=(const T& rOther)
    {
      Any(rOther).Swap(*this);
      return *this;
    }

    /**
     * Assignment operator.
     */
    Any& operator=(const Any& rOther)
    {
      Any(rOther).Swap(*this);
      return *this;
    }

  public:
    /**
     * Checks if instance is empty.
     * @return true if instance is empty, otherwise false
     */
    kt_bool IsEmpty() const
    {
      return !m_pContent;
    }

    /**
     * Gets the typeid of the contained value if instance is non-empty, otherwise typeid(void).
     */
    const std::type_info& GetType() const
    {
      return m_pContent ? m_pContent->GetType() : typeid(void);
    }

  private:
    class PlaceHolder
    {
    public:

      virtual ~PlaceHolder()
      {
      }

    public:
      virtual const std::type_info & GetType() const = 0;

      virtual PlaceHolder * Clone() const = 0;
    };

    template<typename T>
    class Holder : public PlaceHolder
    {
    public:
      Holder(const T & value)
        : held(value)
      {
      }

    public:
      virtual const std::type_info & GetType() const
      {
        return typeid(T);
      }

      virtual PlaceHolder * Clone() const
      {
        return new Holder(held);
      }

    public:
      T held;

    private: 
      Holder & operator=(const Holder &);
    };

  public:
    /**
     * Empty Any 
     */
    static const Any Empty;

  private:
    template<typename T>
    friend T * any_cast(Any *);

    PlaceHolder* m_pContent;
  }; // class Any

  /**
   * Cast for extracting a value of a given type from an Any
   * @param pAny Any value to cast
   * @returns a qualified pointer to the value content if successful, otherwise NULL is returned
   */
  template<typename T>
  T * any_cast(Any* pAny)
  {
    return pAny && pAny->GetType() == typeid(T) ? &static_cast<Any::Holder<T> *>(pAny->m_pContent)->held : NULL;
  }

  /**
   * Cast for extracting a const value of a given type from an const Any
   * @param pAny Any value to cast
   * @returns a qualified pointer to the value content if successful, otherwise NULL is returned
   */
  template<typename T>
  inline const T * any_cast(const Any* pAny)
  {
    return any_cast<T>(const_cast<Any*>(pAny));
  }

  /**
   * Cast for extracting a reference value of a given type from an Any
   * Throws an Karto::Exception if unable to cast reference
   * @param rAny Any value to cast
   * @returns a copy of the value content if successful
   */
  template<typename T>
  inline T any_cast(const Any& rAny)
  {
    const T* pResult = any_cast<T>(&rAny);
    if (!pResult)
    {
      throw karto::Exception("Unable to perform any_cast");
    }

    return *pResult;
  }

  //@}

}

#endif // __OpenKarto_Any_h__
