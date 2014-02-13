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

#ifndef __OpenKarto_MetaClassHelper_h__
#define __OpenKarto_MetaClassHelper_h__

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  /** 
   * Helper class to create MetaClass. Allows for setting attributes, parameters, constructors and base class information
   */
  template <typename T>
  class MetaClassHelper
  {
  public:
    /**
     * Constructor
     */
    MetaClassHelper(MetaClass& target)
      : m_pMetaClass(&target)
      , m_pAttributes(m_pMetaClass)
    {
    }

    /**
     * Add static base class
     */
    template <typename U>
    MetaClassHelper<T>& Base()
    {
      const MetaClass& baseClass = GetMetaClassByType<U>();
      karto::String baseName = baseClass.GetName();

      karto_forEach(List<const MetaClass*>, &m_pMetaClass->m_BaseClasses)
      {
        assert((*iter)->GetName() != baseName);
      }

      m_pMetaClass->m_BaseClasses.Add(&baseClass);

      return *this;
    }

    /**
     * Add static attribute
     */
    MetaClassHelper<T>& Attribute(const karto::String& rAttributeName)
    {
      return Attribute(rAttributeName, "");
    }

    /**
     * Add static attribute and value
     */
    template <typename U>
    MetaClassHelper<T>& Attribute(const karto::String& rAttributeName, const U& rValue)
    {
      assert(m_pAttributes && !m_pAttributes->HasAttribute(rAttributeName));

      m_pAttributes->AddAttribute(rAttributeName, rValue);

      return *this;
    }

    /**
     * Add Parameter - not implemented
     */
    template <typename F1, typename F2>
    MetaClassHelper<T>& Parameter(const karto::String& rParameterName, F1 accessor1, F2 accessor2)
    {
      return *this;
    }

    /**
     * Add constructor with zero arguments
     */
    MetaClassHelper<T>& Constructor0()
    {
      MetaConstructor* pConstructor = new MetaConstructorImpl0<T>;
      m_pMetaClass->m_Constructors.Add(pConstructor);

      return *this;
    }

    /**
     * Add constructor with one argument
     */ 
    template <typename A0>
    MetaClassHelper<T>& Constructor1()
    {
      MetaConstructor* pConstructor = new MetaConstructorImpl1<T, A0>;
      m_pMetaClass->m_Constructors.Add(pConstructor);

      return *this;
    }

    /**
     * Add constructor with two argument
     */ 
    template <typename A0, typename A1>
    MetaClassHelper<T>& Constructor2()
    {
      MetaConstructor* pConstructor = new MetaConstructorImpl2<T, A0, A1>;
      m_pMetaClass->m_Constructors.Add(pConstructor);

      return *this;
    }

  private:
    MetaClass* m_pMetaClass;
    MetaAttribute* m_pAttributes;
  };
  
  // @endcond

  //@}

}

#endif // __OpenKarto_MetaClassHelper_h__
