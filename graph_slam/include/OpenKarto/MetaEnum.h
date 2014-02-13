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

#ifndef __OpenKarto_MetaEnum_h__
#define __OpenKarto_MetaEnum_h__

#include <OpenKarto/MetaType.h>
#include <OpenKarto/MetaEnumHelper.h>
#include <OpenKarto/MetaEnumManager.h>
#include <OpenKarto/String.h>
#include <OpenKarto/Referenced.h>
#include <OpenKarto/List.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Enum key value container used by MetaEnum
   */
  struct EnumPair
  {
    /**
     * Equality operator checks equality between two EnumPair's.
     * @param rOther
     * @return true if equal, false otherwise
     */
    kt_bool operator == (const EnumPair& rOther) const
    {
      return rOther.name == name && rOther.value == value;
    }

    /** 
     * Name 
     */
    karto::String name;

    /** 
     * Value
     */
    kt_int64s value;
  };


  /**
   * Type declaration of EnumPair List
   */
  typedef List< EnumPair > EnumPairList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct FindByName
  {
    FindByName(const karto::String& rName)
      : m_name(rName) 
    {

    }

    kt_bool operator()(const EnumPair& rOther) const
    {
      return rOther.name == m_name;
    }

    karto::String m_name;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct FindByValue
  {
    FindByValue(kt_int64s value) 
      : m_value(value) 
    {
    }

    kt_bool operator()(const EnumPair& rOther) const
    {
      return rOther.value == m_value;
    }

    kt_int64s m_value;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaEnumPrivate;

  /**
   * A MetaEnum stores information about an enum like enum name and
   * enum values see EnumPair.
   */
  class KARTO_EXPORT MetaEnum : public Referenced
  {
  public:
    /**
     * Registers a new MetaEnum by name.
     * @param rName Name of new MetaEnum
     * @return helper class for creating MetaEnum
     */
    template <typename T>
    static MetaEnumHelper Register(const karto::String& rName)
    {
      MetaEnum& newEnum = MetaEnumManager::GetInstance().RegisterNew(rName, KartoTypeId<T>::Get(false));
      return MetaEnumHelper(newEnum);
    }

  public:
    /**
     * Gets the name of the MetaEnum.
     * @return name of MetaEnum
     */
    const karto::String& GetName() const;

    /** 
     * Checks if enum have a defined EnumPair by specified name.
     * @param rName 
     * @return true of EnumPair exists, false otherwise
     */
    kt_bool HasName(const karto::String& rName) const;

    /** 
     * Checks if enum have a defined EnumPair by specified value.
     * @param value
     * @return true of EnumPair exists, false otherwise
     */
    kt_bool HasValue(kt_int64s value) const;

    /**
     * Gets the associated EnumPair name with the specified value.
     * @param value
     * @return enum name 
     * @throws Exception if no EnumPair is defined for value
     */
    const karto::String& GetName(kt_int64s value) const;
    
    /**
     * Gets the associated EnumPair value with the specified name.
     * @param rName
     * @return enum value 
     * @throws Exception if no EnumPair is defined for name
     */
    kt_int64s GetValue(const karto::String& rName) const;

    /** 
     * Gets the number of EnumPair's registered with this MetaEnum.
     * @return number of EnumPair's
     */
    kt_size_t GetSize() const;
    
    /**
     * Gets a reference to the EnumPair at index.
     * @param index
     * @return EnumPair at index
     * @throws Exception if index out of range
     */
    const EnumPair& GetPair(kt_size_t index) const;

  public:
    /**
     * Equality operator checks equality between two MetaEnum's.
     * @param rOther
     * @return true if equal, false otherwise
     */
    kt_bool operator==(const MetaEnum& rOther) const;

    /**
     * Inequality operator checks inequality between two MetaEnum's.
     * @param rOther
     * @return true if unequal, false otherwise
     */
    kt_bool operator!=(const MetaEnum& rOther) const;

  protected:
    /**
     * Adds EnumPair.
     * @param rPair EnumPair to add
     */
    void AddEnumPair(const EnumPair& rPair);

  private:
    friend class MetaEnumHelper;
    friend class MetaEnumManager;

  private:
    MetaEnum(const karto::String& rName);
    ~MetaEnum();

  private:
    MetaEnumPrivate* m_pPrivate;
  };

  //@}

}

#endif // __OpenKarto_MetaEnum_h__
