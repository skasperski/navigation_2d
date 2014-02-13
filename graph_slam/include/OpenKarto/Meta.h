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

#ifndef __OpenKarto_Meta_h__
#define __OpenKarto_Meta_h__

#include <OpenKarto/MetaType.h>
#include <OpenKarto/MetaClass.h>
#include <OpenKarto/MetaClassManager.h>
#include <OpenKarto/MetaEnum.h>
#include <OpenKarto/MetaEnumManager.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Gets the number of MetaClass's registered.
   * @return number of MetClass's registered
   */
  inline kt_size_t GetRegisteredMetaClassSize()
  {
    return MetaClassManager::GetInstance().GetSize();
  }

  /**
   * Gets a registered MetaClass by index. Use GetRegisteredMetaClassSize to get index range. 
   * @param index 
   * @return reference to registered MetaClass
   * @throws Exception if index is out of range
   */
  inline const MetaClass& GetMetaClassByIndex(kt_size_t index)
  {
    return MetaClassManager::GetInstance()[index];
  }

  /**
   * Gets a registered MetaClass by name.
   * @param rName Class name
   * @return reference to registered MetaClass
   * @throws Exception if there is no registered MetaClass by specified name
   */
  inline const MetaClass& GetMetaClassByName(const karto::String& rName)
  {
    return MetaClassManager::GetInstance().GetByName(rName);
  }

  /**
   * Gets a registered MetaClass by object.
   * @param rObject object
   * @return reference to registered MetaClass
   * @throws Exception if there is no registered MetaClass by object
   */
  template <typename T>
  const MetaClass& GetMetaClassByObject(const T& rObject)
  {
    return MetaClassManager::GetInstance().GetById(GetTypeId(rObject));
  }

  /**
   * Gets a registered MetaClass by type.
   * @return reference to registered MetaClass
   * @throws Exception if there is no registered MetaClass by type T
   */
  template <typename T>
  const MetaClass& GetMetaClassByType()
  {
    return MetaClassManager::GetInstance().GetById(GetTypeId<T>());
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Gets the number of MetaEnum's registered.
   * @return number of MetaEnum's registered
   */
  inline kt_size_t GetRegisteredMetaEnumSize()
  {
    return MetaEnumManager::GetInstance().GetSize();
  }

  /**
   * Gets a registered MetaEnum by index. Use GetRegisteredMetaClassSize to get index range. 
   * @param index 
   * @return reference to registered MetaEnum
   * @throws Exception if index is out of range
   */
  inline const MetaEnum& GetMetaEnumByIndex(kt_size_t index)
  {
    return MetaEnumManager::GetInstance()[index];
  }

  /**
   * Gets a registered MetaEnum by name.
   * @param rName enum name
   * @return reference to registered MetaEnum
   * @throws Exception if there is no registered MetaEnum by specified name
   */
  inline const MetaEnum& GetMetaEnumByName(const karto::String& rName)
  {
    return MetaEnumManager::GetInstance().GetByName(rName);
  }

  /**
   * Gets a registered MetaEnum by object.
   * @param rObject object
   * @return reference to registered MetaEnum
   * @throws Exception if there is no registered MetaEnum by object
   */
  template <typename T>
  const MetaEnum& GetMetaEnumByObject(const T& rObject)
  {
    return MetaEnumManager::GetInstance().GetById(GetTypeId<T>(rObject));
  }

  /**
   * Gets a registered MetaEnum by type.
   * @return reference to registered MetaEnum
   * @throws Exception if there is no registered MetaEnum by type T
   */
  template <typename T>
  const MetaEnum& GetMetaEnumByType()
  {
    return MetaEnumManager::GetInstance().GetById(GetTypeId<T>());
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  //@cond EXCLUDE
  /**
   * Check that meta type is registered.
   * Internal method. Please don't call.
   */
  KARTO_EXPORT void CheckTypeRegistered(const char* pName, void (*registerFunc)());
  // @endcond

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Macro for adding a C++ class to the Meta system.
   */
  #define KARTO_TYPE(type) \
    template <> struct KartoTypeId<type> \
    { \
      static const char* Get(kt_bool = true) {return #type;} \
    }; \

  /**
   * Macro for adding a C++ class to the Meta system with a registration function as a callback.
   * The registration function will be call the first time the provided class is accessed.
   */
  #define KARTO_AUTO_TYPE(type, registerFunc) \
    template <> struct KartoTypeId<type> \
    { \
      static const char* Get(kt_bool checkRegister = true) \
      { \
        if (checkRegister) \
          CheckTypeRegistered(#type, registerFunc); \
        return #type; \
      } \
    }; \

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Macro for getting the right runtime type info.
   */
  #define KARTO_RTTI() \
    public: virtual const char* GetKartoClassId() const {return GetKartoTypeIdTemplate(this);} \
    private:
  
  //@}

}

#endif // __OpenKarto_Meta_h__
