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

#ifndef __OpenKarto_MetaEnumManager_h__
#define __OpenKarto_MetaEnumManager_h__

#include <OpenKarto/String.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class MetaEnum;

  struct MetaEnumManagerPrivate;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Internal class used to manage all MetaEnums. Please don't use unless you 
   * know what you re doing!
   */
  class KARTO_EXPORT MetaEnumManager
  {
  public:
    /**
     * Get instance of MetaEnumManager (singleton)
     */
    static MetaEnumManager& GetInstance();

  public:
    /**
     * Register new MetaEnum class with name and id
     * @param rName
     * @param rId
     * @return new MetaEnum class
     */
    MetaEnum& RegisterNew(const karto::String& rName, const karto::String& rId);

    /**
     * Gets MetaEnum by name
     * @param rName
     * @return MetaEnum
     * @throw Exception if no MetaEnum registered by name
     */
    const MetaEnum& GetByName(const karto::String& rName) const;

    /**
     * Gets MetaEnum by id
     * @param rId
     * @return MetaEnum
     * @throw Exception if no MetaEnum registered by id
     */
    const MetaEnum& GetById(const karto::String& rId) const;

    /**
     * Checks if MetaEnum exists by id
     * @param rId
     * @return true if MetaEnum exists; false otherwise
     */
    kt_bool EnumExists(const karto::String& rId) const;

    /**
     * Gets number of registered MetaEnums 
     * @return number of MetaEnums
     */
    kt_size_t GetSize() const;

    /**
     * Clear all registered meta enums.
     * @Note Do not call. Called by Environment on Terminate.
     */
    void Clear();

  public:
    /**
     * Index operator, get MetaEnum at index
     * @param index
     * @return MetaEnum&
     */
    const MetaEnum& operator [] (kt_size_t index) const;

  private:
    /**
     * Default constructor
     */
    MetaEnumManager();

    /**
     * Default destructor
     */
    ~MetaEnumManager();

  private:
    /**
     * Restrict the copy constructor
     */
    MetaEnumManager(const MetaEnumManager&);

    /**
     * Restrict the assignment operator
     */
    const MetaEnumManager& operator=(const MetaEnumManager&);

  private:
    MetaEnumManagerPrivate* m_pPrivate;
  };

  // @endcond

  //@}

}

#endif // __OpenKarto_MetaEnumManager_h__
