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

#ifndef __OpenKarto_MetaClassManager_h__
#define __OpenKarto_MetaClassManager_h__

#include <OpenKarto/String.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class MetaClass;

  struct MetaClassManagerPrivate;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Internal class used to manage all MetaClasses. Please don't use unless you 
   * know what you re doing!
   */
  class KARTO_EXPORT MetaClassManager
  {
  public:
    /**
     * Get instance of MetaClassManager (singleton)
     */
    static MetaClassManager& GetInstance();

  public:
    /**
     * Register new MetaClass class with name and id
     * @param rName
     * @param rId
     * @return new MetaClass class
     * @throw Exception if MetaClass is already registered by name
     */
    MetaClass& RegisterNew(const karto::String& rName, const karto::String& rId);

    /**
     * Gets MetaClass by name
     * @param rName
     * @return MetaClass
     * @throw Exception if no MetaClass registered by name
     */
    const MetaClass& GetByName(const karto::String& rName) const;

    /**
     * Gets MetaClass by id
     * @param rId
     * @return MetaClass
     * @throw Exception if no MetaClass registered by id
     */
    const MetaClass& GetById(const karto::String& rId) const;

    /**
     * Checks if MetaClass exists by id
     * @param rId
     * @return true if MetaClass exists; false otherwise
     */
    kt_bool ClassExists(const karto::String& rId) const;

    /**
     * Gets number of registered MetaClasses 
     * @return number of MetaClasses
     */
    kt_size_t GetSize() const;

    /**
     * Clear all registered meta classes.
     * @Note Do not call. Called by Environment on Terminate.
     */
    void Clear();

  public:
    /**
     * Index operator, get MetaClass at index
     * @param index
     * @return MetaEnum&
     */
    const MetaClass& operator [] (kt_size_t index) const;

  private:
    /**
     * Default constructor
     */
    MetaClassManager();

    /**
     * Default destructor
     */
    ~MetaClassManager();

  private:
    /**
     * Restrict the copy constructor
     */
    MetaClassManager(const MetaClassManager&);

    /**
     * Restrict the assignment operator
     */
    const MetaClassManager& operator=(const MetaClassManager&);

  private:
    MetaClassManagerPrivate* m_pPrivate;
  };

  // @endcond

  //@}

}

#endif // __OpenKarto_MetaClassManager_h__
