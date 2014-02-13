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

#ifndef __OpenKarto_Object_h__
#define __OpenKarto_Object_h__

#include <OpenKarto/Object.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Module parameters
   */
  class KARTO_EXPORT ModuleParameters : public Object
  {
    KARTO_RTTI();

  public:
    /**
     * ModuleParameters
     * @param rName name
     */
    ModuleParameters(const Identifier& rName);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~ModuleParameters();
    //@endcond
    
  private:
    // restrict the following functions
    ModuleParameters(const ModuleParameters&);
    const ModuleParameters& operator=(const ModuleParameters&);
  }; // ModuleParameters

  /**
   * Register Parameters with MetaClassManager
   */
  KARTO_TYPE(ModuleParameters);

  /**
   * Type declaration of Parameters managed by SmartPointer
   */
  typedef SmartPointer<ModuleParameters> ParametersPtr;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Abstract custom item object
   */
  class KARTO_EXPORT CustomItem : public Object
  {
    KARTO_RTTI();

  public:
    /**
     * Custom item with empty name
     */
    CustomItem();

    /**
     * Custom item with the given name
     * @param rName name
     */
    CustomItem(const Identifier& rName);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~CustomItem();
    //@endcond
    
  public:
    /**
     * Write out this custom item as a string
     * @return string representation of this custom item
     */
    virtual const String& Write() const = 0;
    
    /**
     * Read in this custom item from a string
     * @param pValue string representation of this custom item
     */
    virtual void Read(const String& pValue) = 0;

  private:
    // restrict the following functions
    CustomItem(const CustomItem&);
    const CustomItem& operator=(const CustomItem&);
  }; // CustomItem

  /**
   * Register CustomItem with MetaClassManager
   */
  KARTO_TYPE(CustomItem);

  /**
   * Type declaration of CustomItem managed by SmartPointer
   */
  typedef SmartPointer<CustomItem> CustomItemPtr;

  /**
   * Type declaration of CustomItem List
   */
  typedef List<CustomItemPtr> CustomItemList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Dataset info.
   * Contains title, author and other information about the dataset
   */
  class KARTO_EXPORT DatasetInfo : public Object
  {
    KARTO_RTTI();

  public:
    /**
     * All information as empty strings
     */
    DatasetInfo();
  
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~DatasetInfo();
    //@endcond
    
  public:
    /**
     * Gets title of the dataset
     * @return title of the dataset
     */
    const String& GetTitle() const;

    /**
     * Gets author(s) information about the dataset
     * @return author(s) information about the dataset
     */
    const String& GetAuthor() const;

    /**
     * Gets description about the dataset
     * @return description about the dataset
     */
    const String& GetDescription() const;

    /**
     * Gets copyright information of the dataset
     * @return copyright information of the dataset
     */
    const String& GetCopyright() const;

  private:
    // restrict the following functions
    DatasetInfo(const DatasetInfo&);
    const DatasetInfo& operator=(const DatasetInfo&);

  private:
    Parameter<String>* m_pTitle;
    Parameter<String>* m_pAuthor;
    Parameter<String>* m_pDescription;
    Parameter<String>* m_pCopyright;
  }; // class DatasetInfo

  /**
   * Register DatasetInfo with MetaClassManager
   */
  KARTO_TYPE(DatasetInfo);

  /**
   * Type declaration of DatasetInfo managed by SmartPointer
   */
  typedef SmartPointer<DatasetInfo> DatasetInfoPtr;

  //@}

}

#endif // __OpenKarto_Object_h__
