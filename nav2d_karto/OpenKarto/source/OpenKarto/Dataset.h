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

#ifndef __OpenKarto_Dataset_h__
#define __OpenKarto_Dataset_h__

#include <OpenKarto/Referenced.h>
#include <OpenKarto/Objects.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct DatasetPrivate;

  /**
   * Container for Karto objects.
   * @deprecated Please use ObjectList instead.
   */
  class KARTO_DEPRECATED KARTO_EXPORT Dataset : public Referenced
  {
  public:
    /**
     * Empty container
     */
    Dataset();

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Dataset();
    //@endcond
    
  public:
    /**
     * Adds given object to this dataset
     * @param pObject object
     */
    void Add(Object* pObject);

    /**
     * Gets information about this dataset
     * @return info about this dataset
     */
    DatasetInfo* GetDatasetInfo();

    /**
     * Empties this dataset
     */
    void Clear();

    /**
     * Gets list of objects contained in this dataset
     * @return list of objects contained in this dataset
     */
    const ObjectList& GetObjects() const;

  public:
    /**
     * Gets the object at the given index
     * @param index index
     * @return object at the given index
     */
    Object* operator [] (kt_int32u index) const;

  private:
    // restrict the following functions
    Dataset(const Dataset&);
    const Dataset& operator=(const Dataset&);

  private:
    DatasetPrivate* m_pDatasetPrivate;
  }; // class Dataset

  /**
   * Type declaration of Dataset managed by SmartPointer
   */
  typedef SmartPointer<Dataset> DatasetPtr;

  //@}

}

#endif // __OpenKarto_Dataset_h__
