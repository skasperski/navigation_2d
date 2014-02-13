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

#ifndef __OpenKarto_MetaAttribute_h__
#define __OpenKarto_MetaAttribute_h__

#include <OpenKarto/String.h>
#include <OpenKarto/Any.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template <typename T> class MetaClassHelper;

  struct MetaAttributePrivate;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Attribute container for MetaClass.
   */
  class KARTO_EXPORT MetaAttribute
  {
  public:
    /**
     * Destructor
     */
    virtual ~MetaAttribute();

  public:
    /**
     * Check if attribute exists in container.
     * @param rId If of attribute
     * @return true if attribute with rID is defines, false otherwise
     */
    kt_bool HasAttribute(const karto::String& rId) const;

    /**
     * Gets attribute with id.
     * @param rId If of attribute
     * @return Attribute with rID
     */
    const Any& GetAttribute(const karto::String& rId) const;

  protected:
    /**
     * Default constructor
     */
    MetaAttribute();

    /**
     * Adds attribute with id and value to container
     * @param rId If of attribute
     * @param rValue Value of attribute
     */
    void AddAttribute(const karto::String& rId, const Any& rValue) const;

  private:
    template <typename T> friend class MetaClassHelper;

    MetaAttributePrivate* m_pPrivate;
  };

  //@}

}

#endif // __OpenKarto_MetaAttribute_h__
