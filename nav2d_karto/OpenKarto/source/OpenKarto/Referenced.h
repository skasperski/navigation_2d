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

#ifndef __OpenKarto_Referenced_h__
#define __OpenKarto_Referenced_h__

#include <OpenKarto/Macros.h>
#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ReferencedPrivate;

  /**
   * Base class for reference-counted objects. Combined with SmartPointer<T>, any subclass of Referenced 
   * will automatically be garbage collected when unreferenced.
   */
  class KARTO_EXPORT Referenced
  {
  public:
    /**
     * Default constructor
     */
    Referenced();

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Referenced();
    //@endcond
    
  public:
    /**
     * Increases the reference count
     * @return reference count
     */    
    kt_int32s Reference() const;

    /**
     * Decreases the reference count
     * @return reference count
     */    
    kt_int32s Unreference() const;

    /**
     * Decreases the reference count.  Does not
     * delete the object if the reference count
     * goes to 0.
     * @return reference count
     */    
    kt_int32s UnreferenceNoDelete() const;

    /**
     * Gets the reference count
     * @return reference count
     */
    kt_int32s GetReferenceCount();

  private:
    // restrict the following functions
    Referenced(const Referenced&);
    const Referenced& operator=(const Referenced&);

  private:
    ReferencedPrivate* m_pReferencedPrivate;
  }; // Referenced

  //@}
}

#endif // __OpenKarto_Referenced_h__
