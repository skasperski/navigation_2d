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

#ifndef __OpenKarto_MetaEnumHelper_h__
#define __OpenKarto_MetaEnumHelper_h__

#include <OpenKarto/String.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  class MetaEnum;

  /** 
   * Helper class to create MetaEnum. Allows for setting enum pair information
   */
  class KARTO_EXPORT MetaEnumHelper
  {
  public:
    /** 
     * Construct the meta enum helper with a MetaEnum
     * @param rMetaEnum The MetaEnum to create
     */
    MetaEnumHelper(MetaEnum& rMetaEnum);

  public:
    /**
     * Specify a enum pair with name and value for the MetaEnum
     * @param rName Name of the enum pair
     * @param value Value of the enum pair
     * @return Helper class for creating MetaEnum
     */
    MetaEnumHelper& Value(const karto::String& rName, kt_int64s value);

  private:
    MetaEnum* m_pMetaEnum;
  };

  // @endcond

  //@}

}

#endif // __OpenKarto_MetaEnumHelper_h__
