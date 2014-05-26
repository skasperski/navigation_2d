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

#ifndef __OpenKarto_Pair_h__
#define __OpenKarto_Pair_h__

#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Pair class
   * Any class that is used with Pair MUST implement operator==.
   */
  template<typename U, typename V>
  class Pair
  {
  public:
    /**
     * Empty pair
     */
    Pair()
    {
    }
    
    /**
     * A pair of the given items 
     * @param rFirst first item
     * @param rSecond second item
     */
    Pair(const U& rFirst, const V& rSecond)
    {
      m_First = rFirst;
      m_Second = rSecond;
    }
    
    /**
     * Gets the first item in this pair
     * @return first item in this pair
     */
    inline U& GetFirst()
    {
      return m_First;
    }
    
    /**
     * Gets the first item in this pair (const version)
     * @return first item in this pair
     */
    inline const U& GetFirst() const
    {
      return m_First;
    }
    
    /**
     * Gets the second item in this pair
     * @return second item in this pair
     */
    inline V& GetSecond()
    {
      return m_Second;
    }

    /**
     * Gets the second item in this pair (const version)
     * @return second item in this pair
     */
    inline const V& GetSecond() const
    {
      return m_Second;
    }
    
    /**
     * Equality operator
     */
    inline kt_bool operator==(const Pair& rPair)
    {
      return (m_First == rPair.m_First) && (m_Second == rPair.m_Second);
    }
    
  private:
    U m_First;
    V m_Second;
  };

  //@}

}

#endif // __OpenKarto_Pair_h__
