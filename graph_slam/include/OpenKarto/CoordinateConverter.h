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

#ifndef __OpenKarto_CoordinateConverter_h__
#define __OpenKarto_CoordinateConverter_h__

#include <OpenKarto/Geometry.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Convert coordinates between world and grid coordinates.
   * In world coordinates, 1.0 = 1 meter, whereas in grid coordinates, 1 = 1 pixel!
   * The conversion from pixels to meters is 1.0 meter = 1 pixel / scale.
   */
  class CoordinateConverter
  {
  public:
    /** 
     * Constructs a converter with a scale of 20.0 (conversion: 1 pixel = 0.05 meters)
     */
    CoordinateConverter()
      : m_Scale(20.0)
    {
    }

  public:
    /**
     * Scales the given value
     * @param value value to scale
     * @return scaled value
     */
    KARTO_DEPRECATED inline kt_double Transform(kt_double value)
    {
      return value * m_Scale;
    }

    /**
     * Converts the given world coordinate to grid coordinates
     * @param rWorld world coordinate
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return equivalent grid coordinate of given world coordinate
     */
    inline Vector2i WorldToGrid(const Vector2d& rWorld, kt_bool flipY = false) const
    {
      kt_double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
      kt_double gridY = 0.0;

      if (flipY == false)
      {
        gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
      }
      else
      {
        gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) * m_Scale;
      }

      return Vector2i(static_cast<kt_int32s>(math::Round(gridX)), static_cast<kt_int32s>(math::Round(gridY)));
    }

    /**
     * Converts the given grid coordinate to world coordinates
     * @param rGrid grid coordinate
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return equivalent world coordinate of given grid coordinate
     */
    inline Vector2d GridToWorld(const Vector2i& rGrid, kt_bool flipY = false) const
    {
      kt_double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
      kt_double worldY = 0.0;
      
      if (flipY == false)
      {
        worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
      }
      else
      {
        worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
      }

      return Vector2d(worldX, worldY);
    }

    /**
     * Gets the scale
     * @return scale
     */
    inline kt_double GetScale() const
    {
      return m_Scale;
    }
    
    /**
     * Sets the scale
     * @param scale
     */
    inline void SetScale(kt_double scale)
    {
      m_Scale = scale;
    }

    /**
     * Gets the offset
     * @return offset
     */
    inline const Vector2d& GetOffset() const
    {
      return m_Offset;
    }

    /**
     * Sets the offset
     * @param rOffset new offset
     */
    inline void SetOffset(const Vector2d& rOffset)
    {
      m_Offset = rOffset;
    }
    
    /**
     * Sets the size
     * @param rSize new size
     */
    inline void SetSize(const Size2<kt_int32s>& rSize)
    {
      m_Size = rSize;
    }

    /**
     * Gets the size
     * @return size
     */
    inline const Size2<kt_int32s>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Gets the resolution
     * @return resolution
     */
    inline kt_double GetResolution() const
    {
      return 1.0 / m_Scale;
    }

    /**
     * Sets the resolution
     * @param resolution new resolution
     */
    inline void SetResolution(kt_double resolution)
    {
      m_Scale = 1.0 / resolution;
    }
    
    /**
     * Gets the bounding box
     * @return bounding box
     */
    inline BoundingBox2 GetBoundingBox() const
    {
      BoundingBox2 box;
      
      kt_double minX = GetOffset().GetX();
      kt_double minY = GetOffset().GetY();
      kt_double maxX = minX + GetSize().GetWidth() * GetResolution();
      kt_double maxY = minY + GetSize().GetHeight() * GetResolution();
      
      box.SetMinimum(GetOffset());
      box.SetMaximum(Vector2d(maxX, maxY));
      return box;
    }
    
  private:
    Size2<kt_int32s> m_Size;
    kt_double m_Scale;

    Vector2d m_Offset;
  }; // CoordinateConverter

  //@}

}

#endif // __OpenKarto_CoordinateConverter_h__
