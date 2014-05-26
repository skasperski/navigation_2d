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

#ifndef __OpenKarto_Grid_h__
#define __OpenKarto_Grid_h__

#include <OpenKarto/Object.h>
#include <OpenKarto/CoordinateConverter.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Functor
   */
  class KARTO_EXPORT Functor
  {
  public:
    /**
     * Functor function
     */
    virtual void operator() (kt_int32u) {};
  }; // Functor

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a grid
   */
  template<typename T>
  class Grid : public Object
  {
    KARTO_RTTI();

  public:
    /**
     * Creates a grid of given size and resolution
     * @param width width
     * @param height height
     * @param resolution resolution
     * @return grid pointer
     */
    static Grid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution)
    {
      Grid* pGrid = new Grid(width, height);
      
      pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);
      
      return pGrid;
    }
    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Grid()
    {
      delete [] m_pData;
      delete m_pCoordinateConverter;
    }
    //@endcond

  public:
    /**
     * Clears out the grid data
     */
    void Clear()
    {
      memset(m_pData, 0, GetDataSize() * sizeof(T));
    }
    
    /**
     * Returns a clone of this grid
     * @return grid clone
     */
    Grid* Clone()
    {
      Grid* pGrid = CreateGrid(GetWidth(), GetHeight(), GetResolution());
      pGrid->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());
      
      memcpy(pGrid->GetDataPointer(), GetDataPointer(), GetDataSize());
      
      return pGrid;
    }
    
    /**
     * Resizes the grid (deletes all old data)
     * @param width new width
     * @param height new height
     */
    virtual void Resize(kt_int32s width, kt_int32s height)
    {
      m_Width = width;
      m_Height = height;
      m_WidthStep = math::AlignValue<kt_int32s>(width, 8);
      
      if (m_pData != NULL)
      {
        delete[] m_pData;
        m_pData = NULL;
      }

      try
      {
        m_pData = new T[GetDataSize()];

        if (m_pCoordinateConverter == NULL)
        {
          m_pCoordinateConverter = new CoordinateConverter();
        }

        m_pCoordinateConverter->SetSize(Size2<kt_int32s>(width, height));
      }
      catch (...)
      {
        m_pData = NULL;

        m_Width = 0;
        m_Height = 0;
        m_WidthStep = 0;
      }
      
      Clear();
    }
    
    /**
     * Checks whether the given coordinate is a valid grid index
     * @param rGrid grid index
     * @return whether the given coordinate is a valid grid index
     */
    inline kt_bool IsValidGridIndex(const Vector2i& rGrid) const
    {
      return (math::IsUpTo(rGrid.GetX(), GetWidth()) && math::IsUpTo(rGrid.GetY(), GetHeight()));
    }
    
    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid grid index
     * @param boundaryCheck whether to check if the grid index is in the grid boundaries
     * @throws Exception if boundaryCheck is true and the grid index falls outside the grid boundaries
     * @return index into the grid data pointer of the given grid coordinate
     */
    virtual kt_int32s GridIndex(const Vector2i& rGrid, kt_bool boundaryCheck = true) const
    {
      if (boundaryCheck == true)
      { 
        if (IsValidGridIndex(rGrid) == false)
        {
          StringBuilder errorMessage;
          errorMessage << "Index (" << rGrid.GetX() << ", " << rGrid.GetY() << ")" << " out of range.  Index must be between [0; " << GetWidth() << ") and [0; " << GetHeight() << ")";
          throw Exception(errorMessage.ToString());          
        }
      }

      kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);
      
      if (boundaryCheck == true)
      {
        assert(math::IsUpTo(index, GetDataSize()));
      }
      
      return index;
    }

    /** 
     * Gets the grid coordinate from an index
     * @param index index
     * @return grid coordinate from index
     */
    Vector2i IndexToGrid(kt_int32s index) const
    {
      Vector2i grid;

      grid.SetY(index / m_WidthStep);
      grid.SetX(index - grid.GetY() * m_WidthStep);

      return grid;
    }
    
    /**
     * Converts the given world coordinate to grid coordinates
     * @param rWorld world coordinate
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return equivalent grid coordinate of given world coordinate
     */
    inline Vector2i WorldToGrid(const Vector2d& rWorld, kt_bool flipY = false) const
    {
      return GetCoordinateConverter()->WorldToGrid(rWorld, flipY);
    }
    
    /**
     * Converts the given grid coordinate to world coordinates
     * @param rGrid grid coordinate
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return equivalent world coordinate of given grid coordinate
     */
    inline Vector2d GridToWorld(const Vector2i& rGrid, kt_bool flipY = false) const
    {      
      return GetCoordinateConverter()->GridToWorld(rGrid, flipY);
    }    

    /**
     * Gets pointer to data at given grid coordinate
     * @param rGrid grid coordinate
     * @return pointer to data at given grid coordinate
     */
    T* GetDataPointer(const Vector2i& rGrid)
    {
      kt_int32s index = GridIndex(rGrid, true);      
      return m_pData + index;
    }

    /**
     * Gets pointer to data at given grid coordinate (const version)
     * @param rGrid grid coordinate
     * @return pointer to data at given grid coordinate
     */
    T* GetDataPointer(const Vector2i& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid, true);      
      return m_pData + index;
    }

    /**
     * Gets the width of this grid
     * @return width of this grid
     */
    inline kt_int32s GetWidth() const
    {
      return m_Width;
    };

    /**
     * Gets the height of this grid
     * @return height of this grid
     */
    inline kt_int32s GetHeight() const
    {
      return m_Height;
    };

    /**
     * Get the size as a Size2<kt_int32s>
     * @return size of this grid
     */
    inline const Size2<kt_int32s> GetSize() const
    {
      return Size2<kt_int32s>(GetWidth(), GetHeight());
    }

    /**
     * Get actual allocated grid width.  Note: The width of the grid is 8-byte aligned.
     * @return width step
     */
    inline kt_int32s GetWidthStep() const
    {
      return m_WidthStep;
    }

    /**
     * Gets the grid data pointer
     * @return data pointer
     */
    inline T* GetDataPointer()
    {
      return m_pData;
    }

    /**
     * Gets the grid data pointer (const version)
     * @return data pointer
     */
    inline T* GetDataPointer() const
    {
      return m_pData;
    }

    /**
     * Gets the allocated grid size in bytes
     * @return allocated grid size in bytes
     */
    inline kt_int32s GetDataSize() const
    {
      return m_WidthStep * GetHeight();
    }

    /**
     * Gets value at given grid coordinate
     * @param rGrid grid coordinate
     * @return value at given grid coordinate
     */
    inline T GetValue(const Vector2i& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid);
      return m_pData[index];
    }
    
    /**
     * Sets value at given grid coordinate
     * @param rGrid grid coordinate
     * @param rValue new value
     */
    inline void SetValue(const Vector2i& rGrid, T rValue) const
    {
      kt_int32s index = GridIndex(rGrid);
      m_pData[index] = rValue;
    }

    /**
     * Gets the coordinate converter for this grid
     * @return coordinate converter for this grid
     */
    inline CoordinateConverter* GetCoordinateConverter() const
    {
      return m_pCoordinateConverter;
    }

    /**
     * Gets the resolution
     * @return resolution
     */
    inline kt_double GetResolution() const
    {
      return GetCoordinateConverter()->GetResolution();
    }
    
    /**
     * Gets this grid's bounding box
     * @return bounding box
     */
    inline BoundingBox2 GetBoundingBox() const
    {
      return GetCoordinateConverter()->GetBoundingBox();
    }
    
    /**
     * Increments all the grid cells from (x0, y0) to (x1, y1);
     * if applicable, apply f to each cell traced
     * @param x0 x0
     * @param y0 y0
     * @param x1 x1
     * @param y1 y1
     * @param f functor
     */
    void TraceLine(kt_int32s x0, kt_int32s y0, kt_int32s x1, kt_int32s y1, Functor* f = NULL)
    {
      kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
      if (steep)
      {
        math::Swap(x0, y0);
        math::Swap(x1, y1);
      }
      if (x0 > x1)
      {
        math::Swap(x0, x1);
        math::Swap(y0, y1);
      }

      kt_int32s deltaX = x1 - x0;
      kt_int32s deltaY = abs(y1 - y0);
      kt_int32s error = 0;
      kt_int32s ystep;
      kt_int32s y = y0;

      if (y0 < y1)
      {
        ystep = 1;
      }
      else
      {
        ystep = -1;
      }

      kt_int32s pointX;
      kt_int32s pointY;
      for (kt_int32s x = x0; x <= x1; x++)
      {
        if (steep)
        {
          pointX = y;
          pointY = x;
        }
        else
        {
          pointX = x;
          pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX)
        {
          y += ystep;
          error -= deltaX;
        }

        Vector2i gridIndex(pointX, pointY);
        if (IsValidGridIndex(gridIndex))
        {
          kt_int32s index = GridIndex(gridIndex, false);
          T* pGridPointer = GetDataPointer();
          pGridPointer[index]++;
          
          if (f != NULL)
          {
            (*f)(index); 
          }
        }
      }
    }

  protected:
    /**
     * Constructs grid of given size
     * @param width width
     * @param height height
     */
    Grid(kt_int32s width, kt_int32s height)
      : m_Width(0)
      , m_Height(0)
      , m_WidthStep(0)
      , m_pData(NULL)

      , m_pCoordinateConverter(NULL)
    {
      Resize(width, height);
    }
    
  private:
    Grid(const Grid&);
    const Grid& operator=(const Grid&);

  private:
    kt_int32s m_Width;       // width of grid
    kt_int32s m_Height;      // height of grid
    kt_int32s m_WidthStep;   // 8-byte aligned width of grid
    T* m_pData;              // grid data

    CoordinateConverter* m_pCoordinateConverter; // coordinate converter to convert between world coordinates and grid coordinates
  }; // Grid

  /**
   * Register Grid<kt_int8u> with MetaClassManager
   */
  KARTO_TYPE(Grid<kt_int8u>);

  /**
   * Register Grid<kt_int32u> with MetaClassManager
   */
  KARTO_TYPE(Grid<kt_int32u>);

  /**
   * Register Grid<kt_float> with MetaClassManager
   */
  KARTO_TYPE(Grid<kt_float>);

  /**
   * Register Grid<kt_double> with MetaClassManager
   */
  KARTO_TYPE(Grid<kt_double>);

  //@}

}

#endif // __OpenKarto_Grid_h__
