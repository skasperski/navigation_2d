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

#ifndef __OpenKarto_GridIndexLookup_h__
#define __OpenKarto_GridIndexLookup_h__

#include <OpenKarto/Grid.h>
#include <OpenKarto/SensorData.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
    
  /**
   * Resizable array whose contents are not preseved upon resizing.
   * \todo Consider replacing with List<kt_int32s>
   */
  class LookupArray
  {
  public:
    /**
     * Constructs empty array
     */
    LookupArray();

    /**
     * Destructor
     */
    virtual ~LookupArray();

  public:
    /**
     * Clears array
     */
    void Clear();

    /**
     * Gets size of this array
     * @return size of this array
     */
    kt_int32u GetSize() const;

    /**
     * Sets size of this array (resize if not big enough)
     * @param size new size
     */
    void SetSize(kt_int32u size);

    /**
     * Gets reference to value at given index
     * @param index index
     * @return reference to value at given index
     */
    inline kt_int32s& operator[](kt_int32u index) 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets value at given index
     * @param index index
     * @return value at given index
     */
    inline kt_int32s operator[](kt_int32u index) const 
    {
      assert(index < m_Size);

      return m_pArray[index]; 
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer()
    {
      return m_pArray;
    }

    /**
     * Gets array pointer (const version)
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer() const
    {
      return m_pArray;
    }

  private:
    kt_int32s* m_pArray;
    kt_int32u m_Capacity;
    kt_int32u m_Size;
  }; // LookupArray

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Create lookup tables for point readings at varying angles in grid.
   * For each angle, grid indexes are calculated for each point reading.
   * This is to speed up finding best angle/position for a localized laser scan
   * 
   * Used heavily in mapper and localizer. 
   * 
   * In the localizer, this is a huge speed up for calculating possible positions.  For each particle,
   * a probability is calculated.  The laser scan is the same, but all grid indexes at all possible angles are
   * calculated.  So when calculating the particle probability at a specific angle, the index table is used
   * to look up probabilities in the probability grid.
   */
  template<typename T>
  class GridIndexLookup
  {
  public:
    /**
     * Construct a GridIndexLookup with a grid
     * @param pGrid grid pointer
     */
    GridIndexLookup(Grid<T>* pGrid)
      : m_pGrid(pGrid)
      , m_Capacity(0)
      , m_Size(0)
      , m_ppLookupArray(NULL)
    {
    }
    
    /**
     * Destructor
     */
    virtual ~GridIndexLookup()
    {
      DestroyArrays();
    }

  public:
    /**
     * Gets the lookup array for a particular angle index
     * @param index angle index
     * @return lookup array
     */
    const LookupArray* GetLookupArray(kt_int32u index) const
    {
      assert(math::IsUpTo(index, m_Size));
      
      return m_ppLookupArray[index];
    }
    
    /**
     * Gets list of angles 
     * @return list of angles
     */
    const List<kt_double>& GetAngles() const
    {
      return m_Angles;
    }
    
    /**
     * Computes lookup table of the points of the given scan for the given angular space
     * @param pScan scan
     * @param angleCenter angle at center
     * @param angleOffset computes lookup arrays for the angles within this offset around angleCenter
     * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
     */
    void ComputeOffsets(LocalizedLaserScan* pScan, kt_double angleCenter, kt_double angleOffset, kt_double angleResolution)
    {
      assert(angleOffset != 0.0);
      assert(angleResolution != 0.0);
 
      kt_int32u nAngles = static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
      SetSize(nAngles);

      //////////////////////////////////////////////////////
      // convert points into local coordinates of scan pose

      const Vector2dList& rPointReadings = pScan->GetPointReadings();

      // compute transform to scan pose
      Transform transform(pScan->GetSensorPose());

      Pose2List localPoints;
      karto_const_forEach(Vector2dList, &rPointReadings)
      {
        // do inverse transform to get points in local coordinates
        Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
        localPoints.Add(vec);
      }

      //////////////////////////////////////////////////////
      // create lookup array for different angles
      kt_double angle = 0.0;
      kt_double startAngle = angleCenter - angleOffset;
      for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * angleResolution;
        ComputeOffsets(angleIndex, angle, localPoints);
      }
      //assert(math::DoubleEqual(angle, angleCenter + angleOffset));
    }

  private:
    /**
     * Computes lookup value of points for given angle
     * @param angleIndex angle index
     * @param angle angle
     * @param rLocalPoints points in local coordinates
     */
    void ComputeOffsets(kt_int32u angleIndex, kt_double angle, const Pose2List& rLocalPoints)
    {
      m_ppLookupArray[angleIndex]->SetSize(static_cast<kt_int32u>(rLocalPoints.Size()));
      m_Angles[angleIndex] = angle;
      
      // set up point array by computing relative offsets to points readings
      // when rotated by given angle
      
      const Vector2d& rGridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();
      
      kt_double cosine = cos(angle);
      kt_double sine = sin(angle);
      
      kt_int32u readingIndex = 0;

      kt_int32s* pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

      karto_const_forEach(Pose2List, &rLocalPoints)
      {
        const Vector2d& rPosition = iter->GetPosition();
        
        // counterclockwise rotation and that rotation is about the origin (0, 0).
        Vector2d offset;
        offset.SetX(cosine * rPosition.GetX() -   sine * rPosition.GetY());
        offset.SetY(  sine * rPosition.GetX() + cosine * rPosition.GetY());
        
        // have to compensate for the grid offset when getting the grid index
        Vector2i gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);
        
        // use base GridIndex to ignore ROI 
        kt_int32s lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

        pAngleIndexPointer[readingIndex] = lookupIndex;

        readingIndex++;
      }
      assert(readingIndex == rLocalPoints.Size());
    }
        
    /**
     * Sets size of lookup table (resize if not big enough)
     * @param size new size
     */
    void SetSize(kt_int32u size)
    {
      assert(size != 0);
      
      if (size > m_Capacity)
      {
        if (m_ppLookupArray != NULL)
        {
          DestroyArrays();
        }
        
        m_Capacity = size;
        m_ppLookupArray = new LookupArray*[m_Capacity];
        for (kt_int32u i = 0; i < m_Capacity; i++)
        {
          m_ppLookupArray[i] = new LookupArray();
        }        
      }
      
      m_Size = size;
      
      m_Angles.Resize(size);
    }

    /**
     * Delete the arrays
     */
    void DestroyArrays()
    {
      for (kt_int32u i = 0; i < m_Capacity; i++)
      {
        delete m_ppLookupArray[i];
      }
      
      delete[] m_ppLookupArray;
      m_ppLookupArray = NULL;      
    }
    
  private:
    Grid<T>* m_pGrid; 

    kt_int32u m_Capacity;
    kt_int32u m_Size;
    
    LookupArray **m_ppLookupArray;

    // for sanity check
    List<kt_double> m_Angles;
  }; // class GridIndexLookup

  //@}

}

#endif // __OpenKarto_GridIndexLookup_h__
