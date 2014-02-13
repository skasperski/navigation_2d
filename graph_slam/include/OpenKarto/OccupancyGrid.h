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

#ifndef __OpenKarto_OccupancyGird_h__
#define __OpenKarto_OccupancyGird_h__

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
   * Valid grid cell states
   */
  typedef enum
  {
    GridStates_Unknown = 0,
    GridStates_Occupied = 100,
    GridStates_Free = 255
  } GridStates;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class CellUpdater;
  class OpenMapper;

  /**
   * Occupancy grid definition. See GridStates for possible grid values.
   */
  class KARTO_EXPORT OccupancyGrid : public Grid<kt_int8u>
  {
    KARTO_RTTI();

  private:
    friend class CellUpdater;
    friend class IncrementalOccupancyGrid;
    
  public:
    /**
     * Occupancy grid of given size
     * @param width width
     * @param height height
     * @param rOffset offset
     * @param resolution resolution
     */
    OccupancyGrid(kt_int32s width, kt_int32s height, const Vector2d& rOffset, kt_double resolution);
    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~OccupancyGrid();
    //@endcond
    
  public:
    /**
     * Occupancy grid from the given scans using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param rScans list of scans
     * @param resolution resolution
     * @return occupancy grid from the given scans using the given resolution
     */
    static OccupancyGrid* CreateFromScans(const LocalizedLaserScanList& rScans, kt_double resolution);

    /**
     * Occupancy grid from the given scans using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param rScans scans
     * @param resolution resolution
     * @deprecated Please use CreateFromScans(const LocalizedLaserScanList& rScans, kt_double resolution)
     * @warning Throws exception in Windows
     * @return occupancy grid from the given scans using the given resolution
     */
    static KARTO_DEPRECATED  OccupancyGrid* CreateFromScans(const std::vector< SmartPointer<LocalizedRangeScan> >& rScans, kt_double resolution);

    /**
     * Occupancy grid from the scans in the given mapper using the given resolution
     * @note Please assign the returned occupancy grid to an OccupancyGridPtr to avoid memory leaks.
     * @param pMapper mapper
     * @param resolution resolution
     * @return occupancy grid from the given scans using the given resolution
     */
    static OccupancyGrid* CreateFromMapper(OpenMapper* pMapper, kt_double resolution);

  public:
    /**
     * Makes a clone
     * @return occupancy grid clone
     */
    OccupancyGrid* Clone() const;

    /** 
     * Checks if the given grid index is free
     * @param rGridIndex grid index
     * @return whether the cell at the given grid index is free space
     */
    inline kt_bool IsFree(const Vector2i& rGridIndex) const
    {
      kt_int8u* pOffsets = (kt_int8u*)GetDataPointer(rGridIndex);
      return (*pOffsets == GridStates_Free);
    }
    
    /**
     * Casts a ray from the given point (up to the given max range)
     * and returns the distance to the closest obstacle
     * @param rPose2 starting point
     * @param maxRange maximum range
     * @return distance to closest obstacle
     */
    kt_double RayCast(const Pose2& rPose2, kt_double maxRange) const;

  protected:
    /**
     * Gets grid of cell hit counts
     * @return grid of cell hit counts
     */
    Grid<kt_int32u>* GetCellHitsCounts()
    {
      return m_pCellHitsCnt;
    }

    /**
     * Get grid of cell pass counts
     * @return grid of cell pass counts
     */
    Grid<kt_int32u>* GetCellPassCounts()
    {
      return m_pCellPassCnt;
    }
    
    /**
     * Calculates grid dimensions from localized laser scans and resolution
     * @param rScans scans
     * @param resolution resolution
     * @param rWidth (output parameter) width
     * @param rHeight (output parameter) height
     * @param rOffset (output parameter) offset
     */
    static void ComputeDimensions(const LocalizedLaserScanList& rScans, kt_double resolution, kt_int32s& rWidth, kt_int32s& rHeight, Vector2d& rOffset);
    
    /**
     * Creates grid using scans 
     * @param rScans scans
     */
    virtual void CreateFromScans(const LocalizedLaserScanList& rScans);
        
    /**
     * Adds the scan's information to this grid's counters (optionally
     * update the grid's cells' occupancy status)
     * @param pScan laser scan
     * @param doUpdate whether to update the grid's cell's occupancy status
     * @return returns false if an endpoint fell off the grid, otherwise true
     */
    kt_bool AddScan(LocalizedLaserScan* pScan, kt_bool doUpdate = false);
    
    /**
     * Traces a beam from the start position to the end position marking
     * the bookkeeping arrays accordingly.
     * @param rWorldFrom start position of beam
     * @param rWorldTo end position of beam
     * @param isEndPointValid is the reading within the range threshold?
     * @param doUpdate whether to update the cells' occupancy status immediately
     * @return returns false if an endpoint fell off the grid, otherwise true
     */
    kt_bool RayTrace(const Vector2d& rWorldFrom, const Vector2d& rWorldTo, kt_bool isEndPointValid, kt_bool doUpdate = false);
    
    /**
     * Updates a single cell's value based on the given counters
     * @param pCell cell
     * @param cellPassCnt cell pass count
     * @param cellHitCnt cell hit count
     */
    void UpdateCell(kt_int8u* pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt);
    
    /**
     * Updates the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
     */
    void UpdateGrid();
    
    /**
     * Resizes the grid (deletes all old data)
     * @param width new width
     * @param height new height
     */
    virtual void Resize(kt_int32s width, kt_int32s height);
    
  protected:
    /**
     * Counters of number of times a beam passed through a cell
     */
    SmartPointer< Grid<kt_int32u> > m_pCellPassCnt;
    
    /**
     * Counters of number of times a beam ended at a cell    
     */
    SmartPointer< Grid<kt_int32u> >  m_pCellHitsCnt;

  private:
    CellUpdater* m_pCellUpdater;

    ////////////////////////////////////////////////////////////
    // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
    // then not many beams will hit the cell!

    // Number of beams that must pass through a cell before it will be considered to be occupied
    // or unoccupied.  This prevents stray beams from messing up the map.
    SmartPointer<Parameter<kt_int32u> > m_pMinPassThrough;

    // Minimum ratio of beams hitting cell to beams passing through cell for cell to be marked as occupied
    SmartPointer<Parameter<kt_double> > m_pOccupancyThreshold;

  private:
    // restrict the following functions
    OccupancyGrid(const OccupancyGrid&);
    const OccupancyGrid& operator=(const OccupancyGrid&);
  }; // OccupancyGrid

  /**
   * Register OccupancyGrid with MetaClassManager
   */
  KARTO_TYPE(OccupancyGrid);

  /**
   * Type declaration of OccupancyGrid managed by SmartPointer
   */
  typedef SmartPointer<OccupancyGrid> OccupancyGridPtr;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Updates the cell at the given index based on the grid's hits and pass counters
   */    
  class KARTO_EXPORT CellUpdater : public Functor
  {
  public:
    /**
     * Cell updater for the given occupancy grid
     * @param pOccupancyGrid occupancy grid
     */
    CellUpdater(OccupancyGrid* pOccupancyGrid)
      : m_pOccupancyGrid(pOccupancyGrid)
    {
    }
    
    /**
     * Updates the cell at the given index based on the grid's hits and pass counters
     * @param index index
     */    
    virtual void operator() (kt_int32u index);
    
  private:
    OccupancyGrid* m_pOccupancyGrid;
  }; // CellUpdater

  //@}

}


#endif // __OpenKarto_OccupancyGird_h__
