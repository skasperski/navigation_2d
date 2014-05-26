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

#include <OpenKarto/OccupancyGrid.h>
#include <OpenKarto/OpenMapper.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  OccupancyGrid::OccupancyGrid(kt_int32s width, kt_int32s height, const Vector2d& rOffset, kt_double resolution) 
    : Grid<kt_int8u>(width, height)
    , m_pCellPassCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
    , m_pCellHitsCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
    , m_pCellUpdater(NULL)
  {
    m_pCellUpdater = new CellUpdater(this);

    if (karto::math::DoubleEqual(resolution, 0.0))
    {
      throw Exception("Resolution cannot be 0");
    }

    m_pMinPassThrough = new Parameter<kt_int32u>(NULL, "MinPassThrough", "", "", 2);
    m_pOccupancyThreshold = new Parameter<kt_double>(NULL, "OccupancyThreshold", "", "", 0.1);

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  OccupancyGrid::~OccupancyGrid()
  {
    delete m_pCellUpdater;
  }

  OccupancyGrid* OccupancyGrid::Clone() const
  {
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(GetWidth(), GetHeight(), GetCoordinateConverter()->GetOffset(), 1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt = m_pCellPassCnt->Clone();
    pOccupancyGrid->m_pCellHitsCnt = m_pCellHitsCnt->Clone();

    return pOccupancyGrid;
  }

  // KARTO_DEPRECATED
#ifdef WIN32
  OccupancyGrid* OccupancyGrid::CreateFromScans(const std::vector< SmartPointer<LocalizedRangeScan> >& /*rScans*/, kt_double /*resolution*/)
  {
    throw Exception("OccupancyGrid::CreateFromScans - Not supported in Windows. Please Use CreateFromScans(const LocalizedLaserScanList& rScans, kt_double resolution).");
  }
#else
  OccupancyGrid* OccupancyGrid::CreateFromScans(const std::vector< SmartPointer<LocalizedRangeScan> >& rScans, kt_double resolution)
  {
    LocalizedLaserScanList scans;
    const_forEach(std::vector< SmartPointer<LocalizedRangeScan> >, &rScans)
    {
      scans.Add(*iter);
    }

    return CreateFromScans(scans, resolution);
  }
#endif

  OccupancyGrid* OccupancyGrid::CreateFromScans(const LocalizedLaserScanList& rScans, kt_double resolution)
  {
    if (rScans.Size() == 0)
    {
      return NULL;
    }

    kt_int32s width, height;
    Vector2d offset;
    OccupancyGrid::ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;      
  }

  OccupancyGrid* OccupancyGrid::CreateFromMapper(OpenMapper* pMapper, kt_double resolution)
  {
    return karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), resolution);
  }

  void OccupancyGrid::ComputeDimensions(const LocalizedLaserScanList& rScans, kt_double resolution, kt_int32s& rWidth, kt_int32s& rHeight, Vector2d& rOffset)
  {
    BoundingBox2 boundingBox;
    karto_const_forEach(LocalizedLaserScanList, &rScans)
    {
      LocalizedLaserScan* pLocalizedLaserScan = *iter;

      if (pLocalizedLaserScan != NULL)
      {
        boundingBox.Add(pLocalizedLaserScan->GetBoundingBox());
      }
    }

    kt_double scale = 1.0 / resolution;
    Size2<kt_double> size = boundingBox.GetSize();

    rWidth = static_cast<kt_int32s>(math::Round(size.GetWidth() * scale));
    rHeight = static_cast<kt_int32s>(math::Round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  void OccupancyGrid::CreateFromScans(const LocalizedLaserScanList& rScans)
  {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    karto_const_forEach(LocalizedLaserScanList, &rScans)
    {
      AddScan(*iter);
    }

    UpdateGrid();
  }

  kt_bool OccupancyGrid::AddScan(LocalizedLaserScan* pScan, kt_bool doUpdate)
  {
    kt_double rangeThreshold = pScan->GetLaserRangeFinder()->GetRangeThreshold();
    kt_double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();
    kt_double minRange = pScan->GetLaserRangeFinder()->GetMinimumRange();

    Vector2d scanPosition = pScan->GetSensorPose().GetPosition();

    // get scan point readings
    const Vector2dList& rPointReadings = pScan->GetPointReadings(false);

    kt_bool scanInGrid = false;

    // draw lines from scan position to all point readings 
    karto_const_forEach(Vector2dList, &rPointReadings)
    {
      Vector2d point = *iter;
      kt_double range = scanPosition.Distance(point);
      kt_bool isEndPointValid = range < (rangeThreshold - KT_TOLERANCE);

      if (range >= maxRange || range < minRange)
      {
        // ignore max range or min range readings
        continue;
      }
      else if (range >= rangeThreshold)
      {
        // trace up to range reading
        kt_double ratio = rangeThreshold / range;
        kt_double dx = point.GetX() - scanPosition.GetX();
        kt_double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      if (RayTrace(scanPosition, point, isEndPointValid, doUpdate))
      {
        scanInGrid = true;
      }
    }

    return scanInGrid;
  }

  kt_double OccupancyGrid::RayCast(const Pose2& rPose2, kt_double maxRange) const
  {
    kt_double scale = GetCoordinateConverter()->GetScale();

    kt_double x = rPose2.GetX();
    kt_double y = rPose2.GetY();
    kt_double theta = rPose2.GetHeading();

    kt_double sinTheta = sin(theta);
    kt_double cosTheta = cos(theta);

    kt_double xStop = x + maxRange * cosTheta;
    kt_double xSteps = 1 + fabs(xStop - x) * scale;

    kt_double yStop = y + maxRange * sinTheta;
    kt_double ySteps = 1 + fabs(yStop - y) * scale;

    kt_double steps = math::Maximum(xSteps, ySteps);
    kt_double delta = maxRange / steps;
    kt_double distance = delta;

    for (kt_int32u i = 1; i < steps; i++)
    {
      kt_double x1 = x + distance * cosTheta;
      kt_double y1 = y + distance * sinTheta;

      Vector2i gridIndex = WorldToGrid(Vector2d(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex))
      {
        distance = (i + 1) * delta;
      }
      else
      {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  kt_bool OccupancyGrid::RayTrace(const Vector2d& rWorldFrom, const Vector2d& rWorldTo, kt_bool isEndPointValid, kt_bool doUpdate)
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    Vector2i gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2i gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    CellUpdater* pCellUpdater = doUpdate ? m_pCellUpdater : NULL;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(), gridTo.GetY(), pCellUpdater);        

    // for the end point
    if (isEndPointValid)
    {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo))
      {
        kt_int32s index = m_pCellPassCnt->GridIndex(gridTo, false);

        kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;

        if (doUpdate)
        {
          (*m_pCellUpdater)(index);
        }
      }
    }      

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  void OccupancyGrid::UpdateCell(kt_int8u* pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt)
  {
    if (cellPassCnt > m_pMinPassThrough->GetValue())
    {
      kt_double hitRatio = static_cast<kt_double>(cellHitCnt) / static_cast<kt_double>(cellPassCnt);

      if (hitRatio > m_pOccupancyThreshold->GetValue())
      {
        *pCell = GridStates_Occupied;
      }
      else
      {
        *pCell = GridStates_Free;
      }
    }      
  }

  void OccupancyGrid::UpdateGrid()
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    // clear grid
    Clear();

    // set occupancy status of cells
    kt_int8u* pDataPtr = GetDataPointer();
    kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    kt_int32u nBytes = GetDataSize();
    for (kt_int32u i = 0; i < nBytes; i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++)
    {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  void OccupancyGrid::Resize(kt_int32s width, kt_int32s height)
  {
    Grid<kt_int8u>::Resize(width, height);

    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void CellUpdater::operator() (kt_int32u index)
  {
    kt_int8u* pDataPtr = m_pOccupancyGrid->GetDataPointer();
    kt_int32u* pCellPassCntPtr = m_pOccupancyGrid->m_pCellPassCnt->GetDataPointer();
    kt_int32u* pCellHitCntPtr = m_pOccupancyGrid->m_pCellHitsCnt->GetDataPointer();

    m_pOccupancyGrid->UpdateCell(&pDataPtr[index], pCellPassCntPtr[index], pCellHitCntPtr[index]);
  }

}