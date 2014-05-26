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

#ifndef __OpenKarto_SensorData_h__
#define __OpenKarto_SensorData_h__

#include <vector>

#include <OpenKarto/List.h>
#include <OpenKarto/Object.h>
#include <OpenKarto/Geometry.h>
#include <OpenKarto/Sensor.h>
#include <OpenKarto/Objects.h>
#include <OpenKarto/PoseTransform.h>
#include <OpenKarto/RigidBodyTransform.h>
#include <OpenKarto/SensorRegistry.h>
#include <OpenKarto/AbstractGpsEstimationManager.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Type declaration of range readings List
   */
  typedef List<kt_double> RangeReadingsList;

  /**
   * Type declaration of kt_double List
   */
  typedef List<kt_double> DoubleList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct SensorDataPrivate;

  /**
   * Base class for all sensor data
   */
  class KARTO_EXPORT SensorData : public Object
  {
    KARTO_RTTI();

  protected:
    /**
     * Sensor data from the sensor with the given identifier
     * @param rSensorIdentifier sensor identifier
     */
    SensorData(const Identifier& rSensorIdentifier);

    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~SensorData();
    //@endcond
    
  public:
    /**
     * Gets sensor data id
     * @return sensor id
     */
    inline kt_int32s GetStateId() const
    {
      return m_StateId;
    }

    /**
     * Sets sensor data id
     * @param stateId new id 
     */
    inline void SetStateId(kt_int32s stateId)
    {
      m_StateId = stateId;
    }

    /**
     * Gets sensor data unique id
     * @return unique id
     */
    inline kt_int32s GetUniqueId() const
    {
      return m_UniqueId;
    }

    /**
     * Sets sensor data unique id
     * @param uniqueId new unique id
     */
    inline void SetUniqueId(kt_int32u uniqueId)
    {
      m_UniqueId = uniqueId;
    }
    
    /**
     * Gets sensor data time
     * @return time
     */
    inline kt_int64s GetTime() const
    {
      return m_Time;
    }

    /**
     * Sets sensor data time
     * @param time new time
     */
    inline void SetTime(kt_int64s time)
    {
      m_Time = time;
    }

    /**
     * Gets the sensor identifier of the sensor that created this sensor data
     * @return sensor identifier
     */
    inline const Identifier& GetSensorIdentifier() const
    {
      return m_SensorIdentifier;
    }

    /**
     * Sets the name of the sensor that created this sensor data
     * @param rSensorIdentifier sensor identifier
     */
    inline void SetSensorIdentifier(const Identifier& rSensorIdentifier)
    {
      m_SensorIdentifier = rSensorIdentifier;
    }

    /**
     * Adds a custom item to this sensor data
     * @param pCustomItem custom item
     */
    void AddCustomItem(CustomItem* pCustomItem);
    
    /**
     * Gets all custom items assigned to this sensor data
     * @return list of custom items
     */
    const CustomItemList& GetCustomItems() const;

    /**
     * Checks if there is a custom item attached to this sensor data
     * @return true if there is one or more items attached, false otherwise
     */
     kt_bool HasCustomItem();

  private:
    // restrict the following functions
    SensorData(const SensorData&);
    const SensorData& operator=(const SensorData&);

  private:
    SensorDataPrivate* m_pSensorDataPrivate;

    /**
     * ID unique to individual sensor
     */
    kt_int32s m_StateId;

    /**
     * ID unique across all sensor data
     */
    kt_int32s m_UniqueId;
    
    /**
     * Name of sensor that created this sensor data
     */
    Identifier m_SensorIdentifier;

    /**
     * Time the sensor data was created
     */
    kt_int64s m_Time;
  }; // SensorData

  /**
   * Register SensorData with MetaClassManager
   */
  KARTO_TYPE(SensorData);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * LaserRangeScan representing the range readings from a laser range finder sensor.
   */
  class KARTO_EXPORT LaserRangeScan : public SensorData
  {
    KARTO_RTTI();

  public:
    /**
     * Laser range scan from the given sensor
     * @param rSensorIdentifier sensor identifier
     */
    LaserRangeScan(const Identifier& rSensorIdentifier);

    /**
     * Laser range scan from the given sensor with the given readings
     * @param rSensorIdentifier sensor identifier
     * @param rRangeReadings list of range readings
     */
    LaserRangeScan(const Identifier& rSensorIdentifier, const RangeReadingsList& rRangeReadings);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~LaserRangeScan();
    //@endcond
    
  public:
    /**
     * Gets the range readings of this scan
     * @return range readings of this scan
     */
    inline const RangeReadingsList& GetRangeReadings() const
    {
      return m_RangeReadings;
    }
  
    /**
     * Sets the range readings of this scan
     * @param rRangeReadings range readings of this scan
     */
    inline void SetRangeReadings(const RangeReadingsList& rRangeReadings)
    {
      m_RangeReadings = rRangeReadings;
    }

    /**
     * Gets the laser range finder sensor that generated this scan
     * @return laser range finder sensor of this scan
     */
    inline LaserRangeFinder* GetLaserRangeFinder() const
    {
      return SensorRegistry::GetInstance()->GetSensorByName<LaserRangeFinder>(GetSensorIdentifier());
    }

  private:
    // restrict the following functions
    LaserRangeScan(const LaserRangeScan&);
    const LaserRangeScan& operator=(const LaserRangeScan&);

  private:
    RangeReadingsList m_RangeReadings;
  }; // LaserRangeScan

  /**
   * Register LaserRangeScan with MetaClassManager
   */
  KARTO_TYPE(LaserRangeScan);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * DrivePose representing the pose value of a drive sensor.
   */
  class DrivePose : public SensorData
  {
    KARTO_RTTI();

  public:
    /**
     * Pose of the given drive sensor
     * @param rSensorIdentifier sensor identifier
     */
    DrivePose(const Identifier& rSensorIdentifier)
      : SensorData(rSensorIdentifier)
    {
    }

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~DrivePose()
    {
    }
    //@endcond

  public:
    /**
     * Gets the odometric pose of this scan
     * @return odometric pose of this scan
     */
    inline const Pose2& GetOdometricPose() const 
    {
      return m_OdometricPose;
    }

    /**
     * Sets the odometric pose of this scan
     * @param rPose new odometric pose
     */
    inline void SetOdometricPose(const Pose2& rPose)
    {
      m_OdometricPose = rPose;
    }

  private:
    // restrict the following functions
    DrivePose(const DrivePose&);
    const DrivePose& operator=(const DrivePose&);

  private:
    /**
     * Odometric pose of robot
     */
    Pose2 m_OdometricPose;
  }; // class DrivePose

  /**
   * Register DrivePose with MetaClassManager
   */
  KARTO_TYPE(DrivePose);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Base class for all localized sensor data
   */
  class KARTO_EXPORT LocalizedObject : public SensorData
  {
    KARTO_RTTI();

  public:
    /**
     * Localized object from the given sensor
     * @param rSensorIdentifier sensor identifier
     */
    LocalizedObject(const Identifier& rSensorIdentifier);
    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~LocalizedObject();
    //@endcond
    
  public:
    /**
     * Gets the odometric pose of this object
     * @return odometric pose of this object
     */
    inline const Pose2& GetOdometricPose() const 
    {
      return m_OdometricPose;
    }
    
    /**
     * Sets the odometric pose of this object
     * @param rOdometricPose new odometric pose
     */
    inline void SetOdometricPose(const Pose2& rOdometricPose)
    {
      m_OdometricPose = rOdometricPose;
    }
    
    /**
     * Gets the (possibly corrected) robot pose at which this object was taken.  The corrected robot pose of the object
     * is usually set by an external module such as a localization or mapping module when it is determined
     * that the original pose was incorrect.  The external module will set the correct pose based on
     * additional sensor data and any context information it has.  If the pose has not been corrected,
     * a call to this method returns the same pose as GetOdometricPose().
     * @return corrected pose
     */
    inline const Pose2& GetCorrectedPose() const 
    {
      return m_CorrectedPose;
    }
    
    /**
     * Moves the object by moving the robot pose to the given location.
     * @param rPose new pose of the robot of this scan
     */
    virtual inline void SetCorrectedPose(const Pose2& rPose)
    {
      m_CorrectedPose = rPose;
    }

    /**
     * Gets the GPS reading of this scan (return value is
     * meaningless if IsGpsReadingValid() returns false).
     * @return GPS reading of this scan
     */
    inline gps::PointGps GetGpsReading() const
    {
      return m_GpsReading;
    }
    
    /**
     * Sets the GPS reading of this scan
     * @param rGpsReading GPS reading of this scan
     */
    inline void SetGpsReading(const gps::PointGps& rGpsReading)
    {
      m_GpsReading = rGpsReading;
      m_IsGpsReadingValid = true;
    }
    
    /**
     * Whether the GPS reading is valid
     * @return whether the GPS reading is valid
     */
    inline kt_bool IsGpsReadingValid() const
    {
      return m_IsGpsReadingValid;
    }
    
    /**
     * Gets the GPS estimate of this scan (return value is
     * meaningless if IsGpsEstimateValid() returns false).
     * @return GPS estimate of this scan
     */
    inline gps::PointGps GetGpsEstimate() const
    {
      if (m_pGpsEstimationManager != NULL)
      {
        return m_pGpsEstimationManager->GetGpsEstimate(this);
      }
      else
      {
        return m_GpsEstimate;
      }
    }
    
    /**
     * Sets the GPS estimate of this scan
     * @param rGpsEstimate GPS estimate of this scan
     */
    inline void SetGpsEstimate(const gps::PointGps& rGpsEstimate)
    {
      if (m_pGpsEstimationManager != NULL)
      {
        m_pGpsEstimationManager->SetGpsEstimate(this, rGpsEstimate);
      }
      else
      {
        m_GpsEstimate = rGpsEstimate;
        m_IsGpsEstimateValid = true;
      }
    }
    
    /**
     * Whether the GPS estimate is valid
     * @return whether the GPS estimate is valid
     */
    inline kt_bool IsGpsEstimateValid() const
    {
      if (m_pGpsEstimationManager != NULL)
      {
        return m_pGpsEstimationManager->IsGpsEstimateValid(this);
      }
      else
      {
        return m_IsGpsEstimateValid;
      }
    }
    
    /**
     * Sets the manager for estimating this scan's GPS coordinate
     * @param pGpsEstimationManager GPS estimation manager
     */
    inline void SetGpsEstimationManager(AbstractGpsEstimationManager* pGpsEstimationManager)
    {
      m_pGpsEstimationManager = pGpsEstimationManager;
    }

  private:
    /**
     * Odometric pose of object
     */
    Pose2 m_OdometricPose;
    
    /**
     * Corrected pose of object calculated by mapper (or some other module)
     */
    Pose2 m_CorrectedPose;

    /**
     * GPS reading of this object (value is meaningless
     * if m_IsGpsReadingValid is false).
     */
    gps::PointGps m_GpsReading;
    
    /**
     * Whether the GPS reading is valid
     */
    kt_bool m_IsGpsReadingValid;

    /**
     * GPS estimate of this object (value is meaningless
     * if m_IsGpsEstimateValid is false).
     */
    gps::PointGps m_GpsEstimate;
    
    /**
     * Whether the GPS estimate is valid
     */
    kt_bool m_IsGpsEstimateValid;

    /**
     * Manages the location of robot in GPS coordinates
     */
    AbstractGpsEstimationManager* m_pGpsEstimationManager;
  }; // LocalizedObject

  /**
   * Register LocalizedObject with MetaClassManager
   */
  KARTO_TYPE(LocalizedObject);

  /**
   * Type declaration of LocalizedObject managed by SmartPointer
   */
  typedef SmartPointer<LocalizedObject> LocalizedObjectPtr;

  /**
   * Type declaration of LocalizedObject List
   */
  typedef List<LocalizedObjectPtr> LocalizedObjectList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class AbstractGpsEstimationManager;

  /**
   * Base class for localized laser scans
   */
  class KARTO_EXPORT LocalizedLaserScan : public LocalizedObject
  {    
    KARTO_RTTI();

  public:
    /**
     * Gets the laser range finder sensor that generated this scan
     * @return laser range finder sensor of this scan
     */
    inline LaserRangeFinder* GetLaserRangeFinder() const
    {
      return SensorRegistry::GetInstance()->GetSensorByName<LaserRangeFinder>(GetSensorIdentifier());
    }

    /**
     * Moves the scan by moving the robot pose to the given location.
     * @param rCorrectedPose new pose of the robot of this scan
     */
    virtual inline void SetCorrectedPose(const Pose2& rCorrectedPose)
    {
      LocalizedObject::SetCorrectedPose(rCorrectedPose);
      
      m_IsDirty = true;
    }
    
    /**
     * Gets barycenter of point readings
     * @return barycenter of point readings
     */
    inline const Pose2& GetBarycenterPose() const 
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedLaserScan*>(this)->Update();
      }
      
      return m_BarycenterPose;
    }
    
    /**
     * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
     * @param useBarycenter whether to use the barycenter as the reference pose
     * @return barycenter if given parameter is true, otherwise scanner pose
     */
    inline Pose2 GetReferencePose(kt_bool useBarycenter) const 
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedLaserScan*>(this)->Update();
      }
      
      return useBarycenter ? GetBarycenterPose() : GetSensorPose();
    }    
    
    /**
     * Computes the position of the sensor
     * @return sensor pose
     */
    inline Pose2 GetSensorPose() const
    {
      return GetSensorAt(GetCorrectedPose());
    }
    
    /**
     * Computes the robot pose from the given sensor pose
     * @param rSensorPose new pose of the sensor
     */
    void SetSensorPose(const Pose2& rSensorPose)
    {
      Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
      kt_double offsetLength = deviceOffsetPose2.GetPosition().Length();
      kt_double offsetHeading = deviceOffsetPose2.GetHeading();
      kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
      kt_double correctedHeading = math::NormalizeAngle(rSensorPose.GetHeading());
      Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                                      offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                                      offsetHeading);
      
      SetCorrectedPose(rSensorPose - worldSensorOffset);
      
      Update();
    }
    
    /**
     * Computes the position of the sensor if the robot were at the given pose
     * @param rPose hypothesized pose
     * @return sensor pose at the given pose
     */
    inline Pose2 GetSensorAt(const Pose2& rPose) const
    {
      return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
    }    
    
    /**
     * Gets the bounding box of this scan
     * @return bounding box of this scan
     */
    inline const BoundingBox2& GetBoundingBox() const
    {
      if (m_IsDirty)
      {
        // throw away constness and do an update!
        const_cast<LocalizedLaserScan*>(this)->Update();
      }
      
      return m_BoundingBox;
    }

    /**
     * Gets the point readings of this scan
     * @param wantFiltered whether filtered points are to be included or not
     * @return list of point readings
     */
    const Vector2dList& GetPointReadings(kt_bool wantFiltered = false) const;
    
    /**
     * Gets the range readings of this scan
     * @return range readings of this scan
     */
    inline const RangeReadingsList& GetRangeReadings() const
    {
      return m_RangeReadings;
    }

    /**
     * Gets the number of range readings
     * @return number of range readings
     */
    inline kt_size_t GetNumberOfRangeReadings() const
    {
      return m_RangeReadings.Size();
    }

  protected:
    /**
     * Localized laser scan from the given sensor
     * @param rSensorIdentifier sensor identifier
     */
    LocalizedLaserScan(const Identifier& rSensorIdentifier);
    virtual ~LocalizedLaserScan();

    /**
     * Computes the point readings, bounding box, and barycenter of the scan
     */
    void Update();
    
    /**
     * Computes point readings in global coordinates
     */
    virtual void ComputePointReadings() = 0;

    /**
     * Gets filtered points readings
     * @return filtered point readings
     */
    virtual const Vector2dList& GetFilteredPointReadings() const
    {
      return m_FilteredPointReadings;
    }
    
    /**
     * Gets raw points readings
     * @return raw point readings
     */
    virtual const Vector2dList& GetUnfilteredPointReadings() const
    {
      return m_UnfilteredPointReadings;
    }

  protected:
    /**
     * List of filtered point readings
     */
    Vector2dList m_FilteredPointReadings;
    
    /**
     * List of unfiltered point readings
     */
    Vector2dList m_UnfilteredPointReadings;
    
    /**
     * List of unfiltered ranges
     */
    RangeReadingsList m_RangeReadings;
    
  private:
    /**
     * Name of sensor that created this scan
     */
    Identifier m_SensorIdentifier;
        
    /**
     * Average of all the point readings
     */
    Pose2 m_BarycenterPose;
    
    /**
     * Bounding box of localized range scan
     */
    BoundingBox2 m_BoundingBox;
    
    /**
     * Internal flag used to update point readings, barycenter and bounding box
     */
    kt_bool m_IsDirty;
    
  }; // LocalizedLaserScan
  
  /**
   * Register LocalizedLaserScan with MetaClassManager
   */
  KARTO_TYPE(LocalizedLaserScan);

  /**
   * Type declaration of LocalizedLaserScan managed by SmartPointer
   */
  typedef SmartPointer<LocalizedLaserScan> LocalizedLaserScanPtr;
  
  /**
   * Type declaration of LocalizedLaserScan List
   */
  typedef List<LocalizedLaserScanPtr> LocalizedLaserScanList;
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Scan that is defined by points
   */
  class KARTO_EXPORT LocalizedPointScan : public LocalizedLaserScan
  {
    KARTO_RTTI();
   
  public:
    /**
     * Point scan from the given points in local coordinates
     * @param rSensorIdentifier identifier of sensor that generated this scan
     * @param rLocalPoints list of points in local coordinates
     */
    LocalizedPointScan(const Identifier& rSensorIdentifier, const Vector2dList& rLocalPoints);
    
    /**
     * Gets the (local) angles to the range readings of this scan
     * @return (local) angles to range readings of this scan
     */
    inline const DoubleList& GetLocalAngles() const
    {
      return m_LocalAngles;
    }    
    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~LocalizedPointScan();
    //@endcond
    
  private:
    /**
     * Computes filtered and unfiltered points
     */
    virtual void ComputePointReadings();

  private:
    LocalizedPointScan(const LocalizedPointScan&);
    const LocalizedPointScan& operator=(const LocalizedPointScan&);
    
  private:
    Vector2dList m_LocalPointReadings;
    DoubleList m_LocalAngles;
  }; // LocalizedPointScan
  
  /**
   * Register LocalizedPointScan with MetaClassManager
   */
  KARTO_TYPE(LocalizedPointScan);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * The LocalizedRangeScan contains range data from a single sweep of a laser range finder sensor
   * in a two-dimensional space and position information. The odometric position is the position 
   * reported by the robot when the range data was recorded. The corrected position is the position
   * calculated by the mapper (or some other module)
   *
   * The following example code creates a LocalizedRangeScan 
   * \code
   *   karto::LaserRangeFinder* pLrf = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Sick_LMS100, "/laser0");
   *   RangeReadingsList readings;
   *   karto::LocalizedRangeScan* pLocalizedRangeScan = new karto::LocalizedRangeScan(pLrf->GetName(), readings);
   *   pLocalizedRangeScan->SetTime(karto::Timestamp().GetUtcTime());
   * \endcode
   *
   * Setting the time is important if any filtering or interpolation is required for the scan
   * 
   * Example of how to to get a human readable time from the scan time.
   * \code
   *   std::cout << "Scan Date: " << karto::LocalDateTime(karto::DateTime(karto::Timestamp::FromUtcTime(pLocalizedRangeScan->GetTime()))).ToString().ToCString() << std::endl;
   * \endcode
   * The LocalDataTime automatically corrects for timezone.
   */
  class KARTO_EXPORT LocalizedRangeScan : public LocalizedLaserScan
  {
    KARTO_RTTI();

  public:
    /**
     * Range scan from the given range finder with the given readings
     * @param rSensorIdentifier identifier of sensor that generated this scan
     * @param rReadings list of range readings
     */
    LocalizedRangeScan(const Identifier& rSensorIdentifier, const RangeReadingsList& rReadings);

    /**
     * Range scan from the given range finder with the given readings
     * @param rSensorIdentifier identifier of sensor that generated this scan
     * @param rRangeReadings vector of range readings
     * @deprecated Please use LocalizedRangeScan(const Name& rSensorIdentifier, const RangeReadingsList& rReadings)
     */
    KARTO_DEPRECATED KARTO_FORCEINLINE LocalizedRangeScan(const Identifier& rSensorIdentifier, std::vector<kt_double>& rRangeReadings)
      : LocalizedLaserScan(rSensorIdentifier)
    {
      m_RangeReadings.Resize(static_cast<kt_int32u>(rRangeReadings.size()));
      if (rRangeReadings.size() > 0)
      {
        memcpy(&(m_RangeReadings[0]), &(rRangeReadings[0]), sizeof(kt_double) * rRangeReadings.size());
      }
    }
  
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~LocalizedRangeScan();
    //@endcond
    
  private:
    /**
     * Computes point readings based on range readings
     * Only range readings within [minimum range; range threshold] are returned
     */
    virtual void ComputePointReadings();
    
  private:
    LocalizedRangeScan(const LocalizedRangeScan&);
    const LocalizedRangeScan& operator=(const LocalizedRangeScan&);
  }; // LocalizedRangeScan

  /**
   * Register LocalizedRangeScan with MetaClassManager
   */
  KARTO_TYPE(LocalizedRangeScan);

  //@}

}

#endif // __OpenKarto_SensorData_h__
