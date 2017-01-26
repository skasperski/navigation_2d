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

#ifndef __OpenKarto_Sensor_h__
#define __OpenKarto_Sensor_h__

#include <OpenKarto/List.h>
#include <OpenKarto/Object.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Type declaration of Vector2d List
   */
  typedef List<Vector2d> Vector2dList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SensorData;

  /**
   * Abstract sensor base class
   */
  class KARTO_EXPORT Sensor : public Object
  {
    KARTO_RTTI();

  protected:
    /**
     * Constructs a sensor with the given name
     * @param rName sensor name
     */
    Sensor(const Identifier& rName);

    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Sensor();
    //@endcond
    
  public:
    /**
     * Gets this sensor's offset
     * @return offset pose
     */
    inline const Pose2& GetOffsetPose() const
    {
      return m_pOffsetPose->GetValue();
    }
    
    /**
     * Sets this sensor's offset
     * @param rPose new offset pose
     */
    inline void SetOffsetPose(const Pose2& rPose)
    {
      m_pOffsetPose->SetValue(rPose);
    }

    /**
     * Validates this sensor
     */
    virtual void Validate() = 0;

    /**
     * Validates sensor data
     * @param pSensorData sensor data
     */
    virtual void Validate(SensorData* pSensorData) = 0;

  private:
    // restrict the following functions
    Sensor(const Sensor&);
    const Sensor& operator=(const Sensor&);

  private:
    /**
     * Sensor offset pose
     */
    Parameter<Pose2>* m_pOffsetPose;
  }; // Sensor

  /**
   * Register Sensor with MetaClassManager
   */
  KARTO_TYPE(Sensor);

  /**
   * Type declaration of Sensor managed by SmartPointer
   */
  typedef SmartPointer<Sensor> SensorPtr;
  
  /**
   * Type declaration of Sensor List
   */
  typedef List<SensorPtr> SensorList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Sensor that provides pose information relative to world coordinates.
   *
   * The user can set the offset pose of the drive sensor.  If no value is provided by the user, the default is no offset,
   * i.e, the sensor is initially at the world origin, oriented along the positive x axis.
   */
  class Drive : public Sensor
  {
    KARTO_RTTI();

  public:
    /**
     * Drive object with the given name
     * @param rName name
     */
    Drive(const Identifier& rName)
      : Sensor(rName)
    {
    }

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Drive()
    {
    }
    //@endcond

  public:
    virtual void Validate()
    {
    }

    /**
     * Sensor data is valid if it is not NULL
     */
    virtual void Validate(SensorData* pSensorData)
    {
      if (pSensorData == NULL)
      {
        throw Exception("SensorData == NULL");
      }
    }

  private:
    // restrict the following functions
    Drive(const Drive&);
    const Drive& operator=(const Drive&);
  }; // class Drive

  /**
   * Register Drive with MetaClassManager
   */
  KARTO_TYPE(Drive);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Enumerated type for valid LaserRangeFinder types
   * @cond EXCLUDE
   * If more laser range finder types are added, make sure
   * to add them to the description of LaserRangeFinder below.
   * @endcond
   */
  typedef enum
  {
    LaserRangeFinder_Custom = 0,

    LaserRangeFinder_Sick_LMS100 = 1,
    LaserRangeFinder_Sick_LMS200 = 2,
    LaserRangeFinder_Sick_LMS291 = 3,

    LaserRangeFinder_Hokuyo_UTM_30LX = 4,
    LaserRangeFinder_Hokuyo_URG_04LX = 5    
  } LaserRangeFinderType;

  /**
   * LaserRangeFinderType auto register callback function 
   */
  KARTO_EXPORT void RegisterLaserRangeFinderType();

  /**
   * Auto register LaserRangeFinderType with MetaEnumManager
   */
  KARTO_AUTO_TYPE(LaserRangeFinderType, &RegisterLaserRangeFinderType);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class LocalizedLaserScan;
  class CoordinateConverter;
  class LaserRangeFinder;

  /**
   * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized laser scan relative to the robot.
   * The user can set an offset pose for the sensor relative to the robot coordinate system. If no value is provided
   * by the user, the sensor is set to be at the origin of the robot coordinate system.
   * The LaserRangeFinder contains parameters for physical laser sensor used by the mapper for scan matching.
   * It also contains information about the maximum range of the sensor and provides a threshold
   * for limiting the range of readings.
   * The optimal value for the range threshold depends on the angular resolution of the scan and
   * the desired map resolution.  RangeThreshold should be set as large as possible while still
   * providing "solid" coverage between consecutive range readings.
   *
   * The following example code creates a LaserRangeFinder with default values for a Sick LMS100 laser
   * \code
   *   karto::LaserRangeFinder* pLrf = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Sick_LMS100, "/laser0");
   * \endcode
   *
   * The following laser types are supported in CreateLaserRangeFinder
   *
   *  LaserRangeFinder_Sick_LMS100 
   *  LaserRangeFinder_Sick_LMS200
   *  LaserRangeFinder_Sick_LMS291
   *  LaserRangeFinder_Hokuyo_UTM_30LX
   *  LaserRangeFinder_Hokuyo_URG_04LX   
   */
  class KARTO_EXPORT LaserRangeFinder : public Sensor
  {
    KARTO_RTTI();

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~LaserRangeFinder();
    //@endcond
    
  public:   
    /**
     * Gets this range finder sensor's minimum range
     * @return minimum range
     */
    inline kt_double GetMinimumRange() const
    {
      return m_pMinimumRange->GetValue();
    }

    /**
     * Sets this range finder sensor's minimum range
     * @param minimumRange new minimum range
     */
    inline void SetMinimumRange(kt_double minimumRange)
    {
      m_pMinimumRange->SetValue(minimumRange);
      
      SetRangeThreshold(GetRangeThreshold());
    }

    /**
     * Gets this range finder sensor's maximum range
     * @return maximum range
     */
    inline kt_double GetMaximumRange() const
    {
      return m_pMaximumRange->GetValue();
    }

    /**
     * Sets this range finder sensor's maximum range
     * @param maximumRange new maximum range
     */
    inline void SetMaximumRange(kt_double maximumRange)
    {
      m_pMaximumRange->SetValue(maximumRange);

      SetRangeThreshold(GetRangeThreshold());
    }

    /**
     * Gets the range threshold
     * @return range threshold
     */
    inline kt_double GetRangeThreshold() const
    {
      return m_pRangeThreshold->GetValue();
    }

    /**
     * Sets the range threshold
     * @param rangeThreshold new range threshold
     */
    void SetRangeThreshold(kt_double rangeThreshold);

    /**
     * Gets this range finder sensor's minimum angle
     * @return minimum angle
     */
    inline kt_double GetMinimumAngle() const
    {
      return m_pMinimumAngle->GetValue();
    }
    
    /**
     * Sets this range finder sensor's minimum angle
     * @param minimumAngle new minimum angle
     */
    inline void SetMinimumAngle(kt_double minimumAngle)
    {
      m_pMinimumAngle->SetValue(minimumAngle);

      Update();
    }

    /**
     * Gets this range finder sensor's maximum angle
     * @return maximum angle
     */
    inline kt_double GetMaximumAngle() const
    {
      return m_pMaximumAngle->GetValue();
    }
    
    /**
     * Sets this range finder sensor's maximum angle
     * @param maximumAngle new maximum angle
     */
    inline void SetMaximumAngle(kt_double maximumAngle)
    {
      m_pMaximumAngle->SetValue(maximumAngle);

      Update();
    }
    
    /**
     * Gets this range finder sensor's angular resolution
     * @return angular resolution
     */
    inline kt_double GetAngularResolution() const
    {
      return m_pAngularResolution->GetValue();
    }
    
    /**
     * Sets this range finder sensor's angular resolution
     * @param angularResolution new angular resolution
     */
    void SetAngularResolution(kt_double angularResolution);

    /**
     * Gets this range finder sensor's laser type
     * @return laser type of this range finder
     */
    inline kt_int64s GetType()
    {
      return m_pType->GetValue();
    }

    /**
     * Gets the number of range readings each localized range scan must contain to be a valid scan.
     * @return number of range readings
     */
    inline kt_int32u GetNumberOfRangeReadings() const
    {
      return m_NumberOfRangeReadings;
    }

    /**
     * Validates this sensor
     */
    virtual void Validate();

    /**
     * Validates sensor data
     * @param pSensorData sensor data
     */    
    virtual void Validate(SensorData* pSensorData);

    /**
     * Gets point readings (potentially scaling readings if given coordinate converter is not null)
     * @param pLocalizedLaserScan scan
     * @param pCoordinateConverter coordinate converter
     * @param ignoreThresholdPoints whether to ignore points that exceed the range threshold
     * @param flipY whether to flip the y-coordinate (useful for drawing applications with inverted y-coordinates)
     * @return list of points from the given scan
     */
    const Vector2dList GetPointReadings(LocalizedLaserScan* pLocalizedLaserScan, CoordinateConverter* pCoordinateConverter, kt_bool ignoreThresholdPoints = true, kt_bool flipY = false) const;

  public:
    /**
     * Creates a laser range finder of the given type and name
     * @param type laser type
     * @param rName name of sensor
     * @return laser range finder
     */
    static LaserRangeFinder* CreateLaserRangeFinder(LaserRangeFinderType type, const Identifier& rName);

  private:
    /**
     * Laser range finder with given name
     * @param rName name
     */
    LaserRangeFinder(const Identifier& rName);
    
    /**
     * Sets the number of range readings based on the minimum and maximum angles of the sensor and the angular resolution
     */
    void Update()
    {
      m_NumberOfRangeReadings = static_cast<kt_int32u>(math::Round((GetMaximumAngle() - GetMinimumAngle()) / GetAngularResolution()));
    }

  private:
    LaserRangeFinder(const LaserRangeFinder&);
    const LaserRangeFinder& operator=(const LaserRangeFinder&);

  private:
    // sensor m_Parameters
    Parameter<kt_double>* m_pMinimumAngle;
    Parameter<kt_double>* m_pMaximumAngle;

    Parameter<kt_double>* m_pAngularResolution;

    Parameter<kt_double>* m_pMinimumRange;
    Parameter<kt_double>* m_pMaximumRange;

    Parameter<kt_double>* m_pRangeThreshold;

    ParameterEnum* m_pType;

    kt_int32u m_NumberOfRangeReadings;    
  }; // LaserRangeFinder

  /**
   * Register LaserRangeFinder with MetaClassManager
   */
  KARTO_TYPE(LaserRangeFinder);

  /**
   * Type declaration of LaserRangeFinder managed by SmartPointer
   */
  typedef SmartPointer<LaserRangeFinder> LaserRangeFinderPtr;
  
  //@}

}

#endif // __OpenKarto_Sensor_h__
