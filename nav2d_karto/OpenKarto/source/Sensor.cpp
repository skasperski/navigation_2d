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

#include <iostream>

#include <OpenKarto/StringHelper.h>
#include <OpenKarto/CoordinateConverter.h>
#include <OpenKarto/Sensor.h>
#include <OpenKarto/SensorData.h>
#include <OpenKarto/SensorRegistry.h>
#include <OpenKarto/Logger.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void RegisterLaserRangeFinderType()
  {
    MetaEnum::Register<LaserRangeFinderType>("LaserRangeFinderType")
      .Value("Custom", LaserRangeFinder_Custom)
      .Value("Sick_LMS100", LaserRangeFinder_Sick_LMS100)
      .Value("Sick_LMS200", LaserRangeFinder_Sick_LMS200)
      .Value("Sick_LMS291", LaserRangeFinder_Sick_LMS291)
      .Value("Hokuyo_UTM_30LX", LaserRangeFinder_Hokuyo_UTM_30LX)
      .Value("Hokuyo_URG_04LX", LaserRangeFinder_Hokuyo_URG_04LX);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  Sensor::Sensor(const Identifier& rIdentifier)
    : Object(rIdentifier)
  {
    m_pOffsetPose = new Parameter< karto::Pose2 >(GetParameterSet(), "OffsetPose", "Offset", "", karto::Pose2());

    SensorRegistry::GetInstance()->RegisterSensor(this);
  }

  Sensor::~Sensor()
  {
    SensorRegistry::GetInstance()->UnregisterSensor(this);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  LaserRangeFinder::LaserRangeFinder(const Identifier& rName)
    : Sensor(rName)
    , m_NumberOfRangeReadings(0)
  {
    m_pMinimumRange = new Parameter<kt_double>(GetParameterSet(), "MinimumRange", "Minimum Range", "", 0.0);
    m_pMaximumRange = new Parameter<kt_double>(GetParameterSet(), "MaximumRange", "Maximum Range", "", 80.0);

    m_pMinimumAngle = new Parameter<kt_double>(GetParameterSet(), "MinimumAngle", "Minimum Angle", "", -KT_PI_2);
    m_pMaximumAngle = new Parameter<kt_double>(GetParameterSet(), "MaximumAngle", "Minimum Angle", "", KT_PI_2);

    m_pAngularResolution = new Parameter<kt_double>(GetParameterSet(), "AngularResolution", "Angular Resolution", "", math::DegreesToRadians(1));

    m_pRangeThreshold = new Parameter<kt_double>(GetParameterSet(), "RangeThreshold", "Range Threshold", "", 12.0);

    m_pType = new ParameterEnum(GetParameterSet(), "Type", "Type", "", (kt_int64s)karto::LaserRangeFinder_Custom);
    const MetaEnum& rMetaEnum = karto::GetMetaEnumByType<karto::LaserRangeFinderType>();
    for (kt_size_t i = 0; i < rMetaEnum.GetSize(); i++)
    {
      const EnumPair& rEnumPair = rMetaEnum.GetPair(i);
      m_pType->DefineEnumValue(rEnumPair.name, rEnumPair.value);
    }
  }

  LaserRangeFinder::~LaserRangeFinder()
  {
  }

  const Vector2dList LaserRangeFinder::GetPointReadings(LocalizedLaserScan* pLocalizedLaserScan, CoordinateConverter* pCoordinateConverter, kt_bool ignoreThresholdPoints, kt_bool flipY) const
  {
    Vector2dList pointReadings;

    Vector2d scanPosition = pLocalizedLaserScan->GetSensorPose().GetPosition();

    // compute point readings
    const Vector2dList& rPoints = pLocalizedLaserScan->GetPointReadings(ignoreThresholdPoints);
    for (kt_int32u i = 0; i < rPoints.Size(); i++)
    {
      Vector2d point = rPoints[i];

      kt_double range = scanPosition.Distance(point);
      kt_double clippedRange = math::Clip(range, GetMinimumRange(), GetRangeThreshold());
      if (karto::math::DoubleEqual(range, clippedRange) == false)
      {
        kt_double ratio = clippedRange / range;
        point.SetX(scanPosition.GetX() + ratio * (point.GetX() - scanPosition.GetX()));
        point.SetY(scanPosition.GetY() + ratio * (point.GetY() - scanPosition.GetY()));
      }

      if (pCoordinateConverter != NULL)
      {
        Vector2i gridPoint = pCoordinateConverter->WorldToGrid(point, flipY);
        point.SetX(gridPoint.GetX());
        point.SetY(gridPoint.GetY());
      }

      pointReadings.Add(point);
    }

    return pointReadings;
  }

  void LaserRangeFinder::SetRangeThreshold(kt_double rangeThreshold)
  {
    // make sure rangeThreshold is within laser range finder range
    m_pRangeThreshold->SetValue(math::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange()));

    if (math::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false)
    {
      Log(LOG_INFORMATION, "Info: clipped range threshold to be within minimum and maximum range!");
    }
  }

  void LaserRangeFinder::SetAngularResolution(kt_double angularResolution)
  {
    if (m_pType->GetValue() == LaserRangeFinder_Custom)
    {
      m_pAngularResolution->SetValue(angularResolution);
    }
    else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS100)
    {
      if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.25)))
      {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));
      }
      else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.50)))
      {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.50));
      }
      else
      {
        String errorMessage;
        errorMessage.Append("Invalid resolution for Sick LMS100: ");
        errorMessage.Append(karto::StringHelper::ToString(angularResolution));
        throw Exception(errorMessage);
      }
    }
    else if (m_pType->GetValue() == LaserRangeFinder_Sick_LMS200 || m_pType->GetValue() == LaserRangeFinder_Sick_LMS291)
    {
      if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.25)))
      {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.25));
      }
      else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(0.50)))
      {
        m_pAngularResolution->SetValue(math::DegreesToRadians(0.50));
      }
      else if (math::DoubleEqual(angularResolution, math::DegreesToRadians(1.00)))
      {
        m_pAngularResolution->SetValue(math::DegreesToRadians(1.00));
      }
      else
      {
        String errorMessage;
        errorMessage.Append("Invalid resolution for Sick LMS291: ");
        errorMessage.Append(karto::StringHelper::ToString(angularResolution));
        throw Exception(errorMessage);
      }
    }
    else
    {
      throw Exception("Can't set angular resolution, please create a LaserRangeFinder of type Custom");
    }

    Update();
  }

  void LaserRangeFinder::Validate()
  {
    Update();

    // check if min < max range!
    if (GetMinimumRange() >= GetMaximumRange())
    {
      assert(false);
      throw Exception("LaserRangeFinder::Validate() - MinimumRange must be less than MaximumRange.  Please set MinimumRange and MaximumRange to valid values.");
    }

    // set range threshold to valid value
    if (math::InRange(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange()) == false)
    {
      kt_double newValue = karto::math::Clip(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange());
      Log(LOG_INFORMATION, String("Updating RangeThreshold from ") + StringHelper::ToString(GetRangeThreshold()) + " to " + StringHelper::ToString(newValue));
      SetRangeThreshold(newValue);
    }
  }

  void LaserRangeFinder::Validate(SensorData* pSensorData)
  {
    LocalizedRangeScan* pScan = dynamic_cast<LocalizedRangeScan*>(pSensorData);

    // verify number of range readings in laser scan matches the number of expected range
    // readings; validation only valid with LocalizedRangeScan (LocalizedPointScan may have
    // variable number of readings)
    if (pScan != NULL && (pScan->GetNumberOfRangeReadings() != GetNumberOfRangeReadings()))
    {
      StringBuilder errorMessage;
      errorMessage << "LaserRangeFinder::Validate() - LocalizedRangeScan contains " << pScan->GetNumberOfRangeReadings() << " range readings, expected " << GetNumberOfRangeReadings();

      //      assert(false);
      throw Exception(errorMessage.ToString());
    }
  }

  LaserRangeFinder* LaserRangeFinder::CreateLaserRangeFinder(LaserRangeFinderType type, const Identifier& rName)
  {
    LaserRangeFinder* pLrf = NULL;

    switch(type)
    {
      // see http://www.hizook.com/files/publications/SICK_LMS100.pdf
      // set range threshold to 18m
    case LaserRangeFinder_Sick_LMS100:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "Sick LMS 100");

        // Sensing range is 18 meters (at 10% reflectivity, max range of 20 meters), with an error of about 20mm
        pLrf->m_pMinimumRange->SetValue(0.0);
        pLrf->m_pMaximumRange->SetValue(20.0);

        // 270 degree range, 50 Hz 
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-135)); 
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(135)); 

        // 0.25 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.25)); 

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 10m
    case LaserRangeFinder_Sick_LMS200:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "Sick LMS 200");

        // Sensing range is 80 meters
        pLrf->m_pMinimumRange->SetValue(0.0);
        pLrf->m_pMaximumRange->SetValue(80.0);

        // 180 degree range, 75 Hz 
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90)); 
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90)); 

        // 0.5 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.5)); 

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 30m
    case LaserRangeFinder_Sick_LMS291:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "Sick LMS 291");

        // Sensing range is 80 meters
        pLrf->m_pMinimumRange->SetValue(0.0);
        pLrf->m_pMaximumRange->SetValue(80.0);

        // 180 degree range, 75 Hz 
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90)); 
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90)); 

        // 0.5 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.5)); 

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/Hokuyo_UTM_LaserRangeFinder_LIDAR.pdf
      // set range threshold to 30m
    case LaserRangeFinder_Hokuyo_UTM_30LX:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "Hokuyo UTM-30LX");

        // Sensing range is 30 meters
        pLrf->m_pMinimumRange->SetValue(0.1);
        pLrf->m_pMaximumRange->SetValue(30.0);

        // 270 degree range, 40 Hz 
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-135));
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(135)); 

        // 0.25 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.25)); 

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/HokuyoURG_Datasheet.pdf
      // set range threshold to 3.5m
    case LaserRangeFinder_Hokuyo_URG_04LX:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "Hokuyo URG-04LX");

        // Sensing range is 4 meters. It has detection problems when scanning absorptive surfaces (such as black trimming). 
        pLrf->m_pMinimumRange->SetValue(0.02);
        pLrf->m_pMaximumRange->SetValue(4.0);

        // 240 degree range, 10 Hz 
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-120)); 
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(120));

        // 0.352 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(0.352)); 

        pLrf->m_NumberOfRangeReadings = 751;
      }
      break;

    case LaserRangeFinder_Custom:
      {
        pLrf = new LaserRangeFinder((rName.ToString() != "") ? rName : "User-Defined LaserRangeFinder");

        // Sensing range is 80 meters.
        pLrf->m_pMinimumRange->SetValue(0.0);
        pLrf->m_pMaximumRange->SetValue(80.0);

        // 180 degree range
        pLrf->m_pMinimumAngle->SetValue(math::DegreesToRadians(-90)); 
        pLrf->m_pMaximumAngle->SetValue(math::DegreesToRadians(90));

        // 1.0 degree angular resolution
        pLrf->m_pAngularResolution->SetValue(math::DegreesToRadians(1.0));

        pLrf->m_NumberOfRangeReadings = 181;
      }
      break;
    }

    if (pLrf != NULL)
    {
      pLrf->m_pType->SetValue(type);

      Pose2 defaultOffset;
      pLrf->SetOffsetPose(defaultOffset);
    }

    return pLrf;
  }

}

