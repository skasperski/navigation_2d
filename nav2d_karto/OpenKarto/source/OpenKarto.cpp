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

#include <OpenKarto/OpenKarto.h>

namespace karto
{
  
  void InitializeOpenKartoMetaClasses()
  {
    MetaClass::Register<AbstractParameter>("AbstractParameter");

    MetaClass::Register<Parameter<kt_bool> >("ParameterBool").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_char> >("ParameterChar").Base<AbstractParameter>();

    MetaClass::Register<Parameter<kt_int8s> >("ParameterInt8s").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int8u> >("ParameterInt8u").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int16s> >("ParameterInt16s").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int16u> >("ParameterInt16u").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int32s> >("ParameterInt32s").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int32u> >("ParameterInt32u").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int64s> >("ParameterInt64s").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_int64u> >("ParameterInt64u").Base<AbstractParameter>();

    MetaClass::Register<Parameter<kt_float> >("ParameterFloat").Base<AbstractParameter>();
    MetaClass::Register<Parameter<kt_double> >("ParameterDouble").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::String> >("ParameterString").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Size2<kt_int32s> > >("ParameterSize2Int32s").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Size2<kt_int32u> > >("ParameterSize2Int32u").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Size2<kt_double> > >("ParameterSize2Double").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Vector2i> >("ParameterVector2i").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector3i> >("ParameterVector3i").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector4i> >("ParameterVector4i").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Vector2iu> >("ParameterVector2iu").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector3iu> >("ParameterVector3iu").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector4iu> >("ParameterVector4iu").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Vector2d> >("ParameterVector2d").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector3d> >("ParameterVector3d").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Vector4d> >("ParameterVector4d").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Quaternion> >("ParameterQuaternion").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Color> >("ParameterColor").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::Pose2> >("ParameterPose2").Base<AbstractParameter>();
    MetaClass::Register<Parameter<karto::Pose3> >("ParameterPose3").Base<AbstractParameter>();

    MetaClass::Register<Parameter<karto::gps::PointGps> >("ParameterPointGps").Base<AbstractParameter>();

    MetaClass::Register<ParameterEnum>("ParameterEnum").Base<Parameter<kt_int64s> >();


    MetaClass::Register<Object>("Object")
      .Base<Parameter<kt_int32s> >();

    // Objects

    MetaClass::Register<ModuleParameters>("ModuleParameters")
      .Base<Object>()
      .Attribute("ObjectType", ObjectType_ModuleParameters);

    MetaClass::Register<CustomItem>("CustomItem")
      .Base<Object>();

    MetaClass::Register<DatasetInfo>("DatasetInfo")
      .Base<Object>()
      .Attribute("ObjectType", ObjectType_DatasetInfo);

    // Sensors

    MetaClass::Register<Sensor>("Sensor")
      .Base<Object>()
      .Attribute("ObjectType", ObjectType_Sensor);

    MetaClass::Register<Drive>("Drive")
      .Base<Sensor>()
      .Attribute("ObjectType", ObjectType_Drive);

    MetaClass::Register<LaserRangeFinder>("LaserRangeFinder")
      .Base<Sensor>()
      .Attribute("ObjectType", ObjectType_LaserRangeFinder)
      .Parameter("MinimumAngle", &LaserRangeFinder::GetMinimumAngle, &LaserRangeFinder::SetMinimumAngle)
      .Parameter("MaximumAngle", &LaserRangeFinder::GetMaximumAngle, &LaserRangeFinder::SetMaximumAngle)
      .Parameter("AngularResolution", &LaserRangeFinder::GetAngularResolution, &LaserRangeFinder::SetAngularResolution)
      .Parameter("MinimumRange", &LaserRangeFinder::GetMinimumRange, &LaserRangeFinder::SetMinimumRange)
      .Parameter("MaximumRange", &LaserRangeFinder::GetMaximumRange, &LaserRangeFinder::SetMaximumRange)
      .Parameter("RangeThreshold", &LaserRangeFinder::GetRangeThreshold, &LaserRangeFinder::SetRangeThreshold);
//      .Parameter("Type", &LaserRangeFinder::GetType, &LaserRangeFinder::SetType);

    // SensorData
    MetaClass::Register<SensorData>("SensorData")
      .Base<Object>()
      .Attribute("ObjectType", ObjectType_SensorData);

    MetaClass::Register<LaserRangeScan>("LaserRangeScan")
      .Base<SensorData>()
      .Attribute("ObjectType", ObjectType_LaserRangeScan);

    MetaClass::Register<DrivePose>("DrivePose")
      .Base<SensorData>()
      .Attribute("ObjectType", ObjectType_DrivePose);

    MetaClass::Register<LocalizedObject>("LocalizedObject")
      .Base<SensorData>()
      .Attribute("ObjectType", ObjectType_LocalizedObject);

    MetaClass::Register<LocalizedLaserScan>("LocalizedLaserScan")
      .Base<LocalizedObject>()
      .Attribute("ObjectType", ObjectType_LocalizedLaserScan);

    MetaClass::Register<LocalizedPointScan>("LocalizedPointScan")
      .Base<LocalizedLaserScan>()
      .Attribute("ObjectType", ObjectType_LocalizedPointScan);

    MetaClass::Register<LocalizedRangeScan>("LocalizedRangeScan")
      .Base<LocalizedLaserScan>()
      .Attribute("ObjectType", ObjectType_LocalizedRangeScan);

    // Grid
    MetaClass::Register<Grid<kt_int8u> >("GridInt8u")
      .Base<Object>();

    MetaClass::Register<Grid<kt_int32u> >("GridInt32u")
      .Base<Object>();

    MetaClass::Register<Grid<kt_float> >("GridFloat")
      .Base<Object>();

    MetaClass::Register<Grid<kt_double> >("GridDouble")
      .Base<Object>();

    MetaClass::Register<OccupancyGrid>("OccupancyGrid")
      .Base<Grid<kt_int8u> >()
      .Attribute("ObjectType", ObjectType_OccupancyGrid);
  }

}
