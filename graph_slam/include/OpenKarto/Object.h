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

#ifndef __ObjectKarto_Object_h__
#define __ObjectKarto_Object_h__

#include <OpenKarto/Identifier.h>
#include <OpenKarto/Parameter.h>
#include <OpenKarto/List.h>
#include <OpenKarto/Meta.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Karto object type check
   */
  #define KARTO_TYPECHECKCAST(__Name__) \
    inline kt_bool Is##__Name__(Object* pObject) \
    { \
      return dynamic_cast<__Name__ *>(pObject) != NULL;\
    } 

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Base class for Karto objects
   */
  class KARTO_EXPORT Object : public Referenced
  {
    KARTO_RTTI();

  public:
    /**
     * Constructs an object with an empty identifier
     */
    Object();

    /**
     * Constructs an object with the given identifier
     * @param rIdentifier
     */
    Object(const Identifier& rIdentifier);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Object();
    //@endcond
    
  public:
    /**
     * Gets the identifier of this object
     * @return identifier of this object
     */
    inline const Identifier& GetIdentifier() const 
    {
      return m_Identifier; 
    }
    
    /**
     * Gets the named parameter
     * @param rParameterName name of parameter
     * @return parameter with given name
     */
    inline AbstractParameter* GetParameter(const String& rParameterName) const
    {
      return m_pParameterSet->GetParameter(rParameterName);
    }

    /**
     * Gets the named parameter
     * @param rParameterName name of parameter
     * @return parameter with given name
     */
    template<typename T>
    inline Parameter<T>* GetParameter(const String& rParameterName) const
    {
      return dynamic_cast<Parameter<T>*>(m_pParameterSet->GetParameter(rParameterName));
    }
   
    /**
     * Sets the parameter with the given name with the given value
     * @param rParameterName name
     * @param rValue new value
     */
    template<typename T>
    inline void SetParameters(const karto::String& rParameterName, const T& rValue)
    {
      AbstractParameter* pAbstractParameters = GetParameter(rParameterName);
      if (pAbstractParameters != NULL)
      {
        pAbstractParameters->SetValueFromString(StringHelper::ToString(rValue));
      }
      else
      {
        String errorMessage;
        errorMessage.Append("Parameter does not exist: ");
        errorMessage.Append(rParameterName);
        errorMessage.Append(String::NewLine());
        errorMessage.Append("Valid parameters are: ");
        errorMessage.Append(String::NewLine());
        const ParameterList& rParameters = m_pParameterSet->GetParameters();
        karto_const_forEach(ParameterList, &rParameters)
        {
          errorMessage.Append("\t" + (*iter)->GetName());
          errorMessage.Append(String::NewLine());
        }
        throw Exception(errorMessage);
      }
    }

    /**
     * Gets the set of parameters
     * @return set of parameters
     */
    inline ParameterSet* GetParameterSet()
    {
      return m_pParameterSet;
    }
    
    /**
     * Gets the parameters
     * @return list of parameters
     */
    inline ParameterList GetParameters()
    {
      return m_pParameterSet->GetParameters();
    }    
    
  private:
    // restrict the following functions
    Object(const Object&);
    const Object& operator=(const Object&);

  private:
    Identifier m_Identifier;

    ParameterSetPtr m_pParameterSet;
  }; // class Object

  /**
   * Register Object with MetaClassManager
   */
  KARTO_TYPE(Object);

  /**
   * Type declaration of Object managed by SmartPointer
   */
  typedef SmartPointer<Object> ObjectPtr;

  /**
   * Type declaration of Object List
   */
  typedef List<ObjectPtr> ObjectList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Karto object type 
   */
  const kt_objecttype ObjectType_None									= 0x00000000;

  /**
   * Karto sensor object type 
   */
  const kt_objecttype ObjectType_Sensor								= 0x00001000;

  /**
   * Karto sensor data object type 
   */
  const kt_objecttype ObjectType_SensorData 					= 0x00002000;

  /**
   * Karto custom item object type 
   */
  const kt_objecttype ObjectType_CustomItem 					= 0x00004000;

  /**
   * Karto grid object type 
   */
  const kt_objecttype ObjectType_Grid       					= 0x00008000;

  /**
   * Karto message object type 
   */
  const kt_objecttype ObjectType_Message     					= 0x00010000;

  /**
   * Karto object object type 
   */
  const kt_objecttype ObjectType_Object								= 0x00020000;

  /**
   * Karto misc object type 
   */
  const kt_objecttype ObjectType_Misc									= 0x10000000;

  /**
   * Karto drive object type 
   */
  const kt_objecttype ObjectType_Drive								= ObjectType_Sensor | 0x01;

  /**
   * Karto laser range finder object type 
   */
  const kt_objecttype ObjectType_LaserRangeFinder			= ObjectType_Sensor | 0x02;

  /**
   * Karto camera object type 
   */
  const kt_objecttype ObjectType_Camera								= ObjectType_Sensor | 0x04;

  /**
   * Karto drive pose object type 
   */
  const kt_objecttype ObjectType_DrivePose						= ObjectType_SensorData | 0x01;

  /**
   * Karto laser range scan object type 
   */
  const kt_objecttype ObjectType_LaserRangeScan		    = ObjectType_SensorData | 0x02;

  /**
   * Karto localized object object type 
   */
  const kt_objecttype ObjectType_LocalizedObject   		= ObjectType_SensorData | 0x04;

  /**
   * Karto localized range scan object type 
   */
  const kt_objecttype ObjectType_LocalizedRangeScan		= ObjectType_SensorData | 0x08;

  /**
   * Karto localized point scan object type 
   */
  const kt_objecttype ObjectType_LocalizedPointScan		= ObjectType_SensorData | 0x10;

  /**
   * Karto localized laser scan object type 
   */
  const kt_objecttype ObjectType_LocalizedLaserScan		= ObjectType_SensorData | 0x20;

  /**
   * Karto default custom item object type 
   */
  const kt_objecttype ObjectType_DefaultCustomItem    = ObjectType_CustomItem | 0x01;

  /**
   * Karto rfid object type 
   */
  const kt_objecttype ObjectType_Rfid                 = ObjectType_CustomItem | 0x02;

  /**
   * Karto camera image object type 
   */
  const kt_objecttype ObjectType_CameraImage      	  = ObjectType_SensorData | 0x40;

  /**
   * Karto occupancy grid object type 
   */
  const kt_objecttype ObjectType_OccupancyGrid    	  = ObjectType_Grid | 0x01;

  /**
   * Karto occupancy grid tile object type 
   */
  const kt_objecttype ObjectType_OccupancyGridTile 	  = ObjectType_Grid | 0x02;

  /**
   * Karto header object type 
   */
  const kt_objecttype ObjectType_Header								= ObjectType_Misc | 0x01;

  /**
   * Karto header object type 
   */
  const kt_objecttype ObjectType_Image								= ObjectType_Misc | 0x02;

  /**
   * Karto parameters object type 
   */
  const kt_objecttype ObjectType_ModuleParameters		  = ObjectType_Object | 0x01;

  /**
   * Karto dataset info object type 
   */
  const kt_objecttype ObjectType_DatasetInfo			    = ObjectType_Object | 0x02;

  /**
   * Karto module object type 
   */
  const kt_objecttype ObjectType_Module               = ObjectType_Object | 0x04;

  /**
   * Karto tiled occupancy grid object type 
   */
  const kt_objecttype ObjectType_TiledOccupancyGrid   = ObjectType_Object | 0x08;

  /**
   * Karto int32s message object type 
   */
  const kt_objecttype ObjectType_Int32sMessage        = ObjectType_Message | 0x01;

  /**
   * Karto int64s message object type 
   */
  const kt_objecttype ObjectType_Int64sMessage        = ObjectType_Message | 0x02;

  /**
   * Karto double message object type 
   */
  const kt_objecttype ObjectType_DoubleMessage        = ObjectType_Message | 0x04;

  /**
   * Karto string message object type 
   */
  const kt_objecttype ObjectType_StringMessage        = ObjectType_Message | 0x08;

  /**
   * Karto dataset object message object type 
   */
  const kt_objecttype ObjectType_DatasetObjectMessage = ObjectType_Message | 0x10;

  /**
   * Karto scans pose update message object type 
   */
  const kt_objecttype ObjectType_ScansPoseUpdateMessage = ObjectType_Message | 0x20;

  //@}

}


#endif // __ObjectKarto_Object_h__
