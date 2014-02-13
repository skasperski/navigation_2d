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

#ifndef __OpenKarto_Parameter_h__
#define __OpenKarto_Parameter_h__

#include <OpenKarto/String.h>
#include <OpenKarto/Geometry.h>
#include <OpenKarto/Event.h>
#include <OpenKarto/SmartPointer.h>
#include <OpenKarto/Meta.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Parameter flags used by the KartoViewer application to control the visibility
   * and read/write access
   */
  enum ParameterFlags
  {
    /**
     * Parameter readable
     */
    ParameterFlag_Read = 0x01,
    /**
     * Parameter writeable
     */
    ParameterFlag_Write = 0x02,
    /**
     * Parameter hidden
     */
    ParameterFlag_Hidden = 0x08,
    /**
     * System parameter
     */
    ParameterFlag_System = 0x10
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Parameter description contains metadata information for parameters like name, description, display name and so on.
   * This class is primarily used by the KartoViwer application to display parameter information to the user
   */ 
  class KARTO_EXPORT ParameterDescription : public Referenced
  {
  public:
    /**
     * Constructs a description of a parameter with the given name, display string, and description
     * @param rName name of the parameter
     * @param rDisplayName how the parameter name is displayed
     * @param rDescription description of the parameter
     */
    ParameterDescription(const karto::String& rName, const karto::String& rDisplayName = "", const karto::String& rDescription = "");
    virtual ~ParameterDescription();

  public:
    /**
     * Gets the name of the parameter
     * @return name of the parameter
     */
    inline const karto::String& GetName() const
    {
      return m_Name;
    }

    /**
     * Gets the display name of the parameter
     * @return display name of the parameter
     */
    inline const karto::String& GetDisplayName() const
    {
      return m_DisplayName;
    }

    /**
     * Gets a description of the parameter
     * @return description of the parameter
     */
    inline const karto::String& GetDescription() const
    {
      return m_Description;
    }

    /**
     * Gets the flags of the parameter
     * @return flags of the parameter
     */
    inline kt_int32s GetFlags() const
    {
      return m_Flags;
    }

    /**
     * Sets the flags of the parameter
     * @param flags flags of the parameter
     */
    inline void SetFlags(kt_int32s flags)
    {
      m_Flags = flags;
    }

    /**
     * Gets the field name at the given index. Note: ParameterDescription only supports up to 4 field names.
     * @param index index
     * @return name of field
     */
    const karto::String& GetFieldName(kt_int32u index) const
    {
      if (index >= 4)
      {
        assert(index < 4);
        throw karto::Exception("ParameterDescription::GetFieldName() - Invalid argument, index must be [0;3]");
      }

      return m_FieldNames[index]; 
    }

    /**
     * Sets the field names
     * @param rX name of the X field
     * @param rY name of the Y field
     * @param rZ name of the Z field
     * @param rW name of the W field
     */
    void SetFieldNames(const karto::String& rX = "X", const karto::String& rY = "Y", const karto::String& rZ = "Z", const karto::String& rW = "W") 
    { 
      m_FieldNames[0] = rX; 
      m_FieldNames[1] = rY; 
      m_FieldNames[2] = rZ; 
      m_FieldNames[3] = rW; 
    }

    /**
     * Gets the number of decimal places
     * @return number of decimal places
     */
    kt_int32s GetNumberOfDecimalPlaces() const
    {
      return m_nDecimalPlaces;
    }

    /**
     * Sets the number of decimal places
     * @param decimalPlaces new number of decimal places
     */
    void SetNumberOfDecimalPlaces(kt_int32s decimalPlaces)
    {
      m_nDecimalPlaces = decimalPlaces;
    }

  private:
    /** 
     * Parameter name
     */
    karto::String m_Name;
    
    /**
     * Parameter display name, used in UI as "pretty" name for parameter
     */
    karto::String m_DisplayName;

    /**
     * Parameter description
     */
    karto::String m_Description;

    /**
     * Parameter field names, currently only up to 4 field names are supported. Field names can be used to
     * change the displayed default field names like (x, y, z) to (myX, myY, myZ)
     */
    karto::String m_FieldNames[4];

    /**
     * Parameter flags
     */
    kt_int32s m_Flags;

    /**
     * Number of decimal places displayed in the UI
     */
    kt_int32s m_nDecimalPlaces;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ParameterSet;

  /** 
   * Abstract base class for parameters
   */ 
  class KARTO_EXPORT AbstractParameter : public Referenced
  {
    KARTO_RTTI();

  public:
    /**
     * Constructs an AbstractParameter with the given ParameterDescription and ParameterSet
     * @param pDescription description of parameter
     * @param pParameterSet set of parameters
     */
    AbstractParameter(ParameterDescription* pDescription, ParameterSet* pParameterSet = NULL);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~AbstractParameter();
    //@endcond

  public:
    /**
     * Parameter event for changed notification
     */
    BasicEvent<EventArguments> Changed;

  public:
    /**
     * Gets the name of this parameter
     * @return name
     */
    inline const karto::String& GetName() const
    {
      return m_pDescription->GetName();
    }

    /**
     * Gets the display name of this parameter
     * @return display name
     */
    inline const karto::String& GetDisplayName() const
    {
      return m_pDescription->GetDisplayName();
    }

    /**
     * Gets the description of this parameter
     * @return description
     */
    inline const karto::String& GetDescription() const
    {
      return m_pDescription->GetDescription();
    }

    /**
     * Gets the flags of this parameter
     * @return flags
     */
    inline kt_int32s GetFlags() const
    {
      return m_pDescription->GetFlags();
    }

    /**
     * Gets the parameter description for this parameter (const version)
     * @return parameter description
     */
    inline const ParameterDescription* GetParameterDescription() const 
    {
      return m_pDescription;
    }

    /**
     * Gets the parameter description for this parameter
     * @return parameter description
     */
    inline ParameterDescription* GetParameterDescription() 
    {
      return m_pDescription;
    }

    /**
     * Gets the parameter value as string.
     * @return value as string
     */
    virtual const karto::String GetValueAsString() const = 0;

    /**
     * Sets the parameter value from string.
     * @param rStringValue value as string
     */
    virtual void SetValueFromString(const karto::String& rStringValue) = 0;

    /**
     * Sets the parameter to its default value
     */
    virtual void SetToDefaultValue() = 0;

  protected:
    /**
     * Initialize parameters
     */
    virtual void InitializeParameters();

  private:
    // restrict the following functions
    AbstractParameter(const AbstractParameter&);
    const AbstractParameter& operator=(const AbstractParameter&);

  private:
    karto::SmartPointer<ParameterDescription> m_pDescription;
    ParameterSet* m_pParameterSet;
  };

  /**
   * Register AbstractParameter with MetaClassManager
   */
  KARTO_TYPE(AbstractParameter);

  /**
   * Type declaration of AbstractParameter List
   */
  typedef List<SmartPointer<AbstractParameter> > ParameterList;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ParameterSetPrivate;

  /**
   * Parameter container. 
   */
  class KARTO_EXPORT ParameterSet : public Referenced
  {
  public:
    /**
     * Default constructor
     */
    ParameterSet();

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~ParameterSet();
    //@endcond

  public:
    /**
     * Adds the parameter to this set. Make sure parameter name is unique
     * @param pParameter parameter to add
     * @throws Exception if parameter with name already exists
     */
    void AddParameter(AbstractParameter* pParameter);

    /**
     * Removes the parameter from this set
     * @param pParameter parameter to remove
     */
    void RemoveParameter(AbstractParameter* pParameter);

    /**
     * Gets the parameter with given name - const version
     * @param rParameterName name of parameter
     * @return parameter of given name
     */
    AbstractParameter* GetParameter(const karto::String& rParameterName) const;

    /**
     * Gets the parameter with given name
     * @param rParameterName name of parameter
     * @return parameter of given name
     */
    AbstractParameter* GetParameter(const karto::String& rParameterName);

    /**
     * Removes all parameters
     */
    void Clear();

    /**
     * Gets all parameters - const version
     * @return ParameterList
     */
    const ParameterList& GetParameters() const;

    /**
     * Gets all parameters
     * @return ParameterList
     */
    ParameterList& GetParameters();

  private:
    // restrict the following functions
    ParameterSet(const ParameterSet&);
    const ParameterSet& operator=(const ParameterSet&);

  private:
    ParameterSetPrivate* m_pPrivate;
  }; // class ParameterSet

  /**
   * Type declaration of ParameterSet managed by SmartPointer
   */
  typedef SmartPointer<ParameterSet> ParameterSetPtr;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Parameter
   */
  template<typename T>
  class Parameter : public AbstractParameter
  {
    KARTO_RTTI();

  public:
    /**
     * Parameter with the given description and given value
     * @param pDescription description of the parameter
     * @param rValue parameter value
     */
    Parameter(ParameterDescription* pDescription, const T& rValue)
      : AbstractParameter(pDescription)
      , m_Value(rValue)
    {
      InitializeParameters();
    }

    /**
     * Parameter for the set of parameters with the given name, display name, description and value
     * @param pParameterSet set of parameters
     * @param rName name of the parameter
     * @param rDisplayName display name of the parameter
     * @param rDescription description of the parameter
     * @param rValue parameter value
     */
    Parameter(ParameterSet* pParameterSet, const karto::String& rName, const karto::String& rDisplayName, const karto::String& rDescription, const T& rValue)
      : AbstractParameter(new ParameterDescription(rName, rDisplayName, rDescription), pParameterSet)
      , m_Value(rValue)
    {
      InitializeParameters();
    }

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Parameter()
    {
    }
    //@endcond

  public:
    /**
     * Gets the value of parameter
     * @return parameter value
     */
    virtual const T& GetValue() const
    {
      return m_Value;
    }

    /**
     * Sets the value of parameter
     * @param rValue value to set parameter to
     */
    virtual void SetValue(const T& rValue)
    {
      kt_bool changed = !CompareValue(rValue);

      if (changed)
      {
        m_Value = rValue;

        Changed.Notify(this, EventArguments::Empty());
      }
    }

    /**
     * Gets the default value of parameter
     * @return parameter value
     */
    virtual T& GetDefaultValue()
    {
      return m_DefaultValue;
    }

    /**
     * Sets the default value of parameter
     * @param rValue value to set as default
     */
    virtual void SetDefaultValue(const T& rValue)
    {
      m_DefaultValue = rValue;
    }

    /**
     * Gets value of parameter as string
     * @return string version of value
     */
    virtual const karto::String GetValueAsString() const
    {
      return karto::StringHelper::ToString(m_Value);
    }

    /**
     * Sets the value of parameter from string
     * @param rStringValue string
     */
    virtual void SetValueFromString(const karto::String& rStringValue)
    {
      T value;
      if (karto::StringHelper::FromString(rStringValue, value))
      {
        SetValue(value);
      }
    }

    /**
     * Set parameter value to default value
     */ 
    virtual void SetToDefaultValue()
    {
      SetValue(m_DefaultValue);
    }

  protected:
    /**
     * Initialize parameters
     */ 
    virtual void InitializeParameters()
    {
      SetDefaultValue(GetValue());
    }

    /**
     * Compares if the given value is equal to current value
     * @param rValue value to compare against
     * @return true if equal
     */ 
    kt_bool CompareValue(const T& rValue)
    {
      return m_Value == rValue;
    }

  private:
    // restrict the following functions
    Parameter(const Parameter&);
    const Parameter& operator=(const Parameter&);

  protected:
    /**
     * Parameter value
     */
    T m_Value;

  private:
    /**
     * Parameter default value
     */
    T m_DefaultValue;
  };

  /**
   * Register Parameter<kt_bool> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_bool>);

  /**
   * Register Parameter<kt_char> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_char>);

  /**
   * Register Parameter<kt_int8s> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int8s>);

  /**
   * Register Parameter<kt_int8u> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int8u>);

  /**
   * Register Parameter<kt_int16s> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int16s>);

  /**
   * Register Parameter<kt_int16u> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int16u>);

  /**
   * Register Parameter<kt_int32s> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int32s>);

  /**
   * Register Parameter<kt_int32u> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int32u>);

  /**
   * Register Parameter<kt_int64s> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int64s>);

  /**
   * Register Parameter<kt_int64u> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_int64u>);

  /**
   * Register Parameter<kt_float> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_float>);

  /**
   * Register Parameter<kt_double> with MetaClassManager
   */
  KARTO_TYPE(Parameter<kt_double>);

  /**
   * Register Parameter<karto::String> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::String>);

  /**
   * Register Parameter<karto::Size2<kt_int32s> > with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Size2<kt_int32s> >);

  /**
   * Register Parameter<karto::Size2<kt_int32u> > with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Size2<kt_int32u> >);

  /**
   * Register Parameter<karto::Size2<kt_double> > with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Size2<kt_double> >);

  /**
   * Register Parameter<karto::Vector2i> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector2i>);

  /**
   * Register Parameter<karto::Vector3i> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector3i>);

  /**
   * Register Parameter<karto::Vector4i> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector4i>);

  /**
   * Register Parameter<karto::Vector2iu> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector2<kt_int32u> >);

  /**
   * Register Parameter<karto::Vector3iu> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector3iu>);

  /**
   * Register Parameter<karto::Vector4iu> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector4iu>);

  /**
   * Register Parameter<karto::Vector2d> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector2<kt_double> >);

  /**
   * Register Parameter<karto::Vector3d> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector3d>);

  /**
   * Register Parameter<karto::Vector4d> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Vector4d>);

  /**
   * Register Parameter<karto::Quaternion> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Quaternion>);

  /**
   * Register Parameter<karto::Color> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Color>);

  /**
   * Register Parameter<karto::Pose2> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Pose2>);

  /**
   * Register Parameter<karto::Pose3> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::Pose3>);

  /**
   * Register Parameter<karto::gps::PointGps> with MetaClassManager
   */
  KARTO_TYPE(Parameter<karto::gps::PointGps>);

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct ParameterEnumPrivate;

  /**
   * Parameter enum class
   */
  class KARTO_EXPORT ParameterEnum : public Parameter<kt_int64s>
  {
    KARTO_RTTI();

  public:
    /**
     * Enumeration parameter 
     * @param pParameterSet set this parameter enumeration will belong to
     * @param rName name
     * @param rDisplayName display name
     * @param rDescription description
     * @param value value
     */
    ParameterEnum(ParameterSet* pParameterSet, const karto::String& rName, const karto::String& rDisplayName, const karto::String& rDescription, kt_int64s value);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~ParameterEnum();
    //@endcond

  public:
    /**
     * Gets the parameter value as string.
     * @return value as string
     * @throws Exception if unable to convert enum to string
     */
    virtual const karto::String GetValueAsString() const;

    /**
     * Sets the parameter value from string.
     * @param rStringValue value as string
     * @throws Exception if unable to set enum value
     */
    virtual void SetValueFromString(const karto::String& rStringValue);

  public:
    /**
     * Defines the enum with the given name as having the given value
     * @param rName name of enum
     * @param value value of enum
     */
    void DefineEnumValue(const String& rName, kt_int64s value);

    /**
     * Gets the list of enum pairs associated with this parameter
     * @return list of enum pairs associated with this parameter
     */
    const EnumPairList GetEnumValues() const;

  private:
    ParameterEnumPrivate* m_pPrivate;
  }; // class ParameterEnum

  /**
   * Register ParameterEnum with MetaClassManager
   */
  KARTO_TYPE(ParameterEnum);

  //@}
}

#endif // __OpenKarto_Parameter_h__
