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

#ifndef __OpenKarto_MetaClass_h__
#define __OpenKarto_MetaClass_h__

#include <OpenKarto/MetaType.h>
#include <OpenKarto/MetaAttribute.h>
#include <OpenKarto/MetaClassManager.h>
#include <OpenKarto/Referenced.h>
#include <OpenKarto/List.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  //@cond EXCLUDE

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  struct MetaArgsPrivate;

  /**
   * Meta argument container for meta constructor
   */
  class KARTO_EXPORT MetaArguments
  {
  public:
    /**
     * Default constructor
     */
    MetaArguments();

    /**
     * Constructs an argument container with one argument
     * @param a0 first argument
     */
    MetaArguments(const Any& a0);

    /**
     * Constructs an argument container with two argument
     * @param a0 first argument
     * @param a1 second argument
     */
    MetaArguments(const Any& a0, const Any& a1);

    /**
     * Constructs an argument container with three argument
     * @param a0 first argument
     * @param a1 second argument
     * @param a2 third argument
     */
    MetaArguments(const Any& a0, const Any& a1, const Any& a2);

    /**
     * Constructs an argument container with three argument
     * @param a0 first argument
     * @param a1 second argument
     * @param a2 third argument
     * @param a3 fourth argument
     */
    MetaArguments(const Any& a0, const Any& a1, const Any& a2, const Any& a3);

    /**
     * Constructs an argument container with three argument
     * @param a0 first argument
     * @param a1 second argument
     * @param a2 third argument
     * @param a3 fourth argument
     * @param a4 fifth argument
     */
    MetaArguments(const Any& a0, const Any& a1, const Any& a2, const Any& a3, const Any& a4);

    /**
     * Destructor
     */
    ~MetaArguments();

  public:
    /**
     * Get number of arguments
     */
    kt_size_t GetCount() const;

  public:
    /**
     * Index operator, get argument at index
     * @param index
     * @return argument at index
     */
    const Any& operator[](kt_size_t index) const;

  public:
    /**
     * Empty argument set
     * @return empty arguments
     */
    static const MetaArguments& Empty()
    {
      static MetaArguments dummy;

      return dummy;
    }

  private:
    MetaArgsPrivate* m_pPrivate;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Check that argument is the correct type
   * @param rAny argument to type check 
   * @return true if Any is not NULL
   */
  template <typename T>
  kt_bool CheckArgumentType(const Any& rAny)
  {
    return any_cast<T>(rAny) != NULL;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Abstract base class for meta class constructor definition
   */
  class MetaConstructor
  {
  public:
    /**
     * Destructor
     */
    virtual ~MetaConstructor()
    {
    }

  public:
    /**
     * Check if number of argument matches the number of arguments defined for meta class. Also 
     * check if arguments are of the right type.
     * @param rArgs arguments to check
     * @return true if arguments matches 
     */
    virtual kt_bool CheckArguments(const MetaArguments& rArgs) const = 0;

    /**
     * Create instance of class with provided arguments
     * @param rArgs arguments used to create class
     * @return pointer to created class
     * @throws Exception if unable to match arguments or create instance of class
     */
    virtual void* Create(const MetaArguments& rArgs) const = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Meta class constructor definition for class with zero arguments
   */
  template <typename T>
  class MetaConstructorImpl0 : public MetaConstructor
  {
  public:
    virtual kt_bool CheckArguments(const MetaArguments& rArgs) const
    {
      return (rArgs.GetCount() == 0);
    }

    virtual void* Create(const MetaArguments&) const
    {
      return new T();
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Meta class constructor definition for class with one argument
   */
  template <typename T, typename A0>
  class MetaConstructorImpl1 : public MetaConstructor
  {
  public:
    virtual kt_bool CheckArguments(const MetaArguments& rArgs) const
    {
      return (rArgs.GetCount() == 1) && CheckArgumentType<A0>(rArgs[0]);
    }

    virtual void* Create(const MetaArguments& rArgs) const
    {
      return new T(any_cast<A0>(rArgs[0]));
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /** 
   * Meta class constructor definition for class with two argument
   */
  template <typename T, typename A0, typename A1>
  class MetaConstructorImpl2 : public MetaConstructor
  {
  public:
    virtual kt_bool CheckArguments(const MetaArguments& rArgs) const
    {
      return (rArgs.GetCount() == 2) && CheckArgumentType<A0>(rArgs[0]) && CheckArgumentType<A1>(rArgs[1]);
    }

    virtual void* Create(const MetaArguments& rArgs) const
    {
      return new T(any_cast<A0>(rArgs[0]), any_cast<A1>(rArgs[1]));
    }
  };

  // @endcond

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template <typename T> class MetaClassHelper;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class MetaConstructor;

  /** 
   * The MetaClass contains information about a C++ class with constructors, attributes, base classes 
   */
  class KARTO_EXPORT MetaClass : public MetaAttribute, public Referenced
  {
  public:
    /**
     * Registers a new MetaClass by name.
     * @param rName Name of new MetaClass
     * @return helper class for creating MetaClass
     */
    template <typename T>
    static MetaClassHelper<T> Register(const karto::String& rName)
    {
      MetaClass& newClass = MetaClassManager::GetInstance().RegisterNew(rName, KartoTypeId<T>::Get(false));
      return MetaClassHelper<T>(newClass);
    }

  public:
    /**
     * Gets the name of the MetaClass
     * @return name of MetaClass
     */
    const karto::String& GetName() const;

    /**
     * Gets the number of base classes
     * @return number of base classes
     */
    kt_size_t GetBaseSize() const;

    /**
     * Gets the base MetaClass at index
     * @param index Base class index
     * @return MetaClass at index
     * @throws Exception if index is out of range
     */
    const MetaClass& GetBase(kt_size_t index) const;

    /**
     * Creates a new instance of the C++ class described by the MetaClass
     * @param rArgs arguments for constructor
     * @return new instance of class described by meta class
     */
    template <typename T>
    T* Create(const MetaArguments& rArgs = MetaArguments::Empty()) const
    {
      void* pObject = NULL;

      karto_const_forEach(List<const MetaConstructor*>, &m_Constructors)
      {
        const MetaConstructor* pConstructor = *iter;

        if (pConstructor->CheckArguments(rArgs))
        {
          pObject = pConstructor->Create(rArgs);
        }
      }

      if (pObject == NULL)
      {
        throw karto::Exception("Unable to create object '" + GetName() + "'. Please verify that .Constructor is defined for MetaClass");
      }

      return static_cast<T*>(pObject);
    }

    /**
     * Destroys an instance of the C++ class described by the MetaClass
     */
    template <typename T>
    void Destroy(const T* pObject) const
    {
      delete pObject;
    }

  public:
    /**
     * Equality operator checks equality between two MetaClasses.
     * @param rOther
     * @return true if equal, false otherwise
     */
    kt_bool operator==(const MetaClass& rOther) const;

    /**
     * Inequality operator checks inequality between two MetaClasses
     * @param rOther
     * @return true if unequal, false otherwise
     */
    kt_bool operator!=(const MetaClass& rOther) const;

  private:
    MetaClass(const karto::String& rName);
    ~MetaClass();

  private:
    template <typename T> friend class MetaClassHelper;
    friend class MetaClassManager;

    karto::String m_Name; 
    List<const MetaClass*> m_BaseClasses;

    List<const MetaConstructor*> m_Constructors;
  };

  //@}

}

#endif // __OpenKarto_MetaClass_h__
