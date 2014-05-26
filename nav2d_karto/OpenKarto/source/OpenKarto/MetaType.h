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

#ifndef __OpenKarto_MetaType_h__
#define __OpenKarto_MetaType_h__

#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{
  
  //@cond EXCLUDE

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // The following code is from the boost library with the following license 
  // and modified to fit into the KartoSDK

  // Copyright 2003 (c) The Trustees of Indiana University.

  // Use, modification, and distribution is subject to the Boost Software
  // License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
  // http://www.boost.org/LICENSE_1_0.txt)

  //    Authors: Jaakko Jarvi (jajarvi at osl.iu.edu)
  //             Jeremiah Willcock (jewillco at osl.iu.edu)
  //             Andrew Lumsdaine (lums at osl.iu.edu)

  template <bool B, class T = void>
  struct enable_if_c 
  {
    typedef T type;
  };

  template <class T>
  struct enable_if_c<false, T> 
  {
  };

  template <class Cond, class T = void> 
  struct enable_if : public enable_if_c<Cond::value, T> 
  {
  };

  template <bool B, class T>
  struct lazy_enable_if_c 
  {
    typedef typename T::type type;
  };

  template <class T>
  struct lazy_enable_if_c<false, T> 
  {
  };

  template <class Cond, class T> 
  struct lazy_enable_if : public lazy_enable_if_c<Cond::value, T>
  {
  };

  template <bool B, class T = void>
  struct disable_if_c
  {
    typedef T type;
  };

  template <class T>
  struct disable_if_c<true, T> 
  {
  };

  template <class Cond, class T = void> 
  struct disable_if : public disable_if_c<Cond::value, T> 
  {
  };

  template <bool B, class T>
  struct lazy_disable_if_c 
  {
    typedef typename T::type type;
  };

  template <class T>
  struct lazy_disable_if_c<true, T>
  {
  };

  template <class Cond, class T> 
  struct lazy_disable_if : public lazy_disable_if_c<Cond::value, T>
  {
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // remove reference
  template<class T>
  struct remove_reference
  {	
    typedef T Type;
  };

  // remove reference
  template<class T>
  struct remove_reference<T&>
  {	
    typedef T Type;
  };

  //// remove rvalue reference
  //template<class T>
  //struct remove_reference<T&&>
  //{	
  //  typedef T Type;
  //};

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // is_pointer template - value == false
  template <typename T> 
  struct is_pointer 
  { 
    enum 
    {
      value = false 
    };
  };

  // is_pointer template - value == true
  template <typename T> 
  struct is_pointer<T*> 
  { 
    enum 
    {
      value = true 
    };
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * This will trigger a compile error if KARTO_TYPE is not defined for object
   */
  template <typename T>
  struct KartoTypeId
  {
    static const char* Get(kt_bool = true)
    {
      // Remember to register your class/enum T with the KARTO_TYPE macro
      return T::KARTO_REGISTER_ME_WITH_KARTO_TYPE();
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Get composed type
   * Generic version
   * KartoType<T>::Type == T
   */
  template <typename T/*, typename E = void*/>
  struct KartoType
  {
    typedef T Type;
  };

  /**
   * KartoType<const T>::Type == KartoType<T>::Type
   */
  template <typename T> 
  struct KartoType<const T>
  {
    typedef typename KartoType<T>::Type Type;
  };

  /**
   * KartoType<T&>::Type == KartoType<T>::Type
   * KartoType<const T&>::Type == KartoType<T>::Type
   */
  template <typename T> 
  struct KartoType<T&>
  {
    typedef typename KartoType<T>::Type Type;
  };

  /**
   * KartoType<T*>::Type == KartoType<T>::Type
   * KartoType<const T*>::Type == KartoType<T>::Type
   */
  template <typename T>
  struct KartoType<T*>
  {
    typedef typename KartoType<T>::Type Type;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Get the Karto type id
   */
  template <typename T>
  const char* GetKartoTypeIdTemplate()         
  {
    return KartoTypeId<typename KartoType<T>::Type>::Get();
  }

  /**
   * Get the Karto type id
   */
  template <typename T>
  const char* GetKartoTypeIdTemplate(const T&) 
  {
    return KartoTypeId<typename KartoType<T>::Type>::Get();
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Karto object information
   * Generic version
   */
  template <typename T, typename E = void>
  struct KartoObjectTraits
  {
    typedef T& RefReturnType;

    static RefReturnType Get(void* pPointer) 
    {
      return *static_cast<T*>(pPointer);
    }
  };

  /**
   * specialized for pointer
   */
  template <typename T>
  struct KartoObjectTraits<T*>
  {
    typedef T* RefReturnType;
    typedef T* PointerType;

    static RefReturnType Get(void* pPointer) 
    {
      return static_cast<T*>(pPointer);
    }
    
    static PointerType GetPointer(T* rValue)
    {
      return rValue;
    }
  };

  /**
   * specialized for reference to non-referenced types
   */
  template <typename T>
  struct KartoObjectTraits<T&, typename disable_if<is_pointer<typename KartoObjectTraits<T>::RefReturnType> >::type>
  {
    typedef T& RefReturnType;
    typedef T* PointerType;

    static RefReturnType Get(void* pPointer) 
    {
      return *static_cast<T*>(pPointer);
    }

    static PointerType GetPointer(T& rValue) 
    {
      return &rValue;
    }
  };

  /**
   * specialized for reference to referenced types - removed the reference!
   */
  template <typename T>
  struct KartoObjectTraits<T&, typename enable_if< is_pointer< typename KartoObjectTraits< T >::RefReturnType > >::type > : KartoObjectTraits<T>
  {
  };

  /**
   * specialized for const types - removes the const!
   */
  template <typename T>
  struct KartoObjectTraits<const T> : KartoObjectTraits<T>
  {
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Compile time check if T implements the KARTO_RTTI (Karto Runtime Type Info)
   * see http://stackoverflow.com/questions/257288/is-it-possible-to-write-a-c-template-to-check-for-a-functions-existence
   */
  template <typename T>
  struct KartoTypeHasRtti
  {
    typedef kt_int16s Yes; 
    typedef kt_int32s No; 

    template <typename U, const char* (U::*)() const>
    struct CheckForMember 
    {
    };

    template <typename U> static Yes HasMemberFunction(CheckForMember<U, &U::GetKartoClassId>*);
    template <typename U> static No HasMemberFunction(...);

    enum
    {
      value = sizeof(HasMemberFunction<typename KartoType<T>::Type>(0)) == sizeof(Yes)
    };
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Get KARTO_RTTI information for dynamic karto type. 
   * NOTE: Base type and derived types my use the KARTO_RTTI macro for this to work!
   */
  template <typename T, typename E = void>
  struct GetKartoTypeIdTemplateRTTI
  {
    typedef KartoObjectTraits<const T&> Traits;

    static const char* Get(const T& rObject)
    {
      typename Traits::PointerType pointer = Traits::GetPointer(rObject);
      return pointer ? pointer->GetKartoClassId() : GetKartoTypeIdTemplate<T>();
    }
  };

  /**
   * Specialized version for type that don't implement KARTO_RTTI
   */
  template <typename T>
  struct GetKartoTypeIdTemplateRTTI< T, typename disable_if< KartoTypeHasRtti<T> >::type >
  {
    static const char* Get(const T&)
    {
      return GetKartoTypeIdTemplate<T>();
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Get the Karto type id
   */
  template <typename T> const char* GetTypeId()                
  {
    return GetKartoTypeIdTemplate<T>();
  }
  
  /**
   * Get the Karto type id
   */
  template <typename T> const char* GetTypeId(const T& rObject) 
  {
    return GetKartoTypeIdTemplateRTTI<T>::Get(rObject);
  }

  // @endcond

  //@}

}

#endif // __OpenKarto_MetaType_h__
