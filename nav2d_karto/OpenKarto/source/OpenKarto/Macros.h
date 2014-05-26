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

#ifndef __OpenKarto_Macros_h__
#define __OpenKarto_Macros_h__

///** \addtogroup OpenKarto */
//@{

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Karto defines for handling deprecated code 
 */
#ifndef KARTO_DEPRECATED
#  if defined(__GNUC__) && (__GNUC__ >= 4 || (__GNUC__==3 && __GNUC_MINOR__>=1))
#    define KARTO_DEPRECATED __attribute__((deprecated))
#  elif defined(__INTEL) || defined(_MSC_VER)
#    define KARTO_DEPRECATED __declspec(deprecated)
#  else
#    define KARTO_DEPRECATED
#  endif
#endif

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Karto defines for forcing inline code 
 */
#ifndef KARTO_FORCEINLINE
#  if defined(__GNUC__) && (__GNUC__ >= 4 || (__GNUC__==3 && __GNUC_MINOR__>=1))
#    define KARTO_FORCEINLINE inline __attribute__((always_inline))
#  elif defined(__INTEL) || defined(_MSC_VER)
#    define KARTO_FORCEINLINE __forceinline
#  else
#    define KARTO_FORCEINLINE
#  endif
#endif

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Karto defines for windows dynamic build 
 */
#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
# if defined( _LIB ) || defined( KARTO_STATIC ) || defined( STATIC_BUILD )
#  define KARTO_EXPORT
# else
#  ifdef KARTO_DYNAMIC
#    define KARTO_EXPORT __declspec(dllexport)
#  else
#    define KARTO_EXPORT __declspec(dllimport)
#  endif // KARTO_DYNAMIC 
# endif
#else
#  define KARTO_EXPORT
#endif 

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/** 
 * Iterate through items in std::vector with iterator iter
 */
#define forEach( listtype, list ) \
  for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

/** 
 * Iterate through items in std::vector with provided iterator iter
 */
#define forEachAs( listtype, list, iter ) \
  for ( listtype::iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

/** 
 * Iterate through items in const std::vector with iterator iter
 */
#define const_forEach( listtype, list ) \
  for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

/** 
 * Iterate through items in const std::vector with provided iterator iter
 */
#define const_forEachAs( listtype, list, iter ) \
  for ( listtype::const_iterator iter = (list)->begin(); iter != (list)->end(); ++iter )

/** 
 * Reverse iterate through items in std::vector with iterator iter
 */
#define forEachR( listtype, list ) \
  for ( listtype::reverse_iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

/** 
 * Reverse iterate through items in const std::vector with iterator iter
 */
#define const_forEachR( listtype, list ) \
  for ( listtype::const_reverse_iterator iter = (list)->rbegin(); iter != (list)->rend(); ++iter )

/** 
 * Iterate through items in karto::List with iterator iter
 */
#define karto_forEach(listtype, list) \
  for ( listtype::Iterator iter = (list)->GetIterator(); iter.HasNext(); iter.Next())

/** 
 * Iterate through items in karto::List with provided iterator iter
 */
#define karto_forEachAs(listtype, list, iter) \
  for ( listtype::Iterator iter = (list)->GetIterator(); iter.HasNext(); iter.Next())

/** 
 * Iterate through items in const karto::List with iterator iter
 */
#define karto_const_forEach(listtype, list) \
  for ( listtype::ConstIterator iter = (list)->GetConstIterator(); iter.HasNext(); iter.Next())

/** 
 * Iterate through items in const karto::List with provided iterator iter
 */
#define karto_const_forEachAs(listtype, list, iter) \
  for ( listtype::ConstIterator iter = (list)->GetConstIterator(); iter.HasNext(); iter.Next())


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/** 
 * Disable annoying compiler warnings
 */

#if defined(__INTEL) || defined(_MSC_VER)

// Disable the warning: 'identifier' : unreferenced formal parameter
#pragma warning(disable:4100)

// Disable the warning: 'identifier' : class 'type' needs to have dll-interface to be used by clients of class 'type2'
#pragma warning(disable:4251)

// Disable the warning: 'identifier' assignment operator could not be generated
#pragma warning(disable:4512)

#endif

#ifdef __INTEL_COMPILER

//// Disable the warning: conditional expression is constant
//#pragma warning(disable:4127)
//
//// Disable the warning: 'identifier' : unreferenced formal parameter
//#pragma warning(disable:4100)
//
//// remark #383: value copied to temporary, reference to temporary used
//#pragma warning(disable:383)
//
//// remark #981: operands are evaluated in unspecified order
//// disabled -> completely pointless if the functions do not have side effects
//#pragma warning(disable:981)
//
//// remark #1418: external function definition with no prior declaration
//#pragma warning(disable:1418)
//
//// remark #1572: floating-point equality and inequality comparisons are unreliable
//// disabled -> everyone knows it, the parser passes this problem deliberately to the user
//#pragma warning(disable:1572)
//
//// remark #10121:
//#pragma warning(disable:10121)

#endif // __INTEL_COMPILER

//@}

#endif // __OpenKarto_Macros_h__
