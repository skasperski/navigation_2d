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

#ifndef __OpenKarto_Types_h__
#define __OpenKarto_Types_h__

#include <assert.h>

#include <cstddef>

/**
  * \defgroup OpenKarto OpenKarto Module
  */
/*@{*/

#if defined(_MSC_VER)

  /**
   * Type declaration of 8 bit integer type
   */
  typedef signed __int8 kt_int8s;

  /**
   * Type declaration of unsigned 8 bit integer type
   */
  typedef unsigned __int8 kt_int8u;

  /**
   * Type declaration of 16 bit integer type
   */
  typedef signed __int16 kt_int16s;

  /**
   * Type declaration of unsigned 16 bit integer type
   */
  typedef unsigned __int16 kt_int16u;

  /**
   * Type declaration of 32 bit integer type
   */
  typedef signed __int32 kt_int32s;

  /**
   * Type declaration of unsigned 32 bit integer type
   */
  typedef unsigned __int32 kt_int32u;

  /**
   * Type declaration of 64 bit integer type
   */
  typedef signed __int64 kt_int64s;

  /**
   * Type declaration of unsigned 64 bit integer type
   */
  typedef unsigned __int64 kt_int64u;

  /**
   * Type declaration of size_t type
   */
  typedef std::size_t kt_size_t;

#else

  #include <stdint.h>

  /**
   * Type declaration of 8 bit integer type
   */
  typedef int8_t kt_int8s;

  /**
   * Type declaration of unsigned 8 bit integer type
   */
  typedef uint8_t kt_int8u;

  /**
   * Type declaration of 16 bit integer type
   */
  typedef int16_t kt_int16s;

  /**
   * Type declaration of unsigned 16 bit integer type
   */
  typedef uint16_t kt_int16u;

  /**
   * Type declaration of 32 bit integer type
   */
  typedef int32_t kt_int32s;

  /**
   * Type declaration of unsigned 32 bit integer type
   */
  typedef uint32_t kt_int32u;

#if defined(__LP64__)
  /**
   * Type declaration of 64 bit integer type
   */
  typedef signed long kt_int64s;

  /**
   * Type declaration of unsigned 64 bit integer type
   */
  typedef unsigned long kt_int64u;
#else
  /**
   * Type declaration of 64 bit integer type
   */
  typedef signed long long kt_int64s;

  /**
   * Type declaration of unsigned 64 bit integer type
   */
  typedef unsigned long long kt_int64u;
#endif

  /**
   * Type declaration of size_t type
   */
  typedef std::size_t kt_size_t;

#endif

/**
 * Type declaration of boolean type
 */
typedef bool kt_bool;

/**
 * Type declaration of char type
 */
typedef char kt_char;

/**
 * Type declaration of float type
 */
typedef float kt_float;

/**
 * Type declaration of double type
 */
typedef double kt_double;

/**
 * Type declaration of karto object type
 */
typedef kt_int32u kt_objecttype;

/**
 * Type declaration of karto hight resolution timer tick type
 */
typedef kt_int64s kt_tick;

/*@}*/

#endif // __OpenKarto_Types_h__
