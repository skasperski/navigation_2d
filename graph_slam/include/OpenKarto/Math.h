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

#ifndef __OpenKarto_Math_h__
#define __OpenKarto_Math_h__

#include <assert.h>
#include <math.h>
#include <float.h>
#include <cstdlib>

#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Platform independent pi definitions
   */
  
  /**
   * PI
   */
  const kt_double KT_PI         =  3.14159265358979323846;
  
  /**
   * 2 * PI
   */
  const kt_double KT_2PI        =  6.28318530717958647692;
  
  /**
   * PI / 2
   */
  const kt_double KT_PI_2       =  1.57079632679489661923;
  
  /**
   * PI / 180
   */
  const kt_double KT_PI_180     =  0.01745329251994329577;
  
  /**
   * 180 / PI
   */
  const kt_double KT_180_PI     = 57.29577951308232087685;
  
  /**
   * Karto tolerance level
   * @cond EXCLUDE
   * Note: \todo 1/26/2011: get assert failure for lrfs with wide angles (see willow dataset)
   * @endcond
   */
  const kt_double KT_TOLERANCE  = 1e-06;
  
  namespace math
  {    
    ///** \addtogroup OpenKarto */
    //@{

    /**
     * Converts degrees into radians
     * @param degrees degrees
     * @return radian equivalent of degrees
     */
    inline kt_double DegreesToRadians(kt_double degrees)
    {
      return degrees * KT_PI_180;
    }
    
    /**
     * Converts radians into degrees
     * @param radians radians
     * @return degree equivalent of radians
     */
    inline kt_double RadiansToDegrees(kt_double radians)
    {
      return radians * KT_180_PI;
    }
    
    /**
     * Square function
     * @param value value
     * @return square of value
     */
    template<typename T>
    inline T Square(T value)
    {
      return (value * value);
    }
    
    /**
     * Round function
     * @param value value
     * @return rounds value to the nearest whole number (as double)
     */
    inline kt_double Round(kt_double value)
    {
      return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
    }
    
    /**
     * Binary minimum function
     * @param value1 value
     * @param value2 value
     * @return the lesser of value1 and value2
     */
    template<typename T>
    inline const T& Minimum(const T& value1, const T& value2)
    {
      return value1 < value2 ? value1 : value2;
    }
    
    /**
     * Binary maximum function
     * @param value1 value
     * @param value2 value
     * @return the greater of value1 and value2
     */
    template<typename T>
    inline const T& Maximum(const T& value1, const T& value2)
    {
      return value1 > value2 ? value1 : value2;
    }
    
    /**
     * Clips a number to the specified minimum and maximum values.
     * @param n number to be clipped
     * @param minValue minimum value
     * @param maxValue maximum value
     * @return the clipped value
     */
    template<typename T> 
    inline const T& Clip(const T& n, const T& minValue, const T& maxValue)
    {
      return Minimum(Maximum(n, minValue), maxValue);
    }
    
    /**
     * Checks whether two numbers are equal within a certain tolerance.
     * @param a value
     * @param b value
     * @return true if a and b differ by at most a certain tolerance.
     */
    inline kt_bool DoubleEqual(kt_double a, kt_double b)
    {
      double delta = a - b; 
      return delta < 0.0 ? delta >= -KT_TOLERANCE : delta <= KT_TOLERANCE;
    }
    
    /**
     * Checks whether value is in the range [0;maximum)
     * @param value value
     * @param maximum maximum
     * @return whether the given value is in the range [0;maximum)
     */
    template<typename T>
    inline kt_bool IsUpTo(const T& value, const T& maximum)
    {
      return (value >= 0 && value < maximum);
    }

    //@cond EXCLUDE
    /**
     * Checks whether value is in the range [0;maximum)
     * Specialized version for unsigned int (kt_int32u) to satisfy Intel compiler
     * @param value value
     * @param maximum maximum
     * @return whether the given value is in the range [0;maximum)
     */
    template<>
    inline kt_bool IsUpTo<kt_int32u>(const kt_int32u& value, const kt_int32u& maximum)
    {
      return (value < maximum);
    }
    //@endcond
    
    /**
     * Checks whether value is in the range [a;b]
     * @param value value
     * @param a a
     * @param b b
     * @return whether the given value is in the range [a;b]
     */
    template<typename T>
    inline kt_bool InRange(const T& value, const T& a, const T& b)
    {
      return (value >= a && value <= b);
    }
    
    /**
     * Normalizes angle to be in the range of [-pi, pi]
     * @param angle to be normalized
     * @return normalized angle
     */
    inline kt_double NormalizeAngle(kt_double angle)
    {
      while (angle < -KT_PI)
      {
        if (angle < -KT_2PI)
        {
          angle += (kt_int32u)(angle / -KT_2PI) * KT_2PI;
        }
        else
        {
          angle += KT_2PI;
        }
      }
      
      while (angle > KT_PI)
      {
        if (angle > KT_2PI)
        {
          angle -= (kt_int32u)(angle / KT_2PI) * KT_2PI;
        }
        else
        {
          angle -= KT_2PI;
        }
      }
      
      assert(math::InRange(angle, -KT_PI, KT_PI));
      
      return angle;
    }
    
    /**
     * Returns an equivalent angle to the first parameter such that the difference
     * when the second parameter is subtracted from this new value is an angle
     * in the normalized range of [-pi, pi], i.e. abs(minuend - subtrahend) <= pi.
     * @param minuend minuend
     * @param subtrahend subtrahend
     * @return normalized angle
     */
    inline kt_double NormalizeAngleDifference(kt_double minuend, kt_double subtrahend)
    {
      while (minuend - subtrahend < -KT_PI)
      {
        minuend += KT_2PI;
      }
      
      while (minuend - subtrahend > KT_PI)
      {
        minuend -= KT_2PI;
      }
      
      return minuend;
    }
    
    /**
     * Aligns a value to the alignValue. 
     * The alignValue should be the power of two (2, 4, 8, 16, 32 and so on)
     * @param value value
     * @param alignValue value to align to
     * @return aligned value
     */
    template<class T>
    inline T AlignValue(size_t value, size_t alignValue = 8)
    {
      return static_cast<T> ((value + (alignValue - 1)) & ~(alignValue - 1));
    }
    
    /**
     * Swaps the value of the given parameters
     * @param x value
     * @param y value
     */
    template<class T>
    inline void Swap(T& x, T& y)
    {
      T temp = x;
      x = y;
      y = temp;
    }

    //@}
  } // Math
    
  //@}
}

#endif // __OpenKarto_Math_h__
