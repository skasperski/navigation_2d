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

#ifndef __OpenKarto_Geometry_h__
#define __OpenKarto_Geometry_h__

#include <OpenKarto/List.h>
#include <OpenKarto/StringHelper.h>
#include <OpenKarto/Math.h>

#include <string.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * A 2-dimensional size (width, height)
   */
  template<typename T>
  class Size2
  {
  public:
    /**
     * A size with a width and height of 0
     */
    Size2()
      : m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * A size with the given width and height
     * @param width width
     * @param height height
     */
    Size2(T width, T height)
      : m_Width(width)
      , m_Height(height)
    {
    }

    /**
     * Copy constructor
     */
    Size2(const Size2& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Gets the width
     * @return the width
     */
    inline const T GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width
     * @param width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Gets the height
     * @return the height
     */
    inline const T GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height
     * @param height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Returns a string representation of this size
     * @return string representation of this size
     */
    inline const karto::String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetWidth()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetHeight()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Size2& operator=(const Size2& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;

      return(*this);
    }

    /**
     * Equality operator
     */
    inline kt_bool operator==(const Size2& rOther) const
    {
      return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Size2& rOther) const
    {
      return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
    }

    /**
     * Write size onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Size2& rSize)
    {
      rStream << rSize.ToString().ToCString();
      return rStream;
    }

  private:
    T m_Width;
    T m_Height;
  }; // class Size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * A 3-dimensional size (width, height, depth)
   */
  template<typename T>
  class Size3
  {
  public:
    /**
     * A size with a width, height, and depth of 0
     */
    inline Size3()
      : m_Width(0.0)
      , m_Height(0.0)
      , m_Depth(0.0)
    {
    }

    /**
     * A size with the given width, height, and depth
     * @param width width
     * @param height height
     * @param depth depth
     **/
    inline Size3(T width, T height, T depth)
      : m_Width(width)
      , m_Height(height)
      , m_Depth(depth)
    {
    }		

    /**
     * Copy constructor
     */
    inline Size3(const Size3& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Returns the width
     * @return the width
     */
    inline T GetWidth() const
    {
      return m_Width;
    }
    
    /**
     * Sets the width
     * @param width width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Returns the height
     * @return the height
     */
    inline T GetHeight() const
    {
      return m_Height;
    }
    
    /**
     * Sets the height
     * @param height height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Returns the depth
     * @return the depth
     */
    inline T GetDepth() const
    {
      return m_Depth;
    }

    /**
     * Sets the depth
     * @param depth depth
     */
    inline void SetDepth(T depth)
    {
      m_Depth = depth;
    }

    /**
     * Returns a string representation of this size object
     * @return string representation of this size object
     */
    inline const karto::String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetWidth()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetHeight()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetDepth()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Size3& operator=(const Size3& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;
      m_Depth = rOther.m_Depth;

      return *this;
    }

    /**
     * Equality operator
     */
    kt_bool operator==(const Size3& rOther) const 
    {
      return m_Width == rOther.m_Width && m_Height == rOther.m_Height && m_Depth == rOther.m_Depth; 
    }
    
    /**
     * Inequality operator
     */
    kt_bool operator!=(const Size3& rOther) const 
    {
      return m_Width != rOther.m_Width || m_Height != rOther.m_Height || m_Depth != rOther.m_Depth; 
    }

  private:
    T m_Width;
    T m_Height;
    T m_Depth;
  }; // class Size3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a 2-dimensional vector (x, y)
   */
  template<typename T>
  class Vector2
  {
  public:
    /**
     * Vector at the origin
     */
    Vector2()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
    }
    
    /**
     * Vector at the given location
     * @param x x
     * @param y y
     */
    Vector2(T x, T y)
    {
      m_Values[0] = x;
      m_Values[1] = y;
    }
    
  public:
    /**
     * Gets the x-coordinate of this vector
     * @return the x-coordinate of the vector
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }
    
    /**
     * Sets the x-coordinate of this vector
     * @param x the x-coordinate of the vector
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }
    
    /**
     * Gets the y-coordinate of this vector
     * @return the y-coordinate of the vector
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }
    
    /**
     * Sets the y-coordinate of this vector
     * @param y the y-coordinate of the vector
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }
        
    /**
     * Floor point operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
    }
    
    /**
     * Ceiling point operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0])
      {
        m_Values[0] = rOther.m_Values[0];
      }
      
      if (rOther.m_Values[1] > m_Values[1])
      {
        m_Values[1] = rOther.m_Values[1];
      }
    }
    
    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]);
    }
    
    /**
     * Returns the length of the vector
     * @return length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }
    
    /**
     * Returns the square of the distance to the given vector
     * @param rOther vector
     * @returns square of the distance to the given vector
     */
    inline kt_double SquaredDistance(const Vector2& rOther) const
    {
      return (*this - rOther).SquaredLength();
    }
    
    /** 
     * Gets the distance to the given vector
     * @param rOther vector
     * @return distance to given vector
     */
    inline kt_double Distance(const Vector2& rOther) const
    {
      return sqrt(SquaredDistance(rOther));
    }
    
    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline const String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetX()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetY()));
      return valueString;
    }

  public:
    /**
     * In-place vector addition
     */
    inline void operator+=(const Vector2& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
    }
    
    /**
     * In-place vector subtraction
     */
    inline void operator-=(const Vector2& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
    }

    /**
     * Vector addition
     */
    inline const Vector2 operator+(const Vector2& rOther) const
    {
      return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
    }
    
    /**
     * Vector subtraction
     */
    inline const Vector2 operator-(const Vector2& rOther) const
    {
      return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
    }
    
    /**
     * In-place scalar division
     */
    inline void operator/=(T scalar)
    {
      m_Values[0] /= scalar;
      m_Values[1] /= scalar;
    }
    
    /**
     * Divides a vector by the scalar
     */
    inline const Vector2 operator/(T scalar) const
    {
      return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
    }
        
    /**
     * Vector dot-product
     */
    inline kt_double operator*(const Vector2& rOther) const
    {
      return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
    }    
    
    /**
     * Scales the vector by the given scalar
     */
    inline const Vector2 operator*(T scalar) const
    {
      return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
    }
    
    /**
     * Subtract the vector by the given scalar
     */
    inline const Vector2 operator-(T scalar) const
    {
      return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
    }

    /**
     * In-place scalar multiplication
     */
    inline void operator*=(T scalar)
    {
      m_Values[0] *= scalar;
      m_Values[1] *= scalar;
    }
    
    /**
     * Equality operator
     */
    inline kt_bool operator==(const Vector2& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
    }
    
    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Vector2& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
    }
    
    /**
     * Less than operator
     * @param rOther vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates and then
     * corresponding y coordinates
     */
    inline kt_bool operator<(const Vector2& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
      {
        return true;
      }
      else if (m_Values[0] > rOther.m_Values[0])
      {
        return false;
      }
      else 
      {
        return (m_Values[1] < rOther.m_Values[1]);
      }
    }

    /**
     * Write vector onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator<<(std::ostream& rStream, const Vector2& rVector)
    {
      rStream << rVector.ToString().ToCString();
      return rStream;
    }
    
  private:
    T m_Values[2];
  }; // class Vector2<T>

  /**
   * Type declaration of kt_int32s Vector2 as Vector2i
   */
  typedef Vector2<kt_int32s> Vector2i;

  /**
   * Type declaration of kt_int32u Vector2 as Vector2iu
   */
  typedef Vector2<kt_int32u> Vector2iu;

  /**
   * Type declaration of kt_double Vector2 as Vector2d
   */
  typedef Vector2<kt_double> Vector2d;
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a 3-dimensional vector (x, y, z)
   */
  template<typename T>
  class Vector3
  {
  public:
    /**
     * Vector at the origin
     */
    Vector3()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
      m_Values[2] = 0;
    }

    /**
     * Vector at the given location
     * @param x x
     * @param y y
     * @param z z
     */
    Vector3(T x, T y, T z)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
    }

    /**
     * Constructs a 3D vector from the given 2D vector with a z-component of 0
     * @param rVector 2D vector
     */
    Vector3(const Vector2<T>& rVector)
    {
      m_Values[0] = rVector.GetX();
      m_Values[1] = rVector.GetY();
      m_Values[2] = 0.0;
    }

    /**
     * Copy constructor
     */
    Vector3(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
    }

  public:
    /**
     * Gets the x-component of this vector
     * @return the x-component
     */
    inline const T& GetX() const 
    {
      return m_Values[0]; 
    }

    /**
     * Sets the x-component of this vector
     * @param x x-component
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x; 
    }

    /**
     * Gets the y-component of this vector
     * @return the y-component
     */
    inline const T& GetY() const 
    {
      return m_Values[1]; 
    }

    /**
     * Sets the y-component of this vector
     * @param y y-component
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y; 
    }

    /**
     * Gets the z-component of this vector
     * @return the z-component
     */
    inline const T& GetZ() const
    {
      return m_Values[2]; 
    }

    /**
     * Sets the z-component of this vector
     * @param z z-component
     */
    inline void SetZ(const T& z)
    {
      m_Values[2] = z; 
    } 

    /**
     * Gets a 2D version of this vector (ignores z-component)
     * @return 2D version of this vector
     */
    inline Vector2<T> GetAsVector2() const
    {
      return Vector2<T>(m_Values[0], m_Values[1]);
    }

    /**
     * Floor vector operator
     * @param rOther vector
     */
    inline void MakeFloor(const Vector3& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] < m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Ceiling vector operator
     * @param rOther vector
     */
    inline void MakeCeil(const Vector3& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] > m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] > m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]) + math::Square(m_Values[2]);
    }

    /**
     * Returns the length of the vector.
     * @return length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }
    
    /** 
     * Normalize the vector
     */ 
    inline void Normalize()
    {
      kt_double length = Length();
      if (length > 0.0)
      {
        kt_double inversedLength = 1.0 / length;

        m_Values[0] *= inversedLength;
        m_Values[1] *= inversedLength;
        m_Values[2] *= inversedLength;
      }
    }

    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline const String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetX()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetY()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetZ()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Vector3& operator=(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];

      return *this;
    }

    /**
     * Vector addition
     */
    inline const Vector3 operator+(const Vector3& rOther) const
    {
      return Vector3(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1], m_Values[2] + rOther.m_Values[2]);
    }

    /**
     * Vector-scalar addition
     */
    inline const Vector3 operator+(kt_double scalar) const
    {
      return Vector3(m_Values[0] + scalar, m_Values[1] + scalar, m_Values[2] + scalar);
    }

    /**
     * Vector subtraction
     */
    inline const Vector3 operator-(const Vector3& rOther) const
    {
      return Vector3(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1], m_Values[2] - rOther.m_Values[2]);
    }

    /**
     * Vector-scalar subtraction
     */
    inline const Vector3 operator-(kt_double scalar) const
    {
      return Vector3(m_Values[0] - scalar, m_Values[1] - scalar, m_Values[2] - scalar);
    }

    /**
     * Scales the vector by the given scalar
     */
    inline const Vector3 operator*(T scalar) const
    {
      return Vector3(m_Values[0] * scalar, m_Values[1] * scalar, m_Values[2] * scalar);
    }

    /**
     * In-place vector addition
     */
    inline Vector3& operator+=(const Vector3& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
      m_Values[2] += rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place vector subtraction
     */    
    inline Vector3& operator-=(const Vector3& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
      m_Values[2] -= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise vector multiplication
     * @param rOther vector
     * @return this vector after multiplying each component with the corresponding component in the other vector
     */
    inline Vector3& operator*=(const Vector3& rOther)
    {
      m_Values[0] *= rOther.m_Values[0];
      m_Values[1] *= rOther.m_Values[1];
      m_Values[2] *= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise vector division
     * @param rOther vector
     * @return this vector after dividing each component with the corresponding component in the other vector
     */
    inline Vector3& operator/=(const Vector3& rOther)
    {
      m_Values[0] /= rOther.m_Values[0];
      m_Values[1] /= rOther.m_Values[1];
      m_Values[2] /= rOther.m_Values[2];

      return *this;
    }

    /**
     * In-place component-wise scalar addition
     * @param rValue value to add to each component
     * @return this vector after adding each component with the given value
     */
    inline Vector3& operator+=(const T& rValue)
    {
      m_Values[0] += rValue;
      m_Values[1] += rValue;
      m_Values[2] += rValue;

      return *this;
    }

    /**
     * In-place component-wise scalar subtraction
     * @param rValue value to subtract from each component
     * @return this vector after subtracting the given value from each component
     */
    inline Vector3& operator-=(const T& rValue)
    {
      m_Values[0] -= rValue;
      m_Values[1] -= rValue;
      m_Values[2] -= rValue;

      return *this;
    }

    /**
     * In-place vector-scalar multiplication
     */    
    inline Vector3& operator*=(const T& rValue)
    {
      m_Values[0] *= rValue;
      m_Values[1] *= rValue;
      m_Values[2] *= rValue;

      return *this;
    }

    /**
     * In-place component-wise vector-scalar division
     * @param rValue value to divide from each component
     * @return this vector after dividing each component with the given value
     */
    inline Vector3& operator/=(const T& rValue)
    {
      m_Values[0] /= rValue;
      m_Values[1] /= rValue;
      m_Values[2] /= rValue;

      return *this;
    }

    /** 
     * Vector cross product
     * @param rOther vector
     * @return cross product of this vector and given vector
     */
    inline const Vector3 operator^(const Vector3& rOther) const
    {
      return Vector3(
        m_Values[1] * rOther.m_Values[2] - m_Values[2] * rOther.m_Values[1],
        m_Values[2] * rOther.m_Values[0] - m_Values[0] * rOther.m_Values[2] ,
        m_Values[0] * rOther.m_Values[1] - m_Values[1] * rOther.m_Values[0]);
    }

    /**
     * Less than operator
     * @param rOther other vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates, then
     * corresponding y coordinates, and then corresponding z coordinates
     */
    inline kt_bool operator<(const Vector3& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
      {
        return true;
      }
      else if (m_Values[0] > rOther.m_Values[0]) 
      {
        return false;
      }
      else if (m_Values[1] < rOther.m_Values[1])
      {
        return true;
      }
      else if (m_Values[1] > rOther.m_Values[1]) 
      {
        return false;
      }
      else
      {
        return (m_Values[2] < rOther.m_Values[2]);
      }
    }

    /**
     * Equality operator
     */
    inline kt_bool operator==(const Vector3& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1] && m_Values[2] == rOther.m_Values[2]);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Vector3& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1] || m_Values[2] != rOther.m_Values[2]);
    }

    /**
     * Write vector onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Vector3& rVector)
    {
      rStream << rVector.ToString().ToCString();
      return rStream;
    }

  private:
    T m_Values[3];
  }; // class Vector3<T>

  /**
   * Type declaration of kt_int32s Vector3 as Vector3i
   */
  typedef Vector3<kt_int32s> Vector3i;

  /**
   * Type declaration of kt_int32u Vector3 as Vector3iu
   */
  typedef Vector3<kt_int32u> Vector3iu;

  /**
   * Type declaration of kt_double Vector3 as Vector3d
   */
  typedef Vector3<kt_double> Vector3d;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a 4-dimensional vector (x, y, z, w)
   */
  template<typename T>
  class Vector4
  {
  public:
    /**
     * Vector at the origin
     */
    Vector4()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
      m_Values[2] = 0;
      m_Values[3] = 0;
    }

    /**
     * Vector at the given location
     * @param x x
     * @param y y
     * @param z z
     * @param w w
     */
    Vector4(T x, T y, T z, T w)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
      m_Values[3] = w;
    }

    /**
     * Copy constructor
     */
    Vector4(const Vector4& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
      m_Values[3] = rOther.m_Values[3];
    }

  public:
    /**
     * Gets the x-component of this vector
     * @return x-component
     */
    inline const T& GetX() const 
    {
      return m_Values[0]; 
    }

    /**
     * Sets the x-component of this vector
     * @param x x
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x; 
    }

    /**
     * Gets the y-component of this vector
     * @return y-component
     */
    inline const T& GetY() const 
    {
      return m_Values[1]; 
    }

    /**
     * Sets the y-component of this vector
     * @param y y
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }

    /**
     * Gets the z-component of this vector
     * @return z-component
     */
    inline const T& GetZ() const
    {
      return m_Values[2];
    }

    /**
     * Sets the z-component of this vector
     * @param z z
     */
    inline void SetZ(const T& z)
    {
      m_Values[2] = z;
    } 

    /**
     * Gets the w-component of this vector
     * @return w-component
     */
    inline const T& GetW() const
    {
      return m_Values[3];
    }

    /**
     * Sets the w-component of this vector
     * @param w w
     */
    inline void SetW(const T& w)
    {
      m_Values[3] = w;
    } 

    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline const String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetX()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetY()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetZ()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetW()));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Vector4& operator = (const Vector4& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
      m_Values[3] = rOther.m_Values[3];

      return *this;
    }

    /**
     * Less than operator
     * @param rOther vector
     * @return true if left vector is 'less' than right vector by comparing corresponding x coordinates, then
     * corresponding y coordinates, then corresponding z coordinates, and then corresponding w coordinates
     */
    inline kt_bool operator<(const Vector4& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
      {
        return true;
      }
      else if (m_Values[0] > rOther.m_Values[0]) 
      {
        return false;
      }
      else if (m_Values[1] < rOther.m_Values[1])
      {
        return true;
      }
      else if (m_Values[1] > rOther.m_Values[1]) 
      {
        return false;
      }
      else if (m_Values[2] < rOther.m_Values[2])
      {
        return true;
      }
      else if (m_Values[2] > rOther.m_Values[2])
      {
        return false;
      }
      else
      {
        return (m_Values[3] < rOther.m_Values[3]);
      }
    }

    /**
     * Equality operator
     */
    inline kt_bool operator==(const Vector4& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1] && m_Values[2] == rOther.m_Values[2] && m_Values[3] == rOther.m_Values[3]);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Vector4& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1] || m_Values[2] != rOther.m_Values[2] || m_Values[3] != rOther.m_Values[3]);
    }

    /**
     * Write vector onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator<<(std::ostream& rStream, const Vector4& rVector)
    {
      rStream << rVector.ToString().ToCString();
      return rStream;
    }

  private:
    T m_Values[4];
  }; // class Vector4<T>

  /**
   * Type declaration of kt_int32s Vector4 as Vector4i
   */
  typedef Vector4<kt_int32s> Vector4i;

  /**
   * Type declaration of kt_int32u Vector4 as Vector4iu
   */
  typedef Vector4<kt_int32u> Vector4iu;

  /**
   * Type declaration of double Vector4 as Vector4d
   */
  typedef Vector4<kt_double> Vector4d;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines an orientation as a quaternion rotation using the positive Z axis as the zero reference.
   * <BR>
   * Q = w + ix + jy + kz <BR>
   * w = c_1 * c_2 * c_3 - s_1 * s_2 * s_3 <BR>
   * x = s_1 * s_2 * c_3 + c_1 * c_2 * s_3 <BR>
   * y = s_1 * c_2 * c_3 + c_1 * s_2 * s_3 <BR>
   * z = c_1 * s_2 * c_3 - s_1 * c_2 * s_3 <BR>
   * where <BR>
   * c_1 = cos(theta/2) <BR>
   * c_2 = cos(phi/2) <BR>
   * c_3 = cos(psi/2) <BR>
   * s_1 = sin(theta/2) <BR>
   * s_2 = sin(phi/2) <BR>
   * s_3 = sin(psi/2) <BR>
   * and <BR>
   * theta is the angle of rotation about the Y axis measured from the Z axis. <BR>
   * phi is the angle of rotation about the Z axis measured from the X axis. <BR>
   * psi is the angle of rotation about the X axis measured from the Y axis. <BR>
   * (All angles are right-handed.)
   */
  class KARTO_EXPORT Quaternion
  {
  public:
    /**
     * Quaternion with default (x=0, y=0, z=0, w=1) values
     */
    inline Quaternion()
    {
      m_Values[0] = 0.0;
      m_Values[1] = 0.0;
      m_Values[2] = 0.0;
      m_Values[3] = 1.0; 
    }

    /**
     * Quaternion using given x, y, z, w values.
     * @param x x
     * @param y y
     * @param z z
     * @param w w
     */
    inline Quaternion(kt_double x, kt_double y, kt_double z, kt_double w)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
      m_Values[3] = w;
    }

    /**
    * Quaternion from given Vector4d
     */
    inline Quaternion(const Vector4d& rVector)
    {
      m_Values[0] = rVector.GetX();
      m_Values[1] = rVector.GetY();
      m_Values[2] = rVector.GetZ();
      m_Values[3] = rVector.GetW();
    }

    /**
     * Copy constructor
     */
    inline Quaternion(const Quaternion& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
      m_Values[3] = rOther.m_Values[3];
    }

  public:
    /**
     * Returns the x-value
     * @return the x-value of this quaternion
     */
    inline kt_double GetX() const
    {
      return m_Values[0]; 
    }

    /**
     * Sets the x-value
     * @param x x-value
     */
    inline void SetX(kt_double x)
    {
      m_Values[0] = x; 
    }

    /**
     * Returns the y-value
     * @return the y-value of this quaternion
     */
    inline kt_double GetY() const
    {
      return m_Values[1]; 
    }

    /**
     * Sets the y-value
     * @param y y-value
     */
    inline void SetY(kt_double y)
    {
      m_Values[1] = y; 
    }

    /**
     * Returns the z-value
     * @return the z-value of this quaternion
     */
    inline kt_double GetZ() const
    {
      return m_Values[2]; 
    }

    /**
     * Sets the z-value
     * @param z z-value
     */
    inline void SetZ(kt_double z)
    {
      m_Values[2] = z; 
    }

    /**
     * Returns the w-value
     * @return the w-value of this quaternion
     */
    inline kt_double GetW() const
    {
      return m_Values[3]; 
    }

    /**
     * Sets the w-value
     * @param w w-value
     */
    inline void SetW(kt_double w)
    {
      m_Values[3] = w; 
    }

    /**
     * This quaternion as a 4D vector
     * @return 4D vector representation of this quaternion
     */
    inline const Vector4d GetAsVector4() const 
    {
      return Vector4d(m_Values[0], m_Values[1], m_Values[2], m_Values[3]);
    }

    /**
     * This quaternion in angle-axis form
     * @param rAngle output parameter angle
     * @param rAxis output parameter axis
     */
    void ToAngleAxis(kt_double& rAngle, karto::Vector3d& rAxis) const;

    /**
     * Computes the equivalent quaternion from the given angle-axis form
     */
    void FromAngleAxis(kt_double angleInRadians, const karto::Vector3d& rAxis);

    /**
     * Converts this quaternion into Euler angles
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
     * @param rYaw yaw
     * @param rPitch pitch
     * @param rRoll roll
     */
    void ToEulerAngles(kt_double& rYaw, kt_double& rPitch, kt_double& rRoll) const;

    /**
     * Set x,y,z,w values of the quaternion based on Euler angles. 
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
     * @param yaw yaw
     * @param pitch pitch
     * @param roll roll
     */
    void FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll);

    /**
     * Length of this quaternion
     * @return length of this quaternion
     */
    inline kt_double Length() const
    {
      return sqrt(Norm());
    }

    /** 
     * Returns the norm of this quaternion
     * @return norm of this quaternion
     */ 
    inline kt_double Norm() const
    {
      return m_Values[0]*m_Values[0] + m_Values[1]*m_Values[1] + m_Values[2]*m_Values[2] + m_Values[3]*m_Values[3];
    }

    /** 
     * Normalize the quaternion
     */ 
    inline void Normalize()
    {
      kt_double length = Length();
      if (length > 0.0)
      {
        kt_double inversedLength = 1.0 / length;

        m_Values[0] *= inversedLength;
        m_Values[1] *= inversedLength;
        m_Values[2] *= inversedLength;
        m_Values[3] *= inversedLength;
      }
    }

    /**
     * String representation of this quaternion
     * @return string representation of this quaternion
     */
    const String ToString() const;

  public:
    /**
     * Assignment operator
     */
    inline Quaternion& operator=(const Quaternion& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
      m_Values[3] = rOther.m_Values[3];

      return(*this);
    }

    /**
     * Quaternion multiplication; note that quaternion multiplication is not commutative (a * b != b * a)
     */
    inline const Quaternion operator*(const Quaternion& rOther) const
    {
      return Quaternion(
        rOther.m_Values[3] * m_Values[0] + rOther.m_Values[0] * m_Values[3] + rOther.m_Values[1] * m_Values[2] - rOther.m_Values[2] * m_Values[1],
        rOther.m_Values[3] * m_Values[1] - rOther.m_Values[0] * m_Values[2] + rOther.m_Values[1] * m_Values[3] + rOther.m_Values[2] * m_Values[0],
        rOther.m_Values[3] * m_Values[2] + rOther.m_Values[0] * m_Values[1] - rOther.m_Values[1] * m_Values[0] + rOther.m_Values[2] * m_Values[3],
        rOther.m_Values[3] * m_Values[3] - rOther.m_Values[0] * m_Values[0] - rOther.m_Values[1] * m_Values[1] - rOther.m_Values[2] * m_Values[2] );
    }
    
    /**
     * Equality operator
     */
    inline kt_bool operator==(const Quaternion& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1] && m_Values[2] == rOther.m_Values[2] && m_Values[3] == rOther.m_Values[3]);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Quaternion& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1] || m_Values[2] != rOther.m_Values[2] || m_Values[3] != rOther.m_Values[3]);
    }

    /**
     * Rotate a vector by this quaternion
     * @param rVector vector
     * @return result of multiplying this quaternion by the given vector
     */
    karto::Vector3d operator*(const karto::Vector3d& rVector) const
    {
      // nVidia SDK implementation
      karto::Vector3d uv, uuv; 
      karto::Vector3d qvec(m_Values[0], m_Values[1], m_Values[2]);

      uv = qvec ^ rVector;
      uuv = qvec ^ uv; 

      uv *= ( 2.0 * m_Values[3] ); 
      uuv *= 2.0; 

      return rVector + uv + uuv;
    }

    /**
     * Write this quaternion onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator<<(std::ostream& rStream, const Quaternion& rQuaternion)
    {
      rStream << rQuaternion.ToString().ToCString();
      return rStream;
    }

  private:
    kt_double m_Values[4];
  }; // class Quaternion

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a 2-dimensional bounding box.
   */
  class KARTO_EXPORT BoundingBox2
  {
  public:
    /*
     * Bounding box of maximal size
     */
    BoundingBox2();

    /**
     * Bounding box with given minimum and maximum values.
     * @param rMinimum minimum value
     * @param rMaximum maximum value
     */
    BoundingBox2(const Vector2d& rMinimum, const Vector2d& rMaximum);

  public:
    /** 
     * Get bounding box minimum
     * @return bounding box minimum
     */
    inline const Vector2d& GetMinimum() const
    { 
      return m_Minimum; 
    }

    /** 
     * Set bounding box minimum
     * @param rMinimum bounding box minimum
     */
    inline void SetMinimum(const Vector2d& rMinimum)
    {
      m_Minimum = rMinimum;
    }

    /** 
     * Get bounding box maximum
     * @return bounding box maximum
     */
    inline const Vector2d& GetMaximum() const
    { 
      return m_Maximum;
    }

    /** 
     * Set bounding box maximum
     * @param rMaximum bounding box maximum
     */
    inline void SetMaximum(const Vector2d& rMaximum)
    {
      m_Maximum = rMaximum;
    }

    /**
     * Get the size of this bounding box
     * @return bounding box size
     */
    inline Size2<kt_double> GetSize() const
    {
      Vector2d size = m_Maximum - m_Minimum;

      return Size2<kt_double>(size.GetX(), size.GetY());
    }

    /**
     * Add vector to bounding box
     * @param rPoint point
     */
    inline void Add(const Vector2d& rPoint)
    {
      m_Minimum.MakeFloor(rPoint);
      m_Maximum.MakeCeil(rPoint);
    }

    /**
     * Add given bounding box to this bounding box
     * @param rBoundingBox bounding box
     */
    inline void Add(const BoundingBox2& rBoundingBox)
    {
      Add(rBoundingBox.GetMinimum());
      Add(rBoundingBox.GetMaximum());
    }
    
    /**
     * Whether the given point is in the bounds of this box
     * @param rPoint point
     * @return whether the given point is in the bounds of this box
     */
    inline kt_bool Contains(const Vector2d& rPoint) const
    {
      return (math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
              math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
    }

    /** 
     * Checks if this bounding box intersects with the given bounding box
     * @param rOther bounding box
     * @return true if this bounding box intersects with the given bounding box
     */
    inline kt_bool Intersects(const BoundingBox2& rOther) const
    { 
      if ((m_Maximum.GetX() < rOther.m_Minimum.GetX()) || (m_Minimum.GetX() > rOther.m_Maximum.GetX()))
      {
        return false;
      }
      
      if ((m_Maximum.GetY() < rOther.m_Minimum.GetY()) || (m_Minimum.GetY() > rOther.m_Maximum.GetY()))
      {
        return false;
      }

      return true;
    }

    /** 
     * Checks if this bounding box contains the given bounding box
     * @param rOther bounding box
     * @return true if this bounding box contains the given bounding box
     */
    inline kt_bool Contains(const BoundingBox2& rOther) const
    {
      if ((m_Maximum.GetX() < rOther.m_Minimum.GetX()) || (m_Minimum.GetX() > rOther.m_Maximum.GetX()))
      {
        return false;
      }
      
      if ((m_Maximum.GetY() < rOther.m_Minimum.GetY()) || (m_Minimum.GetY() > rOther.m_Maximum.GetY()))
      {
        return false;
      }
      
      if ((m_Minimum.GetX() <= rOther.m_Minimum.GetX()) && (rOther.m_Maximum.GetX() <= m_Maximum.GetX()) && 
          (m_Minimum.GetY() <= rOther.m_Minimum.GetY()) && (rOther.m_Maximum.GetY() <= m_Maximum.GetY()))
      {
        return true;
      }

      return false;
    }

  private:
    Vector2d m_Minimum;
    Vector2d m_Maximum;
  }; // class BoundingBox2

  /**
   * Defines a 3-dimensional bounding box
   */
  class KARTO_EXPORT BoundingBox3
  {
  public:
    BoundingBox3();
    virtual ~BoundingBox3();

  public:
    /** 
     * Get bounding box minimum
     * return bounding box minimum
     */
    inline const Vector3d& GetMinimum() const
    { 
      return m_Minimum; 
    }

    /** 
     * Set bounding box minimum
     * @param rMinimum bounding box minimum
     */
    inline void SetMinimum(const Vector3d& rMinimum)
    {
      m_Minimum = rMinimum;
    }

    /** 
     * Get bounding box maximum
     * @return bounding box maximum
     */
    inline const Vector3d& GetMaximum() const
    { 
      return m_Maximum;
    }

    /** 
     * Set bounding box maximum
     * @param rMaximum bounding box maximum
     */
    inline void SetMaximum(const Vector3d& rMaximum)
    {
      m_Maximum = rMaximum;
    }

  private:
    Vector3d m_Minimum;
    Vector3d m_Maximum;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Stores x, y, width and height that represents the location and size of a rectangle.
   * Note that (x, y) is at bottom-left in the mapper!
   */
  template<typename T>
  class Rectangle2
  {
  public:
    /**
     * Rectangle with all parameters set to 0
     */
    Rectangle2()
    {
    }

    /**
     * Rectangle with given parameters
     * @param x x-coordinate of left edge of rectangle
     * @param y y-coordinate of bottom edge of rectangle
     * @param width width of rectangle
     * @param height height of rectangle
     */
    Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y)
      , m_Size(width, height)
    {
    }

    /**
     * Rectangle with given position and size
     * @param rPosition (x,y)-coordinate of rectangle
     * @param rSize size of the rectangle
     */
    Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
      : m_Position(rPosition)
      , m_Size(rSize)
    {
    }

    /**
     * Rectangle with given top-left and bottom-right coordinates
     * @param rTopLeft top-left (x,y)-coordinate of rectangle
     * @param rBottomRight bottom-right (x,y)-coordinate of rectangle
     */
    Rectangle2(const Vector2<T>& rTopLeft, const Vector2<T>& rBottomRight)
      : m_Position(rTopLeft)
      , m_Size(rBottomRight.GetX() - rTopLeft.GetX(), rBottomRight.GetY() - rTopLeft.GetY())
    {
    }

    /**
     * Copy constructor
     */
    Rectangle2(const Rectangle2& rOther)
      : m_Position(rOther.m_Position)
      , m_Size(rOther.m_Size)
    {
    }

  public:
    /**
     * Gets the x-coordinate of the left edge of this rectangle
     * @return the x-coordinate of the left edge of this rectangle
     */
    inline T GetX() const
    {
      return m_Position.GetX();
    }
    
    /**
     * Sets the x-coordinate of the left edge of this rectangle
     * @param rX new x-coordinate for the left edge
     */
    inline void SetX(const T& rX) 
    {
      m_Position.SetX(rX);
    }

    /**
     * Gets the y-coordinate of the bottom edge of this rectangle
     * @return the y-coordinate of the bottom edge of this rectangle
     */
    inline T GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate of the bottom edge of this rectangle
     * @param rY new y-coordinate for the bottom edge
     */
    inline void SetY(const T& rY)
    {
      m_Position.SetY(rY);
    }
    
    /**
     * Gets the width of this rectangle
     * @return the width of this rectangle
     */
    inline T GetWidth() const
    {
      return m_Size.GetWidth();
    }

    /**
     * Sets the width of this rectangle
     * @param rWidth new width
     */
    inline void SetWidth(const T& rWidth)
    {
      m_Size.SetWidth(rWidth);
    }
    
    /**
     * Gets the height of this rectangle
     * @return the height of this rectangle
     */
    inline T GetHeight() const
    {
      return m_Size.GetHeight();
    }

    /**
     * Sets the height of this rectangle
     * @param rHeight new height
     */
    inline void SetHeight(const T& rHeight)
    {
      m_Size.SetHeight(rHeight);
    }
    
    /**
     * Gets the position of this rectangle
     * @return the position of this rectangle
     */    
    inline const Vector2<T>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position of this rectangle
     * @param rX new x position
     * @param rY new y position
     */    
    inline void SetPosition(const T& rX, const T& rY)
    {
      m_Position = Vector2<T>(rX, rY);
    }

    /**
     * Sets the position of this rectangle
     * @param rPosition new position
     */    
    inline void SetPosition(const Vector2<T>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Gets the size of this rectangle
     * @return the size of this rectangle
     */    
    inline const Size2<T>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets the size of this rectangle
     * @param rSize new size
     */    
    inline void SetSize(const Size2<T>& rSize) 
    {
      m_Size = rSize;
    }

    /**
     * Get the left coordinate of this rectangle
     * @return left coordinate of this rectangle
     */
    inline T GetLeft() const
    {
      return m_Position.GetX();
    }

    /**
     * Set the left coordinate of this rectangle
     * param rLeft new left coordinate of this rectangle
     */
    inline void SetLeft(const T& rLeft)
    {
      m_Position.SetX(rLeft);
    }

    /**
     * Get the top coordinate of this rectangle
     * @return top coordinate of this rectangle
     */
    inline T GetTop() const
    {
      return m_Position.GetY();
    }

    /**
     * Set the top coordinate of this rectangle
     * param rTop new top coordinate of this rectangle
     */
    inline void SetTop(const T& rTop)
    {
      m_Position.SetY(rTop);
    }

    /**
     * Get the right coordinate of this rectangle
     * @return right coordinate of this rectangle
     */
    inline T GetRight() const
    {
      return m_Position.GetX() + m_Size.GetWidth();
    }

    /**
     * Set the right coordinate of this rectangle
     * param rRight new right coordinate of this rectangle
     */
    inline void SetRight(const T& rRight)
    {
      m_Size.SetWidth(rRight - m_Position.GetX());
    }

    /**
     * Get the bottom coordinate of this rectangle
     * @return bottom coordinate of this rectangle
     */
    inline T GetBottom() const
    {
      return m_Position.GetY() + m_Size.GetHeight();
    }

    /**
     * Set the bottom coordinate of this rectangle
     * param rBottom new bottom coordinate of this rectangle
     */
    inline void SetBottom(const T& rBottom)
    {
      m_Size.SetHeight(rBottom - m_Position.GetY());
    }

    /**
     * Get the top-left coordinate of this rectangle
     * @return top-left coordinate of this rectangle
     */
    inline Vector2<T> GetTopLeft()
    {
      return Vector2<T>(GetLeft(), GetTop());
    }

    /**
     * Get the top-right coordinate of this rectangle
     * @return top-right coordinate of this rectangle
     */
    inline Vector2<T> GetTopRight()
    {
      return Vector2<T>(GetRight(), GetTop());
    }

    /**
     * Get the bottom-left coordinate of this rectangle
     * @return bottom-left coordinate of this rectangle
     */
    inline Vector2<T> GetBottomLeft()
    {
      return Vector2<T>(GetLeft(), GetBottom());
    }

    /**
     * Get the bottom-right coordinate of this rectangle
     * @return bottom-right coordinate of this rectangle
     */
    inline Vector2<T> GetBottomRight()
    {
      return Vector2<T>(GetRight(), GetBottom());
    }

    /**
     * Gets the center of this rectangle
     * @return the center of this rectangle
     */
    inline const Vector2<T> GetCenter() const
    {
      return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5, m_Position.GetY() + m_Size.GetHeight() * 0.5);
    }

    /**
     * Whether this rectangle contains the given rectangle
     * @param rOther rectangle
     * @return true if this rectangle contains the given rectangle, false otherwise
     */
    inline kt_bool Contains(const Rectangle2<T>& rOther)
    {
      T l1 = m_Position.GetX();
      T r1 = m_Position.GetX();
      if (m_Size.GetWidth() < 0)
        l1 += m_Size.GetWidth();
      else
        r1 += m_Size.GetWidth();
      if (l1 == r1) // null rect
        return false;
      
      T l2 = rOther.m_Position.GetX();
      T r2 = rOther.m_Position.GetX();
      if (rOther.m_Size.GetWidth() < 0)
        l2 += rOther.m_Size.GetWidth();
      else
        r2 += rOther.m_Size.GetWidth();
      if (l2 == r2) // null rect
        return false;

      if (l2 < l1 || r2 > r1)
        return false;

      T t1 = m_Position.GetY();
      T b1 = m_Position.GetY();
      if (m_Size.GetHeight() < 0)
        t1 += m_Size.GetHeight();
      else
        b1 += m_Size.GetHeight();
      if (t1 == b1) // null rect
        return false;

      T t2 = rOther.m_Position.GetY();
      T b2 = rOther.m_Position.GetY();
      if (rOther.m_Size.GetHeight() < 0)
        t2 += rOther.m_Size.GetHeight();
      else
        b2 += rOther.m_Size.GetHeight();
      if (t2 == b2) // null rect
        return false;

      if (t2 < t1 || b2 > b1)
        return false;

      return true;
    }

  public:
    /**
     * Assignment operator
     */
    Rectangle2& operator = (const Rectangle2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Size = rOther.m_Size;
      
      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Rectangle2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Rectangle2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
    }

  private:
    Vector2<T> m_Position;
    Size2<T> m_Size;
  }; // class Rectangle2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Pose3;

  /**
   * Defines a pose (position and heading) in 2-dimensional space.
   */
  class KARTO_EXPORT Pose2
  {
  public:
    /**
     * Pose at the origin with a heading of 0
     */
    Pose2();

    /**
     * Pose with given position and heading
     * @param rPosition position
     * @param heading heading
     **/
    Pose2(const Vector2d& rPosition, kt_double heading = 0);

    /**
     * Pose with given position and heading
     * @param x x-coordinate
     * @param y y-coordinate
     * @param heading heading
     **/
    Pose2(kt_double x, kt_double y, kt_double heading);

    /**
     * Pose2 object from a Pose3 (ignores z-coordinate)
     */
    Pose2(const Pose3& rPose);

    /**
     * Copy constructor
     */
    Pose2(const Pose2& rOther);

  public:
    /**
     * Returns the x-coordinate
     * @return the x-coordinate of this pose
     */
    inline kt_double GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate
     * @param x new x-coordinate
     */
    inline void SetX(kt_double x)
    {
      m_Position.SetX(x);
    }

    /**
     * Returns the y-coordinate
     * @return the y-coordinate of this pose
     */
    inline kt_double GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate
     * @param y new y-coordinate
     */
    inline void SetY(kt_double y)
    {
      m_Position.SetY(y);
    }

    /**
     * Returns the position
     * @return the position of this pose
     */
    inline const Vector2d& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position
     * @param rPosition new position
     */
    inline void SetPosition(const Vector2d& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Returns the heading of the pose (in radians)
     * @return the heading of this pose
     */
    inline kt_double GetHeading() const
    {
      return m_Heading;
    }

    /**
     * Sets the heading
     * @param heading new heading
     */
    inline void SetHeading(kt_double heading)
    {
      m_Heading = heading;
    }

    /** 
     * Return the squared distance between this pose and the given pose
     * @param rOther pose
     * @return squared distance between this pose and the given pose
     */
    inline kt_double SquaredDistance(const Pose2& rOther) const
    {
      return m_Position.SquaredDistance(rOther.m_Position);
    }
    
    /**
     * Return the angle from this pose to the given vector
     * @param rVector vector
     * @return angle to given vector
     */
    inline kt_double AngleTo(const Vector2d& rVector) const
    {
      kt_double angle = atan2(rVector.GetY() - GetY(), rVector.GetX() - GetX());
      return karto::math::NormalizeAngle(angle - GetHeading());
    }
    
    /**
     * Returns a string representation of this pose
     * @param precision precision, default 4
     * @return string representation of this pose
     */
    inline const String ToString(kt_int32u precision = 4) const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(m_Position.GetX(), precision));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(m_Position.GetY(), precision));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(m_Heading, precision));
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Pose2& operator = (const Pose2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Heading = rOther.m_Heading;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator==(const Pose2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator!=(const Pose2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
    }

    /**
     * In-place Pose2 addition
     */
    inline void operator+=(const Pose2& rOther)
    {
      m_Position += rOther.m_Position;
      m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
    }

    /**
     * Pose2 addition
     */
    inline Pose2 operator+(const Pose2& rOther) const
    {
      return Pose2(m_Position + rOther.m_Position, math::NormalizeAngle(m_Heading + rOther.m_Heading));
    }

    /**
     * Pose2 subtraction
     */
    inline Pose2 operator-(const Pose2& rOther) const
    {
      return Pose2(m_Position - rOther.m_Position, math::NormalizeAngle(m_Heading - rOther.m_Heading));
    }

    /**
     * Write this pose onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Pose2& rPose)
    {
      rStream << rPose.ToString();
      return rStream;
    }
    
  private:
    Vector2d m_Position;

    kt_double m_Heading;
  }; // class Pose2

  /**
   * Type declaration of Pose2 List
   */
  typedef List<Pose2> Pose2List;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a position and orientation in 3-dimensional space.
   * Karto uses a right-handed coordinate system with X, Y as the 2-D ground plane and X is forward and Y is left.
   * Values in Vector3 used to define position must have units of meters.
   * The value of angle when defining orientation in two dimensions must be in units of radians.
   * The definition of orientation in three dimensions uses quaternions.  
   */
  class Pose3
  {
  public:
    /**
     * Pose at the origin and quaternion with components (0, 0, 0, 1)
     */
    Pose3()
    {
    }

    /**
     * Pose object at the given position.
     * @param rPosition vector
     */
    Pose3(const Vector3d& rPosition)
      : m_Position(rPosition)
    {
    }

    /**
     * Pose at the given position and orientation.
     * @param rPosition position vector
     * @param rOrientation quaternion orientation
     */
    Pose3(const Vector3d& rPosition, const karto::Quaternion& rOrientation)
      : m_Position(rPosition)
      , m_Orientation(rOrientation)
    {
    }

    /**
     * Copy constructor
     */
    Pose3(const Pose3& rOther)
      : m_Position(rOther.m_Position)
      , m_Orientation(rOther.m_Orientation)
    {
    }

    /**
     * Constructs a pose object from a Pose2.
     * @param rPose pose
     */
    Pose3(const Pose2& rPose)
    {
      m_Position = Vector3d(rPose.GetX(), rPose.GetY(), 0.0);
      m_Orientation.FromEulerAngles(rPose.GetHeading(), 0.0, 0.0);
    }

  public:
    /**
     * Gets the position of this pose
     * @return position of this pose
     */
    inline const Vector3d& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position of this pose
     * @param rPosition new position
     */
    inline void SetPosition(const Vector3d& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Gets the orientation quaternion of this pose
     * @return orientation quaternion
     */
    inline const Quaternion& GetOrientation() const
    {
      return m_Orientation;
    }

    /**
     * Sets the orientation quaternion of this pose
     * @param rOrientation orientation quaternion
     */
    inline void SetOrientation(const Quaternion& rOrientation)
    {
      m_Orientation = rOrientation;
    }

    /**
     * Returns a string representation of this pose
     * @return string representation of this pose
     */
    inline const String ToString() const
    {
      String valueString;
      valueString.Append(GetPosition().ToString());
      valueString.Append(" ");
      valueString.Append(GetOrientation().ToString());
      return valueString;
    }
    
  public:
    /**
     * Assignment operator
     */
    inline Pose3& operator = (const Pose3& rOther)
    {
      m_Position = rOther.m_Position;
      m_Orientation = rOther.m_Orientation;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Pose3& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Orientation == rOther.m_Orientation);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Pose3& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Orientation != rOther.m_Orientation);
    }

    /**
     * Write pose onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Pose3& rPose)
    {
      rStream << rPose.ToString();
      return rStream;
    }

  private:
    Vector3d m_Position;
    Quaternion m_Orientation;
  }; // class Pose3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Defines a 3x3 matrix
   */
  class Matrix3
  {
  public:
    /**
     * Matrix with all elements as 0
     */
    Matrix3()
    {
      Clear();
    }

    /**
     * Copy constructor
     */
    inline Matrix3(const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
    }

  public:
    /**
     * Sets this matrix to the identity matrix
     */
    void SetToIdentity()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));

      for (kt_int32s i = 0; i < 3; i++)
      {
        m_Matrix[i][i] = 1.0;
      }
    }

    /**
     * Sets this matrix to zero matrix
     */
    void Clear()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));
    }

    /**
     * Sets this matrix to be the rotation matrix of a rotation around the given axis
     * @param x x-coordinate of axis
     * @param y y-coordinate of axis
     * @param z z-coordinate of axis
     * @param radians amount of rotation
     */
    void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
    {
      kt_double cosRadians = cos(radians);
      kt_double sinRadians = sin(radians);
      kt_double oneMinusCos = 1.0 - cosRadians;

      kt_double xx = x * x;
      kt_double yy = y * y;
      kt_double zz = z * z;

      kt_double xyMCos = x * y * oneMinusCos;
      kt_double xzMCos = x * z * oneMinusCos;
      kt_double yzMCos = y * z * oneMinusCos;

      kt_double xSin = x * sinRadians;
      kt_double ySin = y * sinRadians;
      kt_double zSin = z * sinRadians;

      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
      m_Matrix[0][1] = xyMCos - zSin;
      m_Matrix[0][2] = xzMCos + ySin;

      m_Matrix[1][0] = xyMCos + zSin;
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
      m_Matrix[1][2] = yzMCos - xSin;

      m_Matrix[2][0] = xzMCos - ySin;
      m_Matrix[2][1] = yzMCos + xSin;
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
    }

    /**
     * Returns transposed version of this matrix
     * @return transposed matrix
     */
    Matrix3 Transpose() const
    {
      Matrix3 transpose;

      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          transpose.m_Matrix[row][col] = m_Matrix[col][row];
        }
      }

      return transpose;
    }

    /**
     * Returns the inverse of the matrix
     * @return matrix inverse
     */
    Matrix3 Inverse() const
    {
      Matrix3 kInverse = *this;
      kt_bool haveInverse = InverseFast(kInverse, 1e-14);
      if (haveInverse == false)
      {
        assert(false);
      }
      return kInverse;
    }
    
    /**
     * Returns a string representation of this matrix
     * @return string representation of this matrix
     */
    inline const String ToString() const
    {
      String valueString;

      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          valueString.Append(StringHelper::ToString(m_Matrix[row][col]));
          valueString.Append(" ");
        }
      }
      
      return valueString;
    }

  public:
    /**
     * Assignment operator
     */
    inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
      return *this;
    }

    /**
     * Matrix element access, allows use of construct mat(r, c)
     * @param row row
     * @param column column
     * @return reference to mat(r,c)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      return m_Matrix[row][column];
    }

    /**
     * Read-only matrix element access, allows use of construct mat(r, c)
     * @param row row
     * @param column column
     * @return element at mat(r,c)
     */
    inline kt_double operator()(kt_int32u row, kt_int32u column) const
    {
      return m_Matrix[row][column];
    }

    /**
     * Matrix multiplication
     */
    Matrix3 operator*(const Matrix3& rOther) const
    {
      Matrix3 product;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] + m_Matrix[row][1]*rOther.m_Matrix[1][col] + m_Matrix[row][2]*rOther.m_Matrix[2][col];
        }
      }

      return product;
    }

    /**
     * Matrix3 and Pose2 multiplication: matrix * pose [3x3 * 3x1 = 3x1]
     */
    inline Pose2 operator*(const Pose2& rPose2) const
    {
      Pose2 pose2;

      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] * rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] * rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] * rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

      return pose2;
    }

    /**
     * In-place matrix addition
     */
    inline void operator+=(const Matrix3& rkMatrix)
    {
      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
        }
      }
    }
    
    /**
     * Equality operator
     */
    inline kt_bool operator==(const Matrix3& rkMatrix)
    {
      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          if (math::DoubleEqual(m_Matrix[row][col], rkMatrix.m_Matrix[row][col]) == false)
          {
            return false;
          }
        }
      }
      
      return true;
    }
    
    /**
     * Write matrix onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Matrix3& rMatrix)
    {
      rStream << rMatrix.ToString();
      return rStream;
    }

  private:
    /**
     * Internal helper method for inverse matrix calculation
     * This code is from the OgreMatrix3 class
     * @param rkInverse output parameter
     * @param fTolerance tolerance
     * @return whether this matrix was successfully inverted; if so,
     * the inverse will be in rkInverse
     */
    kt_bool InverseFast(Matrix3& rkInverse, kt_double fTolerance = KT_TOLERANCE) const
    {
      // Invert a 3x3 using cofactors.  This is about 8 times faster than
      // the Numerical Recipes code which uses Gaussian elimination.
      rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
      rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
      rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
      rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
      rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
      rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
      rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
      rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
      rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];
      
      kt_double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] + m_Matrix[0][1]*rkInverse.m_Matrix[1][0]+ m_Matrix[0][2]*rkInverse.m_Matrix[2][0];
      
      if (fabs(fDet) <= fTolerance)
      {
        return false;
      }
      
      kt_double fInvDet = 1.0/fDet;
      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          rkInverse.m_Matrix[row][col] *= fInvDet;
        }
      }
      
      return true;
    }
    
  private:
    kt_double m_Matrix[3][3];
  }; // class Matrix3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Color
  {
  public:
    /**
      * Black color
      */
    Color()
      : m_Red(0.0)
      , m_Green(0.0)
      , m_Blue(0.0)
      , m_Alpha(1.0)
    {
    }

    /**
      * Copy constructor
      */
    Color(const Color& rOther)
      : m_Red(rOther.m_Red)
      , m_Green(rOther.m_Green)
      , m_Blue(rOther.m_Blue)
      , m_Alpha(rOther.m_Alpha)
    {
    }

    /**
     * Color with given RGBA values
     * @param red red
     * @param green green
     * @param blue blue
     * @param alpha alpha
     */
    Color(kt_double red, kt_double green, kt_double blue, kt_double alpha = 1.0)
      : m_Red(red)
      , m_Green(green)
      , m_Blue(blue)
      , m_Alpha(alpha)
    {
    }

    /**
     * Color with given RGBA values
     * @param red
     * @param green
     * @param blue
     * @param alpha
     */
    Color(kt_int8u red, kt_int8u green, kt_int8u blue, kt_int8u alpha = 255)
      : m_Red((kt_double)red/255.0)
      , m_Green((kt_double)green/255.0)
      , m_Blue((kt_double)blue/255.0)
      , m_Alpha((kt_double)alpha/255.0)
    {
    }

    /**
      * Destructor
      */
    virtual ~Color()
    {
    }

  public:
    /**
     * Red component of this color
     * @return red component
     */
    const kt_double GetRed() const
    {
      return m_Red;
    }

    /**
     * Set the red component of this color
     * param red red component
     */
    void SetRed(kt_double red)
    {
      m_Red = red;
    }

    /**
     * Green component of this color
     * @return green component
     */
    const kt_double GetGreen() const
    {
      return m_Green;
    }

    /**
     * Set the green component of this color
     * param green green component
     */
    void SetGreen(kt_double green)
    {
      m_Green = green;
    }

    /**
     * Blue component of this color
     * @return blue component
     */
    const kt_double GetBlue() const
    {
      return m_Blue;
    }

    /**
     * Set the blue component of this color
     * param blue blue component
     */
    void SetBlue(kt_double blue)
    {
      m_Blue = blue;
    }

    /**
     * Alpha value of this color
     * @return alpha value
     */
    const kt_double GetAlpha() const
    {
      return m_Alpha;
    }

    /**
     * Set the alpha value of this color
     * param alpha alpha value
     */
    void SetAlpha(kt_double alpha)
    {
      m_Alpha = alpha;
    }

    /**
     * Returns a string representation of this color
     * @return string representation of this color
     */
    const String ToString() const
    {
      String valueString;
      valueString.Append(StringHelper::ToString(GetRed()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetGreen()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetBlue()));
      valueString.Append(" ");
      valueString.Append(StringHelper::ToString(GetAlpha()));
      return valueString;
    }

    public:
    /**
     * Equality operator
     */
    inline kt_bool operator == (const Color& rOther) const
    {
      return (m_Red == rOther.m_Red && m_Green == rOther.m_Green && m_Blue == rOther.m_Blue && m_Alpha == rOther.m_Alpha);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Color& rOther) const
    {
      return (m_Red != rOther.m_Red || m_Green != rOther.m_Green || m_Blue != rOther.m_Blue || m_Alpha != rOther.m_Alpha);
    }

    /**
     * Write color onto output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Color& rColor)
    {
      rStream << rColor.ToString();
      return rStream;
    }

  private:
    kt_double m_Red;
    kt_double m_Green;
    kt_double m_Blue;
    kt_double m_Alpha;
  }; // class Color

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
 
  namespace gps
  {
    /**
     * Defines a GPS point
     */
    class PointGps : public karto::Vector2d
    {
    public:
      /**
       * GPS point with longitude and latitude of (0, 0)
       */
      PointGps()
      {
      }
      
      /**
       * Construct GPS point with given longitude and latitude
       * @param latitude latitude
       * @param longitude longitude
       */
      PointGps(kt_double latitude, kt_double longitude)
      {
        SetX(longitude);
        SetY(latitude);
      }
      
      /**
       * Copy constructor
       */
      PointGps(const PointGps& rOther)
      {
        SetX(rOther.GetLongitude());
        SetY(rOther.GetLatitude());
      }
      
    public:
      /**
       * Gets the latitude
       * @return latitude
       */
      inline kt_double GetLatitude() const
      {
        return GetY();
      }
      
      /**
       * Sets the latitude
       * @param latitude new latitude
       */
      inline void SetLatitude(kt_double latitude)
      {
        SetY(latitude);
      }
      
      /**
       * Gets the longitude
       * @return longitude
       */
      inline kt_double GetLongitude() const
      {
        return GetX();
      }
      
      /**
       * Sets the longitude 
       * @param longitude new longitude
       */
      inline void SetLongitude(kt_double longitude)
      {
        SetX(longitude);
      }
      
      /**
       * Returns bearing to given GPS point
       * Reference: http://www.movable-type.co.uk/scripts/latlong.html
       * @param rOther GPS point
       * @return bearing to given GPS point
       */
      inline kt_double GetBearing(const PointGps& rOther)
      {
        kt_double lat1 = math::DegreesToRadians(GetLatitude());
        kt_double long1 = math::DegreesToRadians(GetLongitude());
        kt_double lat2 = math::DegreesToRadians(rOther.GetLatitude());
        kt_double long2 = math::DegreesToRadians(rOther.GetLongitude());
        
        kt_double deltaLong = long2 - long1;
        
        kt_double y = sin(deltaLong) * cos(lat2);
        kt_double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLong);
        
        return math::RadiansToDegrees(atan2(y, x));
      }
      
      /**
       * Adds the given GPS offset to this GPS point
       * @param rOffset amount to offset this GPS point by
       */
      void AddOffset(const PointGps& rOffset)
      {
        AddOffset(rOffset.GetLatitude(), rOffset.GetLongitude());
      }
      
      /**
       * Adds the given GPS offset to this GPS point
       * @param latitude amount to offset this GPS point by in the latitude direction
       * @param longitude amount to offset this GPS point by in the longitude direction
       */
      void AddOffset(kt_double latitude, kt_double longitude)
      {
        SetX(GetLongitude() + longitude);
        SetY(GetLatitude() - latitude);
      }
      
      /**
       * Distance between this GPS point and the given GPS point
       * Reference: http://code.google.com/p/geopy/wiki/GettingStarted
       * @param rOther GPS point
       * @return distance between this point and the given point
       */
      kt_double Distance(const PointGps& rOther)
      {
        kt_double dlon = rOther.GetLongitude() - GetLongitude();

        const kt_double slat1 = sin(math::DegreesToRadians(GetLatitude()));
        const kt_double clat1 = cos(math::DegreesToRadians(GetLatitude()));

        const kt_double slat2 = sin(math::DegreesToRadians(rOther.GetLatitude()));
        const kt_double clat2 = cos(math::DegreesToRadians(rOther.GetLatitude()));

        const kt_double sdlon = sin(math::DegreesToRadians(dlon));
        const kt_double cdlon = cos(math::DegreesToRadians(dlon));

        const kt_double t1 = clat2 * sdlon;
        const kt_double t2 = clat1 * slat2 - slat1 * clat2 * cdlon;
        const kt_double t3 = slat1 * slat2 + clat1 * clat2 * cdlon;
        const kt_double dist = atan2(sqrt(t1*t1 + t2*t2), t3);

        const kt_double earthRadius = 6372.795;
        return dist * earthRadius;
      }

      /**
       * String representation of this GPS point
       * @return string representation of this GPS point
       */
      inline const String ToString() const
      {
        String valueString;
        valueString.Append(StringHelper::ToString(GetLatitude()));
        valueString.Append(" ");
        valueString.Append(StringHelper::ToString(GetLongitude()));
        return valueString;
      }
      
      /**
       * Write this GPS point onto output stream
       */
      friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const PointGps& rPointGps)
      {
        rStream << rPointGps.ToString();
        return rStream;
      }

    public:
      /**
       * Returns a new GPS point that is the result of component-wise addition of this point and the given point
       * @param rOther GPS point
       * @return new point that is the component-wise addition of this point and the given point
       */
      inline const PointGps operator + (const PointGps& rOther) const
      {
        return PointGps(GetLatitude() + rOther.GetLatitude(), GetLongitude() + rOther.GetLongitude());
      }
      
      /**
       * Returns a new GPS coordinate that is the result of component-wise subtraction of the given point from this point
       * @param rOther GPS point
       * @return new point that is the component-wise subtracion of the given point from this point
       */
      inline const PointGps operator - (const PointGps& rOther) const
      {
        return PointGps(GetLatitude() - rOther.GetLatitude(), GetLongitude() - rOther.GetLongitude());
      }
    };

    typedef List<PointGps> PointGpsList;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  inline String StringHelper::ToString(const Size2<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline String StringHelper::ToString(const Vector2<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline String StringHelper::ToString(const Vector3<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline String StringHelper::ToString(const Vector4<T>& rValue)
  {
    return rValue.ToString();
  }

  template<typename T>
  inline static kt_bool FromString(const String& rStringValue, Vector3<T>& rValue)
  {
    karto::String tempString = rStringValue;
    kt_size_t index = tempString.FindFirstOf(" ");
    if (index != -1)
    {
      karto::String stringValue;
      T value;

      // Get X
      stringValue = tempString.SubString(0, index);

      value = 0;
      FromString(stringValue, value);
      rValue.SetX(value);

      // Get Y
      tempString = rStringValue.SubString(index + 1, rStringValue.Size());
      index = tempString.FindFirstOf(" ");

      stringValue = tempString.SubString(0, index);

      value = 0;
      FromString(stringValue, value);
      rValue.SetY(value);

      // Get Z
      tempString = rStringValue.SubString(index + 1, rStringValue.Size());
      index = tempString.FindFirstOf(" ");

      stringValue = tempString.SubString(index + 1, rStringValue.Size());

      value = 0;
      FromString(stringValue, value);
      rValue.SetZ(value);

      return true;
    }

    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  //@}

}

#endif // __OpenKarto_Geometry_h__
