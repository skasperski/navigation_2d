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

#ifndef __OpenKarto_String_h__
#define __OpenKarto_String_h__

#include <stdio.h>
#include <ostream>
#include <sstream>

#include <OpenKarto/Macros.h>
#include <OpenKarto/Types.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

#ifdef _DEBUG
  struct StringPrivate
  {
    std::string m_String;
  };
#else
  struct StringPrivate;
#endif

  /** 
   * Karto string class that wraps the stl string.
   */
  class KARTO_EXPORT String
  {
  public:
    /**
     * Empty string
     */
    String();

    /**
     * String from char
     * @param c character
     */
    String(char c);

    /**
     * String from const char*
     * @param pString C-string
     */
    String(const char* pString);

    /**
     * String from const char* up to the given size
     * @param pString C-string
     * @param size size
     */
    String(const char* pString, kt_int32u size);

    /**
     * Copy constructor
     */
    String(const String& rOther);

    /**
     * Destructor
     */
    virtual ~String();

  public:
    /**
     * Appends a string to this string
     * @param rString string to append to this string
     */
    void Append(const String& rString);

    /**
     * Gets the string as a C-string
     * @return const char* version of this string
     */
    const char* ToCString() const;

    /**
     * Gets the string size
     * @return size of string
     */
    kt_size_t Size() const;

    /**
     * Makes the string into the empty string
     */
    void Clear();

    /**
     * Returns the substring from the given index until the end of this string
     * @param index starting index
     * @return substring from the given index until the end of this string
     */
    String SubString(kt_size_t index) const;

    /**
     * Returns the substring from the given index for the given number of characters
     * @param index starting index
     * @param length number of characters for the substring
     * @return substring from the given index for the given number of characters
     */
    String SubString(kt_size_t index, kt_size_t length) const;
    
    /**
     * Finds the first index where the given string is a substring of this string
     * @param rValue value to look for
     * @return first index where the given string is a substring of this string
     */
    kt_size_t Find(const String& rValue) const;

    /**
     * Finds the first index where the given string is a substring of this string
     * @param rValue value to look for
     * @return first index where the given string is a substring of this string
     */
    kt_size_t FindFirstOf(const String& rValue) const;

    /**
     * Finds the last index where the given string is a substring of this string
     * @param rValue value to look for
     * @return last index where the given string is a substring of this string
     */
    kt_size_t FindLastOf(const String& rValue) const;

    /**
     * Returns a newline character as a string
     * return newline character as a string
     */
    static String NewLine();

    /**
     * Deletes a portion of the string starting at the given index for the given number of characters
     * @param index starting index to delete characters
     * @param length number of characters to delete
     */
    void Erase(kt_size_t index, kt_size_t length);

  public:
    /**
     * Gets the character at the given index as an integer
     * @param index index
     * @return character at the given index as an integer
     */
    int operator[](kt_int32u index) const;

    /**
     * Assignment operator
     */
    String& operator=(const String& rOther);

    /** 
     * Equality operator
     */    
    kt_bool operator==(const String& rOther) const;

    /** 
     * Inequality operator
     */    
    kt_bool operator!=(const String& rOther) const;

    /** 
     * Less than operator
     */    
    kt_bool operator<(const String& rOther) const;

    /** 
     * Greater than operator
     */    
    kt_bool operator>(const String& rOther) const;

    /**
     * Concatenation operator
     */
    String operator+(const String& rOther);

    /**
     * Concatenation operator
     */
    String operator+(const char* pChar);

    /**
     * Returns a new string that results from prepending the given character to the given string
     * @param pChar character to add
     * @param rOther string to add
     * @return string resulting from prepending this string with the given character
     */
    friend String operator+(const char* pChar, const String& rOther)
    {
      String string(pChar);
      string.Append(rOther);
      return string;
    }

    /**
     * Result of concatenating two strings
     * @param rOther1 first string
     * @param rOther2 second string
     * @return concatenation of two strings
     */
    friend String operator+(const String& rOther1, const String& rOther2)
    {
      String string;
      string.Append(rOther1);
      string.Append(rOther2);
      return string;
    }

    /**
     * Write string to output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const String& rString)
    {
      rStream << rString.ToCString();
      return rStream;
    }

  private:
    StringPrivate* m_pStringPrivate;
  }; // class String

  //@}

}

#endif // __OpenKarto_String_h__
