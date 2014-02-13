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

#ifndef __OpenKarto_Name_h__
#define __OpenKarto_Name_h__

#include <OpenKarto/String.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Identifier class for karto resources.
   *
   * Example of valid identifiers:
   * 'Test' -- no scope
   * '/Test' -- no scope; will be parsed to 'Test'
   * '/scope/Test' - 'scope' scope and 'Test' name
   * '/scope/name/Test' - 'scope/name' scope and 'Test' name
   */ 
  class KARTO_EXPORT Identifier
  {
  public:
    /**
     * Empty name
     */
    Identifier();

    /**
     * Constructs an identifier from a const char*
     * @param pString identifier const char*
     */
    Identifier(const char* pString);

    /**
     * Constructs an identifier from a String
     * @param rString identifier string
     */
    Identifier(const String& rString);

    /**
     * Copy constructor
     */
    Identifier(const Identifier& rOther);

    /**
     * Destructor
     */
    virtual ~Identifier();

  public:
    /**
     * Gets the name of this identifier
     * @return name
     */
    const String& GetName() const;

    /**
     * Sets the name
     * @param pName name
     */
    void SetName(const String& pName);

    /**
     * Gets the scope of this identifier
     * @return scope
     */
    const String& GetScope() const;

    /**
     * Sets the scope of this identifier
     * @param rScope new scope
     */    
    void SetScope(const String& rScope);

    /**
     * Gets the size of the identifier (scope and name)
     * @return size of the identifier (scope and name)
     */
    kt_size_t Size() const;

    /**
     * Clears this identifier
     */
    void Clear();

    /**
     * Returns a string representation of this identifier
     * @return string representation of this identifier
     */
    const String& ToString() const;

  public:
    /** 
     * Assignment operator
     */
    Identifier& operator=(const Identifier& rOther);

    /** 
     * Equality operator
     */    
    kt_bool operator==(const Identifier& rOther) const;

    /** 
     * Inequality operator
     */    
    kt_bool operator!=(const Identifier& rOther) const
    {
      return !(*this == rOther);
    }

    /** 
     * Less than operator (An identifier is 'less than' another identifier if it
     * is lexicographically 'less' than the other identifier.)
     */    
    kt_bool operator<(const Identifier& rOther) const;

    /**
     * Writes this identifier onto the output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, const Identifier& rIdentifier)
    {
      rStream << rIdentifier.ToString();
      return rStream;
    }
    
  private:
    /**
     * Parse the given string into an identifier
     * @param rString string
     */
    void Parse(const String& rString);

    /**
     * Validates the given string as an identifier
     * @param rString string
     */
    void Validate(const String& rString);

    /**
     * Formats the identifier
     */
    void Update();

    /**
     * Whether the character is valid as a first character (alphanumeric or /)
     * @param c character
     * @return true if the character is a valid first character
     */
    inline kt_bool IsValidFirst(char c)
    {
      return (isalpha(c) || c == '/');
    }

    /**
     * Whether the character is a valid character (alphanumeric, /, _, or -)
     * @param c character
     * @return true if the character is valid
     */
    inline kt_bool IsValid(char c)
    {
      return (isalnum(c) || c == '/' || c == '_' || c == '-');
    }

  private:
    String m_Name;
    String m_Scope;
    String m_FullName;
  };

  //@}

}

#endif // __OpenKarto_Name_h__
