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

#include <algorithm>
#include <string>
#include <limits>

#include <OpenKarto/String.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

#ifndef _DEBUG
  struct StringPrivate
  {
    std::string m_String;
  };
#endif

  String::String()
    : m_pStringPrivate(new StringPrivate())
  {
  }

  String::String(char c)
    : m_pStringPrivate(new StringPrivate())
  {
    m_pStringPrivate->m_String = c;
  }

  String::String(const char* pString)
    : m_pStringPrivate(new StringPrivate())
  {
    m_pStringPrivate->m_String = pString;
  }

  String::String(const char* pString, kt_int32u size)
    : m_pStringPrivate(new StringPrivate())
  {
    m_pStringPrivate->m_String = std::string(pString, size);
  }

  String::String(const String& rOther)
    : m_pStringPrivate(new StringPrivate())
  {
    m_pStringPrivate->m_String = rOther.m_pStringPrivate->m_String;
  }

  String::~String()
  {
    delete m_pStringPrivate;
  }

  const char* String::ToCString() const
  {
    return m_pStringPrivate->m_String.c_str();
  }

  kt_size_t String::Size() const
  {
    return m_pStringPrivate->m_String.size();
  }

  void String::Append(const String& rString)
  {
    m_pStringPrivate->m_String.append(rString.ToCString());
  }

  String String::SubString(kt_size_t index) const
  {
    return String(m_pStringPrivate->m_String.substr(index).c_str());
  }

  String String::SubString(kt_size_t index, kt_size_t length) const
  {
    return String(m_pStringPrivate->m_String.substr(index, length).c_str());
  }

  kt_size_t String::Find(const String& rValue) const
  {
    return m_pStringPrivate->m_String.find(rValue.ToCString());
  }

  kt_size_t String::FindFirstOf(const String& rValue) const
  {
    return m_pStringPrivate->m_String.find_first_of(rValue.ToCString());
  }

  kt_size_t String::FindLastOf(const String& rValue) const
  {
    return m_pStringPrivate->m_String.find_last_of(rValue.ToCString());
  }

  void String::Erase(kt_size_t index, kt_size_t length)
  {
    m_pStringPrivate->m_String.erase(index, length);
  }

  karto::String String::NewLine()
  {
    return String('\n');
  }

  void String::Clear()
  {
    m_pStringPrivate->m_String.clear();
  }

  //String::operator const char *() const
  //{
  //  return m_pStringPrivate->m_String.c_str();
  //}

  String& String::operator=(const String& rOther)
  {
    if (&rOther != this)
    {
      m_pStringPrivate->m_String = rOther.m_pStringPrivate->m_String;
    }

    return *this;
  }

  kt_bool String::operator==(const String& rOther) const
  {
    return m_pStringPrivate->m_String == rOther.m_pStringPrivate->m_String;
  }

  kt_bool String::operator!=(const String& rOther) const
  {
    return m_pStringPrivate->m_String != rOther.m_pStringPrivate->m_String;
  }

  kt_bool String::operator<( const String& rOther ) const
  {
    return m_pStringPrivate->m_String < rOther.m_pStringPrivate->m_String;
  }

  kt_bool String::operator>( const String& rOther ) const
  {
    return m_pStringPrivate->m_String > rOther.m_pStringPrivate->m_String;
  }

  String String::operator+(const String& rOther)
  {
    return (m_pStringPrivate->m_String + rOther.m_pStringPrivate->m_String).c_str();
  }

  karto::String String::operator+(const char* pChar)
  {
    return (m_pStringPrivate->m_String + std::string(pChar)).c_str();
  }

  int String::operator[]( kt_int32u index ) const
  {
    return m_pStringPrivate->m_String[index];
  }

}