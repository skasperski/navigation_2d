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

#include <OpenKarto/Identifier.h>
#include <OpenKarto/Exception.h>

namespace karto
{

  Identifier::Identifier()
  {
  }

  Identifier::Identifier(const char* pString)
  {
    Parse(pString);
  }

  Identifier::Identifier(const String& rString)
  {
    Parse(rString);
  }

  Identifier::Identifier(const Identifier& rOther)
  {
    Parse(rOther.ToString());
  }

  Identifier::~Identifier()
  {
  }

  const String& Identifier::GetName() const
  {
    return m_Name;
  }

  void Identifier::SetName(const String& rName)
  {
    if (rName.Size() != 0)
    {
      std::string name(rName.ToCString());

      std::string::size_type pos = name.find_last_of('/');
      if (pos != 0 && pos != std::string::npos)
      {
        throw Exception("Name can't contain a scope!");
      }

      m_Name = rName;
    }
    else
    {
      m_Name.Clear();
    }

    Update();
  }

  const String& Identifier::GetScope() const
  {
    return m_Scope;
  }

  void Identifier::SetScope(const String& rScope)
  {
    if (rScope.Size() != 0)
    {
      m_Scope = rScope;
    }
    else
    {
      m_Scope.Clear();
    }

    Update();
  }

  const String& Identifier::ToString() const
  {
    return m_FullName;
  }

  void Identifier::Clear()
  {
    m_Name.Clear();
    m_Scope.Clear();
    m_FullName.Clear();
  }

  void Identifier::Parse(const String& rString)
  {
    if (rString.Size() == 0)
    {
      Clear();
      return;
    }

    std::string id(rString.ToCString());

    std::string::size_type pos = id.find_last_of('/');

    if (pos == std::string::npos)
    {
      m_Name = rString;
    }
    else
    {
      m_Scope = rString.SubString(0, pos);
      m_Name = rString.SubString(pos+1, rString.Size());

      // remove '/' from m_Scope if first!!
      if (m_Scope.Size() > 0 && m_Scope[0] == '/')
      {
        m_Scope = m_Scope.SubString(1, m_Scope.Size());
      }
    }

    Update();
  }

  void Identifier::Validate(const String& rString)
  {
    if (rString.Size() == 0)
    {
      return;
    }

    std::string id(rString.ToCString());

    char c = id[0];
    if (IsValidFirst(c))
    {
      for (size_t i = 1; i < id.size(); ++i)
      {
        c = id[i];
        if (!IsValid(c))
        {
          throw Exception("Invalid character in name. Valid characters must be within the ranges A-Z, a-z, 0-9, '/', '_' and '-'.");
        }
      }
    }
    else
    {
      throw Exception("Invalid first character in name. Valid characters must be within the ranges A-Z, a-z, and '/'.");
    }
  }

  void Identifier::Update()
  {
    m_FullName.Clear();

    if (m_Scope.Size() > 0)
    {
      m_FullName.Append("/");
      m_FullName.Append(m_Scope);
      m_FullName.Append("/");
    }
    m_FullName.Append(m_Name);
  }

  Identifier& Identifier::operator=(const Identifier& rOther)
  {
    if (&rOther != this)
    {
      m_Name = rOther.m_Name;
      m_Scope = rOther.m_Scope;
      m_FullName = rOther.m_FullName;
    }

    return *this;
  }

  kt_bool Identifier::operator==(const Identifier& rOther) const
  {
    return (m_FullName == rOther.m_FullName);
  }

  kt_bool Identifier::operator<(const Identifier& rOther) const
  {
    return m_FullName < rOther.m_FullName;
  }

  kt_size_t Identifier::Size() const
  {
    return m_FullName.Size();
  }

}