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

#include <OpenKarto/Exception.h>

namespace karto
{

  Exception::Exception(const char* pMessage)
    : m_Message(pMessage)
    , m_ErrorCode(0)
  {
  }

  Exception::Exception(const String& rMessage, kt_int32s errorCode)
    : m_Message(rMessage)
    , m_ErrorCode(errorCode)
  {
  }

  Exception::Exception(const Exception& rException)
    : m_Message(rException.m_Message)
    , m_ErrorCode(rException.m_ErrorCode)
  {
  }

  Exception::~Exception()
  {
  }

  Exception& Exception::operator=(const Exception& rException)
  {
    if (&rException != this)
    {
      m_Message = rException.m_Message;
      m_ErrorCode = rException.m_ErrorCode;
    }

    return *this;
  }

  const String& Exception::GetErrorMessage() const
  {
    return m_Message;
  }

  kt_int32s Exception::GetErrorCode()
  {
    return m_ErrorCode;
  }

}