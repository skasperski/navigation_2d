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

#ifndef __OpenKarto_Exception_h__
#define __OpenKarto_Exception_h__

#include <OpenKarto/String.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Root class for all exceptions thrown from Karto.
   */
  class KARTO_EXPORT Exception
  {
  public:    
    /**
     * Exception with given message
     * @param pMessage exception message
     */
    Exception(const char* pMessage);    

    /**
     * Exception with given message
     * @param rMessage exception message
     * @param errorCode error code
     */
    Exception(const String& rMessage = "Karto Exception", kt_int32s errorCode = 0);    
    
    /**
     * Copy constructor
     */
    Exception(const Exception& rOther);
    
    /**
     * Destructor
     */
    virtual ~Exception();
    
  public:
    /**
     * Assignment operator
     */
    Exception& operator=(const Exception& rException);
    
  public:
    /**
     * Gets the error message
     * @return error message
     */
    const String& GetErrorMessage() const;
    
    /**
     * Gets error code
     * @return error code
     */
    kt_int32s GetErrorCode();
    
  public:
    /**
     * Write exception to output stream
     */
    friend KARTO_FORCEINLINE std::ostream& operator << (std::ostream& rStream, Exception& rException)
    {
      rStream << "Karto Fatal Error: " << std::endl;
      rStream << " ==> error code: " << rException.GetErrorCode() << std::endl;
      rStream << " ==> error message: " << rException.GetErrorMessage() << std::endl;
      return rStream;
    }
    
  private:
    String m_Message;
    kt_int32s m_ErrorCode;
  }; // class Exception

  //@}
}

#endif // __OpenKarto_Exception_h__
