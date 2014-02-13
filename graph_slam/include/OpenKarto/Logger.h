/*
 * Karto(tm) Robot Navigation Software - Software Development Kit
 * Release 2.1
 *
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Karto (tm) is a Trademark of SRI International (R).
 *
 * Author(s): Michael A. Eriksen (eriksen@ai.sri.com)
 */

#pragma once

#ifndef __KARTO_LOGGER__
#define __KARTO_LOGGER__

#include <OpenKarto/String.h>
#include <OpenKarto/Event.h>

namespace karto
{
  
  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Log levels
   */
  enum LogLevel 
  {
    /**
     * Disable all logging
     */
    LOG_NONE = 0,

    /**
     * Fatal log level
     */
    LOG_FATAL = 1,

    /**
     * Error log level
     */
    LOG_ERROR = 3,  

    /**
     * Warning log level
     */
    LOG_WARNING = 4,  

    /**
     * Information log level
     */
    LOG_INFORMATION = 6, 

    /**
     * Debug log level
     */
    LOG_DEBUG = 7    
  };

  /** 
   * Logs a message with the specified log level.
   * Wrapper for Poco logging.  Check if OpenKarto is compiled with POCO with USE_POCO=1.
   * @note All releases of Karto from SRI International are compiled with POCO. If you 
   * are manually compiling OpenKarto and don't include POCO, logging is not functional!
   * @param level log level
   * @param rMessage message
   */
  extern KARTO_EXPORT void Log(LogLevel level, const karto::String& rMessage);

  /**
   * Gets the log level
   * @return log level
   */
  extern KARTO_EXPORT LogLevel GetLogLevel();

  /**
   * Sets the log level
   * @param level new log level
   */
  extern KARTO_EXPORT void SetLogLevel(LogLevel level);

  //@cond EXCLUDE
  /**
   * Initialize logging
   * @note Please don't call. Called by Environment::Initialize()
   */
  void InitializeLogger(const karto::String& rApplicationName = "Karto", const String& rLogLevel = "NONE");

  /**
   * Terminate all logging
   * @note Please don't call. Called by Environment::Terminate()
   */
  void TerminateLogger();
  //@endcond
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Log message arguments used for LogMessage events
   */
  class KARTO_EXPORT LogMessageArguments : public EventArguments
  {
  public:
    /**
     * Log message arguments with a log level and log message
     * @param level log level
     * @param rMessage message
     */
    LogMessageArguments(LogLevel level, const karto::String& rMessage)
      : m_Level(level)
      , m_Message(rMessage)
    {
    }

    /**
     * Destructor
     */
    virtual ~LogMessageArguments()
    {
    }

  public:
    /**
     * Gets the log level 
     * @return log level
     */
    const LogLevel GetLevel() const
    {
      return m_Level;
    }

    /**
     * Gets the log message
     * @return log message
     */
    const karto::String& GetMessage() const
    {
      return m_Message;
    }

  private:
    LogLevel m_Level;
    karto::String m_Message;
  };

#ifdef WIN32
  EXPORT_KARTO_EVENT(KARTO_EXPORT, LogMessageArguments)
#endif

  /**
   * Log message event
   */
  extern KARTO_EXPORT BasicEvent<LogMessageArguments> LogMessage;

  //@}

}

#endif // __KARTO_LOGGER__

