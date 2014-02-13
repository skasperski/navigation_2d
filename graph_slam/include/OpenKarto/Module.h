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

#ifndef __OpernKarto_Module_h__
#define __OpernKarto_Module_h__

#include <OpenKarto/Sensor.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /**
   * Abstract base class for Karto modules.
   */
  class KARTO_EXPORT Module : public Object
  {
  public:
    /**
     * Construct a Module
     * @param rName module name
     */
    Module(const Identifier& rName);

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~Module();
    //@endcond
    
  public:
    /**
     * Resets the module
     */ 
    virtual void Reset();

    /**
     * Processes an object
     */ 
    virtual kt_bool Process(karto::Object* pObject);

  private:
    Module(const Module&);
    const Module& operator=(const Module&);

  private:
    SensorList m_Sensors;
  };

  //@}

}


#endif // __OpernKarto_Module_h__
