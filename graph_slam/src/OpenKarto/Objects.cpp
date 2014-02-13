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

#include <OpenKarto/Objects.h>

namespace karto
{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  ModuleParameters::ModuleParameters(const Identifier& rName)
    : Object(rName)
  {
  }

  ModuleParameters::~ModuleParameters()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  CustomItem::CustomItem()
    : Object()
  {
  }

  CustomItem::CustomItem(const Identifier& rName)
    : Object(rName)
  {
  }

  CustomItem::~CustomItem()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  DatasetInfo::DatasetInfo()
    : Object()
  {
    m_pTitle = new Parameter<karto::String>(GetParameterSet(), "Title", "Dataset::Title", "Title of dataset", "");
    m_pAuthor = new Parameter<karto::String>(GetParameterSet(), "Author", "Dataset::Author", "Author of dataset", "");
    m_pDescription = new Parameter<karto::String>(GetParameterSet(), "Description", "Dataset::Description", "Description of dataset", "");
    m_pCopyright = new Parameter<karto::String>(GetParameterSet(), "Copyright", "Dataset::Copyright", "Copyright of dataset", "");
  }

  DatasetInfo::~DatasetInfo()
  {
  }

  const String& DatasetInfo::GetTitle() const
  {
    return m_pTitle->GetValue();
  }

  const String& DatasetInfo::GetAuthor() const
  {
    return m_pAuthor->GetValue();
  }

  const String& DatasetInfo::GetDescription() const
  {
    return m_pDescription->GetValue();
  }

  const String& DatasetInfo::GetCopyright() const
  {
    return m_pCopyright->GetValue();
  }

}