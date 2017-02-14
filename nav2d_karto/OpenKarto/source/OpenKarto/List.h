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

#ifndef __OpenKarto_List_h__
#define __OpenKarto_List_h__

#include <OpenKarto/Exception.h>
#include <OpenKarto/Math.h>
#include <OpenKarto/StringHelper.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<class T>
  class ConstListIterator;

  template<class T>
  class ListIterator;

  /**
   * List class with boundary check.
   * Any class that is used with List MUST implement operator==.
   * Implementation notes: Do NOT use memcpy/memset (does not work with smart pointers)
   */
  template<class T>
  class List
  {
  public:
    /**
     * Type definition for const iterator
     */
    typedef ConstListIterator<T> ConstIterator;
    
    /**
     * Type definition for iterator
     */
    typedef ListIterator<T> Iterator;
    
    /**
     * Constructs an empty list
     */
    List()
      : m_pElements(NULL)
    {
      Reset();
    }
    
    /** 
     * Construct a list of given size
     * @param size desired size
     */
    List(kt_size_t size)
      : m_pElements(NULL)
    {
      Reset();
      Resize(size);
    }
    
    /** 
     * Copy constructor
     */
    List(const List& rOther)
      : m_pElements(NULL)
    {
      Reset();
      Resize(rOther.m_Size);
      
      for (kt_size_t i = 0; i < m_Size; i++)
      {
        m_pElements[i] = rOther.m_pElements[i];
      }
    }
    
    /** 
     * Destructor
     */
    virtual ~List()
    {
      Reset();
    }
    
  public:
    /**
     * Adds the given value to the end of the list.  Runs
     * in amortized O(1) time.
     * @param rValue value to be added
     */
    virtual void Add(const T& rValue)
    {
      // resize
      if (m_Size == m_Capacity)
      {
        EnsureCapacity(m_Capacity * 2 + 1); // +1 in case capacity is 0
      }
      m_pElements[m_Size] = rValue;
      m_Size++;
    }
    
    /**
     * Appends the given list to this list
     * @param rValue list to be added
     */
    virtual void Add(const List& rValue)
    {
      kt_size_t combinedSize = m_Size + rValue.m_Size;
      if (m_Capacity < combinedSize)
      {
        EnsureCapacity(combinedSize);    
      }
      
      for (kt_size_t i = 0; i < rValue.m_Size; i++)
      {
        m_pElements[m_Size + i] = rValue.m_pElements[i];
      }
      
      m_Size = combinedSize;
    }
    
    /**
     * Removes the first instance of the given value (does nothing if the
     * value is not found).  This method runs in O(n) time.
     * @param rValue value to be removed
     * @return true if item was removed, false otherwise
     */
    virtual kt_bool Remove(const T& rValue) 
    {
      for (kt_size_t i = 0; i < m_Size; i++)
      {
        if (m_pElements[i] == rValue)
        {
          RemoveAt(i);
          return true;
        }
      }
      
      return false;
    }
    
    /**
     * Removes an element at index
     * @param index index
     */
    virtual void RemoveAt(kt_size_t index) 
    {
      if (index >= m_Size)
      {
        String message;
        message.Append("Cannot remove item: invalid index: ");
        message.Append(StringHelper::ToString(index));
        
        throw Exception(message);
      }
      
      if (m_Size > 0)
      {
        // move remaining elements
        for (kt_size_t i = index; i < m_Size - 1; i++)
        {
          m_pElements[i] = m_pElements[i + 1];
        }

        // destroy last element by calling its destructor
        m_pElements[m_Size - 1] = T();

        m_Size--;
      }
    }

    /**
     * Check if this list contains the given value
     * @param rValue value to search for
     * @returns true if this list contains the given value
     */
    virtual kt_bool Contains(const T& rValue) const
    {
      for (kt_size_t i = 0; i < m_Size; i++)
      {
        if (m_pElements[i] == rValue)
        {
          return true;
        }
      }
      
      return false;
    }
    
    /**
     * Gets the size of this list
     * @return size of this list
     */
    inline virtual kt_size_t Size() const
    {
      return m_Size;
    }
    
    /**
     * Checks if this list is empty
     * @return true if this list is empty, false otherwise
     */
    inline virtual kt_bool IsEmpty() const
    {
      return m_Size == 0;
    }
    
    /**
     * Clears this list
     */
    virtual void Clear()
    {
      if (m_Size > 0)
      {
        // call destructor on each element 
        for (kt_size_t i = 0; i < m_Size; i++)
        {
          m_pElements[i] = T();
        }
      }
      
      m_Size = 0;
    }
    
    /**
     * Gets the item at the given index
     * @param index index
     * @return item at given index
     */
    inline virtual T& Get(kt_size_t index)
    {
      return (*this)[index];
    }
    
    /**
     * Gets the item at the given index (const version)
     * @param index index
     * @return item at given index
     */
    inline virtual const T& Get(kt_size_t index) const
    {
      return (*this)[index];
    }
    
    /**
     * Finds the given value in the list
     * @param rValue value to search for
     * @return pointer to item in list or NULL if not found
     */
    inline T* Find(const T& rValue) 
    {
      for (kt_size_t i = 0; i < m_Size; i++)
      {
        if (m_pElements[i] == rValue)
        {
          return &m_pElements[i];
        }
      }
      
      return NULL;
    }
    
    /**
     * Sets the item at the given index to the given value
     * @param index index
     * @param rValue new value to place at given index
     */
    void Set(kt_size_t index, const T& rValue)
    {
      if (index >= m_Size)
      {
        assert(false); 
      }
      
      m_pElements[index] = rValue;
    }
    
    /**
     * First item in the list (list must have at least one item)
     * @return first item in the list
     * @throws Exception if list is empty
     */
    virtual const T& Front() const
    {
      if (m_Size == 0)
      {
        assert(false);
        throw Exception("List is empty!");
      }
      
      return Get(0);
    }
    
    /**
     * Last item in the list (list must have at least one item)
     * @return last item in the list
     * @throws Exception if list is empty
     */
    virtual const T& Back() const
    {
      if (m_Size == 0)
      {
        assert(false);
        throw Exception("List is empty!");
      }
      
      return Get(Size() - 1);
    }
    
    /**
     * Resizes the list. Sets the Size() to be newSize so don't use Add() but [] instead!
     * @param newSize new size
     */
    virtual void Resize(kt_size_t newSize)
    {
      if (newSize != m_Size)
      {
        T* pElements = new T[newSize];
        if (m_pElements != NULL)
        {
          kt_size_t smallerSize = karto::math::Minimum(newSize, m_Size);

          for (kt_size_t i = 0; i < smallerSize; i++)
          {
            pElements[i] = m_pElements[i];
          }

          delete [] m_pElements;
        }

        m_pElements = pElements;
        m_Size = newSize;
        m_Capacity = newSize;
      }
    }
    
    /**
     * Reserves capacity (for efficiency purposes). Don't use [] but Add() instead!
     * @param newCapacity new capacity
     */
    void EnsureCapacity(kt_size_t newCapacity)
    {
      kt_size_t oldSize = m_Size;
      Resize(newCapacity);
      if (newCapacity > oldSize)
      {
        m_Size = oldSize;
      }
    }    
    
    /**
     * Searches for value in list using given comparator f.  Elements must be in
     * sorted order for this function to work properly.
     * @param rValue value to search for
     * @param f comparator takes two elements; returns 0 if the elements are equal,
     * a value less than 0 if the first value is less than the second value, and
     * a value greater than 0 if the first value is greater than the second value
     * @return index into list that is element or -1 if not found
     */
    inline kt_int32s BinarySearch(const T& rValue, kt_int32s (*f)(const T& a, const T& b))
    {
      assert(m_Size > 0);

      kt_int32s lo = 0;
      kt_int32s hi = static_cast<kt_int32s>(m_Size) - 1;
      
      while (lo <= hi)
      {
        kt_int32s mid = (lo + hi) / 2;
        kt_int32s comparison = f(m_pElements[mid], rValue);
        if (comparison == 0)
        {
          return mid;
        }
        else if (comparison < 0)
        {
          lo = mid + 1;
        }
        else // comparison > 0
        {
          hi = mid - 1;
        }
      }
      
      return -1;
    }

  public:
    /**
     * Gets the item at the given index
     * @param index index
     * @return item at given index
     */
    inline T& operator[](kt_size_t index)
    {
      if (index >= m_Size)
      {
        assert(false); 
        throw Exception("Out of bounds exception: " + StringHelper::ToString(index) + " (>= " + StringHelper::ToString(m_Size) + ")");
      }
      
      return m_pElements[index];
    }
    
    /**
     * Gets the item at the given index (const version)
     * @param index index
     * @return item at given index
     */
    inline const T& operator[](kt_size_t index) const
    {
      if (index >= m_Size)
      {
        assert(false);
        throw Exception("Out of bounds exception: " + StringHelper::ToString(index) + " (>= " + StringHelper::ToString(m_Size) + ")");
      }
      
      return m_pElements[index];
    }
    
    /**
     * Assignment operator
     */
    List& operator=(const List& rOther)
    {
      if (&rOther != this)
      {
        Reset();
        Resize(rOther.m_Size);
        for (kt_size_t i = 0; i < rOther.m_Size; i++)
        {
          m_pElements[i] = rOther.m_pElements[i];
        }
      }
      
      return *this;
    }
    
    /**
     * Equality operator
     */
    kt_bool operator==(const List& rOther) const
    {
      if (Size() != rOther.Size())
      {
        return false;
      }
      
      for (kt_size_t i = 0; i < rOther.m_Size; i++)
      {
        if (m_pElements[i] != rOther.m_pElements[i])
        {
          return false;
        }
      }
      
      return true;
    }
    
  private:
    void Reset()
    {
      delete [] m_pElements;
      m_pElements = NULL;
      m_Size = 0;
      m_Capacity = 0;
    }
    
  public:
    /**
     * Gets a const list iterator of this list
     * @return const list iterator of this list
     */
    virtual ConstListIterator<T> GetConstIterator() const
    {
      return ConstListIterator<T>(this);
    }

    /**
     * Gets a list iterator of this list
     * @return list iterator of this list
     */
    virtual ListIterator<T> GetIterator()
    {
      return ListIterator<T>(this);
    }
    
  private:
    T* m_pElements;
    kt_size_t m_Size;
    kt_size_t m_Capacity;
  }; // class List<T>
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Const List iterator
   */
  template<class T>
  class ConstListIterator
  {
  public:
    /**
     * Constructs a const interator for the given list
     * @param pList list to iterate over
     */
    ConstListIterator(const List<T>* pList)
      : m_pList(pList)
      , m_Index(0)
    {
      if (pList == NULL)
      {
        throw Exception("Cannot create iterator: List is NULL");
      }
    }

    /**
     * Whether this iterator has any more items
     * @return true if this iterator has more items, false otherwise
     */
    virtual kt_bool HasNext() const
    {
      return m_Index < m_pList->Size();
    }
    
    /**
     * Current item in the iterator; iterator advances beyond item returned
     * @return current item in the iterator
     */
    virtual const T& Next()
    {
      if (m_Index < m_pList->Size())
      {
        m_Index++;
        return m_pList->Get(m_Index - 1);
      }

      throw Exception("Cannot increment iterator: No more items in iterator.");
    }
    
    /**
     * Gets current item; iterator does NOT advance
     * @return current item
     */
    virtual const T& operator*() const
    {
      if (m_Index < m_pList->Size())
      {
        return m_pList->Get(m_Index);
      }

      throw Exception("Cannot dereference iterator: No more items in iterator.");
    }
    
    /**
     * Gets current item; iterator does NOT advance
     * @return current item
     */
    virtual const T* operator->() const
    {
      if (m_Index < m_pList->Size())
      {
        return &m_pList->Get(m_Index);
      }
      
      throw Exception("Cannot dereference iterator: No more items in iterator.");      
    }
    
    /**
     * Advances iterator to the next item
     * @return next item in the iterator beyond the current item
     */
    virtual const T& operator++()
    {
      Next();
      ConstListIterator iter = *this;
      return *iter;
    }
    
    /**
     * Current item in the iterator; iterator advances beyond item returned
     * @return current item in the iterator
     */
    virtual T operator++(int /*dummy*/)
    {
      return Next();
    }
    
    /**
     * Assignment operator
     */
    virtual const ConstListIterator& operator=(const ConstListIterator& rOther)
    {
      m_pList = rOther.m_pList;
      m_Index = rOther.m_Index;
      return *this;
    }
    
    /**
     * Inequality operator
     */
    virtual kt_bool operator!=(const ConstListIterator& rOther) const
    {
      if (m_pList != rOther.m_pList)
      {
        throw Exception("Iterators are not operating on the same list");
      }
      
      return m_Index != rOther.m_Index;
    }
    
  private:
    const List<T>* m_pList;
    kt_size_t m_Index;
  }; // class ConstIterator<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * List iterator
   */
  template<class T>
  class ListIterator
  {
  public:
    /**
     * Constructs an interator for the given list
     * @param pList list to iterate over
     */    
    ListIterator(List<T>* pList)
    : m_pList(pList)
    , m_Index(0)
    {
      if (pList == NULL)
      {
        throw Exception("Cannot create iterator: List is NULL");
      }
    }
    
    /**
     * Whether this iterator has any more items
     * @return true if this iterator has more items, false otherwise
     */
    virtual kt_bool HasNext() const
    {
      return m_Index < m_pList->Size();
    }
    
    /**
     * Current item in the iterator; iterator advances beyond item returned
     * @return current item in the iterator
     */
    virtual T& Next()
    {
      if (m_Index < m_pList->Size())
      {
        m_Index++;
        return m_pList->Get(m_Index - 1);
      }
      
      throw Exception("Cannot increment iterator: No more items in iterator.");
    }
    
    /**
     * Deletes the item at the current index
     */
    void Erase()
    {
      if (m_Index < m_pList->Size())
      {
        m_pList->RemoveAt(m_Index);
      }
      
      throw Exception("Cannot erase item: No more items in iterator.");      
    }
    
    /**
     * Gets current item; iterator does NOT advance
     * @return current item
     */
    virtual T& operator*() const
    {
      if (m_Index < m_pList->Size())
      {
        return m_pList->Get(m_Index);
      }
      
      throw Exception("Cannot dereference iterator: No more items in iterator.");
    }
    
    /**
     * Gets current item; iterator does NOT advance
     * @return current item
     */
    virtual T* operator->() const
    {
      if (m_Index < m_pList->Size())
      {
        return &m_pList->Get(m_Index);
      }
      
      throw Exception("Cannot dereference iterator: No more items in iterator.");      
    }
    
    /**
     * Advances iterator to the next item
     * @return next item in the iterator beyond the current item
     */
    virtual const T& operator++()
    {
      Next();
      ListIterator iter = *this;
      return *iter;
    }
    
    /**
     * Current item in the iterator; iterator advances beyond item returned
     * @return current item in the iterator
     */
    virtual T operator++(int /*dummy*/)
    {
      return Next();
    }
    
    /**
     * Assignment operator
     */
    virtual const ListIterator& operator=(const ListIterator& rOther)
    {
      m_pList = rOther.m_pList;
      m_Index = rOther.m_Index;
      return *this;
    }
    
    /**
     * Inequality operator
     */
    virtual kt_bool operator!=(const ListIterator& rOther) const
    {
      if (m_pList != rOther.m_pList)
      {
        throw Exception("Iterators are not operating on the same list");
      }
      
      return m_Index != rOther.m_Index;
    }
    
  private:
    List<T>* m_pList;
    kt_size_t m_Index;
  }; // class Iterator<T>

  //@}
}

#endif // __OpenKarto_List_h__
