/*
 * List.h
 *
 *  Created on: Mar 19, 2012
 *      Author: papazov
 */

#ifndef LIST_H_
#define LIST_H_

#include <cstdio>

namespace btl1
{
/** An implementation of a doubly linked list. Reinventing the wheel only because in some implementations of the stl::list
  * the size() method has a linear time complexity! */
template<class T>
class List
{
protected:
	class Element
	{
	public:
		Element(T data, Element* prev, Element* next): mData(data), mPrev(prev), mNext(next){}
		virtual ~Element(){}

		Element* next(){ return mNext;}
		const Element* next()const{ return mNext;}
		Element* previous(){ return mPrev;}
		const Element* previous()const{ return mPrev;}

		T& data(){ return mData;}
		T const& data()const{ return mData;}

		friend class List;

	protected:
		T mData;
		Element *mPrev, *mNext;
	};

public:
	class ConstIterator
	{
	protected:
		ConstIterator(const Element* element): mElement(element){}
	public:
		ConstIterator(): mElement(NULL){}
		ConstIterator(const ConstIterator& src): mElement(src.mElement){}
		virtual ~ConstIterator(){}

		void operator=(const ConstIterator& src){ this->mElement = src.mElement;}

		void operator++(){ mElement = mElement->next();}
		void operator--(){ mElement = mElement->previous();}

		const T& operator*(){ return mElement->data();}
		operator bool(){ return mElement;}

		friend class List;

	protected:
		const Element* mElement;
	};

	class Iterator
	{
	protected:
		Iterator(Element* element): mElement(element){}
	public:
		Iterator(): mElement(NULL){}
		Iterator(const Iterator& src): mElement(src.mElement){}
		virtual ~Iterator(){}

		void operator=(const Iterator& src){ this->mElement = src.mElement;}

		void operator++(){ mElement = mElement->next();}
		void operator--(){ mElement = mElement->previous();}

		T& operator*(){ return mElement->data();}
		operator bool(){ return (bool)mElement;}

		friend class List;

	protected:
		Element* mElement;
	};

public:
	List(): mSize(0), mFirst(NULL), mLast(NULL){}
	virtual ~List(){ this->clear();}

	inline void clear();

	inline void push_back(T data);
	inline void push_front(T data);

	/** \brief Erases the element at position 'pos' and returns an iterator pointing to the element after 'pos'. */
	inline Iterator erase(Iterator pos);

	Iterator first(){ return Iterator(mFirst);}
	ConstIterator first()const{ return ConstIterator(mFirst);}
	Iterator last(){ return Iterator(mLast);}
	ConstIterator last()const{ return ConstIterator(mLast);}

	void pop_first(){ this->erase(this->first());}
	void pop_last() { this->erase(this->last());}

	void pop_first(T& data){ data = mFirst->mData; this->erase(this->first());}
	void pop_last(T& data) { data = mLast->mData; this->erase(this->last());}

	int size()const{ return mSize;}

protected:
	int mSize;
	Element *mFirst, *mLast;
};

//=== inline methods ==============================================================================================================

template<class T>
inline void List<T>::clear()
{
	Element *current, *next;

	for ( current = mFirst ; current != NULL ; current = next )
	{
		next = current->next();
		delete current;
	}

	mFirst = mLast = NULL;
	mSize = 0;
}

//=================================================================================================================================

template<class T>
inline void List<T>::push_back(T data)
{
	if ( mLast )
		mLast = mLast->mNext = new Element(data, mLast/*the current last is now previous*/, NULL/*next, i.e., no next*/);
	else
		mFirst = mLast = new Element(data, NULL, NULL);

	++mSize;
}

//=================================================================================================================================

template<class T>
inline void List<T>::push_front(T data)
{
	if ( mFirst )
		mFirst = mFirst->mPrev = new Element(data, NULL/*there is no previous*/, mFirst/*the current first is now next*/);
	else
		mFirst = mLast = new Element(data, NULL, NULL);

	++mSize;
}

//=================================================================================================================================

template<class T>
inline typename List<T>::Iterator List<T>::erase(Iterator pos)
{
	Element *prev = pos.mElement->previous(), *next = pos.mElement->next();

	if ( prev )
	{
		if ( next ) // 'pos' is in the middle of the list
		{
			prev->mNext = next;
			next->mPrev = prev;
		}
		else // There is nothing after 'pos'
		{
			prev->mNext = NULL;
			mLast = prev;
		}
	}
	else if ( next ) // There is nothing before 'element'
	{
		next->mPrev = NULL;
		mFirst = next;
	}
	else // 'element' is the only one in the list
		mFirst = mLast = NULL;

	--mSize;
	delete pos.mElement;

	return Iterator(next);
}

//=================================================================================================================================

} // namespace btl1

#endif /* LIST_H_ */
