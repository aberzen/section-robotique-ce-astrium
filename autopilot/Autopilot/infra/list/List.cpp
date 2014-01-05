/*
 * List.cpp
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#include "../include/List.hpp"
#include "../include/Task.hpp"

namespace infra {

List::List() :
	_count(0),
	_head(NULL),
	_tail(NULL)
{
}

List::List(ListNode* nodeList, uint16_t count) :
	_count(0),
	_head(NULL),
	_tail(NULL)
{
	uint16_t idx;
	for (idx=0 ; idx<count ; idx++)
	{
		this->pushBack(&(nodeList[idx]));
	}
}

List::~List() {
}

/** @brief Push an element at the end of the list
 */
ListStatus List::pushBack(ListNode* node)
{
	ListStatus result = E_LIST_OK;
	if (node == NULL)
	{
		result = E_LIST_BAD_ARG;
	}
	else
	{
		/* Initialize next to NULL */
		List::setNextNode(node,NULL);

		/* Protect against interrupt */
		Task::disableInterrupt();

		if (isFull())
		{
			/* Maximum number of element reached */
			result = E_LIST_FULL;
		}
		else if (this->_count == 0)
		{
			/* Set as head and tail */
			_head = node;
			_tail = node;
			/* Increment count */
			this->_count ++;
		}
		else
		{
			/* Set as tail only */
			List::setNextNode(_tail,node);
			_tail = node;
			/* Increment count */
			this->_count ++;
		}

		/* Enable interrupt */
		Task::enableInterrupt();

	}
	return result;
}

/** @brief Push an element at the end of the list
 */
ListStatus List::pushFront(ListNode* node)
{
	ListStatus result = E_LIST_OK;
	if (node == NULL)
	{
		result = E_LIST_BAD_ARG;
	}
	else
	{
		/* Protect against interrupt */
		Task::disableInterrupt();

		if (isFull())
		{
			/* Maximum number of element reached */
			result = E_LIST_FULL;
		}
		else {
			/* Next element is the head */
			List::setNextNode(node,_head);

			/* Set as head */
			_head = node;

			/* Increment count */
			this->_count ++;

			if (this->_count == 0)
			{
				/* Set as tail */
				_tail = node;

			}
		}

		/* Enable interrupt */
		Task::enableInterrupt();

	}
	return result;
}

/** @brief Push an element at the end of the list
 * @warning Not protected against concurrent access
 */
ListStatus List::pushBackFromISR(ListNode* node)
{
	ListStatus result = E_LIST_OK;
	if (node == NULL)
	{
		result = E_LIST_BAD_ARG;
	}
	else
	{
		/* Initialize next to NULL */
		List::setNextNode(node,NULL);

		/* Protect against interrupt */
		Task::disableInterrupt();

		if (isFull())
		{
			/* Maximum number of element reached */
			result = E_LIST_FULL;
		}
		else if (this->_count == 0)
		{
			/* Set as head and tail */
			_head = node;
			_tail = node;
			/* Increment count */
			this->_count ++;
		}
		else
		{
			/* Set as tail only */
			List::setNextNode(_tail,node);
			_tail = node;
			/* Increment count */
			this->_count ++;
		}

		/* Enable interrupt */
		Task::enableInterrupt();

	}
	return result;
}

/** @brief Push an element at the end of the list
 * @warning Not protected against concurrent access
 */
ListStatus List::pushFrontFromISR(ListNode* node)
{
	ListStatus result = E_LIST_OK;
	if (node == NULL)
	{
		result = E_LIST_BAD_ARG;
	}
	else
	{
		if (isFull())
		{
			/* Maximum number of element reached */
			result = E_LIST_FULL;
		}
		else {
			/* Next element is the head */
			List::setNextNode(node,_head);

			/* Set as head */
			_head = node;

			/* Increment count */
			this->_count ++;

			if (this->_count == 0)
			{
				/* Set as tail */
				_tail = node;

			}
		}
	}
	return result;
}

/** @brief Pop an element from the list
 */
ListStatus List::pop(ListNode** node)
{
	ListStatus result = E_LIST_OK;

	if (node == NULL)
	{
		/* Bad argument */
		result = E_LIST_BAD_ARG;
	}
	else if (this->_count == 0)
	{
		/* Empty */
		(*node) = NULL;
		result = E_LIST_EMPTY;
	}
	else
	{
		/* Protect against interrupt */
		Task::disableInterrupt();

		/* Get head */
		(*node) = this->_head;

		/* Set new head as the next element after the old head */
		this->_head = List::getNextNode(*node);

		/* Decrement count */
		this->_count --;

		/* Note that the next element of the last node is NULL
		 * That ensures that the head is reseted to NULL if there is no
		 * more element in the list.
		 */

		/* Enable interrupt */
		Task::enableInterrupt();
	}
	return result;
}

/** @brief Pop an element from the list
 * @warning Not protected against concurrent access
 */
ListStatus List::popFromISR(ListNode** node)
{
	ListStatus result = E_LIST_OK;

	if (node == NULL)
	{
		/* Bad argument */
		result = E_LIST_BAD_ARG;
	}
	else if (this->_count == 0)
	{
		/* Empty */
		(*node) = NULL;
		result = E_LIST_EMPTY;
	}
	else
	{
		/* Get head */
		(*node) = this->_head;

		/* Set new head as the next element after the old head */
		this->_head = List::getNextNode(*node);

		/* Decrement count */
		this->_count --;

		/* Note that the next element of the last node is NULL
		 * That ensures that the head is reseted to NULL if there is no
		 * more element in the list.
		 */
	}
	return result;
}

} /* namespace arducopter */
