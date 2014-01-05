/*
 * List.h
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#ifndef LIST_H_
#define LIST_H_

#include <stddef.h>
#include <inttypes.h>

#define LIST_MAX_ELTS (0xFFFF)

namespace infra {

typedef enum EStatus{
	E_LIST_OK = 0,
	E_LIST_FULL,
	E_LIST_EMPTY,
	E_LIST_BAD_ARG
} ListStatus;

class ListNode {
public:
	ListNode() :
		_data(NULL),
		_next(NULL){
	};
	~ListNode(){};
	inline void set(void* _data){
		this->_data = _data;
	}

	inline void* get(){
		return this->_data;
	}
private:
	void* _data;
	ListNode *_next;

	friend class List;
};

class List {
public:
	List();
	List(ListNode* nodeList, uint16_t count);
	virtual ~List() ;

	/** @brief Push an element as first element in the list
	 */
	virtual ListStatus pushFront(ListNode* node) ;
	virtual ListStatus pushFrontFromISR(ListNode* node) ;

	/** @brief Push an element as last element in the list
	 */
	virtual ListStatus pushBack(ListNode* node) ;
	virtual ListStatus pushBackFromISR(ListNode* node) ;


	/** @brief Pop the first element from the list
	 */
	virtual ListStatus pop(ListNode** node) ;
	virtual ListStatus popFromISR(ListNode** node) ;

	/** @brief Is the list full */
	inline bool isFull(void);

	/** @brief Get elements count */
	inline uint16_t getCount(void);

protected:
	/** @brief Number of element in the list */
	uint16_t _count;
	/** @brief Head of the list */
	ListNode* _head;
	/** @brief Tail of the list */
	ListNode* _tail;

protected:
	/* Ensure access to ListNode objects in case of inheritence */
	static inline ListNode* getNextNode(ListNode* node);
	static inline void setNextNode(ListNode* node, ListNode* next);
};


inline ListNode* List::getNextNode(ListNode* node)
{
	return node->_next;
}
inline void List::setNextNode(ListNode* node, ListNode* next)
{
	node->_next = next;
}

/** @brief Is the list full */
inline bool List::isFull(void) {
	return this->_count == LIST_MAX_ELTS;
}

/** @brief Is the list full */
inline uint16_t List::getCount(void) {
	return this->_count;
}

} /* namespace arducopter */
#endif /* LIST_H_ */
