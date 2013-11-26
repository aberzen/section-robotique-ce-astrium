/*
 * Task.h
 *
 *  Created on: 8 janv. 2013
 *      Author: Aberzen
 */

#ifndef TASK_HPP_
#define TASK_HPP_

#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
namespace infra {

class Task {
public:
	Task(
			const signed char * pcName,
            unsigned portSHORT usStackDepth,
            unsigned portBASE_TYPE uxPriority) ;

	virtual ~Task() ;

	/** @brief Non returning function executed as the task body function. */
	virtual void init(void);

	/** @brief Non returning function executed as the task body function. */
	virtual void run(void) = 0;

#if INCLUDE_vTaskSuspend == 1
	/** @brief Suspend the task. */
	inline void suspend(void);

	/** @brief Suspend the task. */
	inline void resume(void);

#if  INCLUDE_xTaskResumeFromISR == 1
	/** @brief Resume task execution from ISR. */
	inline void resumeFromISR(void);
#endif
#endif

#if INCLUDE_pcTaskGetTaskName == 1
	inline signed char * getName(void);
#endif

#if INCLUDE_vTaskPriorityGet == 1
	/** @brief Getter method for priority. */
	inline unsigned portBASE_TYPE getPriority(void);
#endif

#if INCLUDE_vTaskPrioritySet == 1
	/** @brief Setter method for priority. */
	inline void setPriority(unsigned portBASE_TYPE uxNewPriority);
#endif

#if INCLUDE_vTaskDelay == 1
	/** @brief Wait for the expected delay */
	static inline void delay(uint16_t xTicksToDelay);
#endif

#if INCLUDE_vTaskDelayUntil == 1
	/** @brief Wait for the expected date */
	static inline void delayUntil(portTickType *pxPreviousWakeTime, portTickType xTimeIncrement);
#endif

	/** @brief Enter critical section */
	static inline void enterCritical(void);

	/** @brief Enter critical section */
	static inline void leaveCritical(void);

	/** @brief Disable interrupt */
	static inline void disableInterrupt(void);

	/** @brief Enable interrupt */
	static inline void enableInterrupt(void);


	inline bool isInitialized();
protected:
	/** @brief Run a frame */
	void runExecFrame(uint8_t iFrame);

protected:
    bool _isInitialized;

private:
    xTaskHandle _pvCreatedTask;
    portBASE_TYPE _status;
};

extern "C" void runTask(void *pvParameters);

#if INCLUDE_vTaskDelay == 1
inline void Task::delay(uint16_t xTicksToDelay) {
	vTaskDelay(xTicksToDelay);
}
#endif

#if INCLUDE_vTaskDelayUntil == 1
inline void Task::delayUntil(portTickType *pxPreviousWakeTime, portTickType xTimeIncrement){
	vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
}
#endif

#if INCLUDE_vTaskSuspend == 1
/** @brief Suspend task execution. */
inline void Task::suspend(void){
	vTaskSuspend(this->_pvCreatedTask);
}

/** @brief Resume task execution. */
inline void Task::resume(void){
	vTaskResume(this->_pvCreatedTask);
}

#if  INCLUDE_xTaskResumeFromISR == 1
/** @brief Resume task execution from ISR. */
inline void Task::resumeFromISR(void){
	vTaskResumeFromISR(this->_pvCreatedTask);
}
#endif
#endif

#if INCLUDE_pcTaskGetTaskName == 1
/** @brief Priority getter function. */
inline signed char * Task::getName(void){
	return pcTaskGetTaskName(this->_pvCreatedTask);
}
#endif

#if INCLUDE_vTaskPriorityGet == 1
/** @brief Priority getter function. */
inline unsigned portBASE_TYPE Task::getPriority(void){
	return uxTaskPriorityGet(this->_pvCreatedTask);
}
#endif

#if INCLUDE_vTaskPrioritySet == 1
/** @brief Priority setter function. */
inline void Task::setPriority(unsigned portBASE_TYPE uxNewPriority){
	return uxTaskPrioritySet(this->_pvCreatedTask, uxNewPriority);
}
#endif

/** @brief Enter critical section
 * @warning Use with care (alter the stack)
 */
inline void Task::enterCritical(void) {
	taskENTER_CRITICAL();
}

/** @brief Leave critical section
 * @warning Use with care (alter the stack)
 */
inline void Task::leaveCritical(void) {
	taskEXIT_CRITICAL();
}

/** @brief Enter critical section
 * @warning Use with care (alter the stack)
 */
inline void Task::disableInterrupt(void) {
	taskDISABLE_INTERRUPTS();
}

/** @brief Leave critical section
 * @warning Use with care (alter the stack)
 */
inline void Task::enableInterrupt(void) {
	taskENABLE_INTERRUPTS();
}

bool Task::isInitialized(){
	return _isInitialized;
}

} /* namespace arducopter */

#else
typedef struct Task Task;
#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void runTask(void *pvParameters);

#ifdef __cplusplus
}
#endif /* __cplusplus */



#endif /* TASK_HPP_ */
