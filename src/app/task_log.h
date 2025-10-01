/**
 * @file    task_log.h
 * @brief   Task log - log data
 * @version 1.0.0
 * @date    10.04.2025
 * @author  BetaTehPro
 */

#ifndef TASK_LOG_H
#define TASK_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "mav.h"
#include <stdint.h>

#include "lfs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/* littlefs and ulog synchronization period */
#define LOG_SYNC_PERIOD_MS 5000

/* log duration */
#define LOG_PERIOD_MS 5000

#define FTP_DELAY 50 /* [ms] */

#define FTP_TRIES 5 /* Attempts to resend data in timeout logic */
/*******************************************************************************
 * Typedefs
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint32_t CACHE_SIZE;
/*******************************************************************************
 * API
 ******************************************************************************/

/* -------------------------------- TASK -------------------------------------*/
void task_log(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* TASK_LOG_H */