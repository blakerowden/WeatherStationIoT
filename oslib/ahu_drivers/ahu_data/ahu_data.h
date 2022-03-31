/**
 * @file ahu_data.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef AHU_DATA_H
#define AHU_DATA_H

/**
 * @brief Process the RAW data into the data struct
 *
 */
void process_rx_data(void);

/**
 * @brief Take continous data streams and package into JSON.
 *
 */
void JSON_thread(void);

#endif // AHU_DATA_H