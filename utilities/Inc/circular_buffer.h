/**
  ******************************************************************************
  * @file : circular_buffer.h
  * @brief : Interface header for circular buffers for serial communications
  * @author : Aaron Hunter
  * @date : 01/15/2025
  *
  ******************************************************************************
*/

#ifndef INC_CIRCBUF_H_
#define INC_CIRCBUF_H_

/*******************************************************************************
 * #INCLUDES                                                                    *
 ******************************************************************************/

#include "stdint.h"

/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define BUFFER_LENGTH 2048

/*******************************************************************************
 * DATATYPES                                                                  *
 ******************************************************************************/

typedef struct circular_buffer {
    int read_index;
    int write_index;
    unsigned char data[BUFFER_LENGTH];
}circular_buffer_t; 

/*******************************************************************************
 * FUNCTION PROTOTYPES                                                        *
 ******************************************************************************/

/**
 * @function : init_buffer(circular_buffer *buf)
 * @brief : initializes a circular buffer
 * @param :  reference to a circular buffer instance   
 * @return : none
 * @author : Aaron Hunter
 */
void init_buffer(circular_buffer_t *buf);

/**
 * @function : is_buffer_empty(circular_buffer *buf)
 * @brief : checks if the buffer is empty
 * @param :  reference to a circular buffer instance   
 * @return : 1 for True, 0 False
 * @author : Aaron Hunter
 */
int8_t is_buffer_empty(circular_buffer_t *buf);

/**
 * @function : is_buffer_full(circular_buffer *buf)
 * @brief : checks if the buffer is full
 * @param :  reference to a circular buffer instance   
 * @return : 1 for True, 0 False
 * @author : Aaron Hunter
 */
int8_t is_buffer_full(circular_buffer_t *buf);

/**
 * @function : write_buffer(circular_buffer *buf, unsigned char c)
 * @brief : writes single char to a circular buffer
 * @param :  reference to a circular buffer instance   
 * @param : unsigned char to be written into the buffer
 * @return : 1 for success, 0 error
 * @author : Aaron Hunter
 */
int8_t write_buffer(circular_buffer_t *buf, unsigned char c);

/**
 * @function : read_buffer(circular_buffer *buf)
 * @brief : removes and returns the char at the read index
 * @param :  reference to a circular buffer instance   
 * @return : unsigned char
 * @author : Aaron Hunter
 */
unsigned char read_buffer(circular_buffer_t *buf);

/**
 * @function : get_num_elements(circular_buffer *buf)
 * @brief : removes and returns the char at the read index
 * @param :  reference to a circular buffer instance   
 * @return : number of elements stored in the buffer
 * @author : Aaron Hunter
 */
int get_num_elements(circular_buffer_t *buf);

#endif /* INC_CIRCBUF_H_ */