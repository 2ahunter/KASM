/**
  ******************************************************************************
  * @file : circular_buffer.C
  * @brief : Source file for circular buffers for serial communications
  * @author : Aaron Hunter
  * @date : 01/15/2025
  *
  ******************************************************************************
*/

/*******************************************************************************
 * #INCLUDES                                                                    *
 ******************************************************************************/

#include "circular_buffer.h"
#include "stddef.h"

/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define TRUE 1
#define FALSE 0
#define SUCCESS 1
#define ERROR 0

/*******************************************************************************
 * DATATYPES                                                                  *
 ******************************************************************************/



/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES                                                 *
 ******************************************************************************/


/*******************************************************************************
 * FUNCTIONS                                                                   *
 ******************************************************************************/

void init_buffer(circular_buffer_t *buf){
    int i;
    buf->read_index = 0; /*initialize read index to 0 */
    buf->write_index = 0; /*initialize write index to 0 */
    for (i = 0; i < BUFFER_LENGTH; i++) { /*initialize data to zero*/
        buf->data[i] = 0;
    } /*end for */
}

int8_t is_buffer_empty(circular_buffer_t *buf) {
    if (buf->read_index == buf->write_index) { /* if read == write then the buffer is empty */
        return TRUE;
    }
    return FALSE;
}

int8_t is_buffer_full(circular_buffer_t *buf){
  /* write index +1 == read index is full, the mod provides wrap around*/
    if ((buf->write_index + 1) % BUFFER_LENGTH == buf->read_index) {
        return TRUE;
    }
    return FALSE;
}

int8_t write_buffer(circular_buffer_t *buf, unsigned char c){
    if (is_buffer_full(buf) == FALSE) {
        buf->data[buf->write_index] = c;
        /*increment the write index and wrap using modulus arithmetic */
        buf->write_index = (buf->write_index + 1) % BUFFER_LENGTH;
        return SUCCESS;
    }
    return ERROR; /*no data written*/
}

unsigned char read_buffer(circular_buffer_t *buf){
      unsigned char val;
    if (is_buffer_empty(buf) == FALSE) {
        val = buf->data[buf->read_index]; /* get the char from the buffer */
        /*increment the read index and wrap using modulus arithmetic */
        buf->read_index = (buf->read_index + 1) % BUFFER_LENGTH;
        return val;
    }
    return ERROR;
}

int get_num_elements(circular_buffer_t *buf){
    if (buf != NULL) {
        if (buf->write_index < buf->read_index) { /*test for wrap around*/
            return (buf->write_index + BUFFER_LENGTH - buf->read_index);
        } else {
            return (buf->write_index - buf->read_index);
        }
    }
    return 0;
}