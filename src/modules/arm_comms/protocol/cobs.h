/* Provides functions for consistent overhead byte stuffing (COBS) */

#ifndef COBS_H
#define COBS_H

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * Byte stuffs an array of bytes
 * unstuffed: unstuffed array of bytes
 * stuffed: array to write stuffed array of bytes
 *          must be able to hold 2 + ceil(len / 256) bytes
 * len: length of unstuffed array of bytes
 *
 * return: length of stuffed array
 */
int cobs_stuff(const uint8_t *unstuffed, uint8_t *stuffed, int len);

/**
 * Byte unstuffs an array of bytes
 * stuffed: stuffed array of bytes
 * unstuffed: array to write unstuffed array
 *            must be able to hold len bytes
 * len: length of stuffed array
 *
 * return: length of unstuffed array or negative value if array could not be unstuffed
 */
int cobs_unstuff(const uint8_t *stuffed, uint8_t *unstuffed, int len);

#ifdef __cplusplus
}
#endif

#endif /* COBS_H */
