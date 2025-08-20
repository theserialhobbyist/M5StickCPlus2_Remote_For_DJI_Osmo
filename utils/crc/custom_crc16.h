#ifndef CUSTOM_CRC16_H
#define CUSTOM_CRC16_H

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The definition of the used algorithm.
 *
 * This is not used anywhere in the generated code, but it may be used by the
 * application code to call algorithm-specific code, if desired.
 */
#define CRC_ALGO_TABLE_DRIVEN 1

/**
 * The type of the CRC values.
 *
 * This type must be big enough to contain at least 16 bits.
 */
typedef uint_fast16_t crc16_t;

/**
 * Calculate the initial crc value.
 *
 * \return     The initial crc value.
 */
static inline crc16_t crc_init(void)
{
    return 0x3aa3;
}

/**
 * Update the crc value with new data.
 *
 * \param[in] crc      The current crc value.
 * \param[in] data     Pointer to a buffer of \a data_len bytes.
 * \param[in] data_len Number of bytes in the \a data buffer.
 * \return             The updated crc value.
 */
crc16_t crc16_update(crc16_t crc, const void *data, size_t data_len);

/**
 * Calculate the final crc value.
 *
 * \param[in] crc  The current crc value.
 * \return     The final crc value.
 */
static inline crc16_t crc16_finalize(crc16_t crc)
{
    return crc;
}

uint16_t calculate_crc16(const uint8_t *data, size_t length);

#ifdef __cplusplus
}           /* closing brace for extern "C" */
#endif

#endif      /* CUSTOM_CRC16_H */
