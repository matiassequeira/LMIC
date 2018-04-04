#ifndef ESLIGHT_PROTOCOL_H_
#define ESLIGHT_PROTOCOL_H_

#include "eslight.pb.h"

/**
 * Encode a message
 * @param buf Out buffer
 * @param buflen Sizeof buf
 * @param msg Pointer to message to be encoded
 */
size_t esl_encode_hello(uint8_t *buf, size_t buflen, esl_Hello *msg);
size_t esl_encode_discover(uint8_t *buf, size_t buflen, esl_Discover *msg);
size_t esl_encode_value_update(uint8_t *buf, size_t buflen, esl_ValueUpdate *msg);

/**
 * Decode a message
 * @param stream Stream to be decode
 * @param out Pointer to output message
 */
uint8_t esl_decode(pb_istream_t *stream, void *out);

#endif /* ESLIGHT_PROTOCOL_H_ */
