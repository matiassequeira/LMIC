#include "protocol.h"
#include <pb_decode.h>
#include <pb_encode.h>

static bool esl_encode_common(pb_ostream_t *stream, uint64_t type, const pb_field_t fields[], const void *msg)
{
    if (!pb_encode_varint(stream, (uint64_t)type))
    	return false;

    if (!pb_encode_delimited(stream, fields, msg))
    	return false;

    return true;
}

size_t esl_encode_hello(uint8_t *buf, size_t buflen, esl_Hello *hello)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buf, buflen);
    if (!esl_encode_common(&stream, esl_MsgType_HELLO, esl_Hello_fields, hello))
    	return 0;

    return stream.bytes_written;
}

size_t esl_encode_discover(uint8_t *buf, size_t buflen, esl_Discover *discover)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buf, buflen);
    if (!esl_encode_common(&stream, esl_MsgType_DISCOVER, esl_Discover_fields, discover))
    	return 0;

    return stream.bytes_written;
}

size_t esl_encode_value_update(uint8_t *buf, size_t buflen, esl_ValueUpdate *valupdate)
{
    pb_ostream_t stream = pb_ostream_from_buffer(buf, buflen);
    if (!esl_encode_common(&stream, esl_MsgType_VALUE_UPDATE, esl_ValueUpdate_fields, valupdate))
    	return 0;

    return stream.bytes_written;
}

uint8_t esl_decode(pb_istream_t *stream, void *out)
{
    uint64_t type = 0;
    const pb_field_t *fields = NULL;

    if (!pb_decode_varint(stream, &type)) {
    	return 0;
    }

    switch (type) {
    case esl_MsgType_HELLO:
    	fields = esl_Hello_fields;
    	break;

    case esl_MsgType_DISCOVER:
    	fields = esl_Discover_fields;
    	break;

    case esl_MsgType_VALUE_UPDATE:
    	fields = esl_ValueUpdate_fields;
    	break;

    default:
    	return 0;
    }

    if (!pb_decode_delimited(stream, fields, out))
    	return 0;

    return type;
}
