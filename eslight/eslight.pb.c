/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9-dev at Fri Nov  3 14:05:30 2017. */

#include "eslight.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t esl_Hello_fields[3] = {
    PB_FIELD(  1, BYTES   , OPTIONAL, STATIC  , FIRST, esl_Hello, hwid, hwid, 0),
    PB_FIELD(  2, UINT32  , OPTIONAL, STATIC  , OTHER, esl_Hello, sensor_count, hwid, 0),
    PB_LAST_FIELD
};

const pb_field_t esl_Discover_fields[3] = {
    PB_FIELD(  1, UINT32  , OPTIONAL, STATIC  , FIRST, esl_Discover, id, id, 0),
    PB_FIELD(  2, BYTES   , OPTIONAL, STATIC  , OTHER, esl_Discover, serial, id, 0),
    PB_LAST_FIELD
};

const pb_field_t esl_ValueUpdate_fields[5] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, esl_ValueUpdate, id, id, 0),
    PB_FIELD(  3, BOOL    , OPTIONAL, STATIC  , OTHER, esl_ValueUpdate, bool_val, id, 0),
    PB_FIELD(  4, INT32   , OPTIONAL, STATIC  , OTHER, esl_ValueUpdate, int_val, bool_val, 0),
    PB_FIELD(  5, FLOAT   , OPTIONAL, STATIC  , OTHER, esl_ValueUpdate, float_val, int_val, 0),
    PB_LAST_FIELD
};




/* @@protoc_insertion_point(eof) */
