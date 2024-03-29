/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_ABA_EEPROM_ABA_EEPROM_PB_H_INCLUDED
#define PB_ABA_EEPROM_ABA_EEPROM_PB_H_INCLUDED
#include <pb.h>
#include "aba.can.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _aba_eeprom_eeCanSpeed {
    uint8_t nsjw;
    uint8_t nseg1;
    uint8_t nseg2;
    uint8_t nscale;
    uint8_t dsjw;
    uint8_t dseg1;
    uint8_t dseg2;
    uint8_t dscale;
    uint8_t tdc;
    uint8_t tdcf;
    uint8_t tdco;
} aba_eeprom_eeCanSpeed;

typedef struct _aba_eeprom_eeCanDefaults {
    bool has_speed;
    aba_eeprom_eeCanSpeed speed;
    aba_can_eCanMode mode;
    aba_can_eCanSpecialMode specialMode;
    uint8_t autoRetryEnabled;
} aba_eeprom_eeCanDefaults;

typedef struct _aba_eeprom_eeprom_content {
    bool has_can0;
    aba_eeprom_eeCanDefaults can0;
    bool has_can1;
    aba_eeprom_eeCanDefaults can1;
    uint16_t crc16;
} aba_eeprom_eeprom_content;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define aba_eeprom_eeCanSpeed_init_default       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define aba_eeprom_eeCanDefaults_init_default    {false, aba_eeprom_eeCanSpeed_init_default, _aba_can_eCanMode_MIN, _aba_can_eCanSpecialMode_MIN, 0}
#define aba_eeprom_eeprom_content_init_default   {false, aba_eeprom_eeCanDefaults_init_default, false, aba_eeprom_eeCanDefaults_init_default, 0}
#define aba_eeprom_eeCanSpeed_init_zero          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define aba_eeprom_eeCanDefaults_init_zero       {false, aba_eeprom_eeCanSpeed_init_zero, _aba_can_eCanMode_MIN, _aba_can_eCanSpecialMode_MIN, 0}
#define aba_eeprom_eeprom_content_init_zero      {false, aba_eeprom_eeCanDefaults_init_zero, false, aba_eeprom_eeCanDefaults_init_zero, 0}

/* Field tags (for use in manual encoding/decoding) */
#define aba_eeprom_eeCanSpeed_nsjw_tag           1
#define aba_eeprom_eeCanSpeed_nseg1_tag          2
#define aba_eeprom_eeCanSpeed_nseg2_tag          3
#define aba_eeprom_eeCanSpeed_nscale_tag         4
#define aba_eeprom_eeCanSpeed_dsjw_tag           5
#define aba_eeprom_eeCanSpeed_dseg1_tag          6
#define aba_eeprom_eeCanSpeed_dseg2_tag          7
#define aba_eeprom_eeCanSpeed_dscale_tag         8
#define aba_eeprom_eeCanSpeed_tdc_tag            9
#define aba_eeprom_eeCanSpeed_tdcf_tag           10
#define aba_eeprom_eeCanSpeed_tdco_tag           11
#define aba_eeprom_eeCanDefaults_speed_tag       1
#define aba_eeprom_eeCanDefaults_mode_tag        2
#define aba_eeprom_eeCanDefaults_specialMode_tag 3
#define aba_eeprom_eeCanDefaults_autoRetryEnabled_tag 4
#define aba_eeprom_eeprom_content_can0_tag       1
#define aba_eeprom_eeprom_content_can1_tag       2
#define aba_eeprom_eeprom_content_crc16_tag      3

/* Struct field encoding specification for nanopb */
#define aba_eeprom_eeCanSpeed_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   nsjw,              1) \
X(a, STATIC,   SINGULAR, UINT32,   nseg1,             2) \
X(a, STATIC,   SINGULAR, UINT32,   nseg2,             3) \
X(a, STATIC,   SINGULAR, UINT32,   nscale,            4) \
X(a, STATIC,   SINGULAR, UINT32,   dsjw,              5) \
X(a, STATIC,   SINGULAR, UINT32,   dseg1,             6) \
X(a, STATIC,   SINGULAR, UINT32,   dseg2,             7) \
X(a, STATIC,   SINGULAR, UINT32,   dscale,            8) \
X(a, STATIC,   SINGULAR, UINT32,   tdc,               9) \
X(a, STATIC,   SINGULAR, UINT32,   tdcf,             10) \
X(a, STATIC,   SINGULAR, UINT32,   tdco,             11)
#define aba_eeprom_eeCanSpeed_CALLBACK NULL
#define aba_eeprom_eeCanSpeed_DEFAULT NULL

#define aba_eeprom_eeCanDefaults_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  speed,             1) \
X(a, STATIC,   SINGULAR, UENUM,    mode,              2) \
X(a, STATIC,   SINGULAR, UENUM,    specialMode,       3) \
X(a, STATIC,   SINGULAR, UINT32,   autoRetryEnabled,   4)
#define aba_eeprom_eeCanDefaults_CALLBACK NULL
#define aba_eeprom_eeCanDefaults_DEFAULT NULL
#define aba_eeprom_eeCanDefaults_speed_MSGTYPE aba_eeprom_eeCanSpeed

#define aba_eeprom_eeprom_content_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  can0,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  can1,              2) \
X(a, STATIC,   SINGULAR, UINT32,   crc16,             3)
#define aba_eeprom_eeprom_content_CALLBACK NULL
#define aba_eeprom_eeprom_content_DEFAULT NULL
#define aba_eeprom_eeprom_content_can0_MSGTYPE aba_eeprom_eeCanDefaults
#define aba_eeprom_eeprom_content_can1_MSGTYPE aba_eeprom_eeCanDefaults

extern const pb_msgdesc_t aba_eeprom_eeCanSpeed_msg;
extern const pb_msgdesc_t aba_eeprom_eeCanDefaults_msg;
extern const pb_msgdesc_t aba_eeprom_eeprom_content_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define aba_eeprom_eeCanSpeed_fields &aba_eeprom_eeCanSpeed_msg
#define aba_eeprom_eeCanDefaults_fields &aba_eeprom_eeCanDefaults_msg
#define aba_eeprom_eeprom_content_fields &aba_eeprom_eeprom_content_msg

/* Maximum encoded size of messages (where known) */
#define aba_eeprom_eeCanDefaults_size            42
#define aba_eeprom_eeCanSpeed_size               33
#define aba_eeprom_eeprom_content_size           92

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
