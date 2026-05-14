#ifndef WAVE_PACKET_H
#define WAVE_PACKET_H

#include "wave_types.h"
#include "wave_processor.h"

/* ---------------------------------------------------------------------------
 * wave_packet — formats the Tier-1 output packet string.
 *
 * Tier-1 packet format:
 *   T1,<session>,UTC=<utc>,GPS=<fix>,LAT=<lat>,LON=<lon>,
 *   Hm0=<hm0>,Tp=<tp>,Tm01=<tm01>,Tm02=<tm02>,
 *   DIR=<dir>,REF=<ref>,Q=<quality>,R1=<r1>,COH=<coh>
 *
 * Fields not yet computed are filled with NaN or placeholder values.
 * --------------------------------------------------------------------------- */

#define WAVE_PACKET_MAX_LEN  256U

typedef struct {
    char     session_id[16];    /* e.g. "S001"                              */
    float    lat;               /* latitude [deg], NaN if no GPS            */
    float    lon;               /* longitude [deg], NaN if no GPS           */
    uint8_t  gps_fix;           /* 1 = GPS fix, 0 = no fix                 */
    float    Hm0;               /* significant wave height [m]             */
    float    Tp;                /* peak period [s]                         */
    float    Tm01;              /* mean period [s]                         */
    float    Tm02;              /* zero-crossing period [s]                */
    float    dir_deg;           /* wave direction [deg], NaN if fallback   */
    WaveMode_t    mode;         /* processing mode used                    */
    WaveQuality_t quality;      /* quality flag                            */
    float    r1;                /* first directional spread parameter      */
    float    coh;               /* coherence at peak                       */
} WavePacket_t;

/* Format the Tier-1 packet into buf[]. Returns number of chars written. */
int wave_packet_format(const WavePacket_t *pkt,
                       char               *buf,
                       uint32_t            buf_size);

#endif /* WAVE_PACKET_H */
