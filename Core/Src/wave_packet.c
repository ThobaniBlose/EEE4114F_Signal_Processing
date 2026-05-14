/*
 * wave_packet.c
 *
 * Formats the Tier-1 output packet string for transmission or logging.
 */

#include "wave_packet.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *mode_ref_label(WaveMode_t mode)
{
    switch (mode) {
        case WAVE_MODE_GEOGRAPHIC:    return "GEOGRAPHIC";
        case WAVE_MODE_BODY_RELATIVE: return "BODY_RELATIVE";
        case WAVE_MODE_FALLBACK:      return "VERTICAL_ONLY";
        default:                      return "UNKNOWN";
    }
}

int wave_packet_format(const WavePacket_t *pkt,
                       char               *buf,
                       uint32_t            buf_size)
{
    if (pkt == 0 || buf == 0 || buf_size == 0U) return -1;

    int n = snprintf(buf, buf_size,
        "T1,%s,UTC=NO_GPS,GPS=%u,LAT=%.4f,LON=%.4f,"
        "Hm0=%.3f,Tp=%.3f,Tm01=%.3f,Tm02=%.3f,"
        "DIR=%.1f,REF=%s,Q=%u,R1=%.3f,COH=%.3f",
        pkt->session_id,
        (unsigned)pkt->gps_fix,
        (double)pkt->lat,
        (double)pkt->lon,
        (double)pkt->Hm0,
        (double)pkt->Tp,
        (double)pkt->Tm01,
        (double)pkt->Tm02,
        (double)pkt->dir_deg,
        mode_ref_label(pkt->mode),
        (unsigned)pkt->quality,
        (double)pkt->r1,
        (double)pkt->coh);

    return (n > 0 && (uint32_t)n < buf_size) ? n : -2;
}
