/*
 * $QNXLicenseC:
 * Copyright 2012, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "ti814x.h"
#include <sys/syslog.h>
#include <netdrvr/ptp.h>

#define TI814X_TS_BUF_SZ 64

typedef struct {
    uint64_t	timestamp;
    uint16_t	seq;
    uint8_t	type;
    uint8_t	port;
} ether_event_t;

ether_event_t tx_buf[TI814X_TS_BUF_SZ];
uint32_t tx_idx = 0;
ether_event_t rx_buf[TI814X_TS_BUF_SZ];
uint32_t rx_idx = 0;
uint64_t ts_push_buf[TI814X_TS_BUF_SZ];
volatile uint32_t ts_push_cidx = 0;
volatile uint32_t ts_push_pidx = 0;
pthread_mutex_t ts_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile uint64_t	ti814x_clock_offset = 0;
intrspin_t		spinner;

/*
 * Using a 26bit scale factor means that we can handle frequencies down
 * to 25MHz without breaking out of the lower 32bits in ti814x_clock_mult.
 */
#define PTP_SCALE 26LL
/* Defaults, will calculate actual values in ti814x_ptp_cal() */
uint32_t	ti814x_clock_freq = 250;	/* in MHz */
uint64_t	ti814x_clock_mult_base = 4;
uint64_t	ti814x_clock_mult =  4 << PTP_SCALE;

static __inline__ void __attribute__((__unused__))
dmb(void)
{
	__asm__ __volatile__("dmb	sy" : : : "memory");
}

static __inline__ void __attribute__((__unused__))
dsb(void)
{
	__asm__ __volatile__("dsb	sy" : : : "memory");
}

void ti814x_process_ptp_interrupt (attach_args_t *attach_args)
{
    uint32_t		reg, type;
    ether_event_t	*buf;
    uint32_t		*idx;
    uint64_t		time = 0;
    static uint8_t	half_rollover = 0;

    InterruptLock(&spinner);
    reg = in32(attach_args->cpsw_base + CPTS_EVENT_HIGH);
    type = (reg & EVENT_TYPE_MASK) >> EVENT_TYPE_SHIFT;

    /* Fetch timestamp for timestamp events */
    if ((type == EVENT_TYPE_PUSH) ||
	(type == EVENT_TYPE_TX) ||
	(type == EVENT_TYPE_RX)) {

	time = in32(attach_args->cpsw_base + CPTS_EVENT_LOW);

	if (half_rollover || ((time & 0x80000000) == 0)) {
	    /* Timestamp is good, scale and add offset */
	    time = (time * ti814x_clock_mult) >> PTP_SCALE;
	    time += ti814x_clock_offset;
	} else {
	    /* Misqueued see 9.2.2.5.3 p1463 in TRM sprugz8f */
	    time = (time * ti814x_clock_mult) >> PTP_SCALE;
	    time += (ti814x_clock_offset -
		     (((1LL << 32) * ti814x_clock_mult) >> PTP_SCALE));
	}
    }

    switch (type) {
    case EVENT_TYPE_PUSH:
	if (((ts_push_cidx == 0) &&
	     (ts_push_pidx == (TI814X_TS_BUF_SZ - 1))) ||
	    (ts_push_pidx == (ts_push_cidx - 1))) {
	    /* Buffer is full but still need to pop to clear the IRQ */
	    break;
	}
        ts_push_buf[ts_push_pidx] = time;
	dmb();
	ts_push_pidx = (ts_push_pidx + 1) % TI814X_TS_BUF_SZ;
	dsb();
	break;

    case EVENT_TYPE_TX:
    case EVENT_TYPE_RX:
	if (type == EVENT_TYPE_TX) {
	    idx = &tx_idx;
	    buf = &tx_buf[*idx];
	} else {
	    idx = &rx_idx;
	    buf = &rx_buf[*idx];
	}
	buf->timestamp = time;
	buf->seq = reg & SEQUENCE_ID_MASK;
	buf->type = (reg & MESSAGE_TYPE_MASK) >> MESSAGE_TYPE_SHIFT;
	buf->port = (reg & PORT_NUMBER_MASK) >> PORT_NUMBER_SHIFT;
	(*idx)++;
	if (*idx >= TI814X_TS_BUF_SZ) {
	    *idx = 0;
	}
	break;

    case EVENT_TYPE_ROLLOVER:
	ti814x_clock_offset += ((1LL << 32) * ti814x_clock_mult) >> PTP_SCALE;
	half_rollover = 0;
	break;

    case EVENT_TYPE_HALF:
	half_rollover = 1;
	break;
    }

    out32(attach_args->cpsw_base + CPTS_EVENT_POP, EVENT_POP);
    dsb();
    InterruptUnlock(&spinner);
}

uint64_t ti814x_get_push_ts (ti814x_dev_t *ti814x)
{
    uint64_t ts = 0;
    /* Normally first time through but just in case */
    int32_t loop = PLL_MAX_LOOP;

    /* May need to wait for the ISR to populate the data */
    do {
	dsb();
	/* Busy spin, will be short */
	if (ts_push_pidx != ts_push_cidx) {
	    ts = ts_push_buf[ts_push_cidx];
	    dmb();
	    ts_push_cidx = (ts_push_cidx + 1) % TI814X_TS_BUF_SZ;
	    break;
	}
	loop--;
    } while (loop > 0);

    if (loop == 0) {
	log(LOG_ERR, "Failed to retrieve push timestamp");
    }
    return ts;
}

void ti814x_set_time (ti814x_dev_t *ti814x, ptp_time_t time)
{
    uint64_t new, now;

    pthread_mutex_lock(&ts_mutex);
    out32(ti814x->cpsw_regs + CPTS_TS_PUSH, TS_PUSH);
    dsb();
    now = ti814x_get_push_ts(ti814x);
    now = now - ti814x_clock_offset;
    new = (time.sec * 1000LL * 1000LL * 1000LL) + time.nsec;

    InterruptLock(&spinner);
    ti814x_clock_offset = new - now;
    InterruptUnlock(&spinner);
    pthread_mutex_unlock(&ts_mutex);
}

void ti814x_get_time (ti814x_dev_t *ti814x, ptp_time_t *time)
{
    uint64_t time_ns;

    pthread_mutex_lock(&ts_mutex);
    out32(ti814x->cpsw_regs + CPTS_TS_PUSH, TS_PUSH);
    dsb();
    time_ns = ti814x_get_push_ts(ti814x);
    time->sec = time_ns / (1000LL * 1000LL * 1000LL);
    time->nsec = time_ns % (1000LL * 1000LL * 1000LL);
    pthread_mutex_unlock(&ts_mutex);
}

void ti814x_get_timestamp (ptp_extts_t *ts, uint32_t port, uint8_t tx)
{
    uint32_t idx;
    ether_event_t *buf;

    if (tx) {
	buf = tx_buf;
    } else {
	buf = rx_buf;
    }
    for (idx = 0; idx < TI814X_TS_BUF_SZ; idx++) {
	if ((buf[idx].timestamp != 0) && (buf[idx].seq == ts->sequence_id) &&
	    (buf[idx].type == ts->msg_type) && (buf[idx].port == port)) {
	    ts->ts.sec = buf[idx].timestamp / (1000LL * 1000LL * 1000LL);
	    ts->ts.nsec = buf[idx].timestamp % (1000LL * 1000LL * 1000LL);
	    return;
	}
    }

    ts->ts.sec = 0;
    ts->ts.nsec = 0;
}

void ti814x_set_compensation (ti814x_dev_t *ti814x, ptp_comp_t comp)
{
    uint64_t	corr;

    ti814x_clock_mult = ti814x_clock_mult_base << PTP_SCALE;
    corr = (comp.comp * ti814x_clock_mult) / (1000 * 1000 * 1000);
    if (comp.positive) {
      ti814x_clock_mult += corr;
    } else {
      ti814x_clock_mult -= corr;
    }
}


int ti814x_ptp_ioctl (ti814x_dev_t *ti814x, struct ifdrv *ifd)
{
    ptp_time_t		time;
    ptp_comp_t		comp;
    ptp_extts_t		ts;
    uint8_t		tx;

    if (ifd != NULL) {
	switch(ifd->ifd_cmd) {

	case PTP_GET_TX_TIMESTAMP:
	case PTP_GET_RX_TIMESTAMP:
	    if (ifd->ifd_len != sizeof(ts)) {
		return EINVAL;
	    }

	    if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			   &ts, sizeof(ts))) {
		    return EINVAL;
		}
	    } else {
		memcpy(&ts, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(ts));
	    }

	    if (ifd->ifd_cmd == PTP_GET_TX_TIMESTAMP) {
		tx = 1;
	    } else {
		tx = 0;
	    }

	    ti814x_get_timestamp(&ts, ti814x->cfg.device_index + 1, tx);
	    if (ISSTACK) {
		return (copyout(&ts, (((uint8_t *)ifd) + sizeof(*ifd)),
				sizeof(ts)));
	    } else {
		memcpy((((uint8_t *)ifd) + sizeof(*ifd)), &ts, sizeof(ts));
		return EOK;
	    }
	    break;

	case PTP_GET_TIME:
	    if (ifd->ifd_len != sizeof(time)) {
		return EINVAL;
	    }
	    ti814x_get_time(ti814x,&time);
	    if (ISSTACK) {
		return (copyout(&time, (((uint8_t *)ifd) + sizeof(*ifd)),
				sizeof(time)));
	    } else {
		memcpy((((uint8_t *)ifd) + sizeof(*ifd)), &time, sizeof(time));
		return EOK;
	    }
	    break;

	case PTP_SET_TIME:
	    if (ifd->ifd_len != sizeof(time)) {
		return EINVAL;
	    }
	    if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			   &time, sizeof(time))) {
		    return EINVAL;
		}
	    } else {
		memcpy(&time, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(time));
	    }
	    ti814x_set_time(ti814x, time);
	    /* Clock has changed so all old ts are invalid */
	    memset(tx_buf, 0, sizeof(tx_buf));
	    memset(rx_buf, 0, sizeof(rx_buf));
	    dmb();
	    return EOK;
	    break;

	case PTP_SET_COMPENSATION:
	    if (ifd->ifd_len != sizeof(comp)) {
		return EINVAL;
	    }
	    if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			   &comp, sizeof(comp))) {
		    return EINVAL;
		}
	    } else {
		memcpy(&comp, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(comp));
	    }
	    ti814x_set_compensation(ti814x, comp);
	    return EOK;
	    break;

	default:
	    log(LOG_ERR, "Unknown PTP ioctl 0x%lx", ifd->ifd_cmd);
	    break;
	}
    }
    return EINVAL;
}

static int ti814x_ptp_cal (ti814x_dev_t *ti814x)
{
    struct timespec	tv_start, tv_end;
    uint32_t		tv_diff;
    uint64_t		start, end, diff;

    /* Going to measure it so reset the multiplier to 1 */
    ti814x_clock_mult = 1 << PTP_SCALE;

    /* Get start time and timestamp */
    out32(ti814x->cpsw_regs + CPTS_TS_PUSH, TS_PUSH);
    dsb();
    clock_gettime(CLOCK_MONOTONIC, &tv_start);
    start = ti814x_get_push_ts(ti814x);

    /* Wait for 0.5 secs, a compromise between accuracy and delay at startup */
    nic_delay(500);

    /* Get end time and timestamp */
    out32(ti814x->cpsw_regs + CPTS_TS_PUSH, TS_PUSH);
    dsb();
    clock_gettime(CLOCK_MONOTONIC, &tv_end);
    end = ti814x_get_push_ts(ti814x);

    tv_diff = tv_end.tv_nsec - tv_start.tv_nsec;

    if (tv_end.tv_sec > tv_start.tv_sec) {
	tv_diff += 1000 * 1000 * 1000;
    }

    diff = end - start;

    if ((diff == 0) || (tv_diff == 0)) {
	log(LOG_ERR, "Error in calculating PTP clock frequency %" PRIu64
	    " count in %d ns", diff, tv_diff);
	return EINVAL;
    }

    ti814x_clock_freq = ((diff * 1000) + (tv_diff / 2)) / tv_diff;

    if ((ti814x_clock_freq < 25) || (ti814x_clock_freq > 1000)) {
	log(LOG_ERR, "PTP clock frequency %dMHz not in range 25MHz - 1000MHz",
	    ti814x_clock_freq);
	log(LOG_ERR, "Start count %" PRIu64 " end count %" PRIu64, start, end);
	return EINVAL;
    }

    ti814x_clock_mult_base = (1000 / ti814x_clock_freq);
    ti814x_clock_mult = ti814x_clock_mult_base << PTP_SCALE;

    if ((ti814x_clock_mult_base * ti814x_clock_freq) != 1000) {
	log(LOG_ERR, "PTP clock frequency %dMHz not a divisor of 1000MHz",
	    ti814x_clock_freq);
	return EINVAL;
    }

    log(LOG_INFO, "PTP clock frequency %dMHz", ti814x_clock_freq);

    return EOK;
}

int ti814x_ptp_init (ti814x_dev_t *ti814x)
{
    uint32_t	val;
    int		rc;

    /* Don't touch the PLL unless asked on the command line */
    if (!ti814x->ptp_enable) {
	return EOK;
    }
    if ((ti814x->ptp_enable - 1) > 4){
	log(LOG_ERR, "Invalid PTP Mux value %d > 4", ti814x->ptp_enable - 1);
	return EINVAL;
    }

    log(LOG_INFO, "Enabling PTP using PLL id %d", ti814x->ptp_enable - 1);

    /* Set the MUX to input from the appropriate PLL */
    val = in32(ti814x->cmr_base + GMAC_CLKCTRL);
    val &= ~GMAC_CLKCTRL_RFT_MASK;
    val |= (ti814x->ptp_enable - 1) << GMAC_CLKCTRL_RFT_SHIFT;
    out32(ti814x->cmr_base + GMAC_CLKCTRL, val);

    /* Start the timer and enable interrupts */
    out32(ti814x->cpsw_regs + CPTS_CONTROL, CPTS_EN);
    val = in32(ti814x->cpsw_regs + C0_MISC_EN);
    val |= EVENTINT_EN;
    out32(ti814x->cpsw_regs + C0_MISC_EN, val);
    out32(ti814x->cpsw_regs + CPTS_INT_ENABLE, TS_PEND_EN);
    dsb();

    rc = ti814x_ptp_cal(ti814x);
    if (rc != EOK) {
	return rc;
    }

    /* Enable Timestamping on ports */
#if defined (J6)
    out32(ti814x->cpsw_regs + TS_LTYPE, 0x88f7);
    out32(ti814x->cpsw_regs + P1_TS_SEQ_MTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) |
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP);
    out32(ti814x->cpsw_regs + P1_CONTROL,
	  TS_107 | TS_320 | TS_319 | TS_132 | TS_131 | TS_130 | TS_129 |
	  TS_TTL_NONZERO | TS_UNI | TS_ANNEX_F | TS_ANNEX_E | TS_ANNEX_D |
	  TS_LTYPE1_EN | TS_TX_EN | TS_RX_EN);
    out32(ti814x->cpsw_regs + P2_TS_SEQ_MTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) |
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP);
    out32(ti814x->cpsw_regs + P2_CONTROL,
	  TS_107 | TS_320 | TS_319 | TS_132 | TS_131 | TS_130 | TS_129 |
	  TS_TTL_NONZERO | TS_UNI | TS_ANNEX_F | TS_ANNEX_E | TS_ANNEX_D |
	  TS_LTYPE1_EN | TS_TX_EN | TS_RX_EN);
#elif defined (J5_ECO)
    out32(ti814x->cpsw_regs + TS_LTYPE, 0x88f7);
    out32(ti814x->cpsw_regs + P1_TS_SEQ_MTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) |
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP);
    out32(ti814x->cpsw_regs + P1_CONTROL, TS_LTYPE1_EN | TS_TX_EN | TS_RX_EN);
    out32(ti814x->cpsw_regs + P2_TS_SEQ_MTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) |
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP);
    out32(ti814x->cpsw_regs + P2_CONTROL, TS_LTYPE1_EN | TS_TX_EN | TS_RX_EN);
#else
    out32(ti814x->cpsw_regs + P1_TS_CTL_CPSW_3GF,
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP | TS_TX_EN | TS_RX_EN);
    out32(ti814x->cpsw_regs + P1_TS_SEQ_LTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) | 0x88f7);
    out32(ti814x->cpsw_regs + P2_TS_CTL_CPSW_3GF,
	  TS_TYPE_SYNC | TS_TYPE_DELAY_REQ | TS_TYPE_PDELAY_REQ |
	  TS_TYPE_PDELAY_RESP | TS_TX_EN | TS_RX_EN);
    out32(ti814x->cpsw_regs + P2_TS_SEQ_LTYPE,
	  (0x1e << TS_SEQ_ID_OFFSET_SHIFT) | 0x88f7);
#endif
    dsb();
    return EOK;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/dm814x/ptp.c $ $Rev: 801808 $")
#endif
