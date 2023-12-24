/*
 * $QNXLicenseC:
 * Copyright 2011, QNX Software Systems. All Rights Reserved.
 *
 * You must obtain a written license from and pay applicable
 * license fees to QNX Software Systems before you may reproduce,
 * modify or distribute this software, or any work that includes
 * all or part of this software.   Free development licenses are
 * available for evaluation and non-commercial purposes.  For more
 * information visit http://licensing.qnx.com or email
 * licensing@qnx.com.
 *
 * This file may contain contributions from others.  Please review
 * this entire file for other proprietary rights or license notices,
 * as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#ifndef SEROMAP_H_
#define SEROMAP_H_

#include <devctl.h>

#define _DCMD_SEROMAP  _DCMD_MISC

/**
 * General mechanism to signal the Bluetooth Enable pin to be pulled up or down
 * byte - 0 - pull the BT_EN pin low
 * byte - 1 - pull the BT_EN pin high
 */
#define DCMD_SEROMAP_BT_EN				__DIOT(_DCMD_SEROMAP, 1, int)

#endif /* SEROMAP_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devc/public/hw/seromap.h $ $Rev: 680332 $")
#endif
