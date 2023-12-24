/*
 * $QNXLicenseC:
 * Copyright 2008, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
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

#include "externs.h"
#include <stdio.h>
#include <pwd.h>
#include "proto.h"
#include <sys/procmgr.h>
#include <login.h>

#define DENY_ALL					\
			PROCMGR_ADN_ROOT		\
			| PROCMGR_ADN_NONROOT	\
			| PROCMGR_AOP_DENY		\
			| PROCMGR_AOP_LOCK

#define DONT_ALLOW_SPAWN
#define DONT_ALLOW_FORK
#define DONT_ALLOW_DLOPEN

int
main(int argc, char *argv[]) 
{
#ifdef VARIANT_v7
	int err;
#endif

	/*
	 * in SMP, the first time to call nanospin_calibrate() could cause the
	 * devc driver to be rescheduled.
	 * nanospin_ns() is called several times right after the UART is disabled,
	 * if the startup serial driver callouts are called in this time window,
	 * it could cause problems.
	 * So we call nanospin_calibrate() before any nanospin_ns() is called
	 * to minimize this problem
	 */
	nanospin_calibrate(0);

	ttyctrl.max_devs = 16;
	ttc(TTC_INIT_PROC, &ttyctrl, 24);

	if (options(argc, argv) == 0) {
		fprintf(stderr, "%s: No serial ports found\n", argv[0]);
		exit(0);
	}

#ifdef PWR_MAN
	if (!get_chip_type())
		fprintf(stderr, "%s: failed to determine chip type\n", argv[0]);
#endif

#ifdef VARIANT_v7
	err = procmgr_ability(0,
			PROCMGR_ADN_NONROOT | PROCMGR_AOP_ALLOW | PROCMGR_AID_SESSION,
			PROCMGR_ADN_NONROOT | PROCMGR_AOP_ALLOW | PROCMGR_AID_KEYDATA,
			PROCMGR_ADN_NONROOT | PROCMGR_AOP_ALLOW | PROCMGR_AID_PRIORITY,
			PROCMGR_AID_EOL);
	if (err == -1) {
		fprintf(stderr,"Unable to obtain necessary privileges before dropping root\n");
		return -1;
	}
#endif

	if (user_parm != NULL) {
		if (set_ids_from_arg(user_parm) < 0) {
			fprintf(stderr, "Unable to obtain credentials for user %s\n", user_parm);
			return -1;
		}
	}

#ifdef VARIANT_v7
	err = procmgr_ability(0,
#ifdef DONT_ALLOW_SPAWN
			DENY_ALL | PROCMGR_AID_SPAWN,
#endif
#ifdef DONT_ALLOW_FORK
			DENY_ALL | PROCMGR_AID_FORK,
#endif
#ifdef DONT_ALLOW_DLOPEN
			DENY_ALL | PROCMGR_AID_PROT_EXEC,
#endif
			PROCMGR_AID_EOL);

	if (err != EOK)
	{
		fprintf(stderr, "procmgr_ability failed! errno=%d\n", err);
	}
#endif /* VARIANT_v7 */

	ttc(TTC_INIT_START, &ttyctrl, 0);
	return 0;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devc/seromap/main.c $ $Rev: 814187 $")
#endif
