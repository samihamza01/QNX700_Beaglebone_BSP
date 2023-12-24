/*
 * $QNXLicenseC:
 * Copyright 2009, QNX Software Systems. 
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


#include <ti814x.h>
#include <sys/malloc.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <device_qnx.h>


#include <sys/mman.h>


//
// this is a callback, made by the bsd media code.  We passed
// a pointer to this function during the ifmedia_init() call
// in bsd_mii_initmedia()
//

void	bsd_mii_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)

{
	ti814x_dev_t *ti814x = ifp->if_softc;

	ti814x->bsd_mii.mii_media_active = IFM_ETHER;
	ti814x->bsd_mii.mii_media_status = IFM_AVALID;

	if (ti814x->force_link != -1) {
		if (ti814x->linkup) {
			ti814x->bsd_mii.mii_media_status |= IFM_ACTIVE;
		}

		// report back the previously forced values
		switch(ti814x->cfg.media_rate) {
			case 0:
			ti814x->bsd_mii.mii_media_active |= IFM_NONE;
			break;  

			case 1000*10:
			ti814x->bsd_mii.mii_media_active |= IFM_10_T;
			break;

			case 1000*100:
			ti814x->bsd_mii.mii_media_active |= IFM_100_TX;
			break;

			case 1000*1000:
			ti814x->bsd_mii.mii_media_active |= IFM_1000_T;
			break;

			default:	// this shouldnt really happen, but ...
			ti814x->bsd_mii.mii_media_active |= IFM_NONE;
			break;
		}
		if (ti814x->cfg.duplex) {
			ti814x->bsd_mii.mii_media_active |= IFM_FDX;
		}

		/* Sort out Flow Control status */
		if ((ti814x->flow & (IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) ==
		    (IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) {
		    ti814x->bsd_mii.mii_media_active |= IFM_FLOW;
		} else {
		    ti814x->bsd_mii.mii_media_active |= ti814x->flow;
		}

	} else if (ti814x->linkup) {  // link is auto-detect and up

		ti814x->bsd_mii.mii_media_status |= IFM_ACTIVE;

		switch(ti814x->cfg.media_rate) {
			case 1000*10:
			ti814x->bsd_mii.mii_media_active |= IFM_10_T;
			break;

			case 1000*100:
			ti814x->bsd_mii.mii_media_active |= IFM_100_TX;
			break;

			case 1000*1000:
			ti814x->bsd_mii.mii_media_active |= IFM_1000_T;
			break;

			default:	// this shouldnt really happen, but ...
			ti814x->bsd_mii.mii_media_active |= IFM_NONE;
			break;
		}

		if (ti814x->cfg.duplex) {
			ti814x->bsd_mii.mii_media_active |= IFM_FDX;
		}
		/* Sort out Flow Control status */
		if ((ti814x->flow & (IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) ==
		    (IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) {
		    ti814x->bsd_mii.mii_media_active |= IFM_FLOW;
		} else {
		    ti814x->bsd_mii.mii_media_active |= ti814x->flow;
		}

		// could move this to event.c so there was no lag
		ifmedia_set(&ti814x->bsd_mii.mii_media, IFM_ETHER|IFM_AUTO);

	} else {	// link is auto-detect and down
		ti814x->bsd_mii.mii_media_active |= IFM_NONE;
	}

	// stuff parameter values with hoked-up bsd values
	ifmr->ifm_status = ti814x->bsd_mii.mii_media_status;
	ifmr->ifm_active = ti814x->bsd_mii.mii_media_active;
}


//
// this is a callback, made by the bsd media code.  We passed
// a pointer to this function during the ifmedia_init() call
// in bsd_mii_initmedia().  This function is called when
// someone makes an ioctl into us, we call into the generic
// ifmedia source, and it make this callback to actually 
// force the speed and duplex, just as if the user had
// set the cmd line options
//

int bsd_mii_mediachange(struct ifnet *ifp)
{
	ti814x_dev_t	*ti814x;
	nic_config_t	*cfg;
	struct ifmedia  *ifm;
	int		 user_duplex;
	int		 user_media;
	int		 user_flow;
	int		 media;

	ti814x = ifp->if_softc;
	cfg = &ti814x->cfg;
	ifm = &ti814x->bsd_mii.mii_media;
	if (ifm->ifm_media & IFM_FDX) {
	    user_duplex = 1;
	} else {
	    user_duplex = 0;
	}
	user_media = ifm->ifm_media & IFM_TMASK;
	user_flow = ifm->ifm_media & IFM_ETH_FMASK;

	if (!(ifp->if_flags & IFF_UP)) {
	    slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): network interface isn't up, ioctl ignored",
		  __FUNCTION__);
	    return 0;
	}

	if (!(ifm->ifm_media & IFM_ETHER)) {
	    slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): network interface - bad media: 0x%X", 
		  __FUNCTION__, ifm->ifm_media);
	    return 0;   // should never happen
	}

	/* Media is changing so link will be down until autoneg completes */
	ti814x->linkup = 0;
	cfg->flags |= NIC_FLAG_LINK_DOWN;
	if_link_state_change(ifp, LINK_STATE_DOWN);

	switch (user_media) {
	case IFM_NONE:      // disable media
	    ti814x->force_link = 0;
	    cfg->media_rate = 0;
	    cfg->duplex = 0;
	    ti814x->flow = 0;

	    callout_stop(&ti814x->mii_callout);
	    MDI_DisableMonitor(ti814x->mdi);
	    MDI_PowerdownPhy(ti814x->mdi, cfg->phy_addr);
	    return 0;
	    break;

	case IFM_AUTO:      // auto-select media
	    ti814x->force_link = -1;
	    cfg->media_rate = -1;
	    cfg->duplex = -1;
	    ti814x->flow = -1;

	    MDI_GetMediaCapable(ti814x->mdi, cfg->phy_addr, &media);
	    /* Enable Pause in autoneg */
	    if ((ti814x->mdi->PhyData[ cfg->phy_addr]->VendorOUI == KENDIN) &&
		(ti814x->mdi->PhyData[ cfg->phy_addr]->Model == KSZ9031)) {
		/* Bug in KSZ9031 PHY */
		media |= MDI_FLOW;
	    } else {
		media |= MDI_FLOW | MDI_FLOW_ASYM;
	    }
	    break;

	case IFM_10_T:      // force 10baseT
	    ti814x->force_link = user_duplex ? MDI_10bTFD : MDI_10bT;
	    media = ti814x->force_link;
	    cfg->media_rate = 10 * 1000;
	    break;

	case IFM_100_TX:    // force 100baseTX
	    ti814x->force_link = user_duplex ? MDI_100bTFD : MDI_100bT;
	    media = ti814x->force_link;
	    cfg->media_rate = 100 * 1000;
	    break;

	case IFM_1000_T:    // force 1000baseT
	    ti814x->force_link = user_duplex ? MDI_1000bTFD : MDI_1000bT;
	    media = ti814x->force_link;
	    cfg->media_rate = 1000 * 1000;
	    break;

	default:			// should never happen
	    slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): network interface - unknown media: 0x%X", 
		  __FUNCTION__, user_media);
	    return 0;
	    break;
	}

	if (user_media != IFM_AUTO) {
	    // Forced flow control
	    ti814x->flow = user_flow;
	    if (user_flow & IFM_FLOW) {
		ti814x->force_link |= MDI_FLOW;
	    }
	    if (user_flow & IFM_ETH_TXPAUSE) {
		ti814x->force_link |= MDI_FLOW_ASYM;
	    }
	    if (user_flow & IFM_ETH_RXPAUSE) {
	      ti814x->force_link |= MDI_FLOW|MDI_FLOW_ASYM;
	    }
	    media |= user_flow;
	}

	MDI_PowerupPhy(ti814x->mdi, cfg->phy_addr);
	MDI_EnableMonitor(ti814x->mdi, 0);
	MDI_SetAdvert(ti814x->mdi, cfg->phy_addr, media);
	MDI_AutoNegotiate(ti814x->mdi, cfg->phy_addr, NoWait);
	callout_msec(&ti814x->mii_callout, 3 * 1000, ti814x_MDI_MonitorPhy,
	    ti814x);

	return 0;
}


//
// called from ti814x_attach() in init.c to hook up
// to the bsd media structure.  Not entirely unlike kissing
// a porcupine, we must do so carefully, because we do not
// want to use the bsd mii management structure, because
// this driver uses link interrupt
//
// NOTE: Must call bsd_mii_finimedia() to free resources.
//

void	bsd_mii_initmedia(ti814x_dev_t *ti814x)

{
	struct ifmedia	*ifm;

	ifm = &ti814x->bsd_mii.mii_media;
	ti814x->bsd_mii.mii_ifp = &ti814x->ecom.ec_if;

	ifmedia_init(ifm, 0, bsd_mii_mediachange, bsd_mii_mediastatus);

	ifmedia_add(ifm, IFM_ETHER|IFM_NONE, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_AUTO, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_10_T, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_10_T|IFM_FDX, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_FLOW, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_ETH_TXPAUSE, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_10_T|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_100_TX, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_100_TX|IFM_FDX, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_FLOW, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_ETH_TXPAUSE, 0, NULL);
	ifmedia_add(ifm, IFM_ETHER|IFM_100_TX|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);

	if (ti814x->link_mode != RMII && !(ti814x->no_gig)) {
	    ifmedia_add(ifm, IFM_ETHER|IFM_1000_T|IFM_FDX, 0, NULL);
	    ifmedia_add(ifm, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_FLOW, 0, NULL);
	    ifmedia_add(ifm, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_ETH_TXPAUSE, 0, NULL);
	    ifmedia_add(ifm, IFM_ETHER|IFM_1000_T|IFM_FDX|IFM_ETH_RXPAUSE, 0, NULL);
	}

	switch (ti814x->cfg.media_rate) {
	case -1:
	    ifm->ifm_media = IFM_ETHER | IFM_AUTO;
	    break;
	case 10*1000:
	    ifm->ifm_media = IFM_ETHER | IFM_10_T;
	    break;
	case 100*1000:
	    ifm->ifm_media = IFM_ETHER | IFM_100_TX;
	    break;
	case 1000*1000:
	    if (ti814x->link_mode != RMII && !(ti814x->no_gig)) {
		ifm->ifm_media = IFM_ETHER | IFM_1000_T;
	    } else {
		slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		      "%s(): Gigabit set but not supported, setting none",
		      __FUNCTION__);
	    }
	    break;
	default:
	    slogf(_SLOGC_NETWORK, _SLOG_WARNING,
		  "%s(): Unknown initial media, forcing none",
		  __FUNCTION__);
	    /* Fallthrough */
	case 0:
	    ifm->ifm_media = IFM_ETHER | IFM_NONE;
	    break;
	}

	if (ti814x->cfg.duplex == 1) {
	    ifm->ifm_media |= IFM_FDX;
	}

	if (ti814x->flow != -1) {
	    if ((ti814x->flow & (IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) ==
		(IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE)) {
		ifm->ifm_media |= IFM_FLOW;
	    } else {
		ifm->ifm_media |= ti814x->flow;
	    }
	}

	ifmedia_set(ifm, ifm->ifm_media);
}

// Free any memory associated with the bsd mii.
// ifmedia_add() allocates memory and must be freed by
// ifmedia_delete_instance().
//
void bsd_mii_finimedia(ti814x_dev_t *ti814x)
{
	ifmedia_delete_instance(&ti814x->bsd_mii.mii_media, IFM_INST_ANY);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/dm814x/bsd_media.c $ $Rev: 804097 $")
#endif
