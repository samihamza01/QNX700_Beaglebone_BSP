/*
 * $QNXLicenseC:
 * Copyright 2011, QNX Software Systems.
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

#include	"ti814x.h"
#include	"nxp.h"

/*****************************************************************************/
/* PHY read routine                                                          */
/*****************************************************************************/

uint16_t ti814x_mdi_read (void *hdl, uint8_t phyid, uint8_t reg)
{
	ti814x_dev_t	*ti814x = (ti814x_dev_t *) hdl;
	int				timeout = 1000;
	uint16_t		data = 0;
	uint32_t		mdio_useraccess_reg;

	if (ti814x->emu_phy != -1 && ti814x->emu_phy == ti814x->cfg.device_index) {
		return (emu_phy_read (ti814x, phyid, reg));
		}

	switch(ti814x->cfg.device_index) {
		case 0: mdio_useraccess_reg = MDIOUSERACCESS0; break;
		case 1: mdio_useraccess_reg = MDIOUSERACCESS1; break;
		default: return(0xffff);
	}

	while (timeout) {
		if (! (inle32 (ti814x->cpsw_regs + mdio_useraccess_reg) & GO))
			break;
		nanospin_ns (1000);
		timeout--;
	}

	if (timeout <= 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d timeout waiting for ready",	__FUNCTION__, __LINE__);
		return (0xffff);
	}

	outle32 (ti814x->cpsw_regs + mdio_useraccess_reg, GO | REGADR(reg) | PHYADR(phyid));

	while (timeout) {
		if (! (inle32 (ti814x->cpsw_regs + mdio_useraccess_reg) & GO))
			break;
		nanospin_ns (1000);
		timeout--;
	}

	if (timeout <= 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d timeout waiting for data",	__FUNCTION__, __LINE__);
		return (0xffff);
	}

	// if (!(inle32(mdio+mdio_useraccess_reg) & ACK))
	//	 slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d ACK BIT NOT SET: BAD",	__FUNCTION__, __LINE__);
	data = ((uint16_t) inle32 (ti814x->cpsw_regs + mdio_useraccess_reg) & 0xffff);
	if (ti814x->cfg.verbose & DEBUG_MII_RW) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d devid=%d phyid=%d reg=%d data=0x%04x",
			  __FUNCTION__, __LINE__, ti814x->cfg.device_index, phyid, reg, data);
	}
	return data;
}

/*****************************************************************************/
/* PHY write routine                                                         */
/*****************************************************************************/

void ti814x_mdi_write (void *hdl, uint8_t phyid, uint8_t reg, uint16_t data)
{
	ti814x_dev_t	*ti814x = (ti814x_dev_t *) hdl;
	int				timeout = 1000;
	uint32_t		mdio_useraccess_reg;

	if (ti814x->cfg.verbose & DEBUG_MII_RW) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d - devid=%d phyid=%d reg=%d data=0x%04x",
			__FUNCTION__, __LINE__, ti814x->cfg.device_index, phyid, reg, data);
	}

	if (ti814x->emu_phy != -1 && ti814x->emu_phy == ti814x->cfg.device_index) {
		return;
		}

	switch(ti814x->cfg.device_index) {
		case 0: mdio_useraccess_reg = MDIOUSERACCESS0; break;
		case 1: mdio_useraccess_reg = MDIOUSERACCESS1; break;
		default:
			slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d -Invalid device index %d",
						__FUNCTION__, __LINE__, ti814x->cfg.device_index);
			return;
	}

	while (timeout) {
		if (! (inle32 (ti814x->cpsw_regs + mdio_useraccess_reg) & GO))
			break;
		nanospin_ns (1000);
		timeout--;
	}
	if (timeout <= 0)
		return;

	outle32 (ti814x->cpsw_regs + mdio_useraccess_reg, GO | WRITE | REGADR(reg) | PHYADR(phyid) | data);

	timeout = 1000;
	while (timeout) {
		if (!(inle32(ti814x->cpsw_regs + mdio_useraccess_reg) & GO))
			break;
		nanospin_ns(1000);
		timeout--;
	}

	if (timeout <= 0) {
		 slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d - write operation timeout",
		  __FUNCTION__, __LINE__);
	}
}

/*****************************************************************************/
/*  PHY read/write routines called from devctl                               */
/*****************************************************************************/

int	ti814x_phy_funcs (ti814x_dev_t *ti814x, struct ifdrv *ifd)

{
    phy_access_t phy;
    int	         timeout;

	if (! ti814x->phy_init) {
		outle32 (ti814x->cpsw_regs + MDIOCONTROL, MDIO_ENABLE | MDIO_FAULT | MDIO_FAULTENB | MDIO_CLK_DIVISOR);
		timeout = 1000;
		while (timeout) {
			if (!(inle32 (ti814x->cpsw_regs + MDIOCONTROL) & MDIO_IDLE))
				break;
			nanospin_ns (1000);
			timeout--;
			}
		if (timeout <= 0) {
			slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "PHY init timeout");
			return (-1);
			}
		ti814x->phy_init = 1;
		}
	if (copyin ((((uint8_t *)ifd) + sizeof(*ifd)), &phy, sizeof(phy))) {
	    return (EINVAL);
		}
	if (strcmp (ifd->ifd_name, "dm0") && strcmp (ifd->ifd_name, "dm1")) {
		return (EINVAL);
		}
	if (ifd->ifd_data == NULL) {
		return (EINVAL);
		}
	if (ifd->ifd_len != sizeof (phy_access_t)) {
		return (EINVAL);
		}
	if (phy.phy_id >= 32) {
		return (EINVAL);
		}
	if (ifd->ifd_cmd == READ_PHY_REG)
		phy.phy_data = ti814x_mdi_read (ti814x, phy.phy_id, phy.phy_reg);
	else
		ti814x_mdi_write (ti814x, phy.phy_id, phy.phy_reg, phy.phy_data);
	copyout (&phy, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(phy));
	return (EOK);
}

/*****************************************************************************/
/* Callback routine to check for link up/down                                */
/*****************************************************************************/

void ti814x_mdi_callback (void *handle, uint8_t phyaddr, uint8_t linkstate)

{
	ti814x_dev_t		*ti814x = handle;
	int					i, mode, gig_en=0;
	uint32_t			reg, mac_control_reg;
	char				*s = 0;
	struct ifnet		*ifp = &ti814x->ecom.ec_if;
	nic_config_t		*cfg = &ti814x->cfg;
	uint16_t		advert, lpadvert;
	uint32_t		q1_send, q2_send, q3_send;
	uint32_t		port_bw, percent_reg, value;

	if (cfg->verbose & DEBUG_MII) {
		slogf(_SLOGC_NETWORK, _SLOG_INFO, "%s:%d phyaddr=%d linkstate=%d devidx=%d",
				__FUNCTION__, __LINE__, phyaddr, linkstate, cfg->device_index);
	}

	switch (linkstate) {
		case	MDI_LINK_UP:
		if ((i = MDI_GetActiveMedia (ti814x->mdi, cfg->phy_addr, &mode)) != MDI_LINK_UP) {
			if (cfg->verbose & DEBUG_MII)
				slogf(_SLOGC_NETWORK, _SLOG_INFO, "MDI_GetActiveMedia returned %x phy_addr: %d", i, cfg->phy_addr);
			mode = 0;
		}

		switch (mode) {
			case	MDI_10bTFD:
				s = "10 BaseT Full Duplex";
				cfg->media_rate = 10000L;
				cfg->duplex = 1;
				break;
			case	MDI_10bT:
				s = "10 BaseT Half Duplex";
				cfg->duplex = 0;
				cfg->media_rate = 10000L;
				break;
			case	MDI_100bTFD:
				s = "100 BaseT Full Duplex";
				cfg->duplex = 1;
				cfg->media_rate = 100000L;
				break;
			case	MDI_100bT:
				s = "100 BaseT Half Duplex";
				cfg->duplex = 0;
				cfg->media_rate = 100000L;
				break;
			case	MDI_100bT4:
				s = "100 BaseT4";
				cfg->duplex = 0;
				cfg->media_rate = 100000L;
				break;
			case MDI_1000bT:
				s = "1000 BaseT Half Duplex !!!NOT SUPPORTED!!!";
				cfg->duplex = 0;
				cfg->media_rate = 1000 * 1000L;
				gig_en=1;
				break;
			case MDI_1000bTFD:
				s = "1000 BaseT Full Duplex";
				cfg->duplex = 1;
				cfg->media_rate = 1000 * 1000L;
				gig_en=1;
				break;
			default:
				s = "Unknown";
				cfg->duplex = 0;
				cfg->media_rate = 0L;
				break;
			}
		if (cfg->verbose & DEBUG_MASK) {
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s:%d - Link up (%s) if=%d", __FUNCTION__, __LINE__, s, cfg->device_index);
		}

		/* Set Duplex on MAC */
		switch (ti814x->cfg.device_index) {
			case 0: mac_control_reg = ti814x->cpsw_regs + SL1_MAC_CONTROL; break;
			case 1: mac_control_reg = ti814x->cpsw_regs + SL2_MAC_CONTROL; break;
			default:
				slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s:%d - Invalid device index (%d)",
								__FUNCTION__, __LINE__, ti814x->cfg.device_index );
				return;
		}
		reg = inle32 (mac_control_reg);
		reg &= ~(FULLDUPLEX | EXT_EN | GIG_MODE);

		if (cfg->duplex)
			reg |= FULLDUPLEX;
		else
			reg &= ~FULLDUPLEX;

		if (cfg->media_rate == 100000L)
			reg |= MACIFCTL_A;
		else
			reg &= ~MACIFCTL_A;

		if (cfg->media_rate == 10000L)
			reg |= EXT_EN;

		if (gig_en)
#ifndef	J6
			reg |= (GIG_MODE | EXT_EN);
#else
			reg |= GIG_MODE;
#endif
		if (ti814x->flow == -1) {
		    /* Flow control was autoneg'd, set what we got in the MAC */
		    advert = ti814x_mdi_read(ti814x, cfg->phy_addr, MDI_ANAR);
		    lpadvert = ti814x_mdi_read(ti814x, cfg->phy_addr, MDI_ANLPAR);
		    if (advert & MDI_FLOW) {
			if (lpadvert & MDI_FLOW) {
			    /* Enable Tx and Rx of Pause */
			    ti814x->flow_status = IFM_ETH_RXPAUSE |
						  IFM_ETH_TXPAUSE;
			} else if ((advert & MDI_FLOW_ASYM) &&
				   (lpadvert & MDI_FLOW_ASYM)) {
			    /* Enable Rx of Pause */
			    ti814x->flow_status = IFM_ETH_RXPAUSE;
			} else {
			    /* Disable all pause */
			    ti814x->flow_status = 0;
			}
		    } else if ((advert & MDI_FLOW_ASYM) &&
			       (lpadvert & MDI_FLOW) &&
			       (lpadvert & MDI_FLOW_ASYM)) {
			/* Enable Tx of Pause */
			ti814x->flow_status = IFM_ETH_TXPAUSE;
		    } else {
			/* Disable all pause */
			ti814x->flow_status = 0;
		    }
		} else if (ti814x->flow == IFM_FLOW) {
		    ti814x->flow_status = IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE;
		} else {
		    ti814x->flow_status = ti814x->flow;

		}
		reg &= ~(RX_FLOW_EN | TX_FLOW_EN);
		if (ti814x->flow_status & IFM_ETH_RXPAUSE) {
		    reg |= TX_FLOW_EN;
		}
		if (ti814x->flow_status & IFM_ETH_TXPAUSE) {
		    reg |= RX_FLOW_EN;
		}

		outle32 (mac_control_reg, reg);

		/*
		 * Recalculate the send percentage
		 * If link is down then ignore for now.
		 * Send percent will be set on link speed change
		 * when link comes up.
		 */
		port_bw = ti814x->cfg.media_rate / 1000; /* In Mbps */
		if (port_bw) {
		    if (!ti814x->cfg.device_index) {
			percent_reg = P1_SEND_PERCENT;
			q1_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(1 * 2 * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
			q2_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(2 * 2 * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
			q3_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(3 * 2 * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
		    } else {
			percent_reg = P2_SEND_PERCENT;
			q1_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(((1 * 2) + 1) * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
			q2_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(((2 * 2) + 1) * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
			q3_send = (in32(ti814x->cpsw_regs + TX_PRI0_RATE +
					(((3 * 2) + 1) * sizeof(uint32_t)))) >>
					PRI_SEND_SHIFT;
		    }
		    value = (((q3_send * 100) + port_bw - 1) / port_bw) << 16;
		    value |= (((q2_send * 100) + port_bw - 1) / port_bw) << 8;
		    value |= ((q1_send * 100) + port_bw - 1) / port_bw;
		    out32(ti814x->cpsw_regs + percent_reg, value);
		}

		cfg->flags &= ~NIC_FLAG_LINK_DOWN;
		ti814x->linkup = 1;
		if_link_state_change(ifp, LINK_STATE_UP);
                if (ti814x->tx_q_len[0]) {
		    /*
		     * There were some Tx descriptors in use
		     * when the link went down. Kick the start
		     * to ensure things keep running.
		     */
		    NW_SIGLOCK(&ifp->if_snd_ex, ti814x->iopkt);
		    ti814x_start(ifp);
		}
		break;

	case	MDI_LINK_DOWN:
		cfg->media_rate = cfg->duplex = -1;
		MDI_AutoNegotiate(ti814x->mdi, cfg->phy_addr, NoWait);
		cfg->flags |= NIC_FLAG_LINK_DOWN;
		ti814x->linkup = 0;

		if (cfg->verbose & DEBUG_MASK) {
			slogf(_SLOGC_NETWORK, _SLOG_INFO, "%s:%d: Link down %d",
				  __FUNCTION__, __LINE__, cfg->lan);
		}

		if_link_state_change(ifp, LINK_STATE_DOWN);
		break;

	default:
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s:%d: Unknown link state %hhu",
			   __FUNCTION__, __LINE__, linkstate);
		break;
		}
}

/*****************************************************************************/
/* Check for link up/down condition of NXP PHY.                              */
/*****************************************************************************/

void	ti814x_br_phy_state (ti814x_dev_t *ti814x)

{
	uint16_t		phy_data, snr;
	nic_config_t	*cfg = &ti814x->cfg;
	struct ifnet    *ifp = &ti814x->ecom.ec_if;
	uint32_t		reg, mac_control_reg;
	char			err[60];

	err[0] = 0;
	phy_data = ti814x_mdi_read (ti814x, cfg->phy_addr, INT_STATUS_REG);
	if (ti814x->brmast) {
		if (phy_data & TRAINING_FAILED) {
			phy_data = ti814x_mdi_read (ti814x, cfg->phy_addr, EXT_CONTROL_REG);
			ti814x_mdi_write (ti814x, cfg->phy_addr, EXT_CONTROL_REG, (phy_data | TRAINING_RESTART));
			return;
		}
	}
	if (phy_data & PHY_INT_ERR) {
		if (phy_data & PHY_INIT_FAIL)
			strcat (err, "PHY Init failure - ");
		if (phy_data & LINK_STATUS_FAIL)
			strcat (err, "Link status failure - ");
		if (phy_data & SYM_ERR)
			strcat (err, "Symbol error - ");
		if (phy_data & CONTROL_ERR)
			strcat (err, "SMI control error - ");
		if (phy_data & UV_ERR)
			strcat (err, "Under voltage error - ");
		if (phy_data & TEMP_ERR)
			strcat (err, "Temperature error");
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: PHY INT ERR 0x%x - %s", __func__, phy_data, err);
		err[0] = 0;
	}
	phy_data = ti814x_mdi_read (ti814x, cfg->phy_addr, COMM_STATUS_REG);
	snr = (phy_data & SNR_MASK) >> SNR_SHIFT;
	if (snr != ti814x->br_snr && (phy_data & LINK_UP)) {
		ti814x->br_snr = snr;
		if (!snr)
			strcat (err, "Worse than class A");
		else
			sprintf (err, "class %c", snr + 0x40);
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s: SNR %s", __func__, err);
	}
	if (phy_data & COMM_STATUS_ERR) {
		if (phy_data & SSD_ERR)
			strcat (err, "SSD error - ");
		if (phy_data & ESD_ERR)
			strcat (err, "ESD error - ");
		if (phy_data & RX_ERR)
			strcat (err, "Receive error - ");
		if (phy_data * TX_ERR)
			strcat (err, "Transmit error");
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: COMM STATUS ERR 0x%x - %s", __func__, phy_data, err);
		err[0] = 0;
	}
	if ((cfg->flags & NIC_FLAG_LINK_DOWN) && (phy_data & LINK_UP)) {
		/* Link was down and is now up */
		if (cfg->verbose) {
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s(): Link up", __func__);
		}
		ti814x->linkup = 1;
		cfg->flags &= ~NIC_FLAG_LINK_DOWN;
		if_link_state_change (ifp, LINK_STATE_UP);
		phy_data = ti814x_mdi_read (ti814x, cfg->phy_addr, BASIC_CONTROL);
		cfg->media_rate = (phy_data & BC_SPEED_SEL) ? 100000L : 10000L;
		cfg->duplex = (phy_data & BC_DUPLEX) ? 1 : 0;
		mac_control_reg = ti814x->cpsw_regs + SL1_MAC_CONTROL;
		reg = inle32 (mac_control_reg);
		reg &= ~(FULLDUPLEX | EXT_EN | GIG_MODE);
		if (cfg->duplex)
			reg |= FULLDUPLEX;
		else
			reg &= ~FULLDUPLEX;

		if (cfg->media_rate == 100000L)
			reg |= MACIFCTL_A;
		else
			reg &= ~MACIFCTL_A;

		if (cfg->media_rate == 10000L)
			reg |= EXT_EN;
		outle32 (mac_control_reg, reg);

		if (!IFQ_IS_EMPTY (&ifp->if_snd)) {
			/*
			 * Packets were still in the queue when the link
			 * went down, call start to get them moving again.
			 */
			NW_SIGLOCK (&ifp->if_snd_ex, ti814x->iopkt);
			ti814x_start (ifp);
		}
    }
    else if (((cfg->flags & NIC_FLAG_LINK_DOWN) == 0) && ((phy_data & LINK_UP) == 0)) {
		/* Link was up and is now down */
		if (cfg->verbose) {
			slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s(): Link down", __func__);
		}
		ti814x->linkup = 0;
		ti814x->br_snr = 0;
		cfg->flags |= NIC_FLAG_LINK_DOWN;
		if_link_state_change (ifp, LINK_STATE_DOWN);
    }
	phy_data = ti814x_mdi_read (ti814x, cfg->phy_addr, EXTERN_STATUS_REG);
	if (phy_data) {
		switch (phy_data) {
			case	UV_VDDA_3V3:
				strcat (err, "Undervoltage 3V3");
				break;
			case	UV_VDDD_1V8:
				strcat (err, "Undervoltage VDDD 1V8");
				break;
			case	UV_VDDA_1V8:
				strcat (err, "Undervoltage VDDA 1V8");
				break;
			case	UV_VDDIO:
				strcat (err, "Undervoltage VDDIO");
				break;
			case	TEMP_HIGH:
				strcat (err, "Temperature high");
				break;
			case	TEMP_WARN:
				strcat (err, "Temperature warning");
				break;
			case	SHORT_DETECT:
				strcat (err, "Short circuit detected");
				break;
			case	OPEN_DETECT:
				strcat (err, "Open circuit detected");
				break;
			case	POL_DETECT:
				strcat (err, "Polarity inversion detected");
				break;
			case	INTL_DETECT:
				strcat (err, "Interleave detect");
				break;
			default:
				break;
		}
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: External status 0x%x - %s", __func__, phy_data, err);
	}
}

/*******************************************************************************/
/* Setup for NXP TJA1100 BroadReach PHY                                        */
/*******************************************************************************/

int		ti814x_init_broadreach_phy (ti814x_dev_t *ti814x)

{
	int			phy_addr = ti814x->cfg.phy_addr;
	int			timeout = 100;
	uint16_t	phy_data;

	phy_data = ti814x_mdi_read (ti814x, phy_addr, INT_STATUS_REG);
	if (phy_data & PHY_INIT_FAIL) {
		phy_data = BC_RESET;
		ti814x_mdi_write (ti814x, phy_addr, BASIC_CONTROL, phy_data);
		while (timeout) {
			if (!(phy_data = ti814x_mdi_read (ti814x, phy_addr, BASIC_CONTROL) & BC_RESET))
				break;
			timeout--;
			nanospin_ns (1000);
		}
		if (!timeout) {
			slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: PHY did not come out of reset", __func__);
			return -1;
		}
		phy_data = ti814x_mdi_read (ti814x, phy_addr, INT_STATUS_REG);
	}
	if (phy_data & PHY_INIT_FAIL) {
		slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s: PHY initialization failed", __func__);
		return -1;
	}

	phy_data = ti814x_mdi_read (ti814x, phy_addr, EXT_CONTROL_REG);
	phy_data |= CONFIG_ENABLE;
	ti814x_mdi_write (ti814x, phy_addr, EXT_CONTROL_REG, phy_data);
	nic_delay (2);
	phy_data = ti814x_mdi_read (ti814x, phy_addr, CONFIG_REG_1);
	phy_data &= ~(MASTER_SLAVE | AUTO_OP | (MII_MODE << MII_MODE_SHIFT));
	if (ti814x->brmast) {
		phy_data |= MASTER_SLAVE;
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s: PHY set to master mode", __func__);
	}
	else {
		slogf (_SLOGC_NETWORK, _SLOG_INFO, "%s: PHY set to slave mode", __func__);
	}
	ti814x_mdi_write (ti814x, phy_addr, CONFIG_REG_1, phy_data);
	phy_data = ti814x_mdi_read (ti814x, phy_addr, CONFIG_REG_2);
	phy_data |= JUMBO_ENABLE;
	ti814x_mdi_write (ti814x, phy_addr, CONFIG_REG_2, phy_data);
	phy_data = ((NORMAL_MODE << POWER_MODE_SHIFT) | LINK_CONTROL | CONFIG_INH | WAKE_REQUEST);
	ti814x_mdi_write (ti814x, phy_addr, EXT_CONTROL_REG, phy_data);
	nic_delay (5);
	return 0;
}

/*******************************************************************************/
/* Check to see if we have a BroadReach PHY                                    */
/*******************************************************************************/

int		ti814x_is_br_phy (ti814x_dev_t *ti814x)

{
	uint32_t phy_id = 0, vendor= 0, model=0;
	uint32_t phy_addr = ti814x->cfg.phy_addr;
	static int is_br = -1;

	if (is_br == -1) {
		// Make sure we got the good phy address before we do the mii_read
		if (phy_addr == -1) {
			if (ti814x->cfg.verbose & DEBUG_MII) {
				slogf (_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PhyAddr = %d", __func__, phy_addr);
			}
		} else {
			phy_id = (ti814x_mdi_read (ti814x, phy_addr, MDI_PHYID_1)) << 16;
			phy_id |= ti814x_mdi_read (ti814x, phy_addr, MDI_PHYID_2);

			vendor = PHYID_VENDOR (phy_id);
			model = PHYID_MODEL (phy_id);
			if (ti814x->cfg.verbose & DEBUG_MII)
				slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "%s: Vendor %x - Model %x", __func__, vendor, model);

			switch (vendor) {
				case	NXP:
					switch (model) {
						case	TJA1100:
						case	TJA1100_1:
							is_br = 1;
							break;
						default:
							is_br = 0;
							break;
					}
					break;
				default:
					is_br = 0;
					break;
			}
		}
	}
	return is_br;
}

/*****************************************************************************/
/*                                                                           */
/*****************************************************************************/

void ti814x_MDI_MonitorPhy (void *arg)

{
	ti814x_dev_t	*ti814x = arg;
	struct  ifnet	*ifp;

	ifp = &ti814x->ecom.ec_if;

	/* Reap if not recently done */
	if (!ti814x->tx_reaped) {
	    NW_SIGLOCK(&ifp->if_snd_ex, ti814x->iopkt);
	    ti814x_reap_pkts(ti814x, 0);
	    NW_SIGUNLOCK(&ifp->if_snd_ex, ti814x->iopkt);
	}
	ti814x->tx_reaped = 0;

#ifndef SWITCHMODE
	/*
	 * Only monitor the link if it's the 2nd link - the first uses
	 * interrupts, unless emu_phy is specified or it is an ECO
	 *
	 * And:
	 *   1) the user forces it, or
	 *   2) the link state is unknown, or
	 *   3) there's no traffic.
	 */

	if (
#ifndef J5_ECO
	    (ti814x->cfg.device_index || (ti814x->emu_phy != -1)) &&
#endif
	    (ti814x->probe_phy ||
	     (ti814x->cfg.flags & NIC_FLAG_LINK_DOWN) ||
	     (ti814x->cfg.media_rate <= 0) ||
	     !ti814x->pkts_received)) {
		if (!(ti814x_is_br_phy (ti814x))) {
			MDI_MonitorPhy(ti814x->mdi);
		}
		else {
			ti814x_br_phy_state (ti814x);
		}
	}
	ti814x->pkts_received = 0;
#endif

	callout_msec(&ti814x->mii_callout, 3 * 1000,
		     ti814x_MDI_MonitorPhy, ti814x);
}

/*****************************************************************************/
/* MII PHY Interface Routines...                                             */
/*****************************************************************************/
#define NIC_MDI_PRIORITY 10 // mii timer

int ar8031_phy_setup(ti814x_dev_t *ti814x)

{
    nic_config_t        *cfg         = &ti814x->cfg;
    int                 phy_idx     = cfg->phy_addr;
    unsigned short val;

    /* Enable AR8031 to output a 125MHz clk from CLK_25M */
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x7);
    ti814x_mdi_write(ti814x, phy_idx, 0xe, 0x8016);
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x4007);
    val = ti814x_mdi_read(ti814x, phy_idx, 0xe);

    val &= 0xffe3;
    val |= 0x18;
    ti814x_mdi_write(ti814x, phy_idx, 0xe, val);

    /* Introduce tx clock delay to meet data setup and hold times */
    ti814x_mdi_write(ti814x, phy_idx, 0x1d, 0x5);
    val = ti814x_mdi_read(ti814x, phy_idx, 0x1e);
    val |= 0x0100;
    ti814x_mdi_write(ti814x, phy_idx, 0x1e, val);

    /*
     * Disable SmartEEE
     * The Tx delay can mean late pause and bad timestamps.
     */
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x3);
    ti814x_mdi_write(ti814x, phy_idx, 0xe, 0x805d);
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x4003);
    val = ti814x_mdi_read(ti814x, phy_idx, 0xe);
    val &= ~(1 << 8);
    ti814x_mdi_write(ti814x, phy_idx, 0xe, val);

    /* As above for EEE (802.3az) */
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x7);
    ti814x_mdi_write(ti814x, phy_idx, 0xe, 0x3c);
    ti814x_mdi_write(ti814x, phy_idx, 0xd, 0x4007);
    ti814x_mdi_write(ti814x, phy_idx, 0xd,0);

    return 0;
}

/*****************************************************************************/
/* Find a PHY                                                                */
/*****************************************************************************/

int ti814x_findphy(ti814x_dev_t *ti814x)

{
	int					timeout, i;
	int					phy_found = 0;
	uint16_t			reg, reg1;
	volatile uint32_t	phyid;
	int					devidx;
	nic_config_t		*cfg = &ti814x->cfg;

	outle32 (ti814x->cpsw_regs + MDIOCONTROL, MDIO_ENABLE | MDIO_FAULT | MDIO_FAULTENB | MDIO_CLK_DIVISOR);
	ti814x->phy_init = 1;

	/* Wait for MDIO to be IDLE */
	timeout = 1000;
	while (timeout) {
		if (!(inle32 (ti814x->cpsw_regs + MDIOCONTROL) & MDIO_IDLE))
			break;
		nanospin_ns (1000);
		timeout--;
	}
	if (timeout <= 0) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "PHY init timeout");
		return (-1);
	}

	/* Look for an active PHY */
	devidx = cfg->device_index;

	if (ti814x->emu_phy != -1 && ti814x->emu_phy == devidx) {
		emu_phy_init (ti814x);
	}

	/* 4.3 mdio_fs_v1p5p1: Each of the 32 bits of this register (ALIVE) is set if the
	 * most recent access to the PHY with address corresponding to the register
	 * bit number was acknowledged by the PHY, the bit is reset if the PHY
	 * fails to acknowledge the access.
	 */
	/* As such we need to iterate through all 32 possible PHYS, read from them
	 * and only then will the ALIVE bits be representative of what's actually
	 * present on the board */

	if (ti814x->phy_idx != -1) {
		ti814x_mdi_read (ti814x, ti814x->phy_idx, MDI_BMSR);
	}
	else {
		for (i = 0; i < 32; i++) {
			ti814x_mdi_read(ti814x, i, MDI_BMSR);
			nanospin_ns (50000);
		}
	}

	phyid = inle32 (ti814x->cpsw_regs + MDIOALIVE);

	if (cfg->verbose & DEBUG_MII) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "MDIOALIVE=0x%08x", phyid);
	}
	if (ti814x->emu_phy == -1) {
		if ((phyid) && (phyid != 0xffffffff)) {
			if (ti814x->phy_idx == -1) {
				for (i = 0; i < 32; i++) {
					if (phyid & (1 << i)) {
						if (!devidx)
							break;	/* Phy matched device index */
						else
							devidx--;   /* decrement phy index for next loop */
					}
				}
				phyid = i;
				cfg->phy_addr = phyid;
				phy_found = 1;
			}
			else {
				phyid = (phyid & (1 << ti814x->phy_idx));
				if (phyid) {
					phy_found = 1;
					cfg->phy_addr = ti814x->phy_idx;
					phyid = ti814x->phy_idx;
				}
			}
		}
	}
	else {
		phy_found = 1;
		phyid = cfg->phy_addr;
	}
	if (phy_found) {
		if (cfg->verbose & DEBUG_MII) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,
				"%s:%d - MII transceiver found at address %d",
				__FUNCTION__, __LINE__, cfg->phy_addr);
		}
	}

	if ((!phy_found) || (phyid==32)) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s:%d - No PHY found",
			__FUNCTION__, __LINE__);
		return (-1);
	}

	reg = ti814x_mdi_read(ti814x, phyid, MDI_BMSR);
	reg1 = ti814x_mdi_read(ti814x, phyid, MDI_EMSR);
	if (reg == 0xffff)
		return (-1);
	if (reg & BMSR_EXT_STATUS) {
		if (!(reg1 & 0xf000))
			ti814x->no_gig = 1;
	}
	else
		ti814x->no_gig = 1;

	return (EOK);
}

/*****************************************************************************/
/* MII PHY Initialization                                                    */
/*****************************************************************************/

int ti814x_initphy(ti814x_dev_t *ti814x)
{
    int			rc;
    uint16_t		reg, reg1;
    uint32_t		phyid;
    nic_config_t	*cfg;
    struct ifnet	*ifp;

    cfg = &ti814x->cfg;
    ifp = &ti814x->ecom.ec_if;

    if (ti814x_findphy(ti814x) != EOK) {
		return -1;
    }

	if (ti814x_is_br_phy (ti814x)) {
		slogf (_SLOGC_NETWORK, _SLOG_DEBUG1, "%s: Broadreach PHY found", __func__);
		if (ti814x_init_broadreach_phy (ti814x))
			return -1;
		cfg->connector = NIC_CONNECTOR_MII;
		return 0;
	}

    phyid = cfg->phy_addr;

    rc = MDI_Register_Extended(ti814x, ti814x_mdi_write, ti814x_mdi_read,
	ti814x_mdi_callback, (mdi_t **)&ti814x->mdi, NULL, 0, 0);
    if (rc != MDI_SUCCESS) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,
		    "%s:%d - MDI_Register_Extended failed rc=%d",
		    __FUNCTION__, __LINE__, rc);
	return -1;
    }

    cfg->connector = NIC_CONNECTOR_MII;

    /* Permit the MDIO State machine to monitor the link status */
    if (ti814x->emu_phy == -1) {
	switch (ti814x->cfg.device_index) {
	case 0:
	    outle32(ti814x->cpsw_regs + MDIOUSERPHYSEL0,
		LINKINT_ENABLE | phyid);
	    break;
	case 1:
	    outle32(ti814x->cpsw_regs + MDIOUSERPHYSEL1,
		LINKINT_ENABLE | phyid);
	    break;
	default:
	    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "Invalid Device Index");
	    break;
	}
    }

    if (MDI_InitPhy(ti814x->mdi, phyid) != MDI_SUCCESS) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR, "InitPhy failed");
		return -1;
    }

    MDI_ResetPhy(ti814x->mdi, phyid, WaitBusy);

    /* Fixup specific PHY brokenness */
    if (ti814x->fix_mii_clk) {
	reg = ti814x_mdi_read (ti814x, phyid, MDI_PHYID_1);
	reg1 = ti814x_mdi_read (ti814x, phyid, MDI_PHYID_2);
	if (reg == 0x22 && reg1 == 0x1556) { /* Micrel RMII PHY */
	    reg = ti814x_mdi_read(ti814x, phyid, 0x1f);
	    reg |= (1<<7);	/* Set 50Mhz clock bit */
	    ti814x_mdi_write(ti814x, phyid, 0x1f, reg);
	    if (cfg->verbose) {
		reg = ti814x_mdi_read (ti814x, phyid, 0x1f);
		slogf(_SLOGC_NETWORK, _SLOG_DEBUG1,
		    "PHY Reg 0x1f set to - %x", reg);
	    }
	}
    }

    if ((ti814x->mdi->PhyData[cfg->phy_addr]->VendorOUI == KENDIN) &&
	(ti814x->mdi->PhyData[cfg->phy_addr]->Model == KSZ8051)) {
	ti814x_mdi_write(ti814x, phyid, 0x16, 0x2);
    }

#ifdef AM335X
    if ((ti814x->mdi->PhyData[cfg->phy_addr]->VendorOUI == ATHEROS) &&
	(ti814x->mdi->PhyData[cfg->phy_addr]->Model == AR8031)) {
	slogf(_SLOGC_NETWORK, _SLOG_DEBUG1,
	    "AM335X board with Atheros AR8031 PHY");

	/* Set the AM335x to generate the RMII ref clock. */
	reg = inle32(ti814x->cmr_base + GMII_SET);
	reg &= ~(GMII0_RMII_CLKIN | GMII1_RMII_CLKIN);
	outle32(ti814x->cmr_base + GMII_SET, reg);

	/*
	 * As per TI SPRZ360F errata advisory 1.0.10, the AM335X does not
	 * support internal delay mode so the Atheros phy must be configured
	 * to add an internal delay on its tx clk input.
	 */
	ar8031_phy_setup(ti814x);
    }
#endif

    /* Start off down */
    MDI_PowerdownPhy(ti814x->mdi, phyid);
    cfg->flags |= NIC_FLAG_LINK_DOWN;
    if_link_state_change(ifp, LINK_STATE_DOWN);

    return EOK;
}

/*****************************************************************************/
/* MII PHY De-register Interface                                             */
/*****************************************************************************/

void ti814x_finiphy(ti814x_dev_t *ti814x)
{
    MDI_DeRegister(&ti814x->mdi);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/dm814x/mii.c $ $Rev: 804097 $")
#endif
