/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2010 Advanced Micro Devices, Inc.
 * Copyright (C) 2015 Timothy Pearson <tpearson@raptorengineeringinc.com>, Raptor Engineering
 * Copyright (C) 2017 Damien Zammit <damien@zamaudio.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <inttypes.h>
#include "mct_d.h"
#include "mct_d_gcc.h"

/* This is for socket FT3 Fam16h
 */

static void Get_ChannelPS_Cfg0_D(u8 MAAdimms, u8 Speed, u8 MAAload,
				u8 DATAAload, u32 *AddrTmgCTL, u32 *ODC_CTL,
				u8 *CMDmode);


void mctGet_PS_Cfg_D(struct MCTStatStruc *pMCTstat,
			 struct DCTStatStruc *pDCTstat, u32 dct)
{
	Get_ChannelPS_Cfg0_D(pDCTstat->MAdimms[dct], pDCTstat->Speed,
				pDCTstat->MAload[dct], pDCTstat->DATAload[dct],
				&(pDCTstat->CH_ADDR_TMG[dct]), &(pDCTstat->CH_ODC_CTL[dct]),
				&pDCTstat->_2Tmode);
}

/*
 *  In: MAAdimms   - number of DIMMs on the channel
 *    : Speed      - Speed (see DCTStatstruc.Speed for definition)
 *    : MAAload    - number of address bus loads on the channel
 *    : DATAAload  - number of ranks on the channel
 * Out: AddrTmgCTL - Address Timing Control Register Value
 *    : ODC_CTL    - Output Driver Compensation Control Register Value
 *    : CMDmode    - CMD mode
 */
static void Get_ChannelPS_Cfg0_D(u8 MAAdimms, u8 Speed, u8 MAAload,
				u8 DATAAload, u32 *AddrTmgCTL, u32 *ODC_CTL,
				u8 *CMDmode)
{
	*AddrTmgCTL = 0;
	*ODC_CTL = 0;
	*CMDmode = 1;

	if (Speed == 4 || Speed == 5) {
		*AddrTmgCTL = 0x00000000;
	} else if (Speed == 6) {
		if (MAAdimms == 1) {
			if (DATAAload == 1)
				*AddrTmgCTL = 0x003D3D3D;
			else
				*AddrTmgCTL = 0x00000000;
		} else {
			*AddrTmgCTL = 0x00000000;
		}
	} else if (Speed == 7) {
		if (MAAdimms == 1) {
			if (DATAAload == 1)
				*AddrTmgCTL = 0x003D3D3D;
			else
				*AddrTmgCTL = 0x00003D3D;
		} else {
			*AddrTmgCTL = 0x00000000;
		}
	} else if (Speed >= 8) {
		if (MAAdimms == 1) {
			if (DATAAload == 1)
				*AddrTmgCTL = 0x003C3C3C;
			else
				*AddrTmgCTL = 0x00003C3C;
		} else {
			*AddrTmgCTL = 0x00000000;
		}
	} else {
		*AddrTmgCTL = 0x00000000;
	}

	if (Speed == 4) {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x00002222;
		} else {
			*ODC_CTL = 0x10222323;
		}
	} else if (Speed == 5) {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x00002222;
		} else {
			*ODC_CTL = 0x20222323;
		}
	} else if (Speed == 6) {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x10002222;
		} else {
			*ODC_CTL = 0x30222323;
		}
	} else if (Speed == 7) {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x20112222;
		} else {
			*ODC_CTL = 0x30222323;
		}
	} else if (Speed == 8) {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x30332222;
		} else {
			*ODC_CTL = 0x30222323;
		}
	} else {
		if (MAAdimms == 1) {
			*ODC_CTL = 0x30332222;
		} else {
			*ODC_CTL = 0x30222323;
		}
	}
}
