/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#ifndef _RTGALAXY_SOC_H_
#define _RTGALAXY_SOC_H_

#define RTGALAXY_DEFAULT_LEXRA_MEMBASE	0x01b00000
#define RTGALAXY_DEFAULT_HIGHMEM_START	0x02000000

#define RTGALAXY_CHIPID_VENUS		0x1281
#define RTGALAXY_CHIPID_NEPTUNE		0x1282
#define RTGALAXY_CHIPID_MARS		0x1283
#define RTGALAXY_CHIPID_JUPITER		0x1284

static inline int rtgalaxy_is_venus_soc(void)
{
	return (rtgalaxy_info.chip_id == RTGALAXY_CHIPID_VENUS);
}

static inline int rtgalaxy_is_neptune_soc(void)
{
	return (rtgalaxy_info.chip_id == RTGALAXY_CHIPID_NEPTUNE);
}

static inline int rtgalaxy_is_mars_soc(void)
{
	return (rtgalaxy_info.chip_id == RTGALAXY_CHIPID_MARS);
}

static inline int rtgalaxy_is_jupiter_soc(void)
{
	return (rtgalaxy_info.chip_id == RTGALAXY_CHIPID_JUPITER);
}

#endif
