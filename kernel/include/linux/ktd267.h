/*
*	KTD267 FLASH LED specific header file.
*
*
*/

#ifndef __KTD267_H__
#define __KTD267_H__

enum ktd267_mode {
	KTD267_MODE_FLASH,
	KTD267_MODE_MOVIE,
	KTD267_MODE_TERMINATE,
};

struct ktd267_platform_data {
	signed int	gpio_ENM;
	signed int	gpio_ENF;
};

extern struct ktd267_platform_data ktd267data;

int ktd267_set_mode(enum ktd267_mode mode);

#endif
