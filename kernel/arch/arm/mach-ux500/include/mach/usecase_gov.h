/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License terms: GNU General Public License (GPL) version 2
 */
#ifndef __MACH_USECASE_GOV_H
#define __MACH_USECASE_GOV_H

enum usecase_req_type {
	ENABLE_MAX_LIMIT = 0,
	DISABLE_MAX_LIMIT = 1,
	ENABLE_MIN_LIMIT = 2,
	DISABLE_MIN_LIMIT = 3,
};

enum usecase_req_cmd {
	REQ_RESET_VALUE = -1,
	REQ_NO_CHANGE = 0,
};

int set_usecase_config(int enable, int max, int min);

#endif /* __MACH_USECASE_GOV_H */
