/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_BATTERY_H_
#define APPLICATION_BATTERY_H_

/** Measure the battery voltage.
 *
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
int battery_sample(void);

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	uint16_t lvl_mV;

	/** Battery voltage at #lvl_percent remaining life. */
	uint8_t lvl_percent;
};

#endif /* APPLICATION_BATTERY_H_ */
