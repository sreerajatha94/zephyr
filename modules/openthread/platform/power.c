/*
 * Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME net_openthread_power
#define LOG_LEVEL       CONFIG_OPENTHREAD_PLATFORM_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>

#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/toolchain.h>
#include <openthread/tasklet.h>

#include "platform-zephyr.h"
#include "power.h"

struct platform_power_state {
	bool initialized;
	bool deep_sleep_allowed;
	bool deep_sleep_locked;
	bool milli_armed;
	bool milli_fired;
	bool micro_armed;
	bool micro_fired;
	uint32_t next_wake_ms;
	uint32_t next_wake_us;
	uint32_t pending_signals;
	uint32_t processed_signals;
	uint32_t processing_signals;
};

static struct platform_power_state power_state;

OT_TOOL_WEAK bool platformPowerAllowDeepSleepCallback(void)
{
	return true;
}

static bool alarm_is_imminent_32(bool armed, uint32_t now, uint32_t deadline, uint32_t threshold)
{
	if (!armed) {
		return false;
	}

	return (int32_t)(deadline - now) < (int32_t)threshold;
}

static bool platform_power_platform_event_pending(void)
{
	if (power_state.milli_fired || power_state.micro_fired || platformRadioIsPending()) {
		return true;
	}

	if (IS_ENABLED(CONFIG_OPENTHREAD_COPROCESSOR) && platformUartIsPending()) {
		return true;
	}

	return false;
}

static bool platform_power_driver_pass_pending(void)
{
	return power_state.pending_signals != power_state.processed_signals ||
	       platform_power_platform_event_pending();
}

bool platformPowerHasPendingEvents(otInstance *instance)
{
	if (!power_state.initialized) {
		return false;
	}

	return platform_power_driver_pass_pending() ||
	       (instance != NULL && otTaskletsArePending(instance));
}

static bool platform_power_should_lock_deep_sleep(otInstance *instance)
{
	const uint32_t min_sleep_ms = CONFIG_OPENTHREAD_PLATFORM_DEEP_SLEEP_MIN_MS;
	const uint32_t min_sleep_us = min_sleep_ms * USEC_PER_MSEC;

	if (!power_state.deep_sleep_allowed || !platformPowerAllowDeepSleepCallback() ||
	    platformPowerHasPendingEvents(instance)) {
		return true;
	}

	if (alarm_is_imminent_32(power_state.milli_armed, otPlatAlarmMilliGetNow(),
				 power_state.next_wake_ms, min_sleep_ms)) {
		return true;
	}

	if (IS_ENABLED(CONFIG_OPENTHREAD_PLATFORM_USEC_TIMER) &&
	    alarm_is_imminent_32(power_state.micro_armed, otPlatAlarmMicroGetNow(),
				 power_state.next_wake_us, min_sleep_us)) {
		return true;
	}

	return false;
}

static void platform_power_set_deep_sleep_lock(bool lock)
{
	if (lock && !power_state.deep_sleep_locked) {
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
		power_state.deep_sleep_locked = true;
	} else if (!lock && power_state.deep_sleep_locked) {
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
		power_state.deep_sleep_locked = false;
	}
}

static void platform_power_update_constraint(void)
{
	platform_power_set_deep_sleep_lock(platform_power_should_lock_deep_sleep(NULL));
}

static void platform_power_update_constraint_instance(otInstance *instance)
{
	platform_power_set_deep_sleep_lock(platform_power_should_lock_deep_sleep(instance));
}

void platformPowerInit(void)
{
	unsigned int key = irq_lock();

	power_state = (struct platform_power_state){
		.initialized = true,
		.deep_sleep_allowed = true,
		/*
		 * OpenThread startup submits initial work outside of otSysEventSignalPending().
		 * Start conservatively and release the deep-sleep lock after the first process pass.
		 */
		.pending_signals = 1U,
	};
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerProcessBegin(void)
{
	unsigned int key;

	if (!power_state.initialized) {
		return;
	}

	key = irq_lock();
	power_state.processing_signals = power_state.pending_signals;
	irq_unlock(key);
}

void platformPowerProcess(otInstance *instance)
{
	unsigned int key;

	if (!power_state.initialized || instance == NULL) {
		return;
	}

	key = irq_lock();

	if (power_state.pending_signals == power_state.processing_signals) {
		power_state.processed_signals = power_state.pending_signals;
		power_state.milli_fired = false;
		power_state.micro_fired = false;
	}

	platform_power_update_constraint_instance(instance);
	irq_unlock(key);
}

void platformPowerSignalPending(void)
{
	unsigned int key;

	if (!power_state.initialized) {
		return;
	}

	key = irq_lock();
	power_state.pending_signals++;
	platform_power_update_constraint();
	irq_unlock(key);
}

void platformPowerNotifyAlarmMilliStart(uint32_t fire_at_ms)
{
	unsigned int key = irq_lock();

	power_state.milli_armed = true;
	power_state.milli_fired = false;
	power_state.next_wake_ms = fire_at_ms;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerNotifyAlarmMilliStop(void)
{
	unsigned int key = irq_lock();

	power_state.milli_armed = false;
	power_state.milli_fired = false;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerNotifyAlarmMilliFired(void)
{
	unsigned int key = irq_lock();

	power_state.milli_armed = false;
	power_state.milli_fired = true;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerNotifyAlarmMicroStart(uint32_t fire_at_us)
{
	unsigned int key = irq_lock();

	power_state.micro_armed = true;
	power_state.micro_fired = false;
	power_state.next_wake_us = fire_at_us;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerNotifyAlarmMicroStop(void)
{
	unsigned int key = irq_lock();

	power_state.micro_armed = false;
	power_state.micro_fired = false;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerNotifyAlarmMicroFired(void)
{
	unsigned int key = irq_lock();

	power_state.micro_armed = false;
	power_state.micro_fired = true;
	platform_power_update_constraint();

	irq_unlock(key);
}

void platformPowerDeepSleepAllowedSet(bool allowed)
{
	unsigned int key = irq_lock();

	power_state.deep_sleep_allowed = allowed;
	platform_power_update_constraint();

	irq_unlock(key);
}
