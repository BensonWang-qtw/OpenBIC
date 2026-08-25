/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <logging/log.h>
#include <stdlib.h>
#include "libipmi.h"
#include "app_handler.h"
#include "kcs.h"
#include "rg3mxxb12.h"
#include "p3h284x.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "apml.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_i3c.h"
#include "util_sys.h"
#include "util_worker.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_mctp.h"
#include "plat_apml.h"
#include "plat_i3c.h"
#include "plat_isr.h"
#include "pcc.h"

#ifdef ENABLE_PLDM
#include "pldm_oem.h"
#endif

LOG_MODULE_REGISTER(plat_isr, LOG_LEVEL_DBG);

uint8_t hw_event_register[13] = { 0 };

add_sel_info event_work_items[] = {
	{
		.is_init = false,
		.gpio_num = RST_PLTRST_BIC_N,
		.event_type = PLTRST_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_BIOS_POST_CMPLT_BIC_N,
		.event_type = POST_COMPLETED,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_DBP_PRESENT_N,
		.event_type = HDT_PRSNT_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FAST_PROCHOT_N,
		.event_type = FAST_PROCHOT_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_CPU_BIC_THERMTRIP_N,
		.event_type = CPU_THERMAL_TRIP,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_CPU_BIC_PROCHOT_LVT3_N,
		.event_type = SYS_THROTTLE,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_HSC_TIMER_ALT_N,
		.event_type = HSC_OCP,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = IRQ_UV_DETECT_N,
		.event_type = P12V_STBY_UV,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = CPU_SMERR_BIC_N,
		.event_type = SYS_MANAGEMENT_ERROR,
		.assert_type = EVENT_ASSERTED,
	},
};

void init_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(event_work_items); ++index) {
		if (event_work_items[index].is_init != true) {
			k_work_init_delayable(&event_work_items[index].add_sel_work,
					      addsel_work_handler);

			event_work_items[index].is_init = true;
		}
	}
}

void addsel_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_sel_info *work_info = CONTAINER_OF(dwork, add_sel_info, add_sel_work);
	struct pldm_addsel_data msg = { 0 };
	msg.event_type = work_info->event_type;
	msg.assert_type = work_info->assert_type;

	if (send_event_log_to_bmc(msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send event log, event type: 0x%x, assert type: 0x%x",
			work_info->event_type, work_info->assert_type);
	};
}

void addsel_wrapper_work_handler(struct k_work *work_item)
{
	sel_work_wrapper *wrap = CONTAINER_OF(work_item, sel_work_wrapper, work);

	if (wrap->sel_data.event_type == 0) {
		LOG_ERR("Invalid wrapper SEL, skip sending SEL.");
		return;
	}

	if (send_event_log_to_bmc(wrap->sel_data) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send SEL: event_type=0x%x, assert_type=0x%x",
			wrap->sel_data.event_type, wrap->sel_data.assert_type);
	}
}

static add_sel_info *find_event_work_items(uint8_t gpio_num)
{
	for (int index = 0; index < ARRAY_SIZE(event_work_items); ++index) {
		if (event_work_items[index].gpio_num == gpio_num) {
			return &event_work_items[index];
		}
	}

	return NULL;
}

/*
 *	TODO: I3C hub is supplied by DC power currently. When DC off, it can't be initialized.
 *  	  Therefore, BIC needs to implement a workaround to reinitialize the I3C hub during DC on.
 *        This part will be removed after the hardware design be modified.
 */
void reinit_i3c_hub()
{
	uint16_t i3c_hub_type = I3C_HUB_TYPE_UNKNOWN;
	// i3c master initial
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS_HUB;
	i3c_msg.target_addr = I3C_ADDR_HUB;

	const int rstdaa_count = 2;
	int ret = 0;

	for (int i = 0; i < rstdaa_count; i++) {
		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
		if (ret != 0) {
			printf("Error to reset daa. count = %d\n", i);
		}
	}

	ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		printf("Error to set daa\n");
	}

	i3c_attach(&i3c_msg);
	init_i3c_hub_type();
	i3c_hub_type = get_i3c_hub_type();

	// Initialize I3C HUB
	if (i3c_hub_type == P3H2840_DEVICE_INFO) {
		if (!p3h284x_i3c_mode_only_init(&i3c_msg, p3h284x_cmd_initial,
						P3H284X_CMD_INITIAL_SIZE)) {
			printk("failed to initialize 1ou p3h284x\n");
		}
	} else if (i3c_hub_type == RTS4902A_DEVICE_INFO) {
		if (!rg3mxxb12_i3c_mode_only_init(&i3c_msg, rts4902a_cmd_initial,
						  RG3MXXB12_CMD_INITIAL_SIZE)) {
			printk("failed to initialize 1ou rts4902a\n");
		}
	} else {
		if (!rg3mxxb12_i3c_mode_only_init(&i3c_msg, rg3mxxb12_cmd_initial,
						  RG3MXXB12_CMD_INITIAL_SIZE)) {
			printk("failed to initialize 1ou rg3mxxb12\n");
		}
	}

	// Set FF/WF's EID
	send_cmd_to_dev_handler(NULL);
}

void set_ffwf_eid()
{
	// Set FF/WF's EID
	send_cmd_to_dev_handler(NULL);
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == false) && (get_4byte_postcode_ok() == false)) {
		LOG_ERR("FRB3 event assert");
		struct pldm_addsel_data msg = { 0 };
		msg.event_type = FRB3_TIMER_EXPIRE;
		msg.assert_type = EVENT_ASSERTED;
		if (PLDM_SUCCESS != send_event_log_to_bmc(msg)) {
			LOG_ERR("Failed to assert FRE3 event log.");
		};
	} else {
		/* Notification */
		LOG_INF("FRB3 checked pass");
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DEFINE(reinit_i3c_work, reinit_i3c_hub);
K_WORK_DEFINE(set_ffwf_eid_work, set_ffwf_eid);
K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
K_WORK_DELAYABLE_DEFINE(ABORT_FRB2_WDT_THREAD, abort_frb2_wdt_thread);

// for MB_FAST_PROCHOT
#define MB_THROTTLE_WORKQ_STACK_SIZE 1024
#define MB_THROTTLE_WORKQ_PRIORITY K_PRIO_PREEMPT(2)
K_THREAD_STACK_DEFINE(mb_throttle_workq_stack, MB_THROTTLE_WORKQ_STACK_SIZE);
struct k_work_q mb_throttle_work_q;

// for SYS_THROTTLE
#define SYS_THROTTLE_WORKQ_STACK_SIZE 1024
#define SYS_THROTTLE_WORKQ_PRIORITY K_PRIO_PREEMPT(2)
K_THREAD_STACK_DEFINE(sys_throttle_workq_stack, SYS_THROTTLE_WORKQ_STACK_SIZE);
struct k_work_q sys_throttle_work_q;

void init_fastprochot_work_q(void)
{
	k_work_queue_start(&mb_throttle_work_q, mb_throttle_workq_stack,
			   K_THREAD_STACK_SIZEOF(mb_throttle_workq_stack),
			   MB_THROTTLE_WORKQ_PRIORITY, NULL);
}

void init_throttle_work_q(void)
{
	k_work_queue_start(&sys_throttle_work_q, sys_throttle_workq_stack,
			   K_THREAD_STACK_SIZEOF(sys_throttle_workq_stack),
			   SYS_THROTTLE_WORKQ_PRIORITY, NULL);
}

static struct k_work_delayable add_post_timeout_sel_work;
void add_post_timeout_sel_work_handler(struct k_work *work)
{
	struct pldm_addsel_data sel_msg = { 0 };

	sel_msg.assert_type = EVENT_ASSERTED;
	sel_msg.event_type = POST_TIMEOUTED;

	if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
		LOG_ERR("[%s] Failed to send Post-Timeouted assert SEL", __func__);
	} else {
		LOG_INF("[%s] Send Post-Timeouted assert SEL", __func__);
	}
}

void init_post_timeout_event_work(void)
{
	k_work_init_delayable(&add_post_timeout_sel_work, add_post_timeout_sel_work_handler);
}

#define POST_TIMEOUT_SECONDS 1200
K_TIMER_DEFINE(power_on_timer, post_timeout_handler, NULL);
void post_timeout_handler(struct k_timer *timer_id)
{
	LOG_INF("[%s] DC power on over %d seconds. Post timeout", __func__, POST_TIMEOUT_SECONDS);

	k_work_schedule_for_queue(&plat_work_q, &add_post_timeout_sel_work, K_NO_WAIT);
}

#define DC_ON_5_SECOND 5
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);

	bool dc_status = get_DC_status();

	if (dc_status) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_submit(&set_ffwf_eid_work);
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
		k_timer_stop(&power_on_timer);
		k_timer_start(&power_on_timer, K_SECONDS(POST_TIMEOUT_SECONDS), K_NO_WAIT);
		LOG_INF("[%s] Start power on timer", __func__);
	} else {
		k_timer_stop(&power_on_timer);
		LOG_INF("[%s] Stop power on timer", __func__);
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_kcs_ok();
		reset_4byte_postcode_ok();

		set_DC_on_delayed_status();
	}
}

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);

	pal_check_sbrmi_command_code_length();

	if (get_post_status()) {
		set_tsi_threshold();
		disable_mailbox_completion_alert();
		enable_alert_signal();
		read_cpuid();
		LOG_INF("Post complete event assert");
		hw_event_register[12]++;
		k_timer_stop(&power_on_timer);
		LOG_INF("[%s] Stop power on timer", __func__);
	}
}

void ISR_BMC_READY()
{
	sync_bmc_ready_pin();
}

void ISR_WF_BIC_READY()
{
	k_work_submit(&set_ffwf_eid_work);
}

static void SLP3_handler()
{
	if ((gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW)) {
		// add sel to bmc
		struct pldm_addsel_data msg = { 0 };
		msg.event_type = POWER_ON_SEQUENCE_FAIL;
		msg.assert_type = EVENT_ASSERTED;
		if (PLDM_SUCCESS != send_event_log_to_bmc(msg)) {
			LOG_ERR("Failed to send blade fail power-on sequence assert event log.");
		};
		hw_event_register[0]++;
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);

#define POWER_ON_SEQUENCE_FAIL_DELAY_S 10
void ISR_SLP3()
{
	if (gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) {
		LOG_INF("slp3");
		k_work_schedule_for_queue(&plat_work_q, &SLP3_work,
					  K_SECONDS(POWER_ON_SEQUENCE_FAIL_DELAY_S));
	} else {
		if (k_work_cancel_delayable(&SLP3_work) != 0) {
			LOG_ERR("Failed to cancel delayable work.");
		}
	}
}

void ISR_DBP_PRSNT()
{
	/* HDT_PRSENT_N --> Event trigger and save log when signal falling and also
    check RST_RSMRST_BMC_N / normal power. */
	if ((gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) && (get_DC_status())) {
		add_sel_info *event_item = find_event_work_items(FM_DBP_PRESENT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_DBP_PRESENT_N, gpio_get(FM_DBP_PRESENT_N));
			return;
		}

		if ((gpio_get(FM_DBP_PRESENT_N) == GPIO_HIGH)) {
			LOG_INF("ISR_DBP_PRSNT deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_DBP_PRSNT assert");
			hw_event_register[1]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void ISR_MB_THROTTLE()
{
	int gpio_state = gpio_get(FAST_PROCHOT_N);

	if ((gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) && get_DC_status()) {
		const add_sel_info *event_item = find_event_work_items(FAST_PROCHOT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FAST_PROCHOT_N, gpio_state);
			return;
		}

		static sel_work_wrapper mb_throttle_work[20];
		static int mb_index = 0;
		sel_work_wrapper *wrap = &mb_throttle_work[mb_index++ % 20];
		wrap->sel_data.event_type = FAST_PROCHOT_ASSERT;
		wrap->sel_data.assert_type =
			(gpio_state == GPIO_LOW) ? EVENT_ASSERTED : EVENT_DEASSERTED;
		if (gpio_state == GPIO_LOW) {
			hw_event_register[2]++;
		}
		int ret = -1;
		k_work_init_delayable(&wrap->work, addsel_wrapper_work_handler);
		ret = k_work_schedule_for_queue(&mb_throttle_work_q, &wrap->work, K_NO_WAIT);
		if (ret != 1) {
			LOG_ERR("Fail MB_THROTTLE Kwork failed, %d", ret);
		}
	} else {
		LOG_ERR("Fail ISR_MB_THROTTLE");
	}
}

void ISR_SOC_THMALTRIP()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		LOG_INF("ISR_SOC_THMALTRIP assert");
		hw_event_register[3]++;
		add_sel_info *event_item = find_event_work_items(FM_CPU_BIC_THERMTRIP_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x", FM_CPU_BIC_THERMTRIP_N);
			return;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void ISR_SYS_THROTTLE()
{
	int gpio_state_sys_throttle = gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N);

	if ((gpio_get(RST_CPU_RESET_BIC_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		const add_sel_info *event_item = find_event_work_items(FM_CPU_BIC_PROCHOT_LVT3_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_CPU_BIC_PROCHOT_LVT3_N, gpio_state_sys_throttle);
			return;
		}

		static sel_work_wrapper sys_throttle_work[20];
		static int sys_index = 0;
		sel_work_wrapper *wrap = &sys_throttle_work[sys_index++ % 20];
		wrap->sel_data.event_type = event_item->event_type;
		wrap->sel_data.assert_type =
			(gpio_state_sys_throttle == GPIO_LOW) ? EVENT_ASSERTED : EVENT_DEASSERTED;
		if (gpio_state_sys_throttle == GPIO_LOW) {
			hw_event_register[4]++;
		}
		int ret = -1;
		k_work_init_delayable(&wrap->work, addsel_wrapper_work_handler);
		ret = k_work_schedule_for_queue(&sys_throttle_work_q, &wrap->work, K_NO_WAIT);
		if (ret != 1) {
			LOG_ERR("Fail SYS_THROTTLE Kwork failed, %d", ret);
		}
	} else {
		LOG_ERR("Fail ISR_SYS_THROTTLE");
	}
}

void ISR_HSC_OC()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		add_sel_info *event_item = find_event_work_items(FM_HSC_TIMER_ALT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_HSC_TIMER_ALT_N, gpio_get(FM_HSC_TIMER_ALT_N));
			return;
		}

		if (gpio_get(FM_HSC_TIMER_ALT_N) == GPIO_HIGH) {
			LOG_INF("ISR_HSC_OC deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_HSC_OC assert");
			hw_event_register[5]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void ISR_UV_DETECT()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		add_sel_info *event_item = find_event_work_items(IRQ_UV_DETECT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				IRQ_UV_DETECT_N, gpio_get(IRQ_UV_DETECT_N));
			return;
		}

		if (gpio_get(IRQ_UV_DETECT_N) == GPIO_HIGH) {
			LOG_INF("ISR_UV_DETECT deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_UV_DETECT assert");
			hw_event_register[9]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void IST_PLTRST()
{
	LOG_INF("IST_PLTRST assert");
	hw_event_register[10]++;

	add_sel_info *event_item = find_event_work_items(RST_PLTRST_BIC_N);
	if (event_item == NULL) {
		LOG_ERR("Fail to find event items, gpio num: 0x%x", RST_PLTRST_BIC_N);
		return;
	}

	// Reset WDT
	k_work_schedule_for_queue(&plat_work_q, &ABORT_FRB2_WDT_THREAD, K_NO_WAIT);

	k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);

	k_timer_stop(&power_on_timer);
	k_timer_start(&power_on_timer, K_SECONDS(POST_TIMEOUT_SECONDS), K_NO_WAIT);
	LOG_INF("[%s] Start power on timer", __func__);
}

static void APML_ALERT_handler(struct k_work *work)
{
	uint8_t status = 0;
	if (apml_read_byte(apml_get_bus(), SB_RMI_ADDR, SBRMI_STATUS, &status))
		LOG_ERR("Failed to read RMI status.");

	if ((status & 0x02) && (apml_write_byte(apml_get_bus(), SB_RMI_ADDR, SBRMI_STATUS, 0x02)))
		LOG_ERR("Failed to clear SwAlertSts.");

	pldm_msg translated_msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC;
	uint8_t bmc_interface = pal_get_bmc_interface();

	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		translated_msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		translated_msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		translated_msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		translated_msg.ext_params.ep = MCTP_EID_BMC;
	}

	translated_msg.hdr.pldm_type = PLDM_TYPE_OEM;
	translated_msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	translated_msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + CPUID_SIZE);

	if (!ptr) {
		LOG_ERR("Failed to allocate memory for APML alert event.");
		return;
	}

	ptr->cmd_code = APML_ALERT;
	ptr->data_length = CPUID_SIZE;
	const uint8_t *cpuid_ptr = get_cpuid();
	memcpy(ptr->messages, cpuid_ptr, CPUID_SIZE);

	translated_msg.buf = (uint8_t *)ptr;
	translated_msg.len = sizeof(struct pldm_oem_write_file_io_req) + CPUID_SIZE;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &translated_msg, rbuf, resp_len)) {
		LOG_ERR("APML alert mctp_pldm_read fail");
		goto exit;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		goto exit;
	}

exit:
	SAFE_FREE(ptr);

	return;
}

K_WORK_DELAYABLE_DEFINE(APML_ALERT_work, APML_ALERT_handler);
void ISR_APML_ALERT()
{
	hw_event_register[11]++;
	LOG_INF("APML_ALERT detected");
	k_work_schedule_for_queue(&plat_work_q, &APML_ALERT_work, K_NO_WAIT);
}

void ISR_CPU_SMERR_BIC()
{
	if (get_DC_status() == true) {
		add_sel_info *event_item = find_event_work_items(CPU_SMERR_BIC_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x", CPU_SMERR_BIC_N);
			return;
		}

		if ((gpio_get(CPU_SMERR_BIC_N) == GPIO_LOW)) {
			LOG_INF("ISR_CPU_SMERR_BIC assert");
			event_item->assert_type = EVENT_ASSERTED;
		}
		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}
