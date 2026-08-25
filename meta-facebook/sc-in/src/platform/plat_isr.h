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

#ifndef PLAT_ISR_H
#define PLAT_ISR_H

#include <stdint.h>
#include "pldm.h"

extern uint8_t hw_event_register[13];

typedef struct _add_sel_info {
	bool is_init;
	uint8_t gpio_num;
	volatile uint8_t event_type;
	volatile uint8_t assert_type;
	struct k_work_delayable add_sel_work;
} add_sel_info;

typedef struct {
	struct k_work_delayable work;
	struct pldm_addsel_data sel_data;
} sel_work_wrapper;

void add_post_timeout_sel_work_handler(struct k_work *work);
void init_post_timeout_event_work(void);
void post_timeout_handler(struct k_timer *timer_id);
void ISR_DC_ON();
void ISR_POST_COMPLETE();
void ISR_BMC_READY();
void ISR_WF_BIC_READY();
void sync_bmc_ready_pin();
void reinit_i3c_hub();
void set_ffwf_eid();
void ISR_SLP3();
void ISR_DBP_PRSNT();
void ISR_MB_THROTTLE();
void ISR_SOC_THMALTRIP();
void ISR_SYS_THROTTLE();
void ISR_HSC_OC();
void ISR_UV_DETECT();
void IST_PLTRST();
void ISR_APML_ALERT();
void ISR_CPU_SMERR_BIC();

void init_event_work();
void addsel_work_handler(struct k_work *work_item);
void init_throttle_work_q();
void init_fastprochot_work_q();

#endif
