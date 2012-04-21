/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ASM__ARCH_OEM_RAPI_CLIENT_H
#define __ASM__ARCH_OEM_RAPI_CLIENT_H

/*
 * OEM RAPI CLIENT Driver header file
 */

#include <linux/types.h>
#include <mach/msm_rpcrouter.h>

// LG_FW : 2011.07.07 moon.yongho : saving webdload status variable to eMMC. ----------[[
#include "../../../../../lge/include/lg_diag_cfg.h"
// LG_FW : 2011.07.07 moon.yongho -----------------------------------------------------]]

enum {
	OEM_RAPI_CLIENT_EVENT_NONE = 0,

	/*
	 * list of oem rapi client events
	 */
#ifdef CONFIG_LGE_SUPPORT_RAPI
	
	LG_FW_RAPI_START = 100,
	LG_FW_RAPI_CLIENT_EVENT_GET_LINE_TYPE = LG_FW_RAPI_START,
	LG_FW_TESTMODE_EVENT_FROM_ARM11 = LG_FW_RAPI_START + 1,
	LG_FW_A2M_BATT_INFO_GET = LG_FW_RAPI_START + 2,
	LG_FW_A2M_PSEUDO_BATT_INFO_SET = LG_FW_RAPI_START + 3,
	LG_FW_MEID_GET = LG_FW_RAPI_START + 4,
	/* LGE_CHANGE_S 
	 * SUPPORT TESTMODE FOR AIRPLAN MODE
	 * 2010-07-12 taehung.kim@lge.com
	 */
	LG_FW_SET_OPERATION_MODE = LG_FW_RAPI_START + 5,
	LG_FW_A2M_BLOCK_CHARGING_SET = LG_FW_RAPI_START + 6,
#endif 
	LG_FW_MANUAL_TEST_MODE = LG_FW_RAPI_START + 8,
#ifdef CONFIG_LGE_SUPPORT_RAPI
	LGE_RPC_HANDLE_REQUEST = LG_FW_RAPI_START + 9,
#endif
#ifdef CONFIG_LGE_DLOAD_SRD 
	LG_FW_REQUEST_SRD_RPC = LG_FW_RAPI_START + 10,  //kabjoo.choi
#endif 	

#ifdef CONFIG_LGE_DLOAD_SRD 
	LG_FW_RAPI_ERI_DIAG_WRITE= LG_FW_RAPI_START + 11,   //uts dll
#endif 	
//#ifndef CONFIG_LGE_ERI_DOWNLOAD
	LGE_REQUEST_ERI_RPC = LG_FW_RAPI_START + 12,
//#endif
	OEM_RAPI_CLIENT_SET_ROOTING_NV_FROM_KERNEL =LG_FW_RAPI_START + 13,

// LG_FW : 2011.07.07 moon.yongho : saving webdload status variable to eMMC. ----------[[
#ifdef LG_FW_WEB_DOWNLOAD	
	LG_FW_WEB_DLOAD_STATUS= LG_FW_RAPI_START + 14, 	
#endif /*LG_FW_WEB_DOWNLOAD*/	
// LG_FW : 2011.07.07 moon.yongho -----------------------------------------------------]]


// LG_FW [START] : 2011.08.05 jongan.kim : get SW_VER from modem for EUT
	LG_FW_SW_VERSION_GET = LG_FW_RAPI_START + 15,
	LG_FW_SUB_VERSION_GET = LG_FW_RAPI_START + 16,
// LG_FW [END] : 2011.08.05 jongan.kim

	
//DID BACKUP  support	kabjoo.choi
	LG_FW_DID_BACKUP_REQUEST= LG_FW_RAPI_START + 17,	
	
//START FOTA_LGE_BSP miracle.kim@lge.com 2011-08-09 for fota misc info
#ifdef CONFIG_LGE_FOTA_MISC_INFO
	LGE_REMOTE_RPC_REQUEST_VAL = LG_FW_RAPI_START + 18,
#endif	
//END FOTA_LGE_BSP miracle.kim@lge.com 2011-08-09 for fota misc info



	LG_MSG_UNIFIEDMSGTOOL_FROM_ARM11 = 200, //#ifdef LG_SMS_PC_TEST
	LG_FW_GET_MANUAL_MODE = LG_MSG_UNIFIEDMSGTOOL_FROM_ARM11 + 3,
	LG_FW_PUT_MANUAL_MODE = LG_MSG_UNIFIEDMSGTOOL_FROM_ARM11 + 4,
	OEM_RAPI_CLIENT_EVENT_MAX

};

//START FOTA_LGE_BSP miracle.kim@lge.com 2011-08-09 for fota misc info
#ifdef CONFIG_LGE_SUPPORT_RAPI
enum {
        LGE_CLIENT_CMD_START = 0,
#ifdef CONFIG_LGE_SYNC_CMD
        LGE_SYNC_REQUEST = 1,
#endif
        LGE_SW_VERSION_INFO = 2,
        LGE_MIN_INFO = 3,
        LGE_TESTMODE_MANUAL_TEST_INFO = 4,
        LGE_CLIENT_CMD_MAX = 0xF,
};
#endif

struct oem_rapi_client_streaming_func_cb_arg {
	uint32_t  event;
	void      *handle;
	uint32_t  in_len;
	char      *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_cb_ret {
	uint32_t *out_len;
	char *output;
};

struct oem_rapi_client_streaming_func_arg {
	uint32_t event;
	int (*cb_func)(struct oem_rapi_client_streaming_func_cb_arg *,
		       struct oem_rapi_client_streaming_func_cb_ret *);
	void *handle;
	uint32_t in_len;
	char *input;
	uint32_t out_len_valid;
	uint32_t output_valid;
	uint32_t output_size;
};

struct oem_rapi_client_streaming_func_ret {
	uint32_t *out_len;
	char *output;
};

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret);

int oem_rapi_client_close(void);

struct msm_rpc_client *oem_rapi_client_init(void);

#endif
