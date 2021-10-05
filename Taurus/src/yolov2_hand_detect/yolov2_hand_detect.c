/*
 * Copyright (c) 2021 HiSilicon (Shanghai) Technologies CO., LIMITED.
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "sample_comm_nnie.h"
#include "nnie_sample_plug.h"

#include "hi_ext_util.h"
#include "mpp_help.h"
#include "ai_plug.h"
#include "hisignalling.h"

#define PLUG_UUID "\"hi.yolov2_hand_detect\""
#define PLUG_DESC "\"手部检测(yolov2)\"" // UTF8 encode

#define FRM_WIDTH 512
#define FRM_HEIGHT 512
//#define FRM_WIDTH 640
//#define FRM_HEIGHT 384
#define MODEL_FILE_HAND "./plugs/inst_yolov2_inst.wk" // darknet framework wk model

#define THRESH_MIN 0.7
#define DETECT_OBJ_MAX 32 // todo:need test
#define PIRIOD_NUM_MAX 49 // Logs are printed when the number of targets is detected
#define DRAW_RETC_THICK 2 // Draw the width of the line
#define SCORE_MAX 4096

/*
static OsdSet* g_osdsTrash = NULL;
static HI_S32 g_osd0Trash = -1;
*/
static uintptr_t g_handModel = 0;

static const char YOLO2_FD_PROF[] = "{"
"\"uuid\": " PLUG_UUID ","
"\"desc\": " PLUG_DESC ","
"\"frmWidth\": " HI_TO_STR(FRM_WIDTH) ","
"\"frmHeight\": " HI_TO_STR(FRM_HEIGHT) ","
"\"butt\": 0"
"}";

static const char *Yolo2FdProf(void)
{
    return YOLO2_FD_PROF;
}
//static int uart_fd=-1;
static HI_S32 Yolo2FdLoad(uintptr_t *model, OsdSet *osds)
{
    SAMPLE_SVP_NNIE_CFG_S *self = NULL;
    HI_S32 ret;
    /*
    g_osdsTrash = osds;
    HI_ASSERT(g_osdsTrash);
    g_osd0Trash = OsdsCreateRgn(g_osdsTrash);
    HI_ASSERT(g_osd0Trash >= 0);
    */
    ret = Yolo2Create(&self, MODEL_FILE_HAND);
    *model = ret < 0 ? 0 : (uintptr_t)self;
    LOGI("Yolo2FdLoad ret:%d\n", ret);
    //uart_fd=uartOpenInit();
    //HI_ASSERT(uart_fd>=0);
    return ret;
}

/*
static void Yolo2TrashAddTxt(const RectBox box, const DetectObjInfo resBuf, uint32_t color)
{
    HI_OSD_ATTR_S osdRgn;
    char osdTxt[TINY_BUF_SIZE];
    HI_CHAR *trash_name = NULL;
     //HI_ASSERT(g_osdsTrash);
    switch (resBuf.cls) {
            case 0u: trash_name = "background";
                break;
            case 1u: trash_name = "Missing_hole_rotation";
                break;
            case 2u: trash_name = "Mouse_bite_rotation";
                break;
            case 3u: trash_name = "Open_circuit_rotation";
                break;
            case 4u: trash_name = "Short_rotation";
                break;
            case 5u: trash_name = "Spur_rotation";
                break;
            case 6u: trash_name = "Spurious_copper_rotation";
                break;
            default:
                trash_name = "Unkown";
                break;
        }

    uint32_t score = (resBuf.score) * HI_PER_BASE / SCORE_MAX;
	int res = snprintf_s(osdTxt, sizeof(osdTxt), sizeof(osdTxt) - 1, "%d_%s,%d %%", resBuf.cls, trash_name, score);
	HI_ASSERT(res > 0);

	int osdId = OsdsCreateRgn(g_osdsTrash);
	HI_ASSERT(osdId >= 0);

    int x = box.xmin / HI_OVEN_BASE * HI_OVEN_BASE;
    int y = (box.ymin - 30) / HI_OVEN_BASE * HI_OVEN_BASE; // 30: empirical value
    if (y < 0) {
        LOGD("osd_y < 0, y=%d\n", y);
        OsdsDestroyRgn(g_osdsTrash, osdId);
    } else {
        TxtRgnInit(&osdRgn, osdTxt, x, y, color, 16, 24);
        OsdsSetRgn(g_osdsTrash, osdId, &osdRgn);
    }
}
*/

HI_S32 HandDetectInit()
{
    return Yolo2FdLoad(&g_handModel, NULL);
}

static HI_S32 Yolo2FdUnload(uintptr_t model)
{
    Yolo2Destory((SAMPLE_SVP_NNIE_CFG_S *)model);
     //close(uart_fd);
    return HI_SUCCESS;
}

HI_S32 HandDetectExit()
{
    return Yolo2FdUnload(g_handModel);
}

static HI_S32 Yolo2FdCal(uintptr_t model, VIDEO_FRAME_INFO_S *srcFrm, VIDEO_FRAME_INFO_S *dstFrm, HI_CHAR **resJson)
{
    SAMPLE_SVP_NNIE_CFG_S *self = (SAMPLE_SVP_NNIE_CFG_S *)model; // reference to SDK sample_comm_nnie.h
    IVE_IMAGE_S img;                                              // referece to SDK hi_comm_ive.h
    DetectObjInfo objs[DETECT_OBJ_MAX] = {0};
    RectBox boxs[DETECT_OBJ_MAX] = {0};
    int objNum;
    int ret;

    // performance statistical variable
    static int64_t yuv2RgbCost = 0;
    static int64_t calCost = 0;
    static int piriodNum = 0;
    int64_t begTime;
    int biggestBoxIndex;
    RectBox objBoxs[DETECT_OBJ_MAX] = {0};
    RectBox remainingBoxs[DETECT_OBJ_MAX] = {0};
    int num = 0;
    int weight;
    int height;

    
    // YUV to RGB
    begTime = HiClockMs();
    ret = FrmToOrigImg((VIDEO_FRAME_INFO_S *)srcFrm, &img);
    HI_EXP_RET(ret != HI_SUCCESS, ret, "hand_detect_cal FAIL, for YUV2RGB FAIL, ret=%#x\n", ret);
    yuv2RgbCost += (HiClockMs() - begTime);

    // calculate
    begTime = HiClockMs();
    ret = Yolo2CalImg(self, &img, THRESH_MIN, objs, HI_ARRAY_SIZE(objs), &objNum);
    IveImgDestroy(&img);
    HI_EXP_RET(ret < 0, ret, "hand_detect_cal FAIL, for cal FAIL, ret=%d\n", ret);
    calCost += (HiClockMs() - begTime);
    LOGI("objNum:%d\n", objNum);
    //OsdsClear(g_osdsTrash);
    for (int i = 0; i < objNum; i++)
    {
        RectBox *box = &objs[i].box;
         RectBoxTran(box, FRM_WIDTH, FRM_HEIGHT,
             dstFrm->stVFrame.u32Width, dstFrm->stVFrame.u32Height);
        LOGI("yolo2_out: {%d, %d, %d, %d}\n",
             box->xmin, box->ymin, box->xmax, box->ymax);
        boxs[i] = *box;
            
            //weight=boxs[i].xmax-boxs[i].xmax;
            //height=boxs[i].ymax- boxs[i].ymin;
            
        //MppFrmDrawRect(dstFrm, boxs + i, RGB888_RED, 2);
        Yolo2TrashAddTxt(boxs[i], objs[i], ARGB1555_WHITE);      
    }
    *resJson = DetectObjsToJson(objs, objNum, NULL);
      /*
      if (++piriodNum > PIRIOD_NUM_MAX) {
        LOGD("yolo: num=%d, pcvt=%jd, pcal=%jd\n",
            piriodNum, yuv2RgbCost / piriodNum, calCost / piriodNum);
        yuv2RgbCost = 0;
        calCost = 0;
        piriodNum = 0;
    }
    */
    
    if (objNum > 0) {
        MppFrmDrawRects(dstFrm, boxs, objNum, RGB888_RED, DRAW_RETC_THICK);
    }
    
    // 叠加图形到resFrm中
    //IveImgDestroy(&img);
    return ret;
}

static HI_S32 HandDetect(uintptr_t model, IVE_IMAGE_S *srcYuv, DetectObjInfo boxs[])
{
    SAMPLE_SVP_NNIE_CFG_S *self = (SAMPLE_SVP_NNIE_CFG_S *)model;
    int objNum;
    int ret;
    ret = Yolo2CalImg(self, srcYuv, THRESH_MIN, boxs, DETECT_OBJ_MAX, &objNum);
    HI_EXP_RET(ret < 0, ret, "Hand detect Yolo2CalImg FAIL, for cal FAIL\n");

    return objNum;
}

HI_S32 HandDetectCal(IVE_IMAGE_S *srcYuv, DetectObjInfo resArr[])
{
    int ret;
    ret = HandDetect(g_handModel, srcYuv, resArr);
    return ret;
}

static const AiPlug G_HAND_DETECT_ITF = {
    .Prof = Yolo2FdProf,
    .Load = Yolo2FdLoad,
    .Unload = Yolo2FdUnload,
    .Cal = Yolo2FdCal,
};


/*
const AiPlug* AiPlugItf(uint32_t* magic)
{
    if (magic) {
        *magic = AI_PLUG_MAGIC;
    }

    return (AiPlug*)&G_HAND_DETECT_ITF;
}
*/