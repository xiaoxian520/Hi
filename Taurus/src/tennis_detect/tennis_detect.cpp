#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hi_ext_util.h"
#include "mpp_help.h"
#include "ai_plug.h"

#include "sample_comm_nnie.h"
#include "nnie_sample_plug.h"
#include "yolov2_hand_detect.h"
#include "hisignalling.h"

using namespace std;
using namespace cv;

#define PLUG_UUID          "\"hi.tennis_detect\""
#define PLUG_DESC          "\"网球检测(传统算子)\""  // UTF8 encode

#define FRM_WIDTH          512
#define FRM_HEIGHT        512
#define TENNIS_OBJ_MAX     256
#define DRAW_RETC_THICK    2
#define SCORE_MAX   4096
#define DETECT_OBJ_MAX  32

#define TXT_BEGX            20
#define TXT_BEGY            20
#define FONT_WIDTH          32
#define FONT_HEIGHT         40

static OsdSet* g_osdsTennis = NULL;
static int g_osd0Tennis = -1;
int flag1=0;
int flag2=0;

static const char TENNIS_DETECT[] = "{"
    "\"uuid\": " PLUG_UUID ","
    "\"desc\": " PLUG_DESC ","
    "\"frmWidth\": " HI_TO_STR(FRM_WIDTH) ","
    "\"frmHeight\": " HI_TO_STR(FRM_HEIGHT) ","
    "\"butt\": 0"
"}";

static const char* TennisDetectProf(void)
{
    return TENNIS_DETECT;
}
static int uart_fd=-1;
static int TennisDetectLoad(uintptr_t* model, OsdSet* osds)
{
    HI_S32 ret = 1;

    g_osdsTennis = osds;
    HI_ASSERT(g_osdsTennis);
    g_osd0Tennis = OsdsCreateRgn(g_osdsTennis);
    HI_ASSERT(g_osd0Tennis >= 0);
    *model = 1;
    LOGI("TennisDetectLoad success\n");
    HandDetectInit();
    uart_fd=uartOpenInit();
    HI_ASSERT(uart_fd>=0);
	return ret;
}

static int TennisDetectUnload(uintptr_t model)
{
    (void)model;
    OsdsClear(g_osdsTennis);
    HandDetectExit();
    close(uart_fd);
    return HI_SUCCESS;
}

typedef struct tagIPC_IMAGE{
    HI_U64 u64PhyAddr;
    HI_U64 u64VirAddr;
    HI_U32 u32Width;
    HI_U32 u32Height;
} IPC_IMAGE;

HI_S32 yuvFrame2rgb(VIDEO_FRAME_INFO_S *srcFrame, IPC_IMAGE *dstImage)
{
    IVE_HANDLE hIveHandle;
    IVE_SRC_IMAGE_S pstSrc;
    IVE_DST_IMAGE_S pstDst;
    IVE_CSC_CTRL_S stCscCtrl;
    HI_S32 s32Ret = 0;
    stCscCtrl.enMode = IVE_CSC_MODE_PIC_BT709_YUV2RGB; //IVE_CSC_MODE_VIDEO_BT601_YUV2RGB;
    pstSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
    pstSrc.au64VirAddr[0] = srcFrame->stVFrame.u64VirAddr[0];
    pstSrc.au64VirAddr[1] = srcFrame->stVFrame.u64VirAddr[1];
    pstSrc.au64VirAddr[2] = srcFrame->stVFrame.u64VirAddr[2];
 
    pstSrc.au64PhyAddr[0] = srcFrame->stVFrame.u64PhyAddr[0];
    pstSrc.au64PhyAddr[1] = srcFrame->stVFrame.u64PhyAddr[1];
    pstSrc.au64PhyAddr[2] = srcFrame->stVFrame.u64PhyAddr[2];
 
    pstSrc.au32Stride[0] = srcFrame->stVFrame.u32Stride[0];
    pstSrc.au32Stride[1] = srcFrame->stVFrame.u32Stride[1];
    pstSrc.au32Stride[2] = srcFrame->stVFrame.u32Stride[2];
 
    pstSrc.u32Width = srcFrame->stVFrame.u32Width;
    pstSrc.u32Height = srcFrame->stVFrame.u32Height;
 
    pstDst.enType = IVE_IMAGE_TYPE_U8C3_PACKAGE;
    pstDst.u32Width = pstSrc.u32Width;
    pstDst.u32Height = pstSrc.u32Height;
    pstDst.au32Stride[0] = pstSrc.au32Stride[0];
    pstDst.au32Stride[1] = 0;
    pstDst.au32Stride[2] = 0;

    s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&pstDst.au64PhyAddr[0], (void **)&pstDst.au64VirAddr[0],
        "User", HI_NULL, pstDst.u32Height*pstDst.au32Stride[0] * 3);
    if (HI_SUCCESS != s32Ret) {       
        HI_MPI_SYS_MmzFree(pstDst.au64PhyAddr[0], (void *)pstDst.au64VirAddr[0]);
        LOGE("HI_MPI_SYS_MmzFree err\n");
        return s32Ret;
    }

    s32Ret = HI_MPI_SYS_MmzFlushCache(pstDst.au64PhyAddr[0], (void *)pstDst.au64VirAddr[0],
        pstDst.u32Height*pstDst.au32Stride[0]*3);
    if (HI_SUCCESS != s32Ret) {       
        HI_MPI_SYS_MmzFree(pstDst.au64PhyAddr[0], (void *)pstDst.au64VirAddr[0]);
        return s32Ret;
    }
    memset((void *)pstDst.au64VirAddr[0], 0, pstDst.u32Height*pstDst.au32Stride[0]*3);
    HI_BOOL bInstant = HI_TRUE;

    s32Ret = HI_MPI_IVE_CSC(&hIveHandle, &pstSrc, &pstDst, &stCscCtrl, bInstant);
    if(HI_SUCCESS != s32Ret) {       
        HI_MPI_SYS_MmzFree(pstDst.au64PhyAddr[0], (void *)pstDst.au64VirAddr[0]);
        return s32Ret;
    }

    if (HI_TRUE == bInstant) {
        HI_BOOL bFinish = HI_TRUE;
        HI_BOOL bBlock = HI_TRUE;
        s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);
        while (HI_ERR_IVE_QUERY_TIMEOUT == s32Ret) {
            usleep(100);
            s32Ret = HI_MPI_IVE_Query(hIveHandle,&bFinish,bBlock);
        }
    }
    dstImage->u64PhyAddr = pstDst.au64PhyAddr[0];
    dstImage->u64VirAddr = pstDst.au64VirAddr[0];
    dstImage->u32Width = pstDst.u32Width;
    dstImage->u32Height = pstDst.u32Height;

    return HI_SUCCESS;
}

HI_S32 frame2Mat(VIDEO_FRAME_INFO_S *srcFrame, Mat &dstMat)
{
    HI_U32 w = srcFrame->stVFrame.u32Width;
    HI_U32 h = srcFrame->stVFrame.u32Height;
    int bufLen = w * h * 3;
    HI_U8 *srcRGB = NULL;
    IPC_IMAGE dstImage;
    if (yuvFrame2rgb(srcFrame, &dstImage) != HI_SUCCESS) {
        LOGE("yuvFrame2rgb err\n");
        return HI_FAILURE;
    }
    srcRGB = (HI_U8 *)dstImage.u64VirAddr;
    dstMat.create(h, w, CV_8UC3);
    memcpy(dstMat.data, srcRGB, bufLen * sizeof(HI_U8));
    HI_MPI_SYS_MmzFree(dstImage.u64PhyAddr, (void *)&(dstImage.u64VirAddr));
    return HI_SUCCESS;
}

static void Yolo2TrashAddTxt(const RectBox box, const DetectObjInfo resBuf, uint32_t color)
{
    HI_OSD_ATTR_S osdRgn;
    char osdTxt[TINY_BUF_SIZE];
    HI_CHAR *trash_name = NULL;
     HI_ASSERT( g_osdsTennis);
    switch (resBuf.cls) {
           // flag2++;
            case 0u: trash_name = (char *) "background";
                break;
            case 1u: trash_name = (char *)"Missing_hole_rotation";
                break;
            case 2u: trash_name = (char *)"Mouse_bite_rotation";
                break;
            case 3u: trash_name = (char *)"Open_circuit_rotation";
                break;
            case 4u: trash_name = (char *)"Short_rotation";
                break;
            case 5u: trash_name = (char *)"Spur_rotation";
                break;
            case 6u: trash_name = (char *)"Spurious_copper_rotation";
                break;
            default:
                trash_name = (char *)"Unkown";
                break;
        }

    uint32_t score = (resBuf.score) * HI_PER_BASE / SCORE_MAX;
	int res = snprintf_s(osdTxt, sizeof(osdTxt), sizeof(osdTxt) - 1, "%d_%s,%d %%", resBuf.cls, trash_name, score);
	HI_ASSERT(res > 0);

	int osdId = OsdsCreateRgn( g_osdsTennis);
	HI_ASSERT(osdId >= 0);

    int x = box.xmin / HI_OVEN_BASE * HI_OVEN_BASE;
    int y = (box.ymin - 30) / HI_OVEN_BASE * HI_OVEN_BASE; // 30: empirical value
    if (y < 0) {
        LOGD("osd_y < 0, y=%d\n", y);
        OsdsDestroyRgn( g_osdsTennis, osdId);
    } else {
        TxtRgnInit(&osdRgn, osdTxt, x, y, color, 16, 24);
        OsdsSetRgn( g_osdsTennis, osdId, &osdRgn);
    }
}

/*
    将计算结果打包为resJson.
*/
/*
HI_S32 YieldToOsd( HI_S32 itemNum,HI_CHAR* buf, HI_S32 size)
{
     HI_S32 offset = 0;
     HI_ASSERT( g_osdsTennis);
    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, "Yield: {");
    offset += snprintf_s(buf + offset, size - offset, size - offset - 1,
        "%u%%",  (int)itemNum);
    //HI_ASSERT(offset < sizeof(osdTxt));
    offset += snprintf_s(buf + offset, size - offset, size - offset - 1, " }");
    //HI_ASSERT(offset < sizeof(osdTxt));
     // 叠加图形到resFrm
     return offset;
    //OsdsDestroyRgn( g_osdsTennis, osd);
}
*/
static int TennisDetectCal(uintptr_t model,
    VIDEO_FRAME_INFO_S *srcFrm, VIDEO_FRAME_INFO_S *dstFrm, HI_CHAR** resJson)
{
    (void)model;
    int ret = 0;
    RectBox boxs[TENNIS_OBJ_MAX] = {0};
    IVE_IMAGE_S img;
    DetectObjInfo objs[DETECT_OBJ_MAX] = {0};
    static HI_CHAR prevOsd[NORM_BUF_SIZE] = "";
    HI_CHAR osdBuf[NORM_BUF_SIZE] = "";
    int objNum=0;
    int j=0;
    
    OsdsClear(g_osdsTennis);
    Mat image;
    frame2Mat(srcFrm, image);
    if (image.size == 0) {
        LOGD("image is null\n");
        return HI_FAILURE;
    }

    Mat src = image;
    Mat src1 = src.clone();
    Mat dst, edge, gray, hsv;

    dst.create(src1.size(), src1.type()); // 创建与src同类型和大小的矩阵(dst)
    // imwrite("image.jpg", src1);

    // cvtColor算子用于将图像从一个颜色空间转换到另一个颜色空间的转换
    cvtColor(src1, hsv, COLOR_BGR2HSV); // 将原图转换为HSV图像

    // 对hsv图像进行二值化处理，这里是将绿色背景二值化，该参数根据需求进行调整
    inRange(hsv, Scalar(31, 82, 68), Scalar(65, 248, 255), gray);

    // 利用canny算子进行边缘检测
    Canny(gray, gray, 3, 9, 3);
    vector<vector<Point>> contours;
    findContours(gray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    LOGI("contours.size():%d\n", contours.size());

    //for (int i = 0; i < (int)contours.size(); i++) {
        if (contours.size() > 30) {
            flag1++;
            ret = FrmToOrigImg((VIDEO_FRAME_INFO_S*)srcFrm, &img);
            HI_EXP_RET(ret != HI_SUCCESS, ret, "hand_detect_cal FAIL, for YUV Frm to Img FAIL, ret=%#x\n", ret);
            objNum = HandDetectCal(&img, objs); // Send IMG to the detection net for reasoning
            for (int i = 0; i < objNum; i++) {
                RectBox *box = &objs[i].box;
                RectBoxTran(box, FRM_WIDTH, FRM_HEIGHT,
                    dstFrm->stVFrame.u32Width, dstFrm->stVFrame.u32Height);
                LOGI("yolo2_out: {%d, %d, %d, %d}\n",
                    box->xmin, box->ymin, box->xmax, box->ymax);
                boxs[i] = *box;
               MppFrmDrawRect(dstFrm, boxs + i, RGB888_RED, 2);
                Yolo2TrashAddTxt(boxs[i], objs[i], ARGB1555_WHITE);
                //LOGI("Conformity Rate or conformance rate:(%u%%)\n",(flag2*100)/flag1);
            }
            MppFrmDestroy(srcFrm);
            IveImgDestroy(&img);
        }
         

        /*
        Rect ret1 = boundingRect(Mat(contours[i]));
        ret1.x -= 5;
        ret1.y -= 5;
        ret1.width += 10;
        ret1.height += 10;

        if ((ret1.width > 20) && (ret1.height > 20)) {
            LOGD("ret x:%d, y:%d, width:%d, height:%d\n", ret1.x, ret1.y, ret1.width, ret1.height);
            boxs[j].xmin = ret1.x * 2;
            boxs[j].ymin = (int)(ret1.y * 1.5);
            boxs[j].xmax = boxs[j].xmin + ret1.width * 2;
            boxs[j].ymax = boxs[j].ymin + (int)ret1.height * 1.5;
            j++;
        }
        LOGD("dstfrm width:%d, Height:%d\n", dstFrm->stVFrame.u32Width, dstFrm->stVFrame.u32Height);
	}
    if (j > 0 && j <= 25) {
        LOGI("box num:%d\n", j);
        MppFrmDrawRects(dstFrm, boxs, j, RGB888_RED, DRAW_RETC_THICK);
    }
    */
    // 打包计算结果为resJson
   
    //YieldToOsd(j,osdBuf, sizeof(osdBuf));  
    HI_CHAR *jsonBuf = DetectObjsToJson(objs, objNum, NULL);
    *resJson = jsonBuf;
    if (objNum > 0) {
        flag2++;
        //MppFrmDrawRects(dstFrm, boxs, objNum, RGB888_RED, DRAW_RETC_THICK);
        usbUartSendRead(uart_fd,FIST);  
        usleep(100*100);
    }
     j=(flag2*100)/flag1;
    LOGI("Conformity Rate or conformance rate:(%u%%)\n",(flag2*100)/flag1);

    /*
    YieldToOsd(j,osdBuf, sizeof(osdBuf));
        // 叠加图形到resFrm中
     HI_OSD_ATTR_S rgn;
    TxtRgnInit(&rgn, osdBuf, TXT_BEGX, TXT_BEGY, ARGB1555_YELLOW2, FONT_WIDTH, FONT_HEIGHT);
    OsdsSetRgn(g_osdsTennis, g_osd0Tennis, &rgn);
    LOGI("CNN trash classify: %s\n", osdBuf);
    */
    //YieldToOsd(j); 
    
    //*resJson = TennisDetectToJson(boxs, j, NULL);
    return ret;
}

static const AiPlug G_TENNIS_DETECT_ITF = {
    .Prof = TennisDetectProf,
    .Load = TennisDetectLoad,
    .Unload = TennisDetectUnload,
    .Cal = TennisDetectCal,
};

const AiPlug* AiPlugItf(uint32_t* magic)
{
    if (magic) {
        *magic = AI_PLUG_MAGIC;
    }

    return (AiPlug*)&G_TENNIS_DETECT_ITF;
}
