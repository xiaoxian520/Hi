

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

/* Link Header Files */
#include <link_service.h>
#include <link_platform.h>
#include <histreaming.h>
#include <hi_io.h>
#include <hi_early_debug.h>
#include <hi_gpio.h>

static void ControlLight(hi_gpio_value v)
{
    hi_io_set_func(HI_IO_NAME_GPIO_9, HI_IO_FUNC_GPIO_9_GPIO);
    hi_gpio_set_dir(HI_GPIO_IDX_9, HI_GPIO_DIR_OUT);
    hi_gpio_set_ouput_val(HI_GPIO_IDX_9, v);
}

/**
	@berf  The device side sends the characteristic value to the app side
	@param struct LinkService* ar: histreaming LinkServer structural morphology
	@param const char* property: characteristic value
	@param char* value: send value to app 
	@param int len: send value length
*/
static int GetStatusValue(struct LinkService* ar, const char* property, char* value, int len)
{
    (void)(ar);

    printf("Receive property: %s(value=%s[%d])\n", property, value, len);

    if (strcmp(property, "Status") == 0) {
        strcpy(value, "Opend");
    }

    /*
     * if Ok return 0,
     * Otherwise, any error, return StatusFailure
     */
    return 0;
}
/** 
	@berf recv from app cmd 
	@berf Receive the message sent by the app, and operate the hi3861 device side accordingly
	@param struct LinkService* ar: histreaming LinkServer structural morphology
	@param const char* property: Eigenvalues sent by app
	@param char* value: Value sent by app
	@param int len: Length of APP sent
*/
static int ModifyStatus(struct LinkService* ar, const char* property, char* value, int len)
{
    (void)(ar);

    if (property == NULL || value == NULL) {
        return -1;
    }

    /* modify status property*/
    if (strcmp(property, "status") == 0) {
        if (strcmp(value, "On") == 0) {
            ControlLight(HI_GPIO_VALUE0);
        } else {
            ControlLight(HI_GPIO_VALUE1);
        }
    }

    printf("Receive property: %s(value=%s[%d])\n", property, value,len);

    /*
     * if Ok return 0,
     * Otherwise, any error, return StatusFailure
     */
    return 0;
}

/*
 * It is a Wifi IoT device
 */
static const char* g_wifista_type = "Light";
/**
	@berf The app side obtains the device name
	@param struct LinkService* ar: histreaming LinkServer structural morphology
*/
static const char* GetDeviceType(struct LinkService* ar)
{
    (void)(ar);

    return g_wifista_type;
}

static void *g_link_platform = NULL;

void* histreaming_open(void)
{
    /*server start*/
    hi_u32 ret = hi_gpio_init();
    if (ret != HI_ERR_SUCCESS) {
        printf("===== ERROR ===== gpio -> hi_gpio_init ret:%d\r\n", ret);
        return NULL;
    }
    printf("----- gpio init success-----\r\n");

    LinkService* wifiIot = 0;
    LinkPlatform* link = 0;

    wifiIot = (LinkService*)malloc(sizeof(LinkService));
    if (!wifiIot){
        printf("malloc wifiIot failure\n");
        return NULL;
    }

    wifiIot->get    = GetStatusValue;
    wifiIot->modify = ModifyStatus;
    wifiIot->type = GetDeviceType;

    link = LinkPlatformGet();
    if (!link) {
        printf("get link failure\n");
        return NULL;
    }

    if (link->addLinkService(link, wifiIot, 1) != 0) {
        histreaming_close(link);
        return NULL;
    }

    if (link->open(link) != 0) {
        histreaming_close(link);
        return NULL;
    }

    /* cache link ptr*/
    g_link_platform = (void*)(link);

    return (void*)link;
}

void histreaming_close(void *link)
{
    LinkPlatform *linkPlatform = (LinkPlatform*)(link);
    if (!linkPlatform) {
        return;
    }

    linkPlatform->close(linkPlatform);

    if (linkPlatform != NULL) {
        LinkPlatformFree(linkPlatform);
    }
}

