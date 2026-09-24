#include "mtk_test.h"
#include "mtek_wifi_station.h"
#include <string.h>
MTK_TEST_MAIN_BEGIN
    uint8_t p[32]={8,1};
    uint8_t ap[6]={2,3,4,5,6,7},sta[6]={4,5,6,7,8,9};
    memcpy(p+4,ap,6);memcpy(p+10,sta,6);
    MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==p+10);
    MTK_CHECK(mtk_wifi_station_from_frame(p,23,ap)==NULL);
    p[0]=0x88; MTK_CHECK(mtk_wifi_station_from_frame(p,26,ap)==p+10);
    p[0]=0x80; MTK_CHECK(mtk_wifi_station_from_frame(p,26,ap)==NULL);
    p[0]=8;
    for(unsigned ds=0;ds<4;ds++) {
        p[1]=ds;
        if(ds!=1) MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL);
    }
    p[1]=2;memcpy(p+10,ap,6);memcpy(p+4,sta,6);
    MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==p+4);
    p[4]=1;MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL);
    memset(p+4,0xff,6);MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL);
    memset(p+4,0,6);MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL);
    memcpy(p+4,ap,6);MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL);
    memcpy(p+4,sta,6);memcpy(p+10,sta,6);memcpy(p+16,ap,6);
    MTK_CHECK(mtk_wifi_station_from_frame(p,24,ap)==NULL); /* Addr3 false match */
MTK_TEST_MAIN_END
