#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern "C"
{

#include "SVNETSDK.h"

//#pragma comment (lib,"SV_NET_SDK.lib")

}

#define MAX_DEVICES    (16)

int g_curlUserID = 0;
LONG m_lPlayHandle = -1;
LONG m_lThmlHandle = -1;
/** @fn     fThmlDataCallBack(LONG lRealHandle, char *dwDataType, BYTE *pBuffer, DWORD unWidth, DWORD unHeight, DWORD dwUser)
*  @brief  温度数据回调函数
*  @param	lRealHandle   [IN] - 取流句柄
*			dwDataType	  [IN] - 数据类型
*			pBuffer		  [IN] - 数据缓冲
*			dwBufSize	  [IN] - 数据大小
*			pUser		  [IN] - 用户指针
*  @return	void
*/
void CALLBACK fThmlDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD unWidth, DWORD unHeight, void* pUser)
{
	if (dwDataType == TYPE_THML16)
	{
		
		signed short *ptrS16TempVal = (signed short *)pBuffer;
		//温度数据
		int minVal = 16383;
		int maxVal = -1000;
		int avgVal = 0;
		for (int i = 0; i < unHeight; i+= 16)
		{
			for (int j = 0; j < unWidth; j+= 16)
			{
				if (ptrS16TempVal[i * unWidth + j] > maxVal)
				{
					maxVal = ptrS16TempVal[i * unWidth + j];
				}
				if (ptrS16TempVal[i * unWidth + j] < minVal)
				{
					minVal = ptrS16TempVal[i * unWidth + j];
				}
				avgVal += ptrS16TempVal[i * unWidth + j];
			}
		}
		avgVal = avgVal * 256 / (unHeight * unWidth);

		printf("temp val = %d %d %d\n", maxVal, minVal, avgVal);
		
	}
	else if (dwDataType == TYPE_MTRGN)
	{
		SV_NET_DEV_THMLMTRULE_PARAM_V12 strThmlMtRuleParamV12 = { 0 };
		int ret = 0;
		for (int rgnId = 0; rgnId < 1; rgnId++)
		{
			ret = SV_NET_DEV_GetThmlMtRuleListConfig_V13(pBuffer, rgnId, &strThmlMtRuleParamV12);
			//温度数据
			printf("lRealHandle = %d, id = %d and temp val = %f %f %f\n", lRealHandle, rgnId, strThmlMtRuleParamV12.outTemp[0], strThmlMtRuleParamV12.outTemp[1], strThmlMtRuleParamV12.outTemp[2]);
		}

	}
	else
	{

	}

}


/** @fn     On_Button_thmlData()()
*   @brief
*   @param
*	@param
*   @return bool
*/
void On_Button_thmlData()
{
	SV_NET_DEV_PREVIEWINFO strPreviewInfo = { 0 };
	strPreviewInfo.lChannel = 0;
	strPreviewInfo.hPlayWnd = NULL;// (HWND)widgets.at(videoIndex)->winId();

	strPreviewInfo.byVideoCodingType = 1;	//1--温度数据;3--测温规则数据
	//strPreviewInfo.byProtoType = 1;			//rtsp协议
	strPreviewInfo.dwLinkMode = 4;			//暂不检查该值
	
	strPreviewInfo.byRes1 = 10;		//表示获取帧率0--25

	int nErr = 0;
	int g_bStandardCB = 1;
	
	if (g_bStandardCB) //call back standard stream or not
	{
		m_lPlayHandle = SV_NET_DEV_RealPlay(g_curlUserID, &strPreviewInfo, fThmlDataCallBack, NULL);
		if (m_lPlayHandle < 0)
		{
			nErr = SV_NET_DEV_GetLastError();
			printf("[SV_NET_DEV_RealPlay] code:%d\n", nErr);
			return;
		}

	}
	sleep(1200);

	int ret = SV_NET_DEV_StopRealPlay(m_lPlayHandle, &strPreviewInfo);

}


int checkSum(char *pBuf, int len)
{
	int sum = 0, i;

	for (i = 0; i < len; i++) {
		sum += (unsigned int)pBuf[i] & 0xff;
	}

	return sum;
}
	
int main(int argc, char const *argv[])
{
	int ret = 0;

	char sdkVer[32] = { 0 };
	ret = SV_NET_DEV_VERSION(sdkVer);
	printf("sdk version %s...\n", sdkVer);

	ret = SV_NET_DEV_SDK_Init();

	SV_NET_DEV_USER_LOGIN_INFO strLoginInfo = { 0 };

	char *strIp = "192.168.2.64";
	strcpy(strLoginInfo.sDeviceAddress, strIp);

	strLoginInfo.wPort = 8000;

	//char sUserName[32] = { 0 };
	strcpy(strLoginInfo.sUserName, "admin");

	//char sPassword[32] = { 0 };
	strcpy(strLoginInfo.sPassword, "ipc12345");

	g_curlUserID = SV_NET_DEV_Login(&strLoginInfo, NULL);
	printf("API test: \n client login success...id = %#x\n", g_curlUserID);
	sleep(5);


	On_Button_thmlData();

	return 0;
}
