#ifndef _HC_SV_NET_SDK_H_
#define _HC_SV_NET_SDK_H_

#ifndef _WINDOWS_
#if (defined(_WIN32) || defined(_WIN64))
#include <winsock2.h>
#include <windows.h>    
#endif
#endif

#ifndef __PLAYRECT_defined
#define __PLAYRECT_defined
typedef struct __PLAYRECT
{
	int x;
	int y;
	int uWidth;
	int uHeight;
}PLAYRECT;
#endif

#if (defined(_WIN32)  || defined(_WIN64) ) //windows
#define SV_NET_SDK_API  __declspec(dllexport)
//#define SV_NET_SDK_API  extern "C"__declspec(dllimport)
typedef  unsigned __int64 UINT64;
typedef  signed __int64 INT64;
#elif defined(__linux__) || defined(__APPLE__) //linux
typedef     unsigned int    DWORD;
typedef     unsigned short  WORD;
typedef     unsigned short  USHORT;
typedef     short           SHORT;
typedef     int            LONG;
typedef      unsigned char    BYTE;
//#define     BOOL int
typedef     int 	        BOOL;
typedef     unsigned int       UINT;
typedef     void*             LPVOID;
typedef     void*             HANDLE;
typedef     unsigned int*  LPDWORD;
typedef  unsigned long long UINT64;
typedef  signed long long INT64;
typedef     unsigned int    UINT32;
typedef     int             INT32;
typedef     unsigned short  UINT16;
typedef     short  			INT16;
typedef      unsigned char    UINT8;

#ifndef    TRUE
#define    TRUE    1
#endif
#ifndef    FALSE
#define       FALSE 0
#endif
#ifndef    NULL
#define       NULL 0
#endif

#define __stdcall 
#define CALLBACK  

#define SV_NET_SDK_API extern "C"
typedef unsigned int   COLORKEY;
typedef unsigned int   COLORREF;

#ifndef __HWND_defined
#define __HWND_defined
#if defined(__linux__)
typedef unsigned int HWND;
#else
typedef void* HWND;
#endif
#endif

#ifndef __HDC_defined
#define __HDC_defined
#if defined(__linux__)
typedef struct __DC
{
	void*   surface;        //SDL Surface
	HWND    hWnd;           //HDC window handle
}DC;
typedef DC* HDC;
#else
typedef void* HDC;
#endif
#endif

typedef struct tagInitInfo
{
	int uWidth;
	int uHeight;
}INITINFO;
#endif

typedef struct MT_RULE_INFO {
	WORD x;                
	WORD y;
}DRAW_RULE_INFO, *PDRAW_RULE_INFO;

#define MAX_ID_COUNT    256
#define MAX_STREAM_ID_COUNT    1024
#define STREAM_ID_LEN   32

#define SERIALNO_LEN            48      //序列号长度
#define SOFTWARE_VER_LEN        24      //软件版本长度

#define NAME_LEN		32
#define PASSWD_LEN		16
#define PATHNAME_LEN		128
#define MACADDR_LEN		6
#define PHONENUMBER_LEN		32

#define MAX_SHELTERNUM            4       //设备最大遮挡区域数
#define MAX_ALARMOUT              4       //设备最大报警输出数
#define MAX_CHANNUM               16      //设备最大通道数
#define MAX_DAYS                  7       //每周天数
#define MAX_TIMESEGMENT_V40       16      //每节课信息
#define MAX_TIMESEGMENT           4       //设备最大时间段数

#define SV_NET_DEV_ADDRESS_MAX_LEN 129
#define SV_NET_DEV_LOGIN_USERNAME_MAX_LEN 64
#define SV_NET_DEV_LOGIN_PASSWD_MAX_LEN 64


#define SV_NET_DEV_GET_GBT28181_ACCESS_CFG            3249  //获取GBT28181协议接入配置        
#define SV_NET_DEV_SET_GBT28181_ACCESS_CFG            3250  //设置GBT28181协议接入配置
#define SV_NET_DEV_GET_GBT28181_CHANINFO_CFG        3251  //获取GBT28181协议接入设备的通道信息
#define SV_NET_DEV_SET_GBT28181_CHANINFO_CFG        3252  //设置GBT28181协议接入设备的通道信息
#define SV_NET_DEV_GET_GBT28181_ALARMINCFG            3253  //获取GBT28181协议接入设备的报警信息
#define SV_NET_DEV_SET_GBT28181_ALARMINCFG            3254  //设置GBT28181协议接入设备的报警信息


#if 1
enum PLAY_FRAME_TYPE
{
	TYPE_UNKNOWN = 0,
	TYPE_RGB16,
	TYPE_RGB24,
	TYPE_RGB32,
	TYPE_UYVY,
	TYPE_YV12,
	TYPE_YUY2,
	TYPE_YPUV, /* Y planar, UV interlaced, 4:2:2 */
	TYPE_THML16,
	TYPE_MTRGN,
	TYPE_THML16_RGB16,
};
#endif
//预览接口
typedef struct tagSV_NET_SDK_PREVIEWINFO
{
	LONG lChannel;//通道号
	DWORD dwStreamType;    // 码流类型，0-主码流，1-子码流，2-码流3，3-码流4, 4-码流5,5-码流6,7-码流7,8-码流8,9-码流9,10-码流10
	DWORD dwLinkMode;// 0：TCP方式,1：UDP方式,2：多播方式,3 - RTP方式，4-RTP/RTSP,5-RSTP/HTTP ,6- HRUDP（可靠传输） ,7-RTSP/HTTPS
	HWND hPlayWnd;//播放窗口的句柄,为NULL表示不播放图象
	DWORD bBlocked;  //0-非阻塞取流, 1-阻塞取流, 如果阻塞SDK内部connect失败将会有5s的超时才能够返回,不适合于轮询取流操作.
	DWORD bPassbackRecord; //0-不启用录像回传,1启用录像回传
	BYTE byPreviewMode;//预览模式，0-正常预览，1-延迟预览
	BYTE byStreamID[STREAM_ID_LEN/*32*/];//流ID，lChannel为0xffffffff时启用此参数
	BYTE byProtoType; //应用层取流协议，0-私有协议，1-RTSP协议
	BYTE byRes1;	//表示帧率设置
	BYTE byVideoCodingType; //码流数据编解码类型 0-通用编码数据 1-热成像探测器产生的原始数据（温度数据的加密信息，通过去加密运算，将原始数据算出真实的温度值）
	DWORD dwDisplayBufNum; //播放库播放缓冲区最大缓冲帧数，范围1-50，置0时默认为1 
	BYTE byNPQMode;	//NPQ是直连模式，还是过流媒体 0-直连 1-过流媒体
	BYTE byRes[215];
}SV_NET_DEV_PREVIEWINFO, *LPSV_NET_DEV_PREVIEWINFO;

typedef struct tagSV_NET_DEV_ETHERPARAM
{
	DWORD	devIp;
	DWORD	devIpMask;
	DWORD	mediaType;		/* network interface type */
	WORD	ipPortNo;		/* command port */
	BYTE	res1[2];
	BYTE	macAddr[6];
	BYTE	res2[2];
}SV_NET_DEV_ETHERPARAM, *LPSV_NET_DEV_ETHERPARAM;

typedef struct tagSV_NET_DEV_NETWORKPARAM
{
	DWORD	length;
	SV_NET_DEV_ETHERPARAM etherCfg[2];
	DWORD	manageHostIp;
	WORD	manageHostPort;
	WORD	httpPort;
	DWORD	ipResolverIpAddr;
	DWORD	mcastAddr;
	DWORD	gatewayIp;
	DWORD	nfsIp;
	BYTE	nfsDirectory[PATHNAME_LEN];
	DWORD	bEnablePPPoE;
	BYTE	pppoeName[NAME_LEN];
	BYTE	pppoePassword[PASSWD_LEN];
	BYTE	res2[4];
	DWORD	pppoeIp;
}SV_NET_DEV_NETWORKPARAM, *LPSV_NET_DEV_NETWORKPARAM;

//SV_NET_DEV_Login_V30()参数结构
typedef struct
{
	BYTE sSerialNumber[SERIALNO_LEN];  //序列号
	BYTE byAlarmInPortNum;		        //报警输入个数
	BYTE byAlarmOutPortNum;		        //报警输出个数
	BYTE byDiskNum;				    //硬盘个数
	BYTE byDVRType;				    //设备类型, 1:DVR 2:ATM DVR 3:DVS ......
	BYTE byChanNum;				    //模拟通道个数
	BYTE byStartChan;			        //起始通道号,例如DVS-1,DVR - 1
	BYTE byAudioChanNum;                //语音通道数
	BYTE byIPChanNum;					//最大数字通道个数，低位  
	BYTE byZeroChanNum;			//零通道编码个数 //2010-01-16
	BYTE byMainProto;			//主码流传输协议类型 0-private, 1-rtsp,2-同时支持private和rtsp
	BYTE bySubProto;				//子码流传输协议类型0-private, 1-rtsp,2-同时支持private和rtsp
	BYTE bySupport;        //能力，位与结果为0表示不支持，1表示支持，
						   //bySupport & 0x1, 表示是否支持智能搜索
						   //bySupport & 0x2, 表示是否支持备份
						   //bySupport & 0x4, 表示是否支持压缩参数能力获取
						   //bySupport & 0x8, 表示是否支持多网卡
						   //bySupport & 0x10, 表示支持远程SADP
						   //bySupport & 0x20, 表示支持Raid卡功能
						   //bySupport & 0x40, 表示支持IPSAN 目录查找
						   //bySupport & 0x80, 表示支持rtp over rtsp
	BYTE bySupport1;        // 能力集扩充，位与结果为0表示不支持，1表示支持
							//bySupport1 & 0x1, 表示是否支持snmp v30
							//bySupport1 & 0x2, 支持区分回放和下载
							//bySupport1 & 0x4, 是否支持布防优先级	
							//bySupport1 & 0x8, 智能设备是否支持布防时间段扩展
							//bySupport1 & 0x10, 表示是否支持多磁盘数（超过33个）
							//bySupport1 & 0x20, 表示是否支持rtsp over http	
							//bySupport1 & 0x80, 表示是否支持车牌新报警信息2012-9-28, 且还表示是否支持SV_NET_DEV_IPPARACFG_V40结构体
	BYTE bySupport2; /*能力，位与结果为0表示不支持，非0表示支持
					 bySupport2 & 0x1, 表示解码器是否支持通过URL取流解码
					 bySupport2 & 0x2,  表示支持FTPV40
					 bySupport2 & 0x4,  表示支持ANR
					 bySupport2 & 0x8,  表示支持CCD的通道参数配置
					 bySupport2 & 0x10,  表示支持布防报警回传信息（仅支持抓拍机报警 新老报警结构）
					 bySupport2 & 0x20,  表示是否支持单独获取设备状态子项
					 bySupport2 & 0x40,  表示是否是码流加密设备*/
	WORD wDevType;              //设备型号
	BYTE bySupport3; //能力集扩展，位与结果为0表示不支持，1表示支持
					 //bySupport3 & 0x1, 表示是否多码流
					 // bySupport3 & 0x4 表示支持按组配置， 具体包含 通道图像参数、报警输入参数、IP报警输入、输出接入参数、
					 // 用户参数、设备工作状态、JPEG抓图、定时和时间抓图、硬盘盘组管理 
					 //bySupport3 & 0x8为1 表示支持使用TCP预览、UDP预览、多播预览中的"延时预览"字段来请求延时预览（后续都将使用这种方式请求延时预览）。而当bySupport3 & 0x8为0时，将使用 "私有延时预览"协议。
					 //bySupport3 & 0x10 表示支持"获取报警主机主要状态（V40）"。
					 //bySupport3 & 0x20 表示是否支持通过DDNS域名解析取流

	BYTE byMultiStreamProto;//是否支持多码流,按位表示,0-不支持,1-支持,bit1-码流3,bit2-码流4,bit7-主码流，bit-8子码流
	BYTE byStartDChan;		//起始数字通道号,0表示无效
	BYTE byStartDTalkChan;	//起始数字对讲通道号，区别于模拟对讲通道号，0表示无效
	BYTE byHighDChanNum;		//数字通道个数，高位
	BYTE bySupport4;
	BYTE byLanguageType;// 支持语种能力,按位表示,每一位0-不支持,1-支持  
						//  byLanguageType 等于0 表示 老设备
						//  byLanguageType & 0x1表示支持中文
						//  byLanguageType & 0x2表示支持英文
	BYTE byRes2[9];		//保留
}SV_NET_DEV_DEVICEINFO, *LPSV_NET_DEV_DEVICEINFO;


typedef void (CALLBACK *fLoginResultCallBack) (LONG lUserID, DWORD dwResult, LPSV_NET_DEV_DEVICEINFO lpDeviceInfo, void* pUser);

//登录接口
typedef struct
{
	char sDeviceAddress[SV_NET_DEV_ADDRESS_MAX_LEN];
	BYTE byUseTransport;    //是否启用能力集透传，0--不启用透传，默认，1--启用透传
	WORD wPort;
	char sUserName[SV_NET_DEV_LOGIN_USERNAME_MAX_LEN];
	char sPassword[SV_NET_DEV_LOGIN_PASSWD_MAX_LEN];
	fLoginResultCallBack cbLoginResult;
	void *pUser;
	BOOL bUseAsynLogin;
	BYTE byProxyType; //0:不使用代理，1：使用标准代理，2：使用EHome代理
	BYTE byUseUTCTime;    //0-不进行转换，默认,1-接口上输入输出全部使用UTC时间,SDK完成UTC时间与设备时区的转换,2-接口上输入输出全部使用平台本地时间，SDK完成平台本地时间与设备时区的转换
	BYTE byLoginMode; //0-Private 1-ISAPI 2-自适应
	BYTE byHttps;    //0-不适用tls，1-使用tls 2-自适应
	LONG iProxyID;    //代理服务器序号，添加代理服务器信息时，相对应的服务器数组下表值
	BYTE byVerifyMode;  //认证方式，0-不认证，1-双向认证，2-单向认证；认证仅在使用TLS的时候生效;
	BYTE byRes3[119];
}SV_NET_DEV_USER_LOGIN_INFO, *LPSV_NET_DEV_USER_LOGIN_INFO;

typedef struct tagSV_NET_DEV_UPGRADE_PARAM
{
	DWORD dwUpgradeType;
	char sFileName[256];
	void *pInbuffer;
	DWORD dwBufferLen;
	char *pUnitIdList[64];
	BYTE  byRes[112];
}SV_NET_DEV_UPGRADE_PARAM, *LPSV_NET_DEV_UPGRADE_PARAM;

//设置完全获取出厂值
typedef struct tagSV_NET_DEV_COMPLETE_RESTORE_INFO_
{
	DWORD   dwSize; //结构体长度
	DWORD   dwChannel; //通道号
	BYTE    byRes[64];
}SV_NET_DEV_COMPLETE_RESTORE_INFO, *LPSV_NET_DEV_COMPLETE_RESTORE_INFO;

//登陆设备返回的设备信息
typedef struct {
	UINT8	DEVName[NAME_LEN];
	UINT8	serialno[SERIALNO_LEN];
	UINT8	softwareVersion[SOFTWARE_VER_LEN];
	UINT8	hardwareVersion[SOFTWARE_VER_LEN];
	//UINT32	softwareVersion;
	//UINT32	softwareBuildDate;
	UINT8	devType[NAME_LEN];
}SV_NET_DEV_STDINFO, *LPSV_NET_DEV_STDINFO;


typedef struct {
	UINT8	DEVName[NAME_LEN];
	UINT8	serialno[SERIALNO_LEN];
	UINT8	softwareVersion[SOFTWARE_VER_LEN];
	UINT8	hardwareVersion[SOFTWARE_VER_LEN];
	//UINT32	softwareVersion;
	//UINT32	softwareBuildDate;
	UINT8	devType[NAME_LEN];

	DWORD dwDEVID;                //DEV ID
	DWORD dwRecycleRecord;        //是否循环录像,0:不是; 1:是
	DWORD dwHardwareVersion;    // 硬件版本,高16位是主版本,低16位是次版本
	BYTE byAlarmInPortNum;        //报警输入个数
	BYTE byAlarmOutPortNum;        //报警输出个数
	BYTE byRS232Num;            // 232串口个数
	BYTE byRS485Num;            // 485串口个数

	BYTE byNetworkPortNum;        //网络口个数
	BYTE byDiskNum;                // 硬盘个数
	BYTE byChanNum;                //DVR 通道个数
	BYTE byStartChan;            //起始通道号,例如DVS-1,DVR - 1

	BYTE byUSBNum;                //USB口的个数
	BYTE byAudioNum;            //语音口的个数
	BYTE byRes[2];
}SV_NET_DEV_STDINFO_EX, *LPSV_NET_DEV_STDINFO_EX;


//热成像设备图像参数信息
typedef struct {
	UINT8	contrast;
	UINT8	brightness;
	UINT8	ideMode;
	UINT8	ideLevel;
	UINT8	nrMode;
	UINT8	nrTempLevel;
	UINT8	nrSpaceLevel;
	UINT8	psdcMode;
}SV_NET_DEV_THMLPIC_PARAM, *LPSV_NET_DEV_THMLPIC_PARAM;

typedef struct {
	UINT8	contrast;
	UINT8	brightness;
	UINT8	ideMode;
	UINT8	ideLevel;
	UINT8	nrMode;
	UINT8	nrTempLevel;
	UINT8	nrSpaceLevel;
	UINT8	psdcMode;

	UINT8 ffcMode; /*0 自动 1手动*/
	UINT8 mirror;  /* 镜像：0 Left;1 Right,;2 Up;3Down */
	UINT8 digitalZoom;  /*数字缩放:0 dsibale  1 enable*/
	UINT8 deadPixelDetect;   /*坏点检测,0 dsibale  1 enable*/

	UINT8 res[20];
}SV_NET_DEV_THMLPIC_PARAM_V13, *LPSV_NET_DEV_THMLPIC_PARAM_V13;

//时间段(子结构)
typedef struct
{
	//开始时间
	BYTE byStartHour;
	BYTE byStartMin;
	//结束时间
	BYTE byStopHour;
	BYTE byStopMin;
}SV_NET_DEV_SCHEDTIME, *LPSV_NET_DEV_SCHEDTIME;

//报警和异常处理结构(子结构)(多处使用)
typedef struct
{
	DWORD    dwHandleType;            /*处理方式,处理方式的"或"结果*/
									  /*0x00: 无响应*/
									  /*0x01: 监视器上警告*/
									  /*0x02: 声音警告*/
									  /*0x04: 上传中心*/
									  /*0x08: 触发报警输出*/
									  /*0x10: Jpeg抓图并上传EMail*/
	BYTE byRelAlarmOut[MAX_ALARMOUT];  //报警触发的输出通道,报警触发的输出,为1表示触发该输出
}SV_NET_DEV_HANDLEEXCEPTION, *LPSV_NET_DEV_HANDLEEXCEPTION;


//信号丢失报警(子结构)
typedef struct
{
	BYTE byEnableHandleVILost;    /* 是否处理信号丢失报警 */
	SV_NET_DEV_HANDLEEXCEPTION strVILostHandleType;    /* 处理方式 */
	SV_NET_DEV_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT];//布防时间
}SV_NET_DEV_VILOST, *LPSV_NET_DEV_VILOST;

//遮挡区域(子结构)
typedef struct
{
	WORD wHideAreaTopLeftX;                /* 遮挡区域的x坐标 */
	WORD wHideAreaTopLeftY;                /* 遮挡区域的y坐标 */
	WORD wHideAreaWidth;                /* 遮挡区域的宽 */
	WORD wHideAreaHeight;                /*遮挡区域的高*/
}SV_NET_DEV_SHELTER, *LPSV_NET_DEV_SHELTER;

//移动侦测(子结构)
typedef struct
{
	BYTE byMotionScope[18][22];    /*侦测区域,共有22*18个小宏块,为1表示改宏块是移动侦测区域,0-表示不是*/
	BYTE byMotionSensitive;        /*移动侦测灵敏度, 0 - 5,越高越灵敏,0xff关闭*/
	BYTE byEnableHandleMotion;    /* 是否处理移动侦测 */
	BYTE byEnableDisplay;    /*启用移动侦测高亮显示，0-否，1-是*/
	char reservedData;
	SV_NET_DEV_HANDLEEXCEPTION strMotionHandleType;    /* 处理方式 */
	SV_NET_DEV_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT];//布防时间
	BYTE byRelRecordChan[MAX_CHANNUM]; //报警触发的录象通道,为1表示触发该通道
}SV_NET_DEV_MOTION, *LPSV_NET_DEV_MOTION;

//遮挡报警(子结构)  区域大小704*576
typedef struct
{
	DWORD dwEnableHideAlarm;                /* 是否启动遮挡报警 ,0-否,1-低灵敏度 2-中灵敏度 3-高灵敏度*/
	WORD wHideAlarmAreaTopLeftX;            /* 遮挡区域的x坐标 */
	WORD wHideAlarmAreaTopLeftY;            /* 遮挡区域的y坐标 */
	WORD wHideAlarmAreaWidth;                /* 遮挡区域的宽 */
	WORD wHideAlarmAreaHeight;                /*遮挡区域的高*/
	SV_NET_DEV_HANDLEEXCEPTION strHideAlarmHandleType;    /* 处理方式 */
	SV_NET_DEV_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT];//布防时间
}SV_NET_DEV_HIDEALARM, *LPSV_NET_DEV_HIDEALARM;

//通道图象结构SDK_V14扩展
typedef struct
{
	DWORD dwSize;
	BYTE sChanName[NAME_LEN];
	DWORD dwVideoFormat;    /* 只读 视频制式 1-NTSC 2-PAL*/
	BYTE byBrightness;      /*亮度,0-255*/
	BYTE byContrast;        /*对比度,0-255*/
	BYTE bySaturation;      /*饱和度,0-255 */
	BYTE byHue;                /*色调,0-255*/
							   //显示通道名
	DWORD dwShowChanName; // 预览的图象上是否显示通道名称,0-不显示,1-显示 区域大小704*576
	WORD wShowNameTopLeftX;                /* 通道名称显示位置的x坐标 */
	WORD wShowNameTopLeftY;                /* 通道名称显示位置的y坐标 */
										   //信号丢失报警
	SV_NET_DEV_VILOST struVILost;
	//移动侦测
	SV_NET_DEV_MOTION struMotion;
	//遮挡报警
	SV_NET_DEV_HIDEALARM struHideAlarm;
	//遮挡  区域大小704*576
	DWORD dwEnableHide;        /* 是否启动遮挡 ,0-否,1-是*/
	SV_NET_DEV_SHELTER struShelter[MAX_SHELTERNUM];
	//OSD
	DWORD dwShowOsd;// 预览的图象上是否显示OSD,0-不显示,1-显示 区域大小704*576
	WORD wOSDTopLeftX;                /* OSD的x坐标 */
	WORD wOSDTopLeftY;                /* OSD的y坐标 */
	BYTE byOSDType;                    /* OSD类型(主要是年月日格式) */
									   /* 0: XXXX-XX-XX 年月日 */
									   /* 1: XX-XX-XXXX 月日年 */
									   /* 2: XXXX年XX月XX日 */
									   /* 3: XX月XX日XXXX年 */
									   /* 4: XX-XX-XXXX 日月年*/
									   /* 5: XX日XX月XXXX年 */
									   /*6: xx/xx/xxxx(月/日/年) */
									   /*7: xxxx/xx/xx(年/月/日) */
									   /*8: xx/xx/xxxx(日/月/年)*/
	BYTE byDispWeek;                /* 是否显示星期 */
	BYTE byOSDAttrib;                /* OSD属性:透明，闪烁 */
									 /* 1: 透明,闪烁 */
									 /* 2: 透明,不闪烁 */
									 /* 3: 闪烁,不透明 */
									 /* 4: 不透明,不闪烁 */
	BYTE byHourOsdType;        /* OSD小时制:0-24小时制,1-12小时制 */
}SV_NET_DEV_PICCFG_EX, *LPSV_NET_DEV_PICCFG_EX;


//码流压缩参数(子结构)(9000扩展)
typedef struct
{
	BYTE byStreamType;        //码流类型 0-视频流, 1-复合流, 表示事件压缩参数时最高位表示是否启用压缩参数
	
	BYTE byResolution;			/*分辨率0-DCIF 1-CIF, 2-QCIF, 3-4CIF,*/
	BYTE byBitrateType;        //码率类型 0:变码率, 1:定码率,0xfe:自动，和源一致
	BYTE byPicQuality;        //图象质量 0-最好 1-次好 2-较好 3-一般 4-较差 5-差,自动，和源一致
							  /*视频码率:0-保留，1-16K(保留)，2-32K，3-48k，4-64K，5-80K，6-96K，7-128K，8-160k，9-192K，10-224K，
							  11-256K，12-320K，13-384K，14-448K，15-512K，16-640K，17-768K，18-896K，19-1024K，20-1280K，21-1536K，22-1792K，23-2048K，
							  24-3072K，25-4096K，26-8192K，27-16384K。最高位(31位)置成1表示是自定义码流，0～30位表示码流值，最小值16k,0xfffffffe，自动，和源一致*/
	DWORD dwVideoBitrate;
	DWORD dwVideoFrameRate;    //帧率 0-全部; 1-1/16; 2-1/8; 3-1/4; 4-1/2; 5-1; 6-2; 7-4; 8-6; 9-8; 10-10; 11-12; 12-16; 13-20; V2.0版本中新加14-15; 15-18; 16-22;
							   //17-25；18-30；19-35；20-40；21-45；22-50；23-55；24-60；25-3;26-5;27-7;28-9;29-100; 30-120;31-24;32-48,33-8.3,0xfffffffe-自动，和源一致
	WORD  wIntervalFrameI;  //I帧间隔,0xfffe 自动，和源一致
	BYTE  byIntervalBPFrame;//0-BBP帧; 1-BP帧; 2-单P帧(2006-08-11 增加单P帧的配置接口，可以改善实时流延时问题)；0xfe-自动，和源一致
	BYTE  byres1;        //保留
	BYTE  byVideoEncType;   //视频编码类型 0-私有264，1-标准h264，2-标准mpeg4，7-M-JPEG，8-MPEG2，9-SVAC, 10-标准h265, 0xfe-自动（和源一致），0xff-无效 
	BYTE  byAudioEncType;   //音频编码类型 0-G722;1-G711_U;2-G711_A;5-MP2L2;6-G276;7-AAC;8-PCM;12-AAC_LC;13-AAC_LD;14-Opus;15-MP3;0xff-无效
	BYTE  byVideoEncComplexity; //视频编码复杂度，0-低，1-中，2高,0xfe:自动，和源一致
	BYTE  byEnableSvc; //0 - 不启用SVC功能；1- 启用SVC功能; 2-自动启用SVC功能
	BYTE  byFormatType; //封装类型，1-裸流，2-RTP封装，3-PS封装，4-TS封装，5-私有，6-FLV，7-ASF，8-3GP,9-RTP+PS（国标：GB28181），0xff-无效
	BYTE  byAudioBitRate; //音频码率 参考 BITRATE_ENCODE_INDEX
	BYTE  byStreamSmooth;//码流平滑 1～100（1等级表示清晰(Clear)，100表示平滑(Smooth)）
	BYTE  byAudioSamplingRate;//音频采样率0-默认,1- 16kHZ, 2-32kHZ, 3-48kHZ, 4- 44.1kHZ,5-8kHZ
	BYTE  bySmartCodec;///*bit0-高性能编码 0-关闭，1-打开，bit1 - 低码率模式 0 - 关闭，1 - 打开*/
	BYTE  byDepthMapEnable;// 深度图使能开关，0-关闭，1-打开；如果开启后，第二通道的子码流（双目）的视频参数都不能配置，默认输出 960*540 的深度图；
						   //平均码率（在SmartCodec使能开启下生效）, 0-0K 1-16K 2-32K 3-48k 4-64K 5-80K 6-96K 7-128K 8-160k 9-192K 10-224K 11-256K 12-320K 13-384K 14-448K 15-512K 16-640K 17-768K 18-896K 19-1024K 20-1280K 21-1536K 22-1792K 23-2048K 24-2560K 25-3072K 26-4096K 27-5120K 28-6144K 29-7168K 30-8192K
						   //最高位(15位)置成1表示是自定义码流, 0-14位表示码流值(MIN- 0 K)。
	WORD  wAverageVideoBitrate;
}SV_NET_DEV_COMPRESSION_INFO_V30, *LPSV_NET_DEV_COMPRESSION_INFO_V30;

//通道压缩参数(9000扩展)
typedef struct
{
	DWORD dwSize;
	SV_NET_DEV_COMPRESSION_INFO_V30 struNormHighRecordPara;    //录像 对应8000的普通
	SV_NET_DEV_COMPRESSION_INFO_V30 struRes;//保留 char reserveData[28];
	SV_NET_DEV_COMPRESSION_INFO_V30 struEventRecordPara;       //事件触发压缩参数
	SV_NET_DEV_COMPRESSION_INFO_V30 struNetPara;               //网传(子码流)
}SV_NET_DEV_COMPRESSIONCFG_V30, *LPSV_NET_DEV_COMPRESSIONCFG_V30;

//图片质量
typedef struct
{
	/*注意：当图像压缩分辨率为VGA时，支持0=CIF, 1=QCIF, 2=D1抓图，
	当分辨率为3=UXGA(1600x1200), 4=SVGA(800x600), 5=HD720p(1280x720),6=VGA,7=XVGA, 8=HD900p
	仅支持当前分辨率的抓图*/

	/* 可以通过能力集获取
	0-CIF，           1-QCIF，           2-D1，         3-UXGA(1600x1200), 4-SVGA(800x600),5-HD720p(1280x720)，
	6-VGA，           7-XVGA，           8-HD900p，     9-HD1080，     10-2560*1920，
	11-1600*304，     12-2048*1536，     13-2448*2048,  14-2448*1200， 15-2448*800，
	16-XGA(1024*768), 17-SXGA(1280*1024),18-WD1(960*576/960*480),      19-1080i,      20-576*576，
	21-1536*1536,     22-1920*1920,      23-320*240,    24-720*720,    25-1024*768,
	26-1280*1280,     27-1600*600,       28-2048*768,   29-160*120,    55-3072*2048,
	64-3840*2160,     70-2560*1440,      75-336*256,
	78-384*256,         79-384*216,        80-320*256,    82-320*192,    83-512*384,
	127-480*272,      128-512*272,       161-288*320,   162-144*176,   163-480*640,
	164-240*320,      165-120*160,       166-576*720,   167-720*1280,  168-576*960,
	180-180*240,      181-360*480,       182-540*720,    183-720*960,  184-960*1280,
	185-1080*1440,      215-1080*720(占位，未测试),  216-360x640(占位，未测试),245-576*704(占位，未测试)
	500-384*288,
	0xff-Auto(使用当前码流分辨率)
	*/
	WORD    wPicSize;
	WORD    wPicQuality;            /* 图片质量系数 0-最好 1-较好 2-一般 */
}SV_NET_DEV_JPEGPARA, *LPSV_NET_DEV_JPEGPARA;


typedef struct
{
	WORD    wPicDepth;
	WORD    wPicQuality;
}SV_NET_DEV_BMPPARA, *LPSV_NET_DEV_BMPPARA;

// 流信息 - 72字节长
typedef struct tagSV_NET_DEV_STREAM_INFO
{
	DWORD dwSize;
	BYTE  byID[STREAM_ID_LEN];      //ID数据
	DWORD dwChannel;                //关联设备通道，等于0xffffffff时，表示不关联
	BYTE  byRes[32];                //保留
}SV_NET_DEV_STREAM_INFO, *LPSV_NET_DEV_STREAM_INFO;

typedef struct tagSV_NET_DEV_MANUAL_RECORD_PARA
{
	SV_NET_DEV_STREAM_INFO struStreamInfo;
	DWORD			lRecordType;
	BYTE			byRes[32];
}SV_NET_DEV_MANUAL_RECORD_PARA, *LPSV_NET_DEV_MANUAL_RECORD_PARA;


#define MTRULE_MAX_POLYGON_POINT_NUM  (10)    //多边形顶点数最大值
typedef struct POINT_S32_S
{
	INT32 s32X;
	INT32 s32Y;
} POINT_S32_S;

//测温规则参数
typedef struct
{
	UINT32                    presetId;          //预置点号
	UINT32                    tempUnit;          //温度单位
	UINT32                    diffAlarmNum;      //区域温差报警对数
	UINT8                     enable;
	UINT8                     paramIndpnd; //规则测温是否支持独立参数(包括发射率、距离、反射温度），1支持，0不支持
	UINT8                     refTempkey;
	UINT8                     regionId;
	UINT8                     name[128];
	float                     refTemp;
	float                     distance;
	float                     emissionRate;
	UINT8					  regionType;
	UINT8                     pointNum;
	POINT_S32_S               pointList[MTRULE_MAX_POLYGON_POINT_NUM];
}SV_NET_DEV_THMLMTRULE_PARAM, *LPSV_NET_DEV_THMLMTRULE_PARAM;

//测温规则参数V12
typedef struct
{
	UINT32                    presetId;          //预置点号
	UINT32                    tempUnit;          //温度单位
	UINT32                    diffAlarmNum;      //区域温差报警对数
	UINT8                     enable;
	UINT8                     paramIndpnd; //规则测温是否支持独立参数(包括发射率、距离、反射温度），1支持，0不支持
	UINT8                     refTempkey;
	UINT8                     regionId;
	UINT8                     name[128];
	float                     refTemp;
	float                     distance;
	float                     emissionRate;
	UINT8					  regionType;
	UINT8                     pointNum;
	POINT_S32_S               pointList[MTRULE_MAX_POLYGON_POINT_NUM];
	float                     outTemp[3];
}SV_NET_DEV_THMLMTRULE_PARAM_V12, *LPSV_NET_DEV_THMLMTRULE_PARAM_V12;


//测温报警规则参数
typedef struct
{
	UINT8                     alarmkey;        //报警开关
	UINT8        			  alarmLevel;      //0-正常 1-预警 2-报警 3-解除预警
	UINT8         			  alarmRule;	   //1 :温度小于 2:温度大于
	UINT8         			  alarmType;	   //1:最高温报警2:最低温报警3:平均温报警4:温差报警	
	float                  	  alarmTmp;        //报警温度
	float                     preAlarmTmp;     //预警温度
	float                     thresholdTmp;    //门限温度
	float                     measureTmpData;
	float                     ruleTmpData;

}SV_NET_DEV_THMLMTALARMRULE_PARAM, *LPSV_NET_DEV_TMLMTALARMRULE_PARAM;

//测温规则参数V12
typedef struct
{
	UINT32                    presetId;          //预置点号
	UINT32                    tempUnit;          //温度单位
	UINT32                    diffAlarmNum;      //区域温差报警对数
	UINT8                     enable;
	UINT8                     paramIndpnd; //规则测温是否支持独立参数(包括发射率、距离、反射温度），1支持，0不支持
	UINT8                     refTempkey;
	UINT8                     regionId;
	UINT8                     name[128];
	float                     refTemp;
	float                     distance;
	float                     emissionRate;
	UINT8					  regionType;
	UINT8                     pointNum;
	POINT_S32_S               pointList[MTRULE_MAX_POLYGON_POINT_NUM];
	float                     outTemp[3];
	POINT_S32_S               outputPoint[3];

	SV_NET_DEV_THMLMTALARMRULE_PARAM strThmlAlarmRuleParam;
}SV_NET_DEV_THMLMTRULE_PARAM_V12Ex, *LPSV_NET_DEV_THMLMTRULE_PARAM_V12Ex;

//测温规则参数V13
typedef struct
{
	UINT32                    presetId;          //预置点号
	UINT32                    tempUnit;          //温度单位
	UINT32                    diffAlarmNum;      //区域温差报警对数
	UINT8                     enable;
	UINT8                     paramIndpnd; //规则测温是否支持独立参数(包括发射率、距离、反射温度），1支持，0不支持
	UINT8                     refTempkey;
	UINT8                     regionId;
	UINT8                     name[128];
	float                     refTemp;
	float                     distance;
	float                     emissionRate;
	UINT8					  regionType;
	UINT8                     pointNum;
	POINT_S32_S               pointList[MTRULE_MAX_POLYGON_POINT_NUM];
	float                     outTemp[3];
	POINT_S32_S               outputPoint[3];
	
	float                     mtComOffset;			//温度补偿值
	UINT8                     res[64];

	SV_NET_DEV_THMLMTALARMRULE_PARAM strThmlAlarmRuleParam;
}SV_NET_DEV_THMLMTRULE_PARAM_V13, *LPSV_NET_DEV_THMLMTRULE_PARAM_V13;

typedef    struct tagSV_NET_DEV_TEMPERATURE_COLOR
{
	/*
	选择0~高温报警类型时，<highTemperature>字段生效,当高于该温度值时，会有进行颜色标注，
	选择1~低温报警类型时, <lowTemperature>字段生效,当低于该温度值时，会有进行颜色标注。
	选择2~区间报警类型时，<highTemperature>、<lowTemperature>字段生效，当在温度在该温度区间时，会有进行颜色标注。
	选择3~保温报警类型时，<highTemperature>、<lowTemperature>字段生效，当温度不在该温度区间时，会有进行颜色标注。
	选择4~为无报警类型，<nullAlarm>字段生效，关闭报警，*/
	BYTE     byType;//测温报警颜色控制类型，0~无报警类型（关闭），1~高温报警类型，2~低温报警类型，3~区间报警类型，4~保温报警类型
	BYTE     byRes1[3];
	int        iHighTemperature;//高温值，-273~10000
	int        iLowTemperature;//低温值，-273~10000
	BYTE     byRes[8];
}SV_NET_DEV_TEMPERATURE_COLOR, *LPSV_NET_DEV_TEMPERATURE_COLOR;

typedef    struct tagSV_NET_DEV_THERMOMETRY_BASICPARAM
{
	DWORD        dwSize;//结构体大小
	BYTE        byEnabled;  //是否使能：0- 否，1- 是
	BYTE        byStreamOverlay; //码流叠加温度信息：0- 否，1- 是
	BYTE         byPictureOverlay;//抓图叠加温度信息：0- 否，1- 是
	BYTE        byMtinfoOnChan;		//可见光通道叠加温度信息
	BYTE        byRawdataOnVideo;		//码流叠加裸流信息
	BYTE        byRawdataOnJpeg;		//抓图叠加裸流信息
	BYTE        byRefreshRawIntval;		//原始数据刷新间隔
	BYTE        byThermometryRange;//测温范围: 0-默认值,1-(-20~150),2-(0~550)（这里以摄氏度为单位计算）,3-(摄氏度:0-650℃；华氏温度:32-1200℉),4-（摄氏度: -40-150℃）,5-(摄氏度: 0~1200℃)（这里以摄氏度为单位计算，根据测温单位设定不同测温范围的显示），6-(摄氏度: -20-120℃,0xff-自动
	BYTE        byThermometryUnit;//测温单位: 0-摄氏度（℃），1-华氏度（℉），2-开尔文(K)。
	BYTE        byThermometryCurve;//测温曲线模式显示方式，0-关闭，1-模式1（横向温度趋势线模式），2-模式2（纵向温度趋势线模式）
	BYTE        byFireImageModea;//消防图像模式，0-保留，1-黑白模式，2-热探测模式，3-火场模式(字段0目前保留，避免与之前接口不兼容)    
	BYTE        byShowTempStripEnable;//显示温度条使能：0- 否，1- 是
	float       fEmissivity;//发射率(发射率 精确到小数点后两位)[0.01, 1.00](即：物体向外辐射能量的本领)
	BYTE         byDistanceUnit;//距离单位: 0-米（m），1-英尺（feet）,2-厘米(centimeter)
	BYTE         byEnviroHumidity;//环境相对湿度，取值范围：0~100%
	BYTE        byRes2[2];
	SV_NET_DEV_TEMPERATURE_COLOR struTempColor;//测温报警颜色
	int            iEnviroTemperature;//环境温度，取值范围：-273~10000摄氏度
	int          iCorrectionVolume;//测温修正量，取值范围：-100~100
								   /* bit0-中心点测温：0-不显示，1-显示；
								   bit1-最高点测温：0-不显示，1-显示；
								   bit2-最低点测温：0-不显示，1-显示；
								   */
	BYTE       bySpecialPointThermType;// 特殊测温点显示
	BYTE       byReflectiveEnabled;//反射温度使能：0- 否，1- 是
	WORD       wDistance;//距离(m)[0, 10000]
	float      fReflectiveTemperature;//反射温度 精确到小数后一位
	float      fAlert;//预警温度阈值，-100.0-1000.0度（精确到小数点后一位）
	float      fAlarm;//报警温度阈值，-100.0-1000.0度（精确到小数点后一位）
	float         fThermalOpticalTransmittance;// 光学透过率, 精确到小数点后3位，范围0.001-1.000，默认1.000
	float      fExternalOpticsWindowCorrection;//外部光学温度，默认值20℃，范围为-40.0~80.0℃，实际显示单位以界面显示为准
	BYTE       byDisplayMaxTemperatureEnabled;// 显示最高温 0-不显示 1-显示
	BYTE       byDisplayMinTemperatureEnabled;// 显示最低温 0-不显示 1-显示
	BYTE       byDisplayAverageTemperatureEnabled;// 显示平均温 0-不显示 1-显示
	BYTE       byThermometryInfoDisplayposition;// 测温信息显示位置 0-保留 1-跟随规则 2-屏幕左上角
	DWORD        dwAlertFilteringTime;//温度预警等待时间,单位秒
	DWORD        dwAlarmFilteringTime;//温度报警等待时间,单位秒
	BYTE       byRes[52];
}SV_NET_DEV_THMLMT_BASICPARAM, *LPSV_NET_DEV_THMLMT_BASICPARAM;

typedef struct tagSV_NET_DEV_MTRULE_CMPOSPARAM
{
	float tempoffset[25];
	unsigned char res[64];
}SV_NET_DEV_MTRULE_CMPOSPARAM, *LPSV_NET_DEV_MTRULE_CMPOSPARAM;

//测温模式配置
typedef    struct tagSV_NET_DEV_THERMOMETRY_MODE
{
	DWORD     dwSize;//结构体大小
	BYTE      byMode;//测温模式，0~普通模式，1~专家模式
	BYTE      byThermometryROIEnabled; //测温ROI使能 0-保留 1-不开启 2-开启（基于互斥兼容考虑）
	BYTE      byRes[62];
}SV_NET_DEV_THERMOMETRY_MODE, *LPSV_NET_DEV_THERMOMETRY_MODE;

//测温二次标定参数结构体
typedef struct {
	float BBTemp;
	float refTemp;
	float distance;
	int x;
	int y;
	BYTE lenType;
	BYTE flg;
	BYTE byRes[2];
}SV_NET_DEV_MTCALIBPOINT, *LPSV_NET_DEV_MTCALIBPOINT;

//校时结构参数
typedef struct
{
	DWORD dwYear;        //年
	DWORD dwMonth;        //月
	DWORD dwDay;        //日
	DWORD dwHour;        //时
	DWORD dwMinute;        //分
	DWORD dwSecond;        //秒
}SV_NET_DEV_TIME, *LPSV_NET_DEV_TIME;

typedef struct tagSV_NET_DEV_TIME_V30
{
	WORD wYear;
	BYTE byMonth;
	BYTE byDay;
	BYTE byHour;
	BYTE byMinute;
	BYTE bySecond;
	BYTE    byISO8601;      /*是否是8601的时间格式，即时差字段是否有效0-时差无效，年月日时分秒为设备本地时间 1-时差有效 */
	WORD	wMilliSec;       //毫秒，精度不够，默认为0
	char    cTimeDifferenceH;  		//与UTC的时差（小时），-12 ... +14，+表示东区, byISO8601为1时有效
	char    cTimeDifferenceM;         	//与UTC的时差（分钟），-30, 30, 45，+表示东区，byISO8601为1时有效
}SV_NET_DEV_TIME_V30, *LPSV_NET_DEV_TIME_V30;

typedef struct tagSV_NET_DEV_NTP
{
	BYTE	ntpServer[64];			/* Domain Name or IP addr of NTP server */
	WORD	interval;				/* adjust time interval(hours) */
	UINT8	enableNTP;				/* enable NPT client */
	signed char timedifferenceH;	/* -12 ... +13 hours */
	signed char timedifferenceM;	/* 0 .. 59 minutes */
	BYTE	res[11];
}SV_NET_DEV_NTP, *LPSV_NET_DEV_NTP;

/*
IP地址
*/
typedef struct
{
	char    sIpV4[16];                        /* IPv4地址 */
	BYTE    byIPv6[128];                        /* 保留 */
}SV_NET_DEV_IPADDR, *LPSV_NET_DEV_IPADDR;

typedef struct tagSV_NET_DEV_ADDRESS
{
	SV_NET_DEV_IPADDR struIP; //IP地址
	WORD wPort;    //端口号
	BYTE byRes[2];
}SV_NET_DEV_ADDRESS, *LPSV_NET_DEV_ADDRESS;

typedef struct tagSV_NET_DEV_PLAY_BY_NAME_PARA
{
	char szFileName[100]; //回放文件名
	BYTE byDownload;    //是否下载 0-否，1-是
	BYTE byRes1[127];
	HWND hWnd;  //回放的窗口句柄，若置为空，SDK仍能收到码流数据，但不解码显示
	SV_NET_DEV_ADDRESS struAddr; //文件所在集群中CS地址信息，该信息文件查找时会返回
							  //如果为空表示本登录地址
	BYTE byRes2[256];
}SV_NET_DEV_PLAY_BY_NAME_PARA, *LPSV_NET_DEV_PLAY_BY_NAME_PARA;



typedef struct {
	BYTE ret[40];
}STREAM_FILE_HEADER;

typedef struct    tagSV_NET_DEV_PALYFILEINFO
{
	UINT32 filelen;
	UINT32 totalSecs;
	UINT32 totalFrames;
	UINT32 streamHeaderLen;
	STREAM_FILE_HEADER streamHeader;
}SV_NET_DEV_PLAYFILEINFO, *LPSV_NET_DEV_PLAYFILEINFO;

typedef struct    tagSV_NET_DEV_FINDFILE
{
	UINT32 recType;
	SV_NET_DEV_TIME startTime;
	SV_NET_DEV_TIME stopTime;
}SV_NET_DEV_FINDFILE, *LPSV_NET_DEV_FINDFILE;

typedef struct    tagSV_NET_DEV_FILEINFO
{
	BYTE fn[32];
	UINT32 fileLen;
	SV_NET_DEV_TIME startTime;
	SV_NET_DEV_TIME stopTime;
}SV_NET_DEV_FILEINFO, *LPSV_NET_DEV_FILEINFO;

#define MAX_FILE_NUM	10
typedef struct    tagSV_NET_DEV_FILELIST
{
	UINT32 fileNum;
	SV_NET_DEV_FILEINFO fileInfo[MAX_FILE_NUM];
}SV_NET_DEV_FILELIST, *LPSV_NET_DEV_FILELIST;



#define MAX_SERVERID_LEN            64 //最大服务器ID的长度
#define MAX_SERVERDOMAIN_LEN        128 //服务器域名最大长度
#define MAX_AUTHENTICATEID_LEN      64 //认证ID最大长度
#define MAX_AUTHENTICATEPASSWD_LEN  32 //认证密码最大长度
#define MAX_SERVERNAME_LEN          64 //最大服务器用户名 
#define MAX_COMPRESSIONID_LEN       64 //编码ID的最大长度
#define MAX_SIPSERVER_ADDRESS_LEN   128 //SIP服务器地址支持域名和IP地址

#define STREAM_ID_LEN   32
#define MAX_STREAM_ID_NUM    30        //最大流ID数目



typedef struct tagSV_NET_DEV_GBT28181_ACCESS_CFG
{
	DWORD       dwSize;
	BYTE       byEnable;//28181协议使能 0-关闭 1-开启
	BYTE       byTransProtocol;//传输协议:0-UDP、1-TCP、默认0-UDP
	WORD       wLocalSipPort;//1024-65535(IPC设备端)
	char       szServerID[MAX_SERVERID_LEN];//服务器ID：64字节字符串，仅限数字
	char       szServerDomain[MAX_SIPSERVER_ADDRESS_LEN];// 服务器域
	char       szSipServerAddress[MAX_SIPSERVER_ADDRESS_LEN];// SIP服务器地址支持域名和IP地址
	WORD       wServerSipPort;//服务器SIP端口：1024-65535
	BYTE       byProtocolVersion;//协议版本 0-GB/T28181-2011(仅支持UDP),1-GB/T28181-2015(支持TCP,UDP),2-GB/T28181-2016
	BYTE        byTCPConnectMod; //TCP连接模式，使用TCP传输协议时有效，0-无效，1-主动模式，2-被动模式
	char       szSipUserName[MAX_SERVERNAME_LEN];//SIP用户名称：64字节字符串(第三方SIP终端请求IPC服务时使用)
	char       szSipAuthenticateID[MAX_AUTHENTICATEID_LEN];//SIP用户认证ID：64字节字符串
	char       szSipAuthenticatePasswd[MAX_AUTHENTICATEPASSWD_LEN];//SIP用户认证密码：32字节字符串(IPC设备注册到SIP服务器时使用)
	DWORD      dwRegisterValid;//注册有效期：单位秒，默认3600；
	BYTE       byHeartbeatInterval;//心跳间隔：单位秒，默认10秒；
	BYTE       byMaxHeartbeatTimeOut;//最大心跳超时次数：默认3次；
	BYTE       byStreamType;// 取流类型0～主码流，1～子码流，2～3码流
	BYTE          byDeviceStatus; //设备是否在线状态，0-保留，1-在线，2-离线
	DWORD      dwRegisterInterval;//注册间隔:注册失败后再次注册的时间间隔,范围60-255s，默认60s
	DWORD      dwAutoAllocChannelID; //是否自动分配通道ID,按位表示，0为手动配置，1为自动分配，bit1-自动分配编码通道ID,bit2-自动分配报警输入通道,bit3-自动分配解码通道ID 
	char       szDeviceDomain[MAX_SIPSERVER_ADDRESS_LEN];// 设备域
	BYTE       byRes4[116];
}SV_NET_DEV_GBT28181_ACCESS_CFG, *LPSV_NET_DEV_GBT28181_ACCESS_CFG;

//GBT28181协议的设备编码通道配置
typedef struct tagSV_NET_DEV_GBT28181_CHANINFO_CFG
{
	DWORD                   dwSize;
	char    szVideoChannelNumID[MAX_COMPRESSIONID_LEN];//设备视频通道编码ID：64字节字符串，仅限数字
	BYTE             byRes[256];
}SV_NET_DEV_GBT28181_CHANINFO_CFG, *LPSV_NET_DEV_GBT28181_CHANINFO_CFG;

//GBT28181协议的报警输入通道配置 条件结构
typedef struct tagSV_NET_DEV_ALARMIN_INFO
{
	SV_NET_DEV_STREAM_INFO struStreamInfo;
	DWORD  dwAlarmInChannel;//报警输入通道号
	BYTE  byRes[32]; //保留
}SV_NET_DEV_ALARMIN_INFO, *LPSV_NET_DEV_ALARMIN_INFO;

//GBT28181协议的报警输入通道配置 配置结构
typedef struct tagSV_NET_DEV_GBT28181_ALARMINCFG
{
	DWORD                   dwSize;
	char    szAlarmInNumID[MAX_COMPRESSIONID_LEN];//设备报警输入编码ID（每一路报警输入不同）：20字节字符串，仅限数字
	BYTE             byRes[256];
}SV_NET_DEV_GBT28181_ALARMINCFG, *LPSV_NET_DEV_GBT28181_ALARMINCFG;


#define UPNP_PORT_NUM                12      //upnp端口映射端口数目

#if 0
/*
IP地址
*/
typedef struct
{
	char    sIpV4[16];                        /* IPv4地址 */
	BYTE    byIPv6[128];                        /* 保留 */
}SV_NET_DEV_IPADDR, *LPSV_NET_DEV_IPADDR;
#endif

typedef struct tagSV_NET_DEV_NAT_PORT
{
	WORD wEnable;         //该端口是否使能映射
	WORD wExtPort;        //映射的外部端口号
	BYTE byRes[12];       //保留
}SV_NET_DEV_NAT_PORT, *LPSV_NET_DEV_NAT_PORT;

typedef struct  tagSV_NET_DEV_NAT_CFG
{
	DWORD dwSize;          //结构体大小
	WORD wEnableUpnp;     //UPNP功能是否启用
	WORD wEnableNat;        //UPNP端口映射（NAT）功能是否启用(保留，与wEnableUpnp保持一致)
	SV_NET_DEV_IPADDR  struIpAddr;      //NAT路由器LAN IP地址
	SV_NET_DEV_NAT_PORT    struHttpPort;   //web server http端口映射配置
	SV_NET_DEV_NAT_PORT    struCmdPort; //命令端口映射配置(8000)
	SV_NET_DEV_NAT_PORT    struRtspPort;  //rtsp端口映射配置
	BYTE byFriendName[64]; //服务名
	BYTE byNatType; //UPNP端口映射类型，0-手动，1-自动
	BYTE            byRes1[3];    //保留
	SV_NET_DEV_NAT_PORT    struHttpsPort;     //https端口映射配置
	SV_NET_DEV_NAT_PORT    struSDKOverTLSPort;  //SDKOverTLS端口映射配置
	BYTE            byres[60];    //保留
}SV_NET_DEV_NAT_CFG, *LPSV_NET_DEV_NAT_CFG;

typedef struct
{
	DWORD  dwEnabled;               //该端口是否被使能映射
	WORD   wInternalPort;           //映射前的端口
	WORD   wExternalPort;           //映射后的端口
	DWORD  dwStatus;                 /*端口映射状态
									 0 未生效
									 1 未生效：映射源端口与目的端口需一致
									 2 未生效:  映射端口号已被使用
									 3 生效
									 */
	SV_NET_DEV_IPADDR    struNatExternalIp;       //映射后的外部地址
	SV_NET_DEV_IPADDR    struNatInternalIp;       //NAT路由器LAN IP地址
	BYTE   byRes[16];               //保留
}SV_NET_DEV_UPNP_PORT_STATE, *LPSV_NET_DEV_UPNP_PORT_STATE;


typedef struct
{
	SV_NET_DEV_UPNP_PORT_STATE strUpnpPort[UPNP_PORT_NUM];     //端口映射状态,数组0 web server端口 数组1 管理端口 数组2 rtsp端口
	BYTE   byRes[200];              //保留
}SV_NET_DEV_UPNP_NAT_STATE, *LPSV_NET_DEV_UPNP_NAT_STATE;


#define PELCO_D 0x00
#define PELCO_P 0x01
#define TILT_UP			21	/* 云台以SS的速度上仰 */
#define TILT_DOWN		22	/* 云台以SS的速度下俯 */
#define PAN_LEFT		23	/* 云台以SS的速度左转 */
#define PAN_RIGHT		24	/* 云台以SS的速度右转 */
#define UP_LEFT			25	/* 云台以SS的速度上仰和左转 */
#define UP_RIGHT		26	/* 云台以SS的速度上仰和右转 */
#define DOWN_LEFT		27	/* 云台以SS的速度下俯和左转 */
#define DOWN_RIGHT		28	/* 云台以SS的速度下俯和右转 */
#define AUTO_PAN		29	/* 云台以SS的速度左右自动扫描 */
#define SET_PRESET      30  /* 设置预置点 */
#define CLE_PRESET      31  /* 清除预置点 */
#define RUN_PRESET      32  /* 使能预置点 */
#define RUN_SEQ			40	/* 开始巡航 */
#define SET_SEQ			41	/* 设置巡航路径 */
#define CLE_SEQ			42	/* 清除单条巡航路径 */
#define CLE_ALL_SEQ		43	/* 清除全部巡航路径 */
#define STOP_SEQ		44	/* 停止巡航 */
//ptz位置信息
typedef struct
{
	WORD wAction;//获取时该字段无效
	WORD wPanPos;//水平参数
	WORD wTiltPos;//垂直参数
	WORD wZoomPos;//变倍参数
}SV_NET_DEV_PTZPOS, *LPSV_NET_DEV_PTZPOS;

typedef struct    tagSV_NET_DEV_PTZ_POSITION
{
	// 是否启用场景，在设置场景行为规则的时候该字段无效，在设置球机本地配置场景位置信息时作为使能位
	BYTE byEnable;
	BYTE byRes1[3];  //保留
	BYTE byPtzPositionName[NAME_LEN]; //场景位置名称
	SV_NET_DEV_PTZPOS struPtzPos; //ptz 坐标
	BYTE byRes2[40];
}SV_NET_DEV_PTZ_POSITION, *LPSV_NET_DEV_PTZ_POSITION;

//巡航点配置(私有IP快球专用)
typedef struct
{
	BYTE    PresetNum;    //预置点
	BYTE    Dwell;        //停留时间
	BYTE    Speed;        //速度
	BYTE    Reserve;    //保留
}SV_NET_DEV_CRUISE_POINT, *LPSV_NET_DEV_CRUISE_POINT;

typedef struct
{
	SV_NET_DEV_CRUISE_POINT struCruisePoint[32];            //最大支持32个巡航点
}SV_NET_DEV_CRUISE_RET, *LPSV_NET_DEV_CRUISE_RET;

//火点检测配置
typedef struct tagSV_NET_DEV_SMOKEDETECTION_CFG
{
	BYTE    byEnable;//使能
	BYTE    bySensitivity; //检测灵敏度: 1~100默认50
	BYTE    byPatrolSensitivity; //巡航检测灵敏度: 1~100默认50
	BYTE    byDoubleCheckSensitivity; //二次过滤灵敏度: 1~100默认50
	WORD    wSensThrd; //检测灵敏度(灰阶阈值)
	BYTE    byRes[54];
}SV_NET_DEV_TFD_CFG, *LPSV_NET_DEV_TFD_CFG;

typedef struct
{
	float f32X;
	float f32Y;
} POINT_FLOAT_S;
typedef struct
{
	POINT_FLOAT_S astPoints[10];  // 报警规则区域多边形顶点数坐标
	UINT8 u8AlarmType;	// 报警类型，如：跨线报警、移动侦测报警、进入区域报警、离开区域报警
	UINT8 u8PointNum;			// 当前绘制的报警规则区域多边形顶点数，取值范围：[2, 10]
	UINT8 u8Time;				// 报警时间，单位：秒，取值范围：[1, 10]，默认值为5
	UINT8 u8Rate;
}SV_NET_DEVTHMLVIBE_RULES;
//智能(VIBE)检测配置
typedef struct tagSV_NET_DEV_VIBEDETECTION_CFG
{
	BYTE    byEnable;//使能
	BYTE    bySensitivity; //检测灵敏度: 1~100默认50
	BYTE    bySensitivity1; //检测灵敏度1: 1~100默认50
	BYTE    u8RuleNum;
	SV_NET_DEVTHMLVIBE_RULES byviberules[8];
	WORD    byDoubleCheckSensitivity; //二次过滤灵敏度: 1~100默认50
	WORD    wSensThrd; //检测灵敏度(灰阶阈值)
	BYTE    byRes[40];
}SV_NET_DEV_VIBE_CFG, *LPSV_NET_DEV_VIBE_CFG;


typedef struct _SC_RECT_U16
{
	UINT16 u16X;		// 矩形相对于坐标原点最近点的 x 坐标
	UINT16 u16Y;		// 矩形相对于坐标原点最近点的 y 坐标
	UINT16 u16Width;	// 矩形的宽
	UINT16 u16Height;	// 矩形的高 
} SC_RECT_U16;

typedef struct _NETTHMLLOD_LIST
{
	UINT32  		preid;							// 预置点
	UINT8			byEnable;						// 使能
	UINT8			byRes0; 							// 报警参数,默认值为50,取值范围[1, 100]
	UINT8			byRes1;							// 报警参考,默认值为50,取值范围[1, 100]
	UINT8			byRes;							// 预留
	SC_RECT_U16 	byRect;							// 检测ROI区域
	UINT16			distThr;						// 检测阈值, 默认值为50,取值范围[1, 100]
	UINT16			areaThr;						// 检测面积, 默认值为50,取值范围[1, 100]
}SV_NET_DEV_THMLLOD_PARAM, *LPSV_NET_DEV_THMLLOD_PARAM;

//报警设备信息
#define MAX_ALARMHOST_ALARMIN_NUM	        64//网络报警主机最大报警输入口数
#define MAX_ALARMHOST_ALARMOUT_NUM	        64//网络报警主机最大报警输出口数

#define EVENT_ALARMIN_TYPE    0x00100000
#define THML_MT_ALARMIN_TYPE 0x00100001
#define THML_TFD_ALARMIN_TYPE 0x00100002
#define THML_MASK_ALARMIN_TYPE 0x00100003
#define THML_MODT_ALARMIN_TYPE 0x00100004
#define THML_CROSS_LINE_ALARMIN_TYPE 0x00110004
#define THML_ENTER_REGION_ALARMIN_TYPE 0x00120004 
#define THML_EXIT_REGION_ALARMIN_TYPE 0x00130004

/*异常处理方式*/
#define NOACTION			0x0					/*无响应*/
#define WARNONMONITOR		0x1					/*监视器上警告*/
#define WARNONAUDIOOUT		0x2					/*声音警告*/
#define UPTOCENTER			0x4					/*上传中心*/
#define TRIGGERALARMOUT		0x8					/*触发报警输出*/
#define TRIGGERRECORD		0x10				/*触发录像*/
#define CATCHPICTURE		0x20				/*抓图*/
#define SENDEMAIL			0x10				/*Send Email*/	

typedef struct
{
	BYTE byUserIDValid;                 /* userid是否有效 0-无效，1-有效 */
	BYTE bySerialValid;                 /* 序列号是否有效 0-无效，1-有效 */
	BYTE byVersionValid;                /* 版本号是否有效 0-无效，1-有效 */
	BYTE byDeviceNameValid;             /* 设备名字是否有效 0-无效，1-有效 */
	BYTE byMacAddrValid;                /* MAC地址是否有效 0-无效，1-有效 */
	BYTE byLinkPortValid;               /* login端口是否有效 0-无效，1-有效 */
	BYTE byDeviceIPValid;               /* 设备IP是否有效 0-无效，1-有效 */
	BYTE bySocketIPValid;               /* socket ip是否有效 0-无效，1-有效 */
	LONG lUserID;                       /* NET_DVR_Login()返回值, 布防时有效 */
	BYTE sSerialNumber[SERIALNO_LEN];	/* 序列号 */
	DWORD dwDeviceVersion;			    /* 版本信息 高16位表示主版本，低16位表示次版本*/
	char sDeviceName[NAME_LEN];		    /* 设备名字 */
	BYTE byMacAddr[MACADDR_LEN];		/* MAC地址 */
	WORD wLinkPort;                     /* link port */
	char sDeviceIP[128];    			/* IP地址 */
	char sSocketIP[128];    			/* 报警主动上传时的socket IP地址 */
	BYTE byIpProtocol;                  /* Ip协议 0-IPV4, 1-IPV6 */
	BYTE byRes2[11];					/* 取出预留的byRes2[0]作一个标志 */
}SV_NET_DEV_ALARMER, *LPSV_NET_DEV_ALARMER;

typedef struct tagSV_NET_DEV_SETUPALARM_PARAM
{
	DWORD dwSize;
	BYTE  byLevel; //布防优先级，0-一等级（高），1-二等级（中），2-三等级（低）
	BYTE  byAlarmInfoType; //上传报警信息类型（抓拍机支持），0-老报警信息（NET_DVR_PLATE_RESULT），1-新报警信息(NET_ITS_PLATE_RESULT)
	BYTE  byRetAlarmTypeV40; //0--返回NET_DVR_ALARMINFO_V30或NET_DVR_ALARMINFO, 1--设备支持NET_DVR_ALARMINFO_V40则返回NET_DVR_ALARMINFO_V40，不支持则返回NET_DVR_ALARMINFO_V30或NET_DVR_ALARMINFO
	BYTE  byRetDevInfoVersion; //CVR上传报警信息回调结构体版本号 0-COMM_ALARM_DEVICE， 1-COMM_ALARM_DEVICE_V40
	BYTE  byRetVQDAlarmType; //VQD报警上传类型，0-上传报报警NET_DVR_VQD_DIAGNOSE_INFO，1-上传报警NET_DVR_VQD_ALARM
							 //1-表示人脸侦测报警扩展(INTER_FACE_DETECTION),0-表示原先支持结构(INTER_FACESNAP_RESULT)
	BYTE  byFaceAlarmDetection;
	//Bit0- 表示二级布防是否上传图片: 0-上传，1-不上传
	//Bit1- 表示开启数据上传确认机制；0-不开启，1-开启
	//Bit6- 表示雷达检测报警(eventType:radarDetection)是否开启实时上传；0-不开启，1-开启（用于web插件实时显示雷达目标轨迹）
	BYTE  bySupport;
	//断网续传类型 
	//bit0-车牌检测（IPC） （0-不续传，1-续传）
	//bit1-客流统计（IPC）  （0-不续传，1-续传）
	//bit2-热度图统计（IPC） （0-不续传，1-续传）
	//bit3-人脸抓拍（IPC） （0-不续传，1-续传）
	//bit4-人脸对比（IPC） （0-不续传，1-续传）
	BYTE  byBrokenNetHttp;
	WORD  wTaskNo;    //任务处理号 和 (上传数据NET_DVR_VEHICLE_RECOG_RESULT中的字段dwTaskNo对应 同时 下发任务结构 NET_DVR_VEHICLE_RECOG_COND中的字段dwTaskNo对应)
	BYTE  byDeployType;    //布防类型：0-客户端布防，1-实时布防
	BYTE  bySubScription;	//订阅，按位表示，未开启订阅不上报  //占位
							//Bit7-移动侦测人车分类是否传图；0-不传图(V30上报)，1-传图(V40上报)
	BYTE  byRes1[2];
	BYTE  byAlarmTypeURL;//bit0-表示人脸抓拍报警上传（INTER_FACESNAP_RESULT）；0-表示二进制传输，1-表示URL传输（设备支持的情况下，设备支持能力根据具体报警能力集判断,同时设备需要支持URL的相关服务，当前是”云存储“）
						 //bit1-表示EVENT_JSON中图片数据长传类型；0-表示二进制传输，1-表示URL传输（设备支持的情况下，设备支持能力根据具体报警能力集判断）
						 //bit2 - 人脸比对(报警类型为COMM_SNAP_MATCH_ALARM)中图片数据上传类型：0 - 二进制传输，1 - URL传输
						 //bit3 - 行为分析(报警类型为COMM_ALARM_RULE)中图片数据上传类型：0 - 二进制传输，1 - URL传输，本字段设备是否支持，对应软硬件能力集中<isSupportBehaviorUploadByCloudStorageURL>节点是否返回且为true
	BYTE  byCustomCtrl;//Bit0- 表示支持副驾驶人脸子图上传: 0-不上传,1-上传
}SV_NET_DEV_SETUPALARM_PARAM, *LPSV_NET_DEV_SETUPALARM_PARAM;

typedef struct tagNET_DVR_ALARMIN_PARAM
{
	DWORD   dwSize;
	BYTE    byName[NAME_LEN];
	WORD    wDetectorType; // DETECTOR_TYPE
	BYTE    byType;     //防区类型，0:即时防区,1-24小时防区,2-延时防区 ,3-内部防区，4-钥匙防区 5-火警防区 6-周界防区 7-24小时无声防区 0xff-无
	BYTE    byUploadAlarmRecoveryReport;    //是否上传防区报警恢复报告，0-不上传，1-上传        
	DWORD	dwParam;    // 防区参数  延时防区延时多长时间, 动环报警主机和自助行报警主机的延时时间通过这个参数来设置 , 具体用哪种设置方式通过能力集中的bySupportAlarmInDelay字段来区别
	SV_NET_DEV_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT];/*布防时间时间段*/
	BYTE    byAssociateAlarmOut[MAX_ALARMHOST_ALARMOUT_NUM];  // 报警输入关联报警输出
	BYTE	byAssociateSirenOut[8];		//  异常处理方式 + 被触发的报警输出U32 + U32
	BYTE	bySensitivityParam;//防区灵敏度参数, 0-10ms、1-250ms、2-500ms、3-750ms
	BYTE	byEnableAlarmIn;
	BYTE	byRes[2];
	BYTE  recordChanTriggered[16];   /* 被触发的录像(此处限制最大通道数为128) */
	BYTE  bEnablePreset[MAX_CHANNUM];   /* 是否调用预置点 */
	BYTE  presetNo[MAX_CHANNUM];    /* 调用的云台预置点序号,一个报警输入可以调用
									 多个通道的云台预置点, 0xff表示不调用预置点。
									 */
	BYTE       bEnablePtzCruise[MAX_CHANNUM];  /* 是否调用巡航 */
	BYTE  ptzCruise[MAX_CHANNUM];    /* 巡航 */
	BYTE       bEnablePtzTrack[MAX_CHANNUM];  /* 是否调用轨迹 */
	BYTE  trackNo[MAX_CHANNUM];    /* 云台的轨迹序号(同预置点) */
}SV_NET_DEV_ALARMIN_PARAM, *LPSV_NET_DEV_ALARMIN_PARAM;

typedef struct tagSV_NET_DEV_ALARMOUT_PARAM
{
	DWORD   dwSize;             // 结构体大小
	BYTE    byName[NAME_LEN];   // 名称
	WORD    wDelay;             // 输出延迟 单位s, 范围：0~3599s 0 表示一直有输出
	WORD	wTriggerIndex;		//触发器号，该参数只能获取
	BYTE    byAssociateAlarmIn[MAX_ALARMHOST_ALARMIN_NUM];   //表示警号跟随的报警输入通道 （多个报警输入同时触发一个警号输出）数组下标0表示报警输入1，依次类推 0-不跟随 1-跟随
	SV_NET_DEV_SCHEDTIME struAlarmTime[MAX_DAYS][MAX_TIMESEGMENT];/*布防时间时间段*/
	BYTE	byModuleType;	//外接触发器类型，1-本地触发器， 2-4路触发器，3-8路触发器 4-单防区触发器，5-32路触发器
	BYTE	byModuleStatus;	//外接触发器状态 1-在线 2-离线
	WORD	wModuleAddress;	//外接触发器地址，扩展模块从0~255，0xFFFF表示无效
	BYTE	byModuleChan;	//外接触发器通道号，从1开始，最大值根据模块类型来决定，0xFF表示无效
	BYTE   	byRes2[55]; 			//保留字节
}SV_NET_DEV_ALARMOUT_PARAM, *LPSV_NET_DEV_ALARMOUT_PARAM;

typedef struct MTALARM_INFO
{
	INT16 maxTemp;	// 这个值放大了10x,使用时需要缩小10倍
	INT16 minTemp;
	INT16 avgTemp;
	INT16 res;
	UINT32 outPoint[2];	//0标识高温点 1标识低温点，高16bit为x，低16bit为y
}SV_NET_DEV_MTALARMINFO;

typedef struct NETRET_MTALARMINFO
{
	UINT32 alarmType;
	UINT32 alarmInNumber;			//用于判断每个规则是否达到预警或报警，按位，第0位表示第0个规则，0表示没达到，1表示达到
	UINT32 decAlarmNum;				//用于判断每个规则是预警还是报警，按位，第0位表示第0个规则，0为预警，1为报警
	char   res[4];                  //预留
	UINT32	channelNo;				/* 按位,第0位对应第0个通道，alarmType为2或3,6时需要设置 */
	UINT32	diskNo;					/* 按位,第0位对应第0个硬盘,dwAlarmType为4,5时需要设置 */
	SV_NET_DEV_MTALARMINFO mtAlarmInfo[25];
}SV_NET_DEV_MTALARMINFO_PARAM;

//SDK接口函数
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SDK_Init();
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SDK_Release();
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_VERSION(char* sdkVerInfo);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetLastError();


//异步登录回调状态宏定义
#define ASYN_LOGIN_SUCC			1		//异步登录成功
#define ASYN_LOGIN_FAILED		0		//异步登录失败

#define EXCEPTION_DEVOFFLINE 0x8006
#define EXCEPTION_AUDIOEXCHANGE 0x8005	//语音对讲时网络异常
#define EXCEPTION_ALARM 0x8004			//报警上传时网络异常
#define EXCEPTION_PREVIEW 0x8003		//网络预览时异常
#define EXCEPTION_SERIAL 0x8002			//透明通道传输时异常
#define EXCEPTION_RECONNECT 0x8001		//预览时重连

void CALLBACK DrawThermotryRule(LONG lRealHandle, HDC hDC, DWORD nUser);

typedef void (CALLBACK *REALDATACALLBACK)(LONG lPlayHandle, DWORD dwDataType, BYTE *pBuffer, DWORD unWidth, DWORD unHeight, void* pUser);

typedef void (CALLBACK *DRAWCALLBACK)(LONG lRealHandle, HDC hDc, DWORD dwUser);

//用户登录
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Login(LPSV_NET_DEV_USER_LOGIN_INFO pLoginInfo, LPSV_NET_DEV_DEVICEINFO lpDeviceInfo);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Logout(LONG lUserID);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_DBGCMD(LPSV_NET_DEV_USER_LOGIN_INFO pLoginInfo, int val);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CHKKEEPALIVE(LONG lUserID);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GETSTDINFO(long lUserID, LPSV_NET_DEV_STDINFO ptrNetDevStdInfo);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GETSTDINFO_EX(long lUserID, LPSV_NET_DEV_STDINFO_EX ptrNetDevStdInfoEx);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SETSTDINFO_EX(long lUserID, LPSV_NET_DEV_STDINFO_EX ptrNetDevStdInfoEx);

typedef void (CALLBACK *EXCEPTIONCALLBACK)(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser);
#ifdef _WIN32
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetExceptionCallBack(UINT nMessage, HWND hWnd, EXCEPTIONCALLBACK fExceptionCallBack, void *pUser);
#elif defined(__linux__) || defined(__APPLE__)
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetExceptionCallBack(UINT reserved1, void* reserved2, EXCEPTIONCALLBACK fExceptionCallBack, void *pUser);
#endif


//长连接透传
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_START_SERIALTRANS(LONG lUserID, LONG lTransType);

//开始播放
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlFPSConfig(LONG lUserID, LONG *fpsVal);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlFPSConfig(LONG lUserID, LONG fpsVal);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_RealPlay(LONG lUserID, LPSV_NET_DEV_PREVIEWINFO lpPreviewInfo, REALDATACALLBACK fRealDataCallBack_V30, void* pUser);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopRealPlayEx(LONG lRealHandle);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopRealPlay(LONG lRealHandle, LPSV_NET_DEV_PREVIEWINFO lpPreviewInfo);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_RigisterDrawFun(LONG lRealHandle, DRAWCALLBACK fDrawCallBack, DWORD dwUser);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SaveRealData(LONG lRealHandle, char *sFileName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopSaveRealData(LONG lRealHandle);

SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_GetPlayPort(LONG *nPort);
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_SetPlayMode(LONG lRealHandle, LONG nPort, BOOL bNormal);
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_PlayStreamData(LONG lRealHandle, LONG nPort, BYTE *pBuffer, unsigned int unWidth, unsigned int unHeight);
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_StopPlay(LONG lRealHandle, LONG nPort);
#if 0
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_InitISP(LONG lRealHandle);
#endif
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_StartISP(LONG lRealHandle, DWORD dwType, DWORD psdcType, BYTE *pBuffer, DWORD unWidth, DWORD unHeight);
SV_NET_SDK_API LONG __stdcall SV_PLAYCTRL_StopISP(LONG lRealHandle);


//回放
/*************************************************
回放时播放控制命令宏定义
**************************************************/
#define SV_NET_DEV_PLAYSTART        1//开始播放
#define SV_NET_DEV_PLAYSTOP        2//停止播放
#define SV_NET_DEV_PLAYPAUSE        3//暂停播放
#define SV_NET_DEV_PLAYRESTART        4//恢复播放

typedef struct    tagSV_NET_DEV_RecordCfg
{
	UINT32 bEnableRecord;
	WORD bAllDayRecord[MAX_DAYS];
	BYTE allDayRecType[MAX_DAYS];
	SV_NET_DEV_SCHEDTIME recTimeSeg[MAX_DAYS][MAX_TIMESEGMENT];
	BYTE segRecType[MAX_DAYS][MAX_TIMESEGMENT];
	UINT32 recordDelay;
	UINT32 preRecordTime;
}SV_NET_DEV_RecordCfg, *LPSV_NET_DEV_RecordCfg;

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FormatHD(LONG lUserID, LONG lHdNo);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ManualRecord(LONG lUserID, LONG lChannel, LONG lAction);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FindFile(LONG lUserID, LONG lChannel, LPSV_NET_DEV_FINDFILE lpFindFile);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FindNextGroupFile(LONG lFindHandle, LPSV_NET_DEV_FILELIST lpFileList);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FindClose(LONG lFindHandle);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlaybackByName(LONG lUserID, char *sFileName, HWND hWnd);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlaybackByTime(LONG lUserID, LONG lChannel, LPSV_NET_DEV_TIME lpStartTime, LPSV_NET_DEV_TIME lpStopTime, HWND hWnd);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopPlayback(LONG lPlayHandle);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetFileByName(LONG lUserID, char *sFileName, char *sSaveFileName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetFileByTime(LONG lUserID, LONG lChannel, LPSV_NET_DEV_TIME lpStartTime, LPSV_NET_DEV_TIME lpStopTime, char *sSaveFileName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopGetFile(LONG lFileHandle);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlaybackCtrl(LONG lPlayHandle, LONG lCtrlCode, LPVOID lpInBuffer, DWORD dwInLen, LPVOID lpOutBuffer, DWORD *lpOutLen);
SV_NET_SDK_API BOOL __stdcall SV_NET_DEV_GetRecordCfg(LONG lUserID, LONG lChannel, LPSV_NET_DEV_RecordCfg lpRecordCfg);
SV_NET_SDK_API BOOL __stdcall SV_NET_DEV_SetRecordCfg(LONG lUserID, LONG lChannel, LPSV_NET_DEV_RecordCfg lpRecordCfg);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlayFile(LONG lUserID, LPSV_NET_DEV_PLAY_BY_NAME_PARA pParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopPlayFile(LONG lPlayHandle);
/*
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FormatHD(LONG lUserID, LONG lHdNo);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ManualRecord(LONG lUserID, LONG lChannel, LONG lAction);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_FindFile(LONG lUserID, LONG lChannel, LPSV_NET_DEV_FINDFILE lpFindFile, LPSV_NET_DEV_FILELIST lpFileList);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlaybackByName(LONG lUserID, char *fileName, LPSV_NET_DEV_PLAYFILEINFO lpPlayFileInfo);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlayBackByTime(LONG lUserID, LONG lChannel, LPSV_NET_DEV_TIME lpStartTime, LPSV_NET_DEV_TIME lpStopTime, HWND hWnd);


SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PlaybackCtrl(LONG lUserID, int ctrlCode, HWND hWnd);
*/
//(LONG lPlayHandle, DWORD dwControlCode, LPVOID lpInBuffer = NULL, DWORD dwInLen = 0, LPVOID lpOutBuffer = NULL, DWORD *lpOutLen = NULL);


//恢复默认值
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_RestoreConfig(LONG lUserID);
//恢复出厂设置
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_RestoreFACTORYConfig(LONG lUserID);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PswdConfig(LONG lUserID, unsigned char *usrname, char *pswd);

//保存参数
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SaveConfig(LONG lUserID);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_WifiConfig(LONG lUserID, char *pBufSSID, char *pBufPSWD);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetWifiConfig(LONG lUserID, char *pBufSSID, char *pBufPSWD);

//图像参数
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlImgParamConfig(LONG lUserID, LPSV_NET_DEV_THMLPIC_PARAM lpThmlImgParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlImgParamConfig(LONG lUserID, LPSV_NET_DEV_THMLPIC_PARAM lpThmlImgParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlImgParamConfigV13(LONG lUserID, LPSV_NET_DEV_THMLPIC_PARAM_V13 lpThmlImgParamV13);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlImgParamConfigV13(LONG lUserID, LPSV_NET_DEV_THMLPIC_PARAM_V13 lpThmlImgParamV13);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetNetParam(LONG lUserID, LPSV_NET_DEV_NETWORKPARAM netWorkParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetNetParam(LONG lUserID, LPSV_NET_DEV_NETWORKPARAM netWorkParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPICParamConfig(LONG lUserID, LPSV_NET_DEV_PICCFG_EX lpPicCfgParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetPICParamConfig(LONG lUserID, LPSV_NET_DEV_PICCFG_EX lpPicCfgParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetSingleParam(LONG lUserID, UINT *value, WORD cmd);

//视频参数
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetVideoCompParamConfig(LONG lUserID, LPSV_NET_DEV_COMPRESSION_INFO_V30 lpnetVideoCompParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetVideoCompParamConfig(LONG lUserID, LPSV_NET_DEV_COMPRESSION_INFO_V30 lpnetVideoCompParam);


SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetFilePath(LONG lUserID, char *sFileName);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CaptureJPEGPicture(LONG lUserID, LONG lChannel, LPSV_NET_DEV_JPEGPARA lpJpegPara, char *sJpegPicBuffer);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CaptureJPEGPicture_V13(LONG lUserID, LONG lChannel, LPSV_NET_DEV_JPEGPARA lpJpegPara, char *sJpegPicBuffer, int *nJpegSize);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CaptureBMPPicture(LONG lUserID, LONG lChannel, LPSV_NET_DEV_BMPPARA lpBmpPara, char *sBmpPicBuffer);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CaptureTEMPData(LONG lUserID, LONG lChannel, SHORT *dTempBuffer, int *nTempSize);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StartDVRRecord(LONG lUserID, LONG lChannel, LONG lRecordType);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopDVRRecord(LONG lUserID, LONG lChannel);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StartManualRecord(LONG lUserID, LPSV_NET_DEV_MANUAL_RECORD_PARA lpManualRecPara);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopManualRecord(LONG lUserID, LPSV_NET_DEV_STREAM_INFO pIDInfo);

//漏油检测
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlLODParamConfig(LONG lUserID, LPSV_NET_DEV_THMLLOD_PARAM lpThmlLODParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlLODParamConfig(LONG lUserID, LPSV_NET_DEV_THMLLOD_PARAM lpThmlLODParam);


//测温
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtRuleListConfig(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM lpThmlMtRuleParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtRuleListConfig(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM lpThmlMtRuleParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_DeltThmlMtRuleListConfig(LONG lUserID, int rgnID);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V12(LONG lUserID, LONG rgnID, LPSV_NET_DEV_THMLMTRULE_PARAM_V12 lpThmlMtRuleParamV12);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V121(LONG lUserID, LONG rgnID, void *pBuffer);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V13(void *pBuffer, LONG rgnID, LPSV_NET_DEV_THMLMTRULE_PARAM_V12 lpThmlMtRuleParamV12);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V131(void *pInBuffer, LONG rgnID, void *pOutBuffer);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V12Ex(LONG lUserID, LONG rgnID, LPSV_NET_DEV_THMLMTRULE_PARAM_V12Ex lpThmlMtRuleParamV12Ex);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V121Ex(LONG lUserID, LONG rgnID, void *pOutBuffer);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V13Ex(void *pBuffer, LONG rgnID, LPSV_NET_DEV_THMLMTRULE_PARAM_V12Ex lpThmlMtRuleParamV12Ex);
SV_NET_SDK_API LONG __stdcall  SV_NET_DEV_GetThmlMtRuleListConfig_V131Ex(void *pInBuffer, LONG rgnID, void *pOutBuffer);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtAndAlarmRuleListConfig(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM lpThmlMtRuleParam, LPSV_NET_DEV_TMLMTALARMRULE_PARAM lpThmlMtAlarmRuleParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtAndAlarmRuleListConfig_V13(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM_V13 lpThmlMtRuleParamV13);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtAndAlarmRuleListConfigEx(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM lpThmlMtRuleParamArr, LPSV_NET_DEV_TMLMTALARMRULE_PARAM lpThmlMtAlarmRuleParamArr);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtAndAlarmRuleListConfigEx_V13(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM_V13 lpThmlMtRuleParamV13Arr);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtRuleParamConfig(LONG lUserID, LPSV_NET_DEV_THMLMTRULE_PARAM lpThmlMtRuleParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtBasicParamConfig(LONG lUserID, LPSV_NET_DEV_THMLMT_BASICPARAM lpThmlMtBasicParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtBasicParamConfig(LONG lUserID, LPSV_NET_DEV_THMLMT_BASICPARAM lpThmlMtBasicParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtCmpOsParam(LONG lUserID, LPSV_NET_DEV_MTRULE_CMPOSPARAM lpThmlMtCmpOsParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlMtModeParamConfig(LONG lUserID, LPSV_NET_DEV_THERMOMETRY_MODE lpThmlMtModeParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtModeParamConfig(LONG lUserID, LPSV_NET_DEV_THERMOMETRY_MODE lpThmlMtModeParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtNormalRegionConfig(LONG lUserID, POINT_S32_S *NormalRegPoints, float *RegTemp);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtNormalRegionConfig_V13(LONG lUserID, unsigned char *pBuffer, POINT_S32_S *NormalRegPoints, float *RegTemp);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtNormalRegionDelete(LONG lUserID, POINT_S32_S *NormalRegPoints);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlMtFSMaxMinVal(LONG lUserID, float *RetTemp);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlManualMtConfig(LONG lUserID, POINT_S32_S point, float *RetTemp);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ThermometryParamCalib(LONG lUserID, LPSV_NET_DEV_MTCALIBPOINT lpThmlMtCalibPointParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_AnalysisThermometryData(LONG lRealHandle, BYTE *pBuffer, signed short *tempBuffer, DWORD unWidth, DWORD unHeight);



//火点检测
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlTFDParamConfig(LONG lUserID, LPSV_NET_DEV_TFD_CFG lpThmlTFDParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlTFDParamConfig(LONG lUserID, LPSV_NET_DEV_TFD_CFG lpThmlTFDParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlVIBEParamConfig(LONG lUserID, LPSV_NET_DEV_VIBE_CFG lpThmlVIBEParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlVIBEParamConfig(LONG lUserID, LPSV_NET_DEV_VIBE_CFG lpThmlVIBEParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_DeltThmlVIBEParamRule(LONG lUserID, int chnl, POINT_S32_S *rectPoint);

//报警
typedef void (CALLBACK *MSGCallBack)(int lCommand, SV_NET_DEV_ALARMER *pAlarmer, char *pAlarmInfo, int dwBufLen, void* pUser);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDVRMessageCallBack(MSGCallBack fMessageCallBack, void* pUser);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetupAlarmChan(LONG lUserID, LPSV_NET_DEV_SETUPALARM_PARAM lpSetupParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CloseAlarmChan(LONG lAlarmHandle);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Config_ListenHost(LONG lUserID, char *sLocalIP, LONG wLocalPort);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Get_ListenHost(LONG lUserID, char *sLocalIP, LONG *wLocalPort);
#if 0
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StartListen(LONG lUserID, LPSV_NET_DEV_SETUPALARM_PARAM lpSetupParam);
#else
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StartListen(char *sLocalIP, WORD wLocalPort, MSGCallBack DataCallback, void* pUserData); 
#endif
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_StopListen(LONG lListenHandle);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetAlarmInCfg(LONG lUserID, int cfgType, LPSV_NET_DEV_ALARMIN_PARAM lpAlarmInParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetAlarmInCfg(LONG lUserID, int cfgType, LPSV_NET_DEV_ALARMIN_PARAM lpAlarmInParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetAlarmOutCfg(LONG lUserID, int cfgType, LPSV_NET_DEV_ALARMOUT_PARAM lpAlarmOutParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetAlarmOutCfg(LONG lUserID, int cfgType, LPSV_NET_DEV_ALARMOUT_PARAM lpAlarmOutParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetAlarmOutStatus(LONG lUserID, LONG *alarmOutStatus);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ManualAlarmOut(LONG lUserID, LONG action);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetAlarmInStatus(LONG lUserID, int *lpAlarmInStatus);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ResetAlarmInStatus(LONG lUserID, LONG lAlarmInChan);


//事件报警功能使能
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetThmlEventAlarmParam(LONG lUserID, int cfgType, LONG eventAlarmEn);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetThmlEventAlarmParam(LONG lUserID, int cfgType, LONG *eventAlarmStatus);

//重启
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_RebootDVR(LONG lUserID);
//关闭DVR
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_ShutDownDVR(LONG lUserID);

//设备升级
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Upgrade(DWORD lUserID, LPSV_NET_DEV_UPGRADE_PARAM lpUpgradeParam);
// 下载文件
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Download(DWORD lUserID, LPSV_NET_DEV_UPGRADE_PARAM lpUpgradeParam);

//手动校时
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDevTimeParamConfig(DWORD lUserID, LPSV_NET_DEV_TIME_V30 lpNetDevTimeParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetDevTimeParamConfig(DWORD lUserID, LPSV_NET_DEV_TIME_V30 lpNetDevTimeParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDevNtpParamConfig(DWORD lUserID, LPSV_NET_DEV_NTP lpNetDevNtpParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetDevNtpParamConfig(DWORD lUserID, LPSV_NET_DEV_NTP lpNetDevNtpParam);
/*
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_AdapterUpgrade(LONG lUserID, char *sFileName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_Upgrade(LONG lUserID, char *sFileName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_VcalibUpgrade(LONG lUserID, LONG lChannel, char const *sFileName);
SV_NET_SDK_API int __stdcall SV_NET_DEV_GetUpgradeState(LONG lUpgradeHandle);
SV_NET_SDK_API int __stdcall SV_NET_DEV_GetUpgradeProgress(LONG lUpgradeHandle);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CloseUpgradeHandle(LONG lUpgradeHandle);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetNetworkEnvironment(DWORD dwEnvironmentLevel);
*/

//机芯控制指令
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CTRL_CORE_FFC(LONG lUserID);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CTRL_CORE_SHUTR(LONG lUserID);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_CTRL_CORE_GEN(LONG lUserID, unsigned int unCmd, unsigned int unVal);


//GBT28181
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetGBT28181AcessConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_ACCESS_CFG lpGbtAcessParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetGBT28181AcessConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_ACCESS_CFG lpGbtAcessParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetGBT28181ChnlParamConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_CHANINFO_CFG lpGbtChnlParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetGBT28181ChnlParamConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_CHANINFO_CFG lpGbtChnlParam);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetGBT28181AlarmInConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_ALARMINCFG lpGbtAlarmInParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetGBT28181AlarmInConfig(LONG lUserID, LPSV_NET_DEV_GBT28181_ALARMINCFG lpGbtAlarmInParam);

//NAT透穿
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetUpnpNatState(LONG lUserID, LPSV_NET_DEV_UPNP_NAT_STATE lpState);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetUpnpNatConfig(LONG lUserID, LPSV_NET_DEV_NAT_CFG lpUpnpParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetUpnpNatConfig(LONG lUserID, LPSV_NET_DEV_NAT_CFG lpUpnpParam);



//云台控制相关接口
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControl(LONG lUserID, DWORD dwPTZCommand, DWORD dwStop);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControl_Other(LONG lUserID, LONG lChannel, DWORD dwPTZCommand, DWORD dwStop);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_TransPTZ(LONG lUserID, char *pPTZCodeBuf, DWORD dwBufSize);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZPreset(LONG lUserID, DWORD dwPTZPresetCmd, DWORD dwPresetIndex);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControlWithSpeed(LONG lUserID, DWORD dwPTZCommand, DWORD dwStop, DWORD dwSpeed);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZTrack(LONG lUserID, DWORD dwPTZTrackCmd);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZCruise(LONG lUserID, DWORD dwPTZCruiseCmd, BYTE byCruiseRoute, BYTE byCruisePoint, WORD wInput);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPTZCruise(LONG lUserID, LONG lChannel, LONG lCruiseRoute, LPSV_NET_DEV_CRUISE_RET lpCruiseRet);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPtzPosition(LONG lUserID, LONG lChannel, LONG lPositionID, LPSV_NET_DEV_PTZ_POSITION lpPtzPosition);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetPtzPosition(LONG lUserID, LONG lChannel, LONG lPositionID, LPSV_NET_DEV_PTZ_POSITION lpPtzPosition);

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDrawRuleEn(LONG lUserID, BYTE byRuleEn);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDrawRuleType(LONG lUserID, BYTE byRuleType);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetDrawRuleList(LONG lUserID, PDRAW_RULE_INFO lpRuleList);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetDrawRuleSign(LONG lUserID, LPDWORD lprSign);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDrawRuleList(LONG lUserID, LONG num, BYTE direc, LONG offset);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDrawFlag(LONG lUserID, HWND hwnd);

#if 0
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_TransPTZ_Other(LONG lUserID, LONG lChannel, char *pPTZCodeBuf, DWORD dwBufSize);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZPreset_Other(LONG lUserID, LONG lChannel, DWORD dwPTZPresetCmd, DWORD dwPresetIndex);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_TransPTZ_EX(LONG lRealHandle, char *pPTZCodeBuf, DWORD dwBufSize);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControl_EX(LONG lRealHandle, DWORD dwPTZCommand, DWORD dwStop);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZPreset_EX(LONG lRealHandle, DWORD dwPTZPresetCmd, DWORD dwPresetIndex);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZCruise_Other(LONG lUserID, LONG lChannel, DWORD dwPTZCruiseCmd, BYTE byCruiseRoute, BYTE byCruisePoint, WORD wInput);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZCruise_EX(LONG lRealHandle, DWORD dwPTZCruiseCmd, BYTE byCruiseRoute, BYTE byCruisePoint, WORD wInput);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZTrack_Other(LONG lUserID, LONG lChannel, DWORD dwPTZTrackCmd);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZTrack_EX(LONG lRealHandle, DWORD dwPTZTrackCmd);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControlWithSpeed_Other(LONG lUserID, LONG lChannel, DWORD dwPTZCommand, DWORD dwStop, DWORD dwSpeed);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControlWithSpeed_EX(LONG lRealHandle, DWORD dwPTZCommand, DWORD dwStop, DWORD dwSpeed);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPTZProtocol_Ex(LONG lUserID, LONG lChannel, SV_NET_DEV_PTZCFG *pPtzcfg);
#endif




typedef struct tagSV_NET_DEV_CRUISEPOINT
{
	BYTE preset;
	BYTE speed;
	WORD stayTime;
}SV_NET_DEV_CRUISEPOINT, *LPSV_NET_DEV_CRUISEPOINT;

typedef struct tagSV_NET_DEV_CRUISEPARAM
{
	DWORD route;
	SV_NET_DEV_CRUISEPOINT point[16];
}SV_NET_DEV_CRUISEPARAM, *LPSV_NET_DEV_CRUISEPARAM;

//通道解码器(云台)参数配置
typedef struct
{
	DWORD dwSize;
	DWORD dwBaudRate;       //波特率(bps)，7－2400，8－4800，9－9600，10－19200， 11－38400;
	BYTE byDataBit;         // 数据有几位 2－7位，3－8位;
	BYTE byStopBit;         // 停止位 0－1位，1－2位;
	BYTE byParity;          // 校验 0－无校验，1－奇校验，2－偶校验;
	BYTE byFlowcontrol;     // 0－无，1－软流控,2-硬流控
	WORD wDecoderType;      //解码器类型  
	WORD wDecoderAddress;    /*解码器地址:1 - 255*/
	BYTE bySetPreset[16];        //此版本暂不支持
	BYTE bySetCruise[16];        //此版本暂不支持
	BYTE bySetTrack[16];        //此版本暂不支持
}SV_NET_DEV_DECODERCFG, *LPSV_NET_DEV_DECODERCFG;


#define BOUD2400   	7
#define BOUD4800   	8
#define BOUD9600   	9
#define BOUD19200  	10
#define BOUD38400  	11

#define DATAB7     	2
#define DATAB8     	3

#define STOPB1			0
#define STOPB2			1

#define NOPARITY	0
#define ODDPARITY	1
#define EVENPARITY	2

#define	NOCTRL		0
#define SOFTCTRL	1		/* xon/xoff flow control */
#define HARDCTRL	2		/* RTS/CTS flow control */

typedef struct tagSV_NET_DEV_PTZ_OSDCFG
{
	DWORD  dwSize;
	BYTE   byZoomStatus; //此版本暂不支持
	BYTE   byPtStatus;    //此版本暂不支持
	BYTE   byPresetStatus;//此版本暂不支持
	BYTE   byPositionDisplayFormat;//此版本暂不支持
	BYTE	byEnablePtzOsd;
	BYTE	byOsdAttrib;
	BYTE	res[2];
	WORD	wPosX;
	WORD	wPosY;
	BYTE   byRes[116];
}SV_NET_DEV_PTZ_OSDCFG, *LPSV_NET_DEV_PTZ_OSDCFG;

typedef struct tagSV_NET_DEV_PTZ_INFO_EX
{
	float fPan;//P值 精确到小数点后3位 范围[0,360.000]
	float fTilt; //Z值 精确到小数点后3位 范围[-90.000,90.000]
	float fVisibleZoom;//可见光zoom 精确到小数点后1位 不会超过[0，10000.0]
	DWORD dwVisibleFocus;//可见光focus[0,65535]
	float fThermalZoom;//热成像zoom 精确到小数点后1位 不会超过[0，10000.0]
	DWORD dwThermalFocus;//热成像focus[0,65535]
}SV_NET_DEV_PTZ_INFO_EX, *LPSV_NET_DEV_PTZ_INFO_EX;

typedef struct tagSV_NET_DEV_PRESET_NAME
{
	DWORD dwSize;
	WORD wPresetNum; //预置点编号
	BYTE byRes1[2]; //字节对齐 
	char byName[NAME_LEN];
	WORD wPanPos;//此版本暂不支持
	WORD wTiltPos;//此版本暂不支持
	WORD wZoomPos;//此版本暂不支持
	BYTE byRes2;
	BYTE byPTZPosExEnable; //此版本暂不支持
	SV_NET_DEV_PTZ_INFO_EX struPtzPosEx;//此版本暂不支持
	BYTE byRes[32];
}SV_NET_DEV_PRESET_NAME, *LPSV_NET_DEV_PRESET_NAME;

#define PTZ_OSD_DISABLE				0
#define PTZ_OSD_TRANS_WINK			1
#define PTZ_OSD_TRANS_NO_WINK		2
#define PTZ_OSD_NO_TRANS_WINK		3
#define PTZ_OSD_NO_TRANS_NO_WINK	4

#define SHUTTER_MODE_AUTO		0
#define SHUTTER_MODE_MANUAL		1

SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControl_EX(LONG lUserID, LONG lChannel, DWORD dwPTZCommand, DWORD dwStop);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZControlWithSpeed_EX(LONG lUserID, LONG lChannel, DWORD dwPTZCommand, DWORD dwStop, DWORD dwSpeed);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZPreset_EX(LONG lUserID, LONG lChannel, DWORD dwPTZPresetCmd, DWORD dwPresetIndex);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_PTZCruise_EX(LONG lUserID, LONG lChannel, DWORD dwPTZCruiseCmd, LPSV_NET_DEV_CRUISEPARAM lpPTZCruiseParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPTZCruiseRoute(LONG lUserID, LONG lChannel, LPSV_NET_DEV_CRUISEPARAM lpPTZCruiseParam);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetDecoderParam(LONG lUserID, LONG lChannel, LPSV_NET_DEV_DECODERCFG lpDecoderCfg);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetDecoderParam(LONG lUserID, LONG lChannel, LPSV_NET_DEV_DECODERCFG lpDecoderCfg);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPtzOsdCfg(LONG lUserID, LONG lChannel, LPSV_NET_DEV_PTZ_OSDCFG lpPtzOsdCfg);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetPtzOsdCfg(LONG lUserID, LONG lChannel, LPSV_NET_DEV_PTZ_OSDCFG lpPtzOsdCfg);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetPresetName(LONG lUserID, LONG lChannel, LPSV_NET_DEV_PRESET_NAME lpPresetName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetPresetName(LONG lUserID, LONG lChannel, LPSV_NET_DEV_PRESET_NAME lpPresetName);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_GetFFCMode(LONG lUserID, int *lpMode);
SV_NET_SDK_API LONG __stdcall SV_NET_DEV_SetFFCMode(LONG lUserID, int mode);

#endif //_HC_SV_NET_SDK_H_
