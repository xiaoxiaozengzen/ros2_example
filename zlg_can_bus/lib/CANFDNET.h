#ifndef CANFDNET_H_
#define CANFDNET_H_

#include "zlgcan/zlgcan.h"

#define SETGETREF_VIRIFY_DEVICE 9
#define SETGETREF_UDP_MULTICAST_LOOPBACK 10
#define SETGETREF_VIRIFY_DEVICE_BY_PASS                                        \
  20 //验证设备的reftype，pData数据类型为指向VerifyDeviceData的指针

// SETREF_ADD_TIMER_SEND_CAN_DIRECT SETREF_ADD_TIMER_SEND_CANFD_DIRECT
// 直接发送定时数据
#define SETREF_ADD_TIMER_SEND_CAN_DIRECT                                       \
  11 // pData Pointer to ZCAN_AUTO_TRANSMIT_OBJ
#define SETREF_ADD_TIMER_SEND_CANFD_DIRECT                                     \
  12 // pData Pointer to ZCANFD_AUTO_TRANSMIT_OBJ

//以下定时发送使用流程，Add
// CAN/CANFD后进入缓存，调用Apply才进行定时发送，Clear会停止之前Apply的数据并清空缓存
#define SETREF_ADD_TIMER_SEND_CAN 21 // pData Pointer to ZCAN_AUTO_TRANSMIT_OBJ
#define SETREF_ADD_TIMER_SEND_CANFD                                            \
  22                               // pData Pointer to ZCANFD_AUTO_TRANSMIT_OBJ
#define SETREF_APPLY_TIMER_SEND 23 // 9 Start Timer Send
#define SETREF_CLEAR_TIMER_SEND 24 // 10 Stop Timer Send & Clear Send List

//设置设备时间戳
#define SETREF_SET_DEV_TIMESTAMP 25 // pData Pointer to UINT64, Timestamp in us
//设置是否发送回显
#define SETREF_SET_TX_ECHO_ENABLE                                              \
  26 //设置设备发送是否开启回显，pData Pointer to UINT,
     // 0:关闭发送回显，1：开启发送回显
#define GETREF_GET_TX_TIMESTAMP 27 //回显数据数量, pData 指向 TxTimeStamp
#define GETREF_GET_BUS_USAGE 28    //获取总线利用率, pData指向 BusUsage

// CANFDNET根据发送帧中的TX_DELAY_SEND_FLAG标志标识当前帧是否使用延时发送
// TX_DELAY_SEND_FLAG
// 0表示正常发送到总线，1表示设备延迟发送模式，支持帧间隔设置，帧间隔单位ms，2字节填入can/canfd帧中的__res0(低8位)和__res1(低8位)字段中
//发送的数据使用ZCAN_Transmit/ZCAN_TransmitFD接口，返回值表示有多少帧已经加入到设备的发送队列中。
#define GETREF_GET_DELAY_SEND_AVALIABLE_COUNT                                  \
  29 //获取设备端延迟发送可用数量 pData Pointer to uint32_t
#define SETREF_CLEAR_DELAY_SEND_QUEUE                                          \
  30 //如果队列发送中有数据因为时间未到未发送，取消设备当前的队列发送

//获取设备定时发送数量，设备调用获取CAN或者CANFD定时发送数量时(REF为31,33)，库和设备进行通讯，获取设备端定时发送列表，然后通过(REF
// 32,34)分别获取CAN,CANFD定时发送帧内容
#define GETREF_GET_DEV_CAN_AUTO_SEND_COUNT                                     \
  31 //获取设备端定时发送CAN帧的数量，pData指向UINT,表示设备端定时发送CAN帧数量
#define GETREF_GET_DEV_CAN_AUTO_SEND_DATA                                      \
  32 //获取设备端定时发送CAN帧的数据，用户根据查询到的CAN帧数量申请内存
     // sizeof(ZCAN_AUTO_TRANSMIT_OBJ) * N，将申请到的内存地址填入pData
#define GETREF_GET_DEV_CANFD_AUTO_SEND_COUNT                                   \
  33 //获取设备端定时发送CANFD帧的数量，pData指向UINT,表示设备端定时发送CANFD帧数量
#define GETREF_GET_DEV_CANFD_AUTO_SEND_DATA                                    \
  34 //获取设备端定时发送CANFD帧的数据，用户根据查询到的CAN帧数量申请内存
     // sizeof(ZCANFD_AUTO_TRANSMIT_OBJ) * N，将申请到的内存地址填入pData

//获取设备状态信息
#define GETREF_GET_DEV_STATE_SYS_INFO                                          \
  35 //包含设备运行时间，序列号，设备名称的json字符串，pData指向ZCAN_RAW_DATA,
     //用户需要申请内存(1k字节即可)，将内存地址和内存大小传填入ZCAN_RAW_DATA对应的字段中
#define GETREF_GET_DEV_STATE_CAN_INFO                                          \
  36 //包含设备通道是否使能，波特率，通道接收数据数量等信息的json字符串，pData指向ZCAN_RAW_DATA,
     //用户需要申请内存(4k字节即可)，将内存地址和内存大小传填入ZCAN_RAW_DATA对应的字段中
#define GETREF_GET_DEV_STATE_RECORDER_INFO                                     \
  37 //包含设备是否记录，记录模式，记录文件大小，sd卡状态等信息的json字符串，pData指向ZCAN_RAW_DATA,
     //用户需要申请内存(1k字节即可)，将内存地址和内存大小传填入ZCAN_RAW_DATA对应的字段中
#define GETREF_GET_DEV_STATE_NET_INFO                                          \
  38 //包含设备网络ip，网关等信息的json字符串，pData指向ZCAN_RAW_DATA,
     //用户需要申请内存(1k字节即可)，将内存地址和内存大小传填入ZCAN_RAW_DATA对应的字段中

//获取设备传送的GPS数据点数量
#define GETREF_GET_DEV_GPS_COUNT                                               \
  39 //获取设备端传输的GPS数据点的数量，pData指向UINT,表示库接收到的GPS数据点数量
#define GETREF_GET_DEV_GPS_DATA                                                \
  40 //获取设备端传输的GPS数据点的数据，pData指向 ZCAN_GPS_DATA
     //用户申请内存空间将地址填入
     // ZCAN_GPS_DATA.pData，可以容纳GPS数据点数量填入ZCAN_GPS_DATA.nFrameCount

//获取LIN 发送Fifo大小
#define GETREF_GET_LIN_TX_FIFO_TOTAL 41     //获取LIN发送缓冲区大小
#define GETREF_GET_LIN_TX_FIFO_AVAILABLE 42 //获取LIN发送缓冲区可用大小

//开启关闭合并接收
#define SETREF_SET_DATA_RECV_MERGE                                             \
  43 //设置合并接收数据，CAN/LIN/GPS以及不同通道的数据合并接收,pData Pointer to
     // UINT, 0:关闭合并接收，1：开启合并接收
#define GETREF_GET_DATA_RECV_MERGE                                             \
  44 //获取合并接收数据状态，pData Pointer to UINT,
     // 0:合并接收关闭，1：合并接收处于开启状态

//查询是否开启发送回显
#define GETREF_GET_TX_ECHO_ENABLE                                              \
  45 //查询设备发送是否开启回显，pData Pointer to UINT,
     // 0:发送回显已关闭，1：发送回显已开启

//设置动态配置（json）
#define SETREF_ADD_DYNAMIC_BY_JSON                                             \
  46 //添加动态配置，pData Pointer to tagZCAN_DYNAMIC_CONFIG_DATA,
     //用户需要申请内存，将内存地址和内存大小传填入ZCAN_RAW_DATA对应的字段中
#define SETREF_APPLY_DYNAMIC_BY_JSON 47 //设置动态配置

//滤波（已弃用）
#define SETREF_ADD_FILTER 50   // pData Pointer to USBCANFDFilterItem
#define SETREF_APPLY_FILTER 51 // pData Pointer to NULL
#define SETREF_CLEAR_FILTER 52 // pData Pointer to NULL

#define SETREF_SET_DYNAMIC_BY_DATA                                             \
  53 //设置动态配置(目前只给滤波用、走的是结构体下发数据)

//动态配置 持久配置 BEGIN
//网卡配置信息（还没支持）
//通用以太网配置
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_MAC                                     \
  "DYNAMIC_CONFIG_NIC_GENNIC_MAC" // 参数类型:string
                                  // 网卡MAC地址，HEX字符串，如：”00:14 : 97 :
                                  // 00 : 01 : 02”
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_DHCP                                    \
  "DYNAMIC_CONFIG_NIC_GENNIC_DHCP" // 参数类型:int
                                   // DHCP是否打开，若打开则自动分配IP，即动态IP；0：关闭，1：打开，默认打开。
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_IP                                      \
  "DYNAMIC_CONFIG_NIC_GENNIC_IP" // 参数类型:string IP地址
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_MASK                                    \
  "DYNAMIC_CONFIG_NIC_GENNIC_MASK" // 参数类型:string 子网掩码
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_GATEWAY                                 \
  "DYNAMIC_CONFIG_NIC_GENNIC_GATEWAY" // 参数类型:string 网关IP
#define ZCAN_DYNAMIC_CONFIG_NIC_GENNIC_ISMASTER                                \
  "DYNAMIC_CONFIG_NIC_GENNIC_ISMASTER" // 车载以太网主从机设置（仅车载以太网有效）0：从机，1：主机
//车载以太网配置
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_MAC                                     \
  "DYNAMIC_CONFIG_NIC_CARNIC_MAC" // 参数类型:string
                                  // 网卡MAC地址，HEX字符串，如：”00:14 : 97 :
                                  // 00 : 01 : 02”
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_DHCP                                    \
  "DYNAMIC_CONFIG_NIC_CARNIC_DHCP" // 参数类型:int
                                   // DHCP是否打开，若打开则自动分配IP，即动态IP；0：关闭，1：打开，默认打开。
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_IP                                      \
  "DYNAMIC_CONFIG_NIC_CARNIC_IP" // 参数类型:string IP地址
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_MASK                                    \
  "DYNAMIC_CONFIG_NIC_CARNIC_MASK" // 参数类型:string 子网掩码
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_GATEWAY                                 \
  "DYNAMIC_CONFIG_NIC_CARNIC_GATEWAY" // 参数类型:string 网关IP
#define ZCAN_DYNAMIC_CONFIG_NIC_CARNIC_ISMASTER                                \
  "DYNAMIC_CONFIG_NIC_CARNIC_ISMASTER" // 车载以太网主从机设置（仅车载以太网有效）0：从机，1：主机
// WIFI网卡信息配置
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_MAC                                    \
  "DYNAMIC_CONFIG_NIC_WIFINIC_MAC" // 参数类型:string
                                   // 网卡MAC地址，HEX字符串，如：”00:14 : 97 :
                                   // 00 : 01 : 02”
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_DHCP                                   \
  "DYNAMIC_CONFIG_NIC_WIFINIC_DHCP" // 参数类型:int
                                    // DHCP是否打开，若打开则自动分配IP，即动态IP；0：关闭，1：打开，默认打开。
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_IP                                     \
  "DYNAMIC_CONFIG_NIC_WIFINIC_IP" // 参数类型:string IP地址
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_MASK                                   \
  "DYNAMIC_CONFIG_NIC_WIFINIC_MASK" // 参数类型:string 子网掩码
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_GATEWAY                                \
  "DYNAMIC_CONFIG_NIC_WIFINIC_GATEWAY" // 参数类型:string 网关IP
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFINIC_ISMASTER                               \
  "DYNAMIC_CONFIG_NIC_WIFINIC_ISMASTER" // 参数类型:int
                                        // 车载以太网主从机设置（仅车载以太网有效）0：从机，1：主机
// WIFI模块配置（仅CANFDDTU-400/600EWGR有WIFI网卡）
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_ENWLAN                              \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_ENWLAN" // 参数类型:int
                                         // 用户一般无需操作;0：禁能 1：使能
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_SSID                                \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_SSID" // 参数类型:string 设备作为 AP
                                       // 热点模式时，为其无线设备的名称，
                                       // SSID广播时可以被搜索到；设备作为
                                       // Station 客户端模式时，为其要连接的无线
                                       // AP
                                       // 或者路由器的名称，名称长度最长为64字符（包括’\0’）
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_WPAPW                               \
  "DYNAMIC_CONFIG_NIC_WIFIMODULEP_WPAPW" // 参数类型:string 设置模块作为 AP
                                         // 时，其他设备需要输入密码；设置模块作为
                                         // Station 时，为连接 AP
                                         // 或者路由器的密码。密码长度最长为64字符（包括’\0’），最少8字符（不包括’\0’）。
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_FREQ                                \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_FREQ" // 参数类型:string 只支持2.4G，不可选
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_CHANNEL                             \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_CHANNEL" // 参数类型:int 工作信道，范围： 1~13
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_WLANMODE                            \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_WLANMODE" // 参数类型:string 0: AP,
                                           // 做热点，可以被其他无线设备连接；1:
                                           // Station,
                                           // 做设备，连接路由器或者热点。
#define ZCAN_DYNAMIC_CONFIG_NIC_WIFIMODULE_WPAMODE                             \
  "DYNAMIC_CONFIG_NIC_WIFIMODULE_WPAMODE" // 参数类型:string
                                          // 加密方式：wpa2_aes、wpa2、open(不加密)
//手机模块配置
#define ZCAN_DYNAMIC_CONFIG_NIC_MOBLIE_ENABLE                                  \
  "DYNAMIC_CONFIG_NIC_MOBLIE_ENABLE" // 参数类型:int
                                     // 使能移动网络，1-启用；0-禁用
//动态配置 持久配置 END

#pragma pack(push, 1)

// verify device
typedef struct tagVerifyDeviceData {
  char inData[16];  //输入数据
  char OutData[16]; //设备返回数据
} VerifyDeviceData;

#if 0
//TX timestamp
typedef struct tagTxTimeStamp
{
    UINT64* pTxTimeStampBuffer;       //allocated by user, size:nBufferTimeStampCount * 8,unit:1us
    UINT    nBufferTimeStampCount;    //buffer timestamp count
    int     nWaitTime;                //Wait Time ms, -1表示等到有数据才返回
}TxTimeStamp;

// Bus usage
typedef struct tagBusUsage
{
    UINT64  nTimeStampBegin;//测量起始时间戳，单位us
    UINT64  nTimeStampEnd;  //测量结束时间戳，单位us
    BYTE    nChnl;          //通道
    BYTE    nReserved;      //保留
    USHORT  nBusUsage;      //总线利用率(%),总线利用率*100展示。取值0~10000，如8050表示80.50%
    UINT    nFrameCount;    //帧数量
}BusUsage;
#endif

typedef struct tagRemoteClient {
  int iIndex;
  UINT port;
  int hClient;
  char szip[32];
} REMOTE_CLIENT;

// GPS
typedef struct tagZCAN_GPS_FRAME {
  union {
    struct {
      UINT btm : 1;         // Bit0 表示时间是否有效
      UINT blatlong : 1;    // Bit1 表示经纬度有效
      UINT bspd : 1;        // Bit2 表示速度是否有效
      UINT bcouseangle : 1; // Bit3 表示航向角是否有效
      UINT baltitude : 1;   // Bit4 表示海拔是否有效
      UINT breserved : 27;  // Bit5-32 保留
    } unionValue;
    UINT rawValue;
  } validflag; //标识数据的有效性
  struct __gps_time {
    USHORT year;
    USHORT mon;
    USHORT day;
    USHORT hour;
    USHORT min;
    USHORT sec;
  } tm;              // 时间
  float latitude;    // 纬度 + north latitude, - south latitude
  float longitude;   // 经度 + east longitude, - west longitude
  float speed;       // km/h
  float courseangle; // 航向角
  float altitude;    // 海拔
  UINT reserved;     // 保留
} ZCAN_GPS_FRAME;

typedef struct tagZCAN_GPS_DATA {
  ZCAN_GPS_FRAME *pData; // 用户申请存放GPS数据的缓冲区地址,需要申请的空间大小为
                         // sizeof(ZCAN_GPS_FRAME) * nFrameCount
  UINT nFrameCount;      // 用户申请存放GPS数据的缓冲区可以存放GPS数据帧的个数
  UINT nRet;             // 实际获取到的GPS数据数量
  int nWaitTime; // 获取gps数据时的等待时间，单位ms, -1表示等到有数据才返回
} ZCAN_GPS_DATA;

typedef struct tagZCAN_RAW_DATA {
  void *pData;   // 用户申请存放数据的缓冲区地址
  UINT nDataLen; // 用户申请存放数据的缓冲区大小
  UINT
      nResultLen; // 输出参数，表示实际数据长度,用户需要检查返回的数据大小，如果nResultLen
                  // > nDataLen
                  // 说明用户申请的空间不足以存放数据，用户需要申请更大的内存空间来容纳数据结果。
  int nWaitTime; // 获取数据时的等待时间，根据不同命令，此参数可能会被忽略
} ZCAN_RAW_DATA;

// zlgcan中canfd_frame.__res0,供ZCANPRO内部使用，用于统一接收数据,为了多通道接收数据连续性
// Flag包含通道信息，是否错误帧，CAN/CANFD/Err信息统一使用canfd_frame结构封装
#define CANFD_FLAG_RES0_FRAME_TYPE_CAN 0
#define CANFD_FLAG_RES0_FRAME_TYPE_CANFD 1
#define CANFD_FLAG_RES0_FRAME_TYPE_ERROR 2
typedef union {
  struct {
    BYTE nChnl : 4;      //通道
    BYTE nType : 3;      //数据类型， 0：CAN;1:CANFD;2:Err
    BYTE nFlagValid : 1; //标识当前Flag字节是否有效，0：无效；1：有效
  } unionValue;
  BYTE nRawValue;
} CANFD_FLAG_RES0;

// LIN
enum {
  MAX_LIN_ID_COUNT = 63,
  MIN_LIN_DLC = 1,
  MAX_LIN_DLC = 8,
  AUTO_LIN_DLC = 255,
};
#define LIN_SEND_SUBSCRIBE 0
#define LIN_SEND_RESPONSE 1
#define LIN_SEND_HEADER 2
#define LIN_SEND_HEADER_AND_DATA 3

// filter
#define CANFDNET_FILTER_COUNT_MAX 16
typedef struct tagCANFDNETFilterItem {
  BYTE nFrameType;   // 帧类型 enumUSBCANFDFrameType
  BYTE nChn;         // 通道
  BYTE nReserved[2]; // 保留
  UINT nStartID;
  UINT nEndID;
} CANFDNETFilterItem;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif
UINT FUNC_CALL ZCAN_GetReference(UINT DeviceType, UINT nDevIndex,
                                 UINT nChnlIndex, UINT nRefType, void *pData);
UINT FUNC_CALL ZCAN_SetReference(UINT DeviceType, UINT nDevIndex,
                                 UINT nChnlIndex, UINT nRefType, void *pData);

#ifdef __cplusplus
}
#endif

#endif // CANFDNET_H_
