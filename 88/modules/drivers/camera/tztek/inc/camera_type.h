#pragma once

typedef char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef signed long long s64;
typedef unsigned long    u64;

typedef struct tagImageInfo
{
        unsigned int dwIndex;
        unsigned int dwChan;
        unsigned int dwTimestampSec;
        unsigned int dwTimestampNsec;
        unsigned int dwDataLength;
        char szSerialNum[32];
        unsigned short dwExposureUsec;
        unsigned short wWidth;
        unsigned short wHeight;
        unsigned short wStatus;
        unsigned char byPixelFormat;
        unsigned char byTimestampType;
        unsigned char res[30];
                                       // unsigned char data[0];
} TImageInfo;

typedef struct tagImageWrap
{
        TImageInfo imgHead;
        unsigned char image[2 * 4096 * 2160];
} TImageWrap;
