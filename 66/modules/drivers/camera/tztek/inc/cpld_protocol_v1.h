 #pragma once
#pragma pack(1)
typedef struct
{
	unsigned int     dwHead;		//CPLD_MSG_HEAD_V1
	unsigned char    bySeq;
	unsigned char    byAction;
	unsigned short   wType;
	unsigned short   wSize;
}TCpldV1MsgHead;


typedef struct
{
	unsigned char  byCheck;
	unsigned int   dwTail;
}TCpldV1MsgTail;

typedef struct 
{
	TCpldV1MsgHead	 stuHead;
	TCpldV1MsgTail	 stuTail;
}TCpldV1TimeStampReq;

typedef struct
{
	TCpldV1MsgHead	 stuHead;
	unsigned char    szTime[21];
	TCpldV1MsgTail	 stuTail;
}TCpldV1TimeStampRes;

typedef struct 
{
	TCpldV1MsgHead	 stuHead;
	unsigned char    szTime[21];
	TCpldV1MsgTail	 stuTail;
}TCpldV1SetInitTimeReq;


typedef struct
{
	TCpldV1MsgHead	stuHead;
	unsigned char   byErrCode;
	TCpldV1MsgTail	stuTail;
}TCpldV1ErrorMsg;

#pragma pop()
