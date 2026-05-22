#ifndef _JPEG_ENCODE_API_H
#define _JPEG_ENCODE_API_H

#if defined(_WIN32) || defined(_WIN64)
#ifndef CAMERA_CALL
#define CAMERA_CALL __stdcall
#endif
#ifdef JPEGENC_EXPORTS
#define JPEGENC_API extern "C" __declspec(dllexport)
#else
#define JPEGENC_API extern "C" __declspec(dllimport)
#endif
#elif defined(__linux__)
#define JPEGENC_API extern "C"
#ifndef CAMERA_CALL
#define CAMERA_CALL
#endif
#else
#define JPEGENC_API
#ifndef CAMERA_CALL
#define CAMERA_CALL
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void(CAMERA_CALL *JPEGENC_CALLBACK)(unsigned char *data, int datalen,
                                            void *userdata);

typedef enum {
  JPEGENC_MODE_MEMORY = 0,
  JPEGENC_MODE_FD = 1,
  JPEGENC_MODE_TZFD = 2,
} EJPEGENC_MODE;

typedef struct tagJPEGENC_PARA {
  unsigned char pixfmt;
  unsigned char userbuf;
  unsigned char res[2];
  int bufsize;
  int width;
  int height;
  int quality;
  int mode;
  int gpuid;

  char res2[128];
} JPEGENC_PARA;
JPEGENC_API int CAMERA_CALL JPEGENC_Init();
JPEGENC_API int CAMERA_CALL JPEGENC_Release();
JPEGENC_API void *CAMERA_CALL JPEGENC_CreateHandle(JPEGENC_PARA *para);
JPEGENC_API int CAMERA_CALL JPEGENC_DestroyHandle(void *handle);
JPEGENC_API int CAMERA_CALL JPEGENC_SetDataCallBack(void *handle,
                                                    JPEGENC_CALLBACK callback,
                                                    void *userdata);
JPEGENC_API int CAMERA_CALL JPEGENC_InputData(void *handle, unsigned char *data,
                                              int datalen);

#ifdef __cplusplus
}
#endif

#endif
