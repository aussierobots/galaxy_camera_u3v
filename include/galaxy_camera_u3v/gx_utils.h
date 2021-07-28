// Copyright 2021 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef GX_UTILS_H_
#define GX_UTILS_H_

#include "galaxy_camera_u3v/GxIAPI.h"
#include "galaxy_camera_u3v/DxImageProc.h"

#include <string>

uint8_t bit_extract(uint16_t value, int begin, int end)
{
    uint16_t mask = (1 << (end - begin)) - 1;
    return static_cast<uint8_t>((value >> begin) & mask);
}

struct CameraDeviceInfo {
  std::string vendor_name;
  std::string model_name;
  std::string serial_number;
  std::string device_version;
  std::string device_firmware_version;
  bool color_filter;
};

struct ImageFormat {
  int64_t sensor_width;   // the width of the sensor, unit: pixel
  int64_t sensor_height;  // the height of the sensor, unit: pixel
  int64_t width_max;      // the maximum width of the image, unit: pixel
  int64_t height_max;     // the maximum height of the image, unit: pixel
  int64_t width;          // the width of the ROI, unit: pixel
  int64_t height;         // the height of the ROI, unit: pixel
  int64_t offset_x;       // relative to the x direction offset in the upper left corner of the sensor unit
  int64_t offset_y;       // relative to the y direction offset in the upper left corner of the sensor unit
};

inline GX_STATUS GetImageFormat(GX_DEV_HANDLE gx_dev_handle, ImageFormat * image_format) {
  GX_STATUS status = GX_STATUS_SUCCESS;

  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_SENSOR_WIDTH, &image_format->sensor_width);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_SENSOR_HEIGHT, &image_format->sensor_height);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_WIDTH_MAX, &image_format->width_max);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_HEIGHT_MAX, &image_format->height_max);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_WIDTH, &image_format->width);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_HEIGHT, &image_format->height);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_OFFSET_X, &image_format->offset_x);
  if (status == GX_STATUS_SUCCESS)
    status = GXGetInt(gx_dev_handle, GX_INT_OFFSET_Y, &image_format->offset_y);

  return status;
}


inline GX_STATUS GetCameraInfo(GX_DEV_HANDLE gx_dev_handle, CameraDeviceInfo *camera_info) {
  size_t size = 0;
  GX_STATUS status = GX_STATUS_SUCCESS;

  // get vendor name
  status = GXGetStringLength(gx_dev_handle, GX_STRING_DEVICE_VENDOR_NAME, &size);
  if (status != GX_STATUS_SUCCESS)
    return status;
  char *p_vendor_name = new char[size];
  status = GXGetString(gx_dev_handle, GX_STRING_DEVICE_VENDOR_NAME, p_vendor_name, &size);
  if (status != GX_STATUS_SUCCESS){
    delete[] p_vendor_name;
    return status;
  }
  camera_info->vendor_name = p_vendor_name;
  delete[] p_vendor_name;

  //get model name
  status = GXGetStringLength(gx_dev_handle, GX_STRING_DEVICE_MODEL_NAME, &size);
  if (status != GX_STATUS_SUCCESS)
    return status;
  char *p_model_name = new char[size];
  status = GXGetString(gx_dev_handle, GX_STRING_DEVICE_MODEL_NAME, p_model_name, &size);
  if (status != GX_STATUS_SUCCESS){
    delete[] p_model_name;
    return status;
  }
  camera_info->model_name = p_model_name;
  delete[] p_model_name;

  // get serial number
  status = GXGetStringLength(gx_dev_handle, GX_STRING_DEVICE_SERIAL_NUMBER, &size);
  if (status != GX_STATUS_SUCCESS)
    return status;
  char *p_serial_number = new char[size];
  status = GXGetString(gx_dev_handle, GX_STRING_DEVICE_SERIAL_NUMBER, p_serial_number, &size);
  if (status != GX_STATUS_SUCCESS){
    delete[] p_serial_number;
    return status;
  }
  camera_info->serial_number = p_serial_number;
  delete[] p_serial_number;

  // get device version
  status = GXGetStringLength(gx_dev_handle, GX_STRING_DEVICE_VERSION, &size);
  if (status != GX_STATUS_SUCCESS)
    return status;
  char *p_device_version = new char[size];
  status = GXGetString(gx_dev_handle, GX_STRING_DEVICE_VERSION, p_device_version, &size);
  if (status != GX_STATUS_SUCCESS){
    delete[] p_device_version;
    return status;
  }
  camera_info->device_version = p_device_version;
  delete[] p_device_version;

  // get firmware version
  status = GXGetStringLength(gx_dev_handle, GX_STRING_DEVICE_FIRMWARE_VERSION, &size);
  if (status != GX_STATUS_SUCCESS)
    return status;
  char *p_device_firmware_version = new char[size];
  status = GXGetString(gx_dev_handle, GX_STRING_DEVICE_FIRMWARE_VERSION, p_device_firmware_version, &size);
  if (status != GX_STATUS_SUCCESS){
    delete[] p_device_firmware_version;
    return status;
  }
  camera_info->device_firmware_version = p_device_firmware_version;
  delete[] p_device_firmware_version;


  status = GXIsImplemented(gx_dev_handle, GX_ENUM_PIXEL_COLOR_FILTER, &camera_info->color_filter);
  if (status != GX_STATUS_SUCCESS) {
    return status;
  }

  return GX_STATUS_SUCCESS;
}



//-------------------------------------------------
/**
\brief Convert frame date to suitable pixel format
\param pParam[in]           pFrameBuffer       FrameData from camera
\return void
*/
//-------------------------------------------------
inline int PixelFormatConvert(PGX_FRAME_BUFFER pFrameBuffer, int64_t i64ColorFilter, u_char* pRGBImageBuf)
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image
            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return -1;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            u_char * pRaw8Image = new unsigned char[pFrameBuffer->nImgSize];
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                delete[] pRaw8Image;
                return -1;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char*)pRaw8Image, pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                delete[] pRaw8Image;
                return -1;
            }
            delete[] pRaw8Image;
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return -1;
        }
    }
    return 0;
}

inline int Raw16toRaw8(PGX_FRAME_BUFFER pFrameBuffer, int64_t i64ColorFilter, u_char* pImageBuf)
{
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW16 to RAW8
    switch (pFrameBuffer->nPixelFormat)
    {
      case GX_PIXEL_FORMAT_BAYER_GR10:
      case GX_PIXEL_FORMAT_BAYER_RG10:
      case GX_PIXEL_FORMAT_BAYER_GB10:
      case GX_PIXEL_FORMAT_BAYER_BG10:
      case GX_PIXEL_FORMAT_BAYER_GR12:
      case GX_PIXEL_FORMAT_BAYER_RG12:
      case GX_PIXEL_FORMAT_BAYER_GB12:
      case GX_PIXEL_FORMAT_BAYER_BG12:
      {
        // Convert to the Raw8 image
        memset(pImageBuf, 0, pFrameBuffer->nWidth*pFrameBuffer->nHeight);
        // emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
        emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_1_8);
        // emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_0_7);
        // emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, pImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_3_10);
        if (emDXStatus != DX_OK)
        {
            printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
            return -1;
        }
        break;
      }
      default:
      {
          printf("Error : PixelFormat Raw16toRaw8 is not supported\n");
          return -1;
      }
    }
    return 0;
}

inline int Raw10PackedtoRaw16(PGX_FRAME_BUFFER pFrameBuffer, u_char* pImageBuf)
{
    VxInt32 emDXStatus = DX_OK;

    memset(pImageBuf, 0, pFrameBuffer->nWidth*pFrameBuffer->nHeight*2);
    emDXStatus = DxRaw10PackedToRaw16((unsigned char*)pFrameBuffer->pImgBuf, pImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight);
    if (emDXStatus != DX_OK)
    {
        printf("DxRaw10PackedToRaw16 Failed, Error Code: %d\n", emDXStatus);
        return -1;
    }

}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return string
*/
//----------------------------------------------------------------------------------
inline const char* GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        return "Error when calling GXGetLastError";
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        return "Failed to allocate memory";
    }

    char * error_string = new char[size];
    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        error_string = "Error when calling GXGetLastError";
    }
    else
    {
        strncpy(error_string, error_info, size);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
    return error_string;
}


#endif