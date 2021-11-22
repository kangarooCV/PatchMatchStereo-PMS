//
// Created by Zhi-E on 2021/11/18.
//

#ifndef PATCHMATCHSTEREO_UTILS_H
#define PATCHMATCHSTEREO_UTILS_H

#include <opencv2/opencv.hpp>
#include "pms_types.h"

/*显示视差图*/
void ShowDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& name);
/*保存视差图*/
void SaveDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);
/*保存视差点云*/
void SaveDisparityCloud(const uint8* img_bytes, const float32* disp_map, const sint32& width, const sint32& height, const std::string& path);




#endif //PATCHMATCHSTEREO_UTILS_H
