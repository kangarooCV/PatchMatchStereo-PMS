//
// Created by Zhi-E on 2021/11/18.
//

#include "utils.h"

void ShowDisparityMap(const float32* disp_map,const sint32& width,const sint32& height, const std::string& name)
{
    // 显示视差图
    const cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    float32 min_disp = float32(width), max_disp = -float32(width);
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = abs(disp_map[i * width + j]);
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = abs(disp_map[i * width + j]);
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
            }
        }
    }

    cv::imshow(name, disp_mat);
    cv::Mat disp_color;
    applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imshow(name + "-color", disp_color);

}

void SaveDisparityMap(const float32* disp_map, const sint32& width, const sint32& height, const std::string& path)
{
    // 保存视差图
    const cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    float32 min_disp = float32(width), max_disp = -float32(width);
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = abs(disp_map[i * width + j]);
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = abs(disp_map[i * width + j]);
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
            }
        }
    }

    cv::imwrite(path + "-d.png", disp_mat);
    cv::Mat disp_color;
    applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imwrite(path + "-c.png", disp_color);
}

void SaveDisparityCloud(const uint8* img_bytes, const float32* disp_map, const sint32& width, const sint32& height, const std::string& path)
{
    float32 B = 193.001;		// 基线
    float32 f = 999.421;		// 焦距
    float32 x0l = 294.182;		// 左视图像主点x0
    float32 y0l = 252.932;		// 左视图像主点y0
    float32 x0r = 326.95975;	// 右视图像主点x0


    // 保存点云
    FILE* fp_disp_cloud = nullptr;
    fopen_s(&fp_disp_cloud, (path + "-cloud.txt").c_str(), "w");
    if (fp_disp_cloud) {
        for (sint32 y = 0; y < height; y++) {
            for (sint32 x = 0; x < width; x++) {
                const float32 disp = abs(disp_map[y * width + x]);
                if (disp == Invalid_Float) {
                    continue;
                }
                float32 Z = B * f / (disp + (x0r - x0l));
                float32 X = Z * (x - x0l) / f;
                float32 Y = Z * (y - y0l) / f;
                fprintf_s(fp_disp_cloud, "%f %f %f %d %d %d\n", X, Y,
                          Z, img_bytes[y * width * 3 + 3 * x + 2], img_bytes[y * width * 3 + 3 * x + 1], img_bytes[y * width * 3 + 3 * x]);
            }
        }
        fclose(fp_disp_cloud);
    }
}