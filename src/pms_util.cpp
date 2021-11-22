/* -*-c++-*- PatchMatchStereo - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
*			  https://github.com/ethan-li-coding
* Describe	: implement of pms_util
*/

#include "stdafx.h"
#include "pms_util.h"
#include <vector>
#include <algorithm>

PColor pms_util::GetColor(const uint8* img_data, const sint32& width, const sint32& height, const sint32& x, const sint32& y)
{
    // 通过图像首地址加上偏移量定位像素的地址位置
	auto* pixel = img_data + y * width * 3 + 3 * x;
    // 加上偏移量就是像素点的坐标
	return {pixel[0], pixel[1], pixel[2]};
}

void pms_util::MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height,
	const sint32 wnd_size)
{
	const sint32 radius = wnd_size / 2;
	const sint32 size = wnd_size * wnd_size;

	// 存储局部窗口内的数据
	std::vector<float32> wnd_data;
	wnd_data.reserve(size);

	for (sint32 y = 0; y < height; y++) {
		for (sint32 x = 0; x < width; x++) {
			wnd_data.clear();

			// 获取局部窗口数据
			for (sint32 r = -radius; r <= radius; r++) {
				for (sint32 c = -radius; c <= radius; c++) {
					const sint32 row = y + r;
					const sint32 col = x + c;
					if (row >= 0 && row < height && col >= 0 && col < width) {
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}

			// 排序
			std::sort(wnd_data.begin(), wnd_data.end());

			if (!wnd_data.empty()) {
				// 取中值
				out[y * width + x] = wnd_data[wnd_data.size() / 2];
			}
		}
	}
}

// 比较函数，用于排序
bool cmp(const std::pair<float32, float32> &a, const std::pair<float32, float32> &b){
    return a.first < b.first;
}

// 中值滤波只考虑了空间域，并未考虑颜色域，所以它对边缘的保持其实不太友好，而加权中值滤波则会考虑窗口内像素和中心像素的色差，色差越小的像素，给其更大的权值，色差越大的像素，给其更小的权值，这样就尽可能避免不同属性的像素彼此影响。
void pms_util::WeightedMedianFilter(const uint8* img_data, const sint32& width, const sint32& height, const sint32& wnd_size, const float32& gamma, const vector<pair<int, int>>& filter_pixels, float32* disparity_map)
{
	const sint32 wnd_size2 = wnd_size / 2;

	// 带权视差集
	std::vector<std::pair<float32, float32>> disps;
	disps.reserve(wnd_size * wnd_size);

    // 对不匹配点进行加权中值滤波
	for (auto& pix : filter_pixels) {
		const sint32 x = pix.first;
		const sint32 y = pix.second;	
		// weighted median filter
		disps.clear();
		const auto& col_p = GetColor(img_data, width, height, x, y);
		float32 total_w = 0.0f;
		for (sint32 r = -wnd_size2; r <= wnd_size2; r++) {
			for (sint32 c = -wnd_size2; c <= wnd_size2; c++) {
				const sint32 yr = y + r;
				const sint32 xc = x + c;
				if (yr < 0 || yr >= height || xc < 0 || xc >= width) {
					continue;
				}
				const auto& disp = disparity_map[yr * width + xc];
				if(disp == Invalid_Float) {
					continue;
				}
				// 计算权值
				const auto& col_q = GetColor(img_data, width, height, xc, yr);
				const auto dc = abs(col_p.r - col_q.r) + abs(col_p.g - col_q.g) + abs(col_p.b - col_q.b);
				const auto w = exp(-dc / gamma);
				total_w += w;

				// 存储带权视差
				disps.emplace_back(disp, w);
			}
		}

		// --- 取加权中值
		// 按视差值排序
		std::sort(disps.begin(), disps.end(), cmp);
		const float32 median_w = total_w / 2;
		float32 w = 0.0f;
		for (auto& wd : disps) {
			w += wd.second;
			if (w >= median_w) {
				disparity_map[y * width + x] = wd.first;
				break;
			}
		}
	}
}


