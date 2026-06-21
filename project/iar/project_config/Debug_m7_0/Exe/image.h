/*
 * image.h
 *
 *  Created on: 2025年12月9日
 *      Author: LST
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_
#include "zf_common_headfile.h"
//二值化＋缩小图像所用所有定义
#define KUAN_X  (188)  // 原图像宽度
#define GAO_Y   (120)  // 原图像高度
#define KUAN_XR (94)   // 处理后的图像宽度（降采样）
#define GAO_YR  (60)   // 处理后的图像高度（降采样）
extern uint8 mt9v03x_image_copy[60][94];  // 降采样后的二值图像
//二值化数组函数声明
extern void binary_tailor();//处理二值化图像函数
//并查集所用所有定义
//定义并查集数组的大小上限 parent[Beacon_quantity+1] 等数组都以此为基础
#define Beacon_quantity (94)  // 最大疑似信标数量
//存储连通域的完整信息 是并查集搜索的最终输出格式。
typedef struct {
    uint16_t area;
    uint8_t min_row, max_row;
    uint8_t min_col, max_col;
    uint32 sum_row;
    uint32 sum_col;
} ComponentInfo;

// 接收connected_components()函数的输出结果
extern ComponentInfo results[Beacon_quantity];  // 结果数组
////在connected_components函数中定义邻域偏移，用于搜索相邻像素
////const int8_t offsets[] = {-1, -width-1, -width, -width+1}; // 八邻域偏移
//并查集函数声明
// 核心搜索函数
void Mean_filtering(uint8* imgIn, int row, int col);
static inline uint16_t find(uint16_t x);
static inline void union_sets(uint16_t x, uint16_t y);
extern uint16_t connected_components(const uint8_t *image, uint8_t height, uint8_t width, ComponentInfo *result);

#endif /* CODE_IMAGE_H_ */
