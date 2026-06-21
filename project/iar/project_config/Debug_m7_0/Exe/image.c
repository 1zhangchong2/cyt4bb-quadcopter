/*
 * image.c
 *
 *  Created on: 2025年12月9日
 *      Author: LST
 */

#include "zf_common_headfile.h"
#include "image.h"
/* 名字：Mean_filtering
 * 功能：均值滤波
 * 参数：识别到的初始图象 高 宽
 * 输出：无
 */
void Mean_filtering(uint8* imgIn, int row, int col)//均值滤波
{
    int i, j;
    for (i = 1; i < row - 1; i++)
    {
        for (j = 1; j < col - 1; j++)
        {
            int sum = 0;
            sum = *(imgIn + i * col + j)
                + *(imgIn + (i - 1) * col + j)
                + *(imgIn + (i + 1) * col + j);
            *(imgIn + i * col + j) = (int)(sum / 3);
        }
    }
    for (i = 0, j = 0; j < col; j++)

        *(imgIn + i * col + j) =
        *(imgIn + (i + 1) * col + j);
    for (i = row - 1, j = 0; j < col; j++)

        *(imgIn + i * col + j) =
        *(imgIn + (i - 1) * col + j);
    for (i = 0, j = 0; i < row - 1; i++)

        *(imgIn + i * col + j) =
        *(imgIn + i * col + j + 1);
    for (i = 0, j = col - 1; i < row - 1; i++)

        *(imgIn + i * col + j) =
        *(imgIn + i * col + j - 1);
}
ComponentInfo results[Beacon_quantity];
uint8 mt9v03x_image_copy[60][94];//二值化缩小处理后的数组
/* 名字：binary_tailor
 * 功能：处理图像（将原图像的188*120处理为94*60 并进行二值化）
 * 参数：无
 * 输出：无（但会得到二值化后的94*60的二维图像数组）
 */
void binary_tailor()//处理图像 将原图像的188*120处理为94*60 并进行二值化
{
    for(int f=0;f<60;f++)
    {
        for(int i=0;i<94;i++)
            {
//                if(f<20)
//                {
//                    if(mt9v03x_image[f*2][i*2]>200)
//                    {
//                        mt9v03x_image_copy[f][i]=255;
//                    }
//                    else
//                    {
//                        mt9v03x_image_copy[f][i]=0;
//                    }
//                }
                if(f<=30)
                {
                    //初始值是40
                    if(mt9v03x_image[f*2][i*2]>40)
                    {
                        mt9v03x_image_copy[f][i]=255;
                    }
                    else
                    {
                        mt9v03x_image_copy[f][i]=0;
                    }
                }

                if(f>30)
                {
                    //初始值是200
                    if(mt9v03x_image[f*2][i*2]>200)
                    {
                        mt9v03x_image_copy[f][i]=255;
                    }
                    else
                    {
                        mt9v03x_image_copy[f][i]=0;
                    }
                }
            }
    }
}
//并查集算法函数
// 在并查集find代码中直接使用的静态数组
static uint16_t parent[Beacon_quantity+1];  // 并查集父节点数组
static uint8_t rank[Beacon_quantity+1];     // 并查集秩数组
static uint16_t labels[GAO_YR][Beacon_quantity];  // 标签矩阵 存储每个像素的临时标签 在搜索过程中使用
static ComponentInfo components[Beacon_quantity+1];  // 组件信息数组 存储每个连通域的属性信息 最终的所有结果都在这里
/* 名字：find
 * 功能：并查集的查找操作，带路径压缩优化。
 * 参数：x（连通域中要查找的节点编号）
 * 输出：节点x所在集合的根节点（该根节点代表了该连通域的最终标签）
 */
static inline uint16_t find(uint16_t x)//搜索加压缩路径
{
    while (parent[x] != x)
    {
        parent[x] = parent[parent[x]];  //路径压缩
        x = parent[x];
    }
    return x;
}
/* 名字：union_sets
 * 功能：并查集的合并操作，使用按秩合并优化。
 * 参数：
 * 输出：无（但会得到已经合并好的连通域）
 */
static inline void union_sets(uint16_t x, uint16_t y)
{
    x = find(x);
    y = find(y);
    if (x == y) return;

    if (rank[x] < rank[y]) parent[y] = x;
    else {
        parent[x] = y;
        if (rank[x] == rank[y]) rank[y]++;
    }
}
/* 名字：connected_components
 * 功能：并查集操作的主函数。
 * 参数：图像 图像高度 图像宽度
 * 输出：
 */
uint16_t connected_components(const uint8_t *image,
                            uint8_t height,
                            uint8_t width,
                            ComponentInfo *result) {
    // 初始化（比memset更快的手动清零）
    for(uint16_t i=0; i<=Beacon_quantity; i++){
        parent[i] = i;
        rank[i] = 0;
    }
    memset(components, 0, sizeof(components));
    for(uint8_t i=0; i<height; i++){
        for(uint8_t j=0; j<width; j++){
            labels[i][j] = 0;
        }
    }

    uint16_t current_label = 0;
    const int8_t offsets[] = {-1, -width-1, -width, -width+1}; // 八邻域偏移

    // 第一遍扫描：标记连通域
    for (uint8_t i = 0; i < height; i++) {
        for (uint8_t j = 0; j < width; j++) {
            if (!image[i*width + j]) continue;

            // 检查四个已处理的邻域
            uint16_t min_neighbor = 0xFFFF;
            for(uint8_t k=0; k<4; k++){
                int8_t ni = i + (offsets[k] / width);
                int8_t nj = j + (offsets[k] % width);
                if(ni >=0 && nj >=0 && nj < width && labels[ni][nj]){
                    uint16_t root = find(labels[ni][nj]);
                    min_neighbor = (root < min_neighbor) ? root : min_neighbor;
                }
            }

            if(min_neighbor == 0xFFFF) {
                // 新标签
                if (++current_label > Beacon_quantity) return 0;
                labels[i][j] = current_label;
                parent[current_label] = current_label;
            } else {
                labels[i][j] = min_neighbor;
                // 合并所有相邻标签
                for(uint8_t k=0; k<4; k++){
                    int8_t ni = i + (offsets[k] / width);
                    int8_t nj = j + (offsets[k] % width);
                    if(ni >=0 && nj >=0 && nj < width && labels[ni][nj]){
                        union_sets(labels[ni][nj], min_neighbor);
                    }
                }
            }
        }
    }

    // 第二遍扫描：统计属性
    uint16_t label_count = 0;
    for (uint8_t i = 0; i < height; i++) {
        for (uint8_t j = 0; j < width; j++) {
            uint16_t lbl = labels[i][j];
            if (!lbl) continue;

            uint16_t root = find(lbl);
            if(components[root].area == 0) {
                // 新组件
                label_count++;
                components[root].area = 1;
                components[root].min_row = i;
                components[root].max_row = i;
                components[root].min_col = j;
                components[root].max_col = j;
                components[root].sum_row = i * 256;
                components[root].sum_col = j * 256;
            } else {
                // 更新边界
                components[root].area++;
                if (i < components[root].min_row) components[root].min_row = i;
                if (i > components[root].max_row) components[root].max_row = i;
                if (j < components[root].min_col) components[root].min_col = j;
                if (j > components[root].max_col) components[root].max_col = j;
                components[root].sum_row += i * 256;
                components[root].sum_col += j * 256;

            }
        }
    }

    // 生成结果
    uint16_t valid_count = 0;
    for(uint16_t i=1; i<=current_label; i++){
        if(components[i].area > 0){
            result[valid_count++] = components[i];
        }
    }

    return valid_count;
}
