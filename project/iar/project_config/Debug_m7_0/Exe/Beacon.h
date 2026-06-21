/*
 * Beacon.h
 *
 *  Created on: 2025年12月15日
 *      Author: LST
 */

#ifndef CODE_BEACON_H_
#define CODE_BEACON_H_
#include "zf_common_headfile.h"
#define Beacon_quantity (94)  // 最大疑似信标数量
#define Celing_Boundary_x                (0.0034)//天花板界限这是二次函数中的|a|值越大越窄可以在这里调节分界线
//注意这个要和Celing_Boundary_sysmmetry配合着改
#define Celing_Boundary_symmetry         (0.64)//天花板界限这是二次函数中的|a|值越大越窄可以在这里调节分界线
//注意这个要和Celing_Boundary_x配合着改Celing_Boundary_x修改值/0.0034*0.64就行
#define Celing_Boundary_y                (50)//天花板界限这是二次函数中的|a|值越大越窄可以在这里调节分界线
extern float a;
extern float b;
extern float c;
extern int sign;
typedef struct
{
        int x;//疑似信标X坐标
        int y;//疑似信标Y坐标
        int y_lentgh;//疑似信标高度
        int x_lentgh;//疑似信标宽度
        int Beacon_area;//疑似信标面积Beacon_area
        int probability;//疑似信标可能是真信标的概率probability
}Light;
extern Light light[Beacon_quantity];//创建一个疑似灯的数据最大信标数为Beacon_quantity
extern int Light_x;//识别到的信标灯坐标
extern int Light_y;//识别到的信标灯坐标
extern int x_lentgh,y_lentgh;
extern int sign;//sign//算法是否找到信标标志找到置为1了
extern int counts;//用于标记有多少个疑似信标数量
extern int area;//最后取出信标概率数值赋值给这个变量了area
extern int Beacon_probability;//最后取出信标概率数值赋值给这个变量了
//函数声明
extern void xunzhaoguangbiao(char p);
void Ideal_environment();
void type_no_sunlight_interference();
void no_Ideal_environment();
#endif /* CODE_BEACON_H_ */
