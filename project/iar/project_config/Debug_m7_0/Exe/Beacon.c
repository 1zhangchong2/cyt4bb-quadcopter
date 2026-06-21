/*
 * Beacon.c
 *
 *  Created on: 2025年12月15日
 *      Author: LST
 */

#include "zf_common_headfile.h"
#include "Beacon.h"

int flagsss=0;//新加的用来急用的意义得思考
int kll=0;
float a = Celing_Boundary_x;
float b = Celing_Boundary_symmetry;
float c = Celing_Boundary_y;
Light light[Beacon_quantity] = {0};//创建一个疑似灯的数据最大信标数为Beacon_quantity
int Light_x = 0;//识别到的信标灯坐标
int Light_y = 0;//识别到的信标灯坐标
int x_lentgh,y_lentgh;
int sign=0;//sign//算法是否找到信标标志找到置为1了
int counts=0;//用于标记有多少个疑似信标数量
int area=0;//最后取出信标概率数值赋值给这个变量了area
int Beacon_probability=0;//最后取出信标概率数值赋值给这个变量了


/* 名字：Ideal_environment
 * 功能：理想环境检测
 * 参数：无
 * 输出：无
 */
//////理想环境检测
void Ideal_environment()
{
    int x_left_border;
    int x_right_border;
    for(int i=118;i>a*94*94-b*94+c;i--)//遍历16-118层//根据摄像头位置高度可以修改的变量
        {
            for(int f=1;f<188;f++)//遍历0到188列//根据摄像头位置高度可以修改的变量
            {
                //i大于多少根据摄像头采集图像中信标灯的位置和实际信标灯和车的位置决定
               if(mt9v03x_image[i][f-1]<200&&mt9v03x_image[i][f]>200&&sign!=1&&i>50)//根据摄像头位置高度可以修改的变量
               {   sign=1;//找到灯
                   x_left_border=f;//
                   Light_y=i;
               }//i大于多少根据摄像头采集图像中信标灯的位置和实际信标灯和车的位置决定
               if(mt9v03x_image[i][f]>200&&mt9v03x_image[i][f+1]<200&&sign==1&&i>50)//根据摄像头位置高度可以修改的变量
               {
                  x_right_border=f;//
                  Light_x=(x_right_border+x_left_border)/2;
                  return;
               }//运用二次函数解决由于摄像头角度导致图像中的天花板灯会比信标还低的情况
               if(mt9v03x_image[i][f]>50&&sign!=1&&i<50&&i>(a*f*f-b*f+c))//根据摄像头位置高度可以修改的变量
               {
                  sign=1;//找到灯
                  Light_x=f;//
                  Light_y=i;//
                  return;
               }
            }
        }
}
/* 名字：no_Ideal_environment
 * 功能：非理想环境检测
 * 参数：无
 * 输出：无
 */
void no_Ideal_environment()
{
    kll=0;
    int count=connected_components(mt9v03x_image_copy[0],GAO_YR,KUAN_XR, results);
        for(uint16_t i=0; i<count; i++){
             light[i].x= (int)results[i].sum_col /256.0f/results[i].area;
             light[i].y= (int)results[i].sum_row /256.0f/results[i].area;
             light[i].Beacon_area=results[i].area;
             light[i].x_lentgh=results[i].max_col-results[i].min_col;
             light[i].y_lentgh=results[i].max_row-results[i].min_row;
             light[i].probability=0;
        }

        ////////////////////////////////
        {//不抗阳光但抗左右脑互博//保证走最侧面得保证路径最好更加丝滑
              int max=0;
              for(int i=0;i<count;i++)
              {
                  if(max<light[i].x&&light[i].y*2>a*light[i].x*light[i].x*4-b*light[i].x*2+c)
                      {
                           flagsss=i;
                           max=light[i].x;
                           kll++;
                      }
              }
              if(kll>0)
              {
                 Light_x=light[flagsss].x*2;
                 Light_y=light[flagsss].y*2;
                 area=light[flagsss].Beacon_area;
                 x_lentgh=light[flagsss].x_lentgh;
                 y_lentgh=light[flagsss].y_lentgh;
                 sign=1;

              }
        }
        ////////////////////////
//        {//抗阳光//走的一定对但是路径就随机了选中目标不稳定把..
//            for(int i=0;i<count;i++)
//                   {
//                       gailvjisuan(i);
//                   }
//                   uint16_t max= light[0].probability;
//                   uint16_t flags=0;
//                   for(int i=1;i<count;i++)
//                   {
//                           if(max<light[i].probability)
//                           {
//                               max=light[i].probability;
//                               flags=i;
//                           }
//                   }
//                   if(light[flags].probability>=Prabability_THRESHOLD)
//                   {
//                       Light_x=light[flags].x;
//                       Light_y=light[flags].y;
//                       area=light[flags].Beacon_area;
//                       x_lentgh=light[flags].x_lentgh;
//                       y_lentgh=light[flags].y_lentgh;
//                       Beacon_probability=light[flags].probability;
//                       sign=1;
////                   }
//        }
}
/* 名字：type_no_sunlight_interference
 * 功能：并查集的合并操作，使用按秩合并优化。
 * 参数：
 * 输出：无（但会得到已经合并好的连通域）
 */
void type_no_sunlight_interference()
{
    binary_tailor();
    no_Ideal_environment();
}
/* 名字：xunzhaoguangbiao
 * 功能：寻找信标主函数代码
 * 参数：p 控制是否为理想环境
 * 输出：无（但最终会得到信标的x y值）
 */
void  xunzhaoguangbiao(char p)
{

    sign=0;//对是否查找到灯标志清0
    counts=0;//对疑似信标清0每次重新寻找疑似信标
    if(p==0)
    Ideal_environment();//如果绝对理想环境可以只用用这个//这个是相信所有识别到的信标灯
    else
    type_no_sunlight_interference();//如果不理想用这个//无阳光干扰
}
