#include "kalman_filter.h"

/**
 *kalman_init - 卡尔曼滤波器初始化
 *@kalman_lcw：卡尔曼滤波器结构体
 *@init_x：待测量的初始值
 *@init_p：后验状态估计值误差的方差的初始值
 */
void kalman_init(kalman_struct *kalman_lcw, float init_x, float init_p)
{
    kalman_lcw->x = init_x;//待测量的初始值，如有中值一般设成中值（如陀螺仪）
    kalman_lcw->p = init_p;//后验状态估计值误差的方差的初始值
    kalman_lcw->A = 1;
    kalman_lcw->H = 1;
    kalman_lcw->q = 10e-2;//10e-6;//2e2;////predict noise convariance 预测（过程）噪声方差 实验发现修改这个值会影响收敛速率
    kalman_lcw->r = 5e2;//10e-5;//测量（观测）噪声方差。以陀螺仪为例，测试方法是：
    //保持陀螺仪不动，统计一段时间内的陀螺仪输出数据。数据会近似正态分布，
    //按3σ原则，取正态分布的(3σ)^2作为r的初始化值
}

/**
 *kalman_filter - 卡尔曼滤波器
 *@kalman_lcw:卡尔曼结构体
 *@measure；测量值
 *返回滤波后的值
 */
float kalman_filter(kalman_struct *kalman_lcw, float measure)
{
    /* Predict */
    kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
    kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    kalman_lcw->gain = kalman_lcw->p * kalman_lcw->H / (kalman_lcw->p * kalman_lcw->H * kalman_lcw->H + kalman_lcw->r);
    kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure - kalman_lcw->H * kalman_lcw->x);
    kalman_lcw->p = (1 - kalman_lcw->gain * kalman_lcw->H) * kalman_lcw->p;

    return kalman_lcw->x;
}

