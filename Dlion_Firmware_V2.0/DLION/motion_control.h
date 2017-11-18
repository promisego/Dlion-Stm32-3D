#ifndef motion_control_h
#define motion_control_h


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：电机控制  版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/

// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_XXX defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
void mc_arc(float *position, float *target, float *offset, unsigned char axis_0, unsigned char axis_1,
  unsigned char axis_linear, float feed_rate, float radius, unsigned char isclockwise, uint8_t extruder);
  
#endif
