/*
 * 3driver.h
 *
 *  Created on: 29 ����. 2016 �.
 *      Author: Home
 */

#ifndef DRIVER3D_H_
#define DRIVER3D_H_

#include "stm32f4xx_hal.h"
#include "lcd.h"
#include "math.h"

#define X_BORDER 240
#define Y_BORDER 320

#define SCALE_X0 X_BORDER/2
#define SCALE_Y0 Y_BORDER/2

#define PERSPECTIVE_COEFICIENT_SCALE 200

//Object3d struct properties
#define OBJECT_VERTEX_QUANTITY_MAX 50
#define OBJECT_EDGE_QUANTITY_MAX 50

//3D point struct
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} XYZ_point;

//Projected 2D point
typedef struct {
	int16_t x;
	int16_t y;
} XY_point;

//Projected edge
typedef struct {
	XY_point* v1;
	XY_point* v2;
	uint16_t color;
} Edge;

typedef struct {

	//Object properties
	int16_t width, depth, heigth;
	XYZ_point centre;

	uint8_t vertex_quantity;
	uint8_t edge_quantity;

	//Start position
	float x_rotate_start;
	float y_rotate_start;
	float z_rotate_start;

	//Position of object origin from global origin
	XYZ_point position;

	//Reference vertexes
	XYZ_point vertex_start[OBJECT_VERTEX_QUANTITY_MAX];

	//2D projection. Dinamic changes in movement
	XY_point vertex[OBJECT_VERTEX_QUANTITY_MAX];
	Edge 	  edge[OBJECT_EDGE_QUANTITY_MAX];

	//Current position
	float x_rotate;
	float y_rotate;
	float z_rotate;

} Object3d__HandleTypeDef;

//Main functions
void Object3d_InitBox(Object3d__HandleTypeDef* obj, int16_t width, int16_t depth, int16_t heigth);

void Object3d_DrawObject(Object3d__HandleTypeDef* obj);
void Object3d_CleanObject(Object3d__HandleTypeDef* obj);
void Object3d_SetRotation(Object3d__HandleTypeDef* obj, float x_angle, float y_angle, float z_angle);

void Object3d_SetPosition(Object3d__HandleTypeDef* obj, int16_t x_pos, int16_t y_pos, int16_t z_pos);
void Object3d_SetStartRotation(Object3d__HandleTypeDef* obj, float x_angle, float y_angle, float z_angle);

void Object3d_SetEdgesColor(Object3d__HandleTypeDef* obj, uint16_t c);
void Object3d_SetEdgeColor(Object3d__HandleTypeDef* obj, uint8_t n, uint16_t c);

void Object3d_SetSize(Object3d__HandleTypeDef* obj,	int16_t w, int16_t d, int16_t h);

//inner functions
void Object3d_DrawEdge(Object3d__HandleTypeDef* obj, uint8_t n);
void Object3d_CleanEdge(Object3d__HandleTypeDef* obj, uint8_t n);

void Object3d_SetBoxVertexes(Object3d__HandleTypeDef* obj);
void Object3d_SetBoxEdges(Object3d__HandleTypeDef* obj);
void Object3d_SetVertex(int16_t x, int16_t y, int16_t z, XYZ_point* point);
void Object3d_SetEdge(XY_point* p1, XY_point* p2, Edge* e);

void Object3d_CalculateObject(Object3d__HandleTypeDef* obj);
XYZ_point Object3d_CalculateVertex(Object3d__HandleTypeDef* obj, uint8_t n);
XY_point Object3d_GetRealXY(XYZ_point* k);



#endif /* DRIVER3D_H_ */
