#pragma once

#ifndef _CONSTANT_H_
#define _CONSTANT_H_

//#define PRINT_DEBUG_MSG

#define WINDOW_WIDTH	800
#define WINDOW_HEIGHT	800
#define LOG_WIDTH		300

#define MAX_LOG_LINE		200
#define LOG_CLASS_NAME		L"Log Window"
#define TEXT_BUFFER_SIZE	4096

#define VGF_LEAF			0.05

#define SOR_MEAN_K				50
#define SOR_STDDEV_MUL_THRES	1.0

#define ICP_MAX_DISTANCE		0.3
#define ICP_MAX_ITERATION		100
#define ICP_TRANSFORM_EPSILON	1E-8
#define ICP_EUCLIDEAN_EPSILON	1.0

#define MAX_CORRECTION_TIME		500.0

#define PI		3.141592f

#define GL_GRID_SIZE	50
#define GL_ROT_RATIO	0.01f
#define GL_MOVE_RATIO	0.01f
#define GL_ZOOM_RATIO	0.01f

#define PORT		9000

#define ID_LOG_EDIT		10

#define WM_REGISTER_THIS		WM_APP
#define	WM_MOUSEWHEEL_PASS		WM_APP + 0x01
#define WM_LOG_SHOW				WM_APP + 0x02

struct PACKET_WRAPPER
{
	unsigned int len;
	unsigned char *pucData;
	HWND hWnd;
};

#endif