#pragma once

#ifndef _NETWORK_MANAGER_H_
#define _NETWORK_MANAGER_H_

#include <boost\bind.hpp>
#include <boost\asio.hpp>
#include <boost\thread.hpp>

#include <Windows.h>

#include "Log.h"
#include "Constant.h"

//#define PRINT_DEBUG_MSG

namespace kukdh1
{
	typedef void (*NMDataCallback)(HWND hWnd, unsigned int uiByteLength, unsigned char *pucData);

	void NetworkManager(HWND hWnd, NMDataCallback function, LogWindow *lWindow = NULL);
}

#endif