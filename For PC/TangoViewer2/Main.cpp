#define WIN32_LEAN_AND_MEAN

#include <Windows.h>
#include <ShellScalingApi.h>
#include <mutex>

#include "Constant.h"
#include "PointCloud.h"
#include "GLWindow.h"
#include "NetworkManager.h"
#include "Stopwatch.h"
#include "Log.h"

#pragma comment(lib, "Shcore")
#pragma comment(linker,"\"/manifestdependency:type='win32' \
name='Microsoft.Windows.Common-Controls' version='6.0.0.0' \
processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

LPCWSTR lpszClass = L"Tango Viewer Server Edition v2.0";
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
void OpenGLDraw();
void NetworkProc(HWND hWnd, unsigned int len, unsigned char *pucData);
DWORD WINAPI ParseData(LPVOID arg);

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpszCmdParam, int nCmdShow)
{
	WNDCLASS wndclass;
	MSG msg;
	HWND hWnd;
	RECT rtWindow;

	SetProcessDpiAwareness(PROCESS_SYSTEM_DPI_AWARE);

	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hbrBackground = (HBRUSH)GetSysColorBrush(COLOR_BTNFACE);
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndclass.hInstance = hInstance;
	wndclass.lpfnWndProc = (WNDPROC)WndProc;
	wndclass.lpszClassName = lpszClass;
	wndclass.lpszMenuName = NULL;
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	RegisterClass(&wndclass);

	SetRect(&rtWindow, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	AdjustWindowRect(&rtWindow, WS_BORDER | WS_CAPTION | WS_SYSMENU, FALSE);

	hWnd = CreateWindow(lpszClass, lpszClass, WS_BORDER | WS_CAPTION | WS_SYSMENU, CW_USEDEFAULT, CW_USEDEFAULT, rtWindow.right - rtWindow.left, rtWindow.bottom - rtWindow.top, NULL, NULL, hInstance, NULL);
	ShowWindow(hWnd, nCmdShow);

	while (GetMessage(&msg, 0, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return (int)msg.wParam;
}

HFONT hFont;
BOOL bDrawFlag;
std::mutex m;

size_t stReadCount;

kukdh1::GLWindow *glWindow;
kukdh1::LogWindow *lWindow;
kukdh1::PointCloud *pcData;

LRESULT CALLBACK WndProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam)
{
	switch (iMessage)
	{
		case WM_CREATE:
			RECT rtGLWindow;

			pcData = new kukdh1::PointCloud();

			hFont = CreateFont(17, 0, 0, 0, 400, 0, 0, 0, DEFAULT_CHARSET, 3, 2, 1, FF_ROMAN, L"Segoe UI");

			SetRect(&rtGLWindow, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
			glWindow = new kukdh1::GLWindow(hWnd, ((LPCREATESTRUCT)lParam)->hInstance, &rtGLWindow, OpenGLDraw);
			lWindow = new kukdh1::LogWindow(hWnd, ((LPCREATESTRUCT)lParam)->hInstance);

			lWindow->SetFont(hFont);
			lWindow->ToggleLogWindow();

			kukdh1::NetworkManager(hWnd, NetworkProc, lWindow);

			return 0;
		case WM_KEYUP:
			switch (wParam)
			{
				case 'B':	//Toggle Object Oriented Bounding Box
					bDrawFlag = !bDrawFlag;
					glWindow->Refresh();
					break;
				case 'R':	//Refresh GLWindow
					glWindow->Refresh();
					break;
				case 'L':	//Toggle LogWindow
					lWindow->ToggleLogWindow();
					break;
				case 'M':	//Save selected item to Map File
					WCHAR *pszTemp;

					lWindow->PrintLog(L"Save selected data to map.\r\n");
				
					pszTemp = (WCHAR *)calloc(MAX_PATH, sizeof(WCHAR));

					wcscat_s(pszTemp, MAX_PATH, L"PointCloudMap.pcdx");

					pcData->ToFile(pszTemp);

					lWindow->PrintLog(L"Save selected data to map Finished\r\n\r\n");

					break;
			}

			return 0;
		case WM_MOVE:
			RECT rtWindow;

			GetWindowRect(hWnd, &rtWindow);
			lWindow->MoveLogWindow(rtWindow.right, rtWindow.top);

			return 0;
		case WM_MOUSEWHEEL:
			glWindow->OnWheelMessage(hWnd, wParam, lParam);
			return 0;
		case WM_DESTROY:
			delete glWindow;
			delete lWindow;
			delete pcData;

			PostQuitMessage(0);

			return 0;
	}

	return DefWindowProc(hWnd, iMessage, wParam, lParam);
}

void OpenGLDraw()
{
	glPushMatrix();

	pcData->DrawOnGLWindow(bDrawFlag);

	glPopMatrix();
}

void NetworkProc(HWND hWnd, unsigned int len, unsigned char *pucData)
{
	HANDLE hThread;
	PACKET_WRAPPER *ppwData;

	ppwData = (PACKET_WRAPPER *)calloc(1, sizeof(PACKET_WRAPPER));

	ppwData->hWnd = hWnd;
	ppwData->len = len;
	ppwData->pucData = (unsigned char *)calloc(len, sizeof(unsigned char));

	memcpy(ppwData->pucData, pucData, len);

	hThread = CreateThread(NULL, 0, ParseData, ppwData, NULL, NULL);
	CloseHandle(hThread);
}

DWORD WINAPI ParseData(LPVOID arg)
{
	PACKET_WRAPPER *ppwData = (PACKET_WRAPPER *)arg;
	kukdh1::PointCloud pcTemp;
	kukdh1::Stopwatch stopwatch;
	unsigned int uiPoints;
	WCHAR szTemp[32];
	time_t tTime;

	memcpy(&uiPoints, ppwData->pucData + 4, 4);

	pcTemp.FromBytestream(ppwData->pucData + 8, uiPoints, ppwData->pucData + 72);

	tTime = time(NULL);
	wsprintf(szTemp, L"Network_%ld", tTime);

	stopwatch.tic();

#ifdef PRINT_DEBUG_MSG
	pcTemp.ApplyStaticalOutlierRemoveFilter(lWindow);
	pcTemp.DownSampling(lWindow);
#else
	pcTemp.ApplyStaticalOutlierRemoveFilter();
	pcTemp.DownSampling();
#endif

	pcTemp.CalculateBoundingBox();
	stopwatch.tok();

	m.lock();

	lWindow->PrintLog(L"Received :\r\n [%d] %s (%.3lfms)\r\n\r\n", stReadCount, szTemp, stopwatch.get());

	lWindow->PrintLog(L"Error correction for [%d] Begin\r\n", stReadCount);

	stopwatch.tic();

#ifdef PRINT_DEBUG_MSG
	pcData->MergePointCloud(pcTemp, lWindow);
#else
	pcData->MergePointCloud(pcTemp);
#endif

	stopwatch.tok();
	
	lWindow->PrintLog(L" [%d] Correction End (%.3lfms)\r\n", stReadCount, stopwatch.get());
	lWindow->PrintLog(L"Error correction for [%d] End\r\n\r\n", stReadCount++);

	if (stopwatch.get() > MAX_CORRECTION_TIME)
	{
#ifdef PRINT_DEBUG_MSG
		pcData->DownSampling(lWindow);
#else
		pcData->DownSampling();
#endif
	}

	m.unlock();

	free(ppwData->pucData);
	free(ppwData);

	glWindow->Refresh();

	return 0;
}