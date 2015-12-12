#pragma once

#ifndef _CRITICAL_SECTION_
#define _CRITICAL_SECTION_

#include <Windows.h>

namespace kukdh1
{
	class CriticalSection
	{
		private:
			CRITICAL_SECTION cs;

		public:
			CriticalSection()
			{
				InitializeCriticalSection(&cs);
			}

			~CriticalSection()
			{
				DeleteCriticalSection(&cs);
			}

			void Enter()
			{
				EnterCriticalSection(&cs);
			}

			void Leave()
			{
				LeaveCriticalSection(&cs);
			}
	};
}

#endif