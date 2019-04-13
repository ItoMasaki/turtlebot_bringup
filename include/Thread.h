/********************************************************
 * SerialPort.h
 *
 * Portable Thread Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/

#ifndef THREAD_HEADER_INCLUDED
#define THREAD_HEADER_INCLUDED

#ifdef WIN32
#ifdef _WINDLL
#define LIBTHREAD_API __declspec(dllexport)
#else
#define LIBTHREAD_API __declspec(dllimport)
#endif

#else
#define LIBTHREAD_API 


#endif // ifdef WIN32


#ifdef WIN32
#include <windows.h>
#define THREAD_ROUTINE DWORD WINAPI
#else
#include <pthread.h>
#define THREAD_ROUTINE void*
#endif


namespace net {
	namespace ysuga {

		class LIBTHREAD_API Mutex {
		private:
#ifdef WIN32
			HANDLE m_Handle;
#else
			pthread_mutex_t m_Handle;
#endif


		public:
			Mutex() {
#ifdef WIN32
				m_Handle = ::CreateMutex(NULL, 0, NULL);
#else
				pthread_mutex_init(&m_Handle, NULL);
#endif
			}

			virtual ~Mutex() {
#ifdef WIN32
				::CloseHandle(m_Handle);
#else
			  pthread_mutex_destroy(&m_Handle);
#endif
			}

		public:
			void Lock() {
#ifdef WIN32
				::WaitForSingleObject(m_Handle, INFINITE);
#else
			  
			  pthread_mutex_lock(&m_Handle);
#endif
			}

			void Unlock() {
#ifdef WIN32
				::ReleaseMutex(m_Handle);
#else
			  pthread_mutex_unlock(&m_Handle);
#endif
			}
		};

		class LIBTHREAD_API Thread
		{
		private:
#ifdef WIN32
			HANDLE m_Handle;
			DWORD m_ThreadId;
#else
			pthread_t m_Handle;
#endif
		public:
			Thread(void);
			virtual ~Thread(void);

		public:
			void Start();

			virtual void Run() {};

			void Join();

			void Exit(unsigned long exitCode);

		public:
			static void Sleep(unsigned long milliSeconds);
		};

	};
};


#endif

/*******************************************************
 * Copyright  2010, ysuga.net all rights reserved.
 *******************************************************/
