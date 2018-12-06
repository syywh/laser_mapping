#ifndef _TIMER_H_
#define _TIMER_H_

#include <chrono>
#include <time.h>
using namespace std::chrono;
#include <inttypes.h>
#include <iostream>
#include <stdint.h>

typedef uint8_t BYTE;
typedef uint32_t DWORD;
typedef int32_t LONG;
typedef int64_t LONGLONG;

typedef union _LARGE_INTEGER {
  struct {
    DWORD LowPart;
    LONG  HighPart;
  };
  struct {
    DWORD LowPart;
    LONG  HighPart;
  } u;
  LONGLONG QuadPart;
} LARGE_INTEGER, *PLARGE_INTEGER;

namespace vill{

	class Timer{

	//public:
	//	Timer(){
	//		QueryPerformanceFrequency(&large_integer);
	//		dff = large_integer.QuadPart;
	//		reset();
	//	}
	//	void reset(){
	//		QueryPerformanceCounter(&large_integer);
	//		c0 = large_integer.QuadPart;
	//	}
	//	/*�Ժ��뷵�� */
	//	double elapsed(void) {
	//		QueryPerformanceCounter(&large_integer);
	//		c2 = large_integer.QuadPart;
	//		return (1.0* (c2 - c0) / dff * 1000);
	//	}
	//private:
	//	LARGE_INTEGER large_integer;
	//	uint64_t c0;
	//	uint64_t c2;
	//	double dff;

	public:
		Timer() :m_begin(high_resolution_clock::now()){	}
		void reset() {
			m_begin=high_resolution_clock::now();
		}
		// ��΢����ΪĬ�Ϸ���
		template<typename Duration=microseconds>
		int64_t elapsed() const{
			return duration_cast<Duration> (high_resolution_clock::now() - m_begin).count();
		}

		template <class Clock>
		void display_precision()
		{
			typedef std::chrono::duration<double, std::nano> NS;
			NS ns = typename Clock::duration(1);
			std::cout << ns.count() << " ns\n";
		}

	private:
		high_resolution_clock::time_point m_begin;

	};
}

#endif
