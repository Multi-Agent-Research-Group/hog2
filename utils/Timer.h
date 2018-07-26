/*
 * $Id: timer.h,v 1.5 2006/10/30 17:45:10 nathanst Exp $
 *
 * timer.h
 * HOG file
 * 
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <functional>

class Timer {
public:
        typedef std::chrono::seconds Interval;
        typedef std::function<void(double)> Timeout;
	Timer();
	
	void StartTimeout(const Interval &interval,
               const Timeout &timeout);
	void StartTimer();
	double TimeCut();
	double EndTimer();
	uint64_t EndTimerNanos();
	double GetElapsedTime();
private:
	std::chrono::high_resolution_clock::time_point startTime;
	double elapsedTime;
        bool running;
        std::thread th;
	
};

#endif // TIMER_H
