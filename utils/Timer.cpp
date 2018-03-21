#include "Timer.h"

Timer::Timer()
{
  elapsedTime = 0;
  running = false;
}


// Have the timer run a function after an interval
void Timer::StartTimeout(const Interval &interval,
               const Timeout &timeout)
{
  StartTimer();
  running = true;

  th = std::thread([=]()
  {
    while(running) {
      std::this_thread::sleep_for(interval);
      TimeCut();
      timeout(GetElapsedTime()*-1.0);
    }
  });       
}

void Timer::StartTimer()
{
  startTime = std::chrono::high_resolution_clock::now();
}

double Timer::TimeCut()
{
  auto stopTime =  std::chrono::high_resolution_clock::now();
  auto difference = stopTime - startTime;
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(difference);
  elapsedTime = time_span.count();

  return elapsedTime;
}

double Timer::EndTimer()
{
  if(running){
    running = false;
    //th.join();
  }
  return TimeCut();
}

uint64_t Timer::EndTimerNanos()
{
  if(running){
    running = false;
    //th.join();
  }
  auto stopTime =  std::chrono::high_resolution_clock::now();
  auto difference = stopTime - startTime;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(difference).count();
}

double Timer::GetElapsedTime()
{
  return elapsedTime;
}
