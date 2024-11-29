//
//  TimeManager.cpp
//

#include "TimeManager.h"

#include <cassert>
#include <chrono>

#include "Sleep.h"

using namespace std;
using namespace chrono;
namespace
{
	const milliseconds MIN_SMOOTH_DURATION(5);

	const int UPDATES_PER_SECOND_DEFAULT = 60;
	const unsigned int MAX_UPDATES_PER_FRAME_DEFAULT = 10;

	const float SMOOTH_FRACTION     = 0.05f;
	const float SMOOTH_ANTIFRACTION = 1.0f - SMOOTH_FRACTION;
}



TimeManager :: TimeManager ()
		: updates_per_second(UPDATES_PER_SECOND_DEFAULT),
		  update_delta_time(1000000 / UPDATES_PER_SECOND_DEFAULT),
		  max_updates_per_frame(MAX_UPDATES_PER_FRAME_DEFAULT),
		//  start_time(),

		  update_count(0),
		//  last_update_time(),
		//  last_update_duration(delta_time),
		  smoothed_updates_per_second((float)(UPDATES_PER_SECOND_DEFAULT)),
		//  next_update_time(),

		  frame_count(0),
		//  last_frame_time(),
		//  last_frame_duration(delta_time),
		  smoothed_frames_per_second((float)(UPDATES_PER_SECOND_DEFAULT))
{
	start_time = system_clock::now();
	last_update_time = start_time;
	last_update_duration = update_delta_time;
	next_update_time = start_time;
	last_frame_time = start_time;
	last_frame_duration = update_delta_time;

	assert(isInvariantTrue());
}

TimeManager :: TimeManager (int updates_per_second_in,
                            unsigned int max_updates_per_frame_in)
		: updates_per_second(updates_per_second_in),
		  update_delta_time(1000000 / updates_per_second_in),
		  max_updates_per_frame(max_updates_per_frame_in),
		//  start_time(),

		  update_count(0),
		//  last_update_time(),
		//  last_update_duration(delta_time),
		  smoothed_updates_per_second((float)(updates_per_second_in)),
		//  next_update_time(),

		  frame_count(0),
		//  last_frame_time(),
		//  last_frame_duration(delta_time),
		  smoothed_frames_per_second((float)(updates_per_second_in))
{
	assert(updates_per_second_in > 0);
	assert(max_updates_per_frame_in > 0);

	start_time = system_clock::now();
	last_frame_time = start_time;
	last_frame_duration = update_delta_time;
	last_update_time = start_time;
	last_update_duration = update_delta_time;
	next_update_time = start_time;

	assert(isInvariantTrue());
}



float TimeManager :: getUpdateDeltaTime () const
{
	return duration<float>(update_delta_time).count();
}

unsigned int TimeManager :: getMaxUpdatesPerFrame () const
{
	return max_updates_per_frame;
}

bool TimeManager :: isUpdateWaiting () const
{
	system_clock::time_point current_time = system_clock::now();
	return next_update_time < current_time;
}

float TimeManager :: getGameDuration () const
{
	system_clock::time_point current_time = system_clock::now();
	return duration<float>(current_time - start_time).count();
}

int TimeManager :: getUpdateCount () const
{
	return update_count;
}

int TimeManager :: getFrameCount () const
{
	return frame_count;
}

float TimeManager :: getUpdateRateAverage () const
{
	return update_count / getGameDuration();
}

float TimeManager :: getFrameRateAverage () const
{
	return frame_count / getGameDuration();
}

float TimeManager :: getUpdateRateInstantaneous () const
{
	assert(last_update_duration.count() > 0);
	float last_update_seconds = duration<float>(last_update_duration).count();
	assert(last_update_seconds > 0.0f);
	return 1.0f / last_update_seconds;
}

float TimeManager :: getFrameRateInstantaneous () const
{
	assert(last_frame_duration.count() > 0);
	float last_frame_seconds = duration<float>(last_frame_duration).count();
	assert(last_frame_seconds > 0.0f);
	return 1.0f / last_frame_seconds;
}

float TimeManager :: getUpdateRateSmoothed () const
{
	return smoothed_updates_per_second;
}

float TimeManager :: getFrameRateSmoothed () const
{
	return smoothed_frames_per_second;
}



void TimeManager :: sleepUntilNextUpdate () const
{
	system_clock::time_point current_time = system_clock::now();
	if(current_time < next_update_time)
	{
		system_clock::duration sleep_time = next_update_time - current_time;
		sleep(duration<double>(sleep_time).count());
	}
}

void TimeManager :: markNextUpdate ()
{
	update_count++;
	system_clock::time_point current_time = system_clock::now();
	last_update_duration = current_time - last_update_time;
	if(last_update_duration < MIN_SMOOTH_DURATION)
		last_update_duration = MIN_SMOOTH_DURATION;
	last_update_time = current_time;
	float last_update_seconds = duration<float>(last_update_duration).count();
	float instantaneous_updates_per_second = 1.0f / last_update_seconds;
	smoothed_updates_per_second = SMOOTH_ANTIFRACTION * smoothed_updates_per_second +
	                              SMOOTH_FRACTION     * instantaneous_updates_per_second;
	next_update_time += update_delta_time;

	assert(isInvariantTrue());
}

void TimeManager :: markNextFrame ()
{
	frame_count++;
	system_clock::time_point current_time = system_clock::now();
	last_frame_duration = current_time - last_frame_time;
	if(last_frame_duration < MIN_SMOOTH_DURATION)
		last_frame_duration = MIN_SMOOTH_DURATION;
	last_frame_time = current_time;
	float last_frame_seconds = duration<float>(last_frame_duration).count();
	float instantaneous_frames_per_second = 1.0f / last_frame_seconds;
	smoothed_frames_per_second = SMOOTH_ANTIFRACTION * smoothed_frames_per_second +
	                             SMOOTH_FRACTION     * instantaneous_frames_per_second;

	assert(isInvariantTrue());
}



bool TimeManager :: isInvariantTrue ()
{
	if(updates_per_second <= 0)
		return false;
	if(update_delta_time != microseconds(1000000 / updates_per_second))
		return false;
	if(duration<float>(update_delta_time).count() <= 0.0f)
		return false;
	if(max_updates_per_frame <= 0)
		return false;

	if(last_update_duration.count() <= 0)
		return false;
	if(last_frame_duration.count() <= 0)
		return false;

	if(update_count < 0)
		return false;
	if(frame_count < 0)
		return false;
	if(smoothed_updates_per_second <= 0)
		return false;
	if(smoothed_updates_per_second <= 0)
		return false;

	return true;
}
