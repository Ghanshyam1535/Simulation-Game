//
//  TimeManager.h
//
#pragma once

#include <chrono>



class TimeManager
{
public:
	TimeManager ();
	TimeManager (int updates_per_second_in,
	             unsigned int max_updates_per_frame_in);

	float getUpdateDeltaTime () const;
	unsigned int getMaxUpdatesPerFrame () const;
	bool isUpdateWaiting () const;
	float getGameDuration () const;
	int getUpdateCount () const;
	int getFrameCount () const;
	float getUpdateRateAverage () const;
	float getFrameRateAverage () const;
	float getUpdateRateInstantaneous () const;
	float getFrameRateInstantaneous () const;
	float getUpdateRateSmoothed () const;
	float getFrameRateSmoothed () const;

	void sleepUntilNextUpdate () const;
	void markNextUpdate ();
	void markNextFrame ();

private:
	bool isInvariantTrue ();

private:
	int updates_per_second;
	std::chrono::microseconds update_delta_time;
	unsigned int max_updates_per_frame;
	std::chrono::system_clock::time_point start_time;

	int update_count;
	std::chrono::system_clock::time_point last_update_time;
	std::chrono::duration<float> last_update_duration;
	float smoothed_updates_per_second;
	std::chrono::system_clock::time_point next_update_time;

	int frame_count;
	std::chrono::system_clock::time_point last_frame_time;
	std::chrono::duration<float> last_frame_duration;
	float smoothed_frames_per_second;
};

