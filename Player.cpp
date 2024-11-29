//
//  Player.cpp
//

#include "Player.h"

#include <cassert>
#include <vector>

#include "ObjLibrary/Vector3.h"

#include "Map.h"
#include "CoordinateSystem.h"
#include "Entity.h"
#include "Fish.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	const double PLAYER_RADIUS = 0.2;

	const double AUTOPILOT_DIVE_DISTANCE =  5.0;  // m
	const double AUTOPILOT_SURFACE_Y     = -1.0;  // m

	const double PURSUIT_TUNING_FACTOR = 0.5;  // s/m, I think
	const double PLAYER_DESIRED_SPEED  = 5.0;  // m/s
}



Player :: Player ()
		: Entity(Vector3::ZERO, PLAYER_RADIUS),
		  m_autopilot_mode(AUTOPILOT_DISABLED),
		  m_autopilot_school(0),
		  m_autopilot_fish(0),
		  m_look_smoothed(1.0, 0.0, 0.0)
{
	assert(isInvariantTrue());
}

Player :: Player (const CoordinateSystem& coords)
		: Entity(coords, PLAYER_RADIUS),
		  m_autopilot_mode(AUTOPILOT_DISABLED),
		  m_autopilot_school(0),
		  m_autopilot_fish(0),
		  m_look_smoothed(1.0, 0.0, 0.0)
{
	assert(isInvariantTrue());
}



bool Player :: isAutopilot () const
{
	assert(isInvariantTrue());

	if(m_autopilot_mode == AUTOPILOT_DISABLED)
		return false;
	else
		return true;
}

std::string Player :: getAutopilotName () const
{
	assert(isInvariantTrue());

	static const string AUTOPILOT_NAME[AUTOPILOT_COUNT] =
	{
		"Disabled",
		"Planning",
		"Going to surface",
		"Swimming along surface",
		"Diving to catch fish",
	};

	assert(m_autopilot_mode < AUTOPILOT_COUNT);
	return AUTOPILOT_NAME[m_autopilot_mode];
}


void Player :: enableAutopilot ()
{
	assert(isInvariantTrue());

	m_autopilot_mode = AUTOPILOT_PLANNING;
	m_look_smoothed  = getForward();

	assert(isInvariantTrue());
}

void Player :: disableAutopilot ()
{
	assert(isInvariantTrue());

	m_autopilot_mode = AUTOPILOT_DISABLED;

	assert(isInvariantTrue());
}

void Player :: updateAutopilot (const Map& map,
                                float delta_time)
{
	assert(isInvariantTrue());
	assert(isAutopilot());
	assert(delta_time >= 0.0);

	// give up if chasing a bad fish
	if(m_autopilot_school >= map.getFishCaughtCount())
		m_autopilot_mode = AUTOPILOT_PLANNING;
	else if(m_autopilot_fish >= map.getFishSchool(m_autopilot_school).getCount())
		m_autopilot_mode = AUTOPILOT_PLANNING;

	// plan if needed
	if(m_autopilot_mode == AUTOPILOT_PLANNING)
		autopilotPlan(map);

	// chase fish
	if(m_autopilot_mode != AUTOPILOT_PLANNING)
	{
		assert(m_autopilot_mode != AUTOPILOT_DISABLED);
		autopilotChase(map, delta_time);
	}

	assert(isInvariantTrue());
}



void Player :: autopilotPlan (const Map& map)
{
	assert(isAutopilot());
	assert(m_autopilot_mode == AUTOPILOT_PLANNING);

	m_autopilot_school = map.findNearestSchool(getPosition(), true);
	if(m_autopilot_school == Map::NOT_FOUND)
		return;  // no fish to catch

	assert(m_autopilot_school < map.getFishSchoolCount());

	unsigned int fish_count = map.getFishSchool(m_autopilot_school).getCount();
	double random01 = rand() / (RAND_MAX + 1.0);  // implicit typecast to double
	m_autopilot_fish = (unsigned int)(random01 * fish_count);
	assert(m_autopilot_fish < fish_count);

	// change autopilot mode (which doesn't matter)
	m_autopilot_mode = AUTOPILOT_SWIM_UP;
}

void Player :: autopilotChase (const Map& map,
                               float delta_time)
{
	assert(isAutopilot());
	assert(m_autopilot_mode != AUTOPILOT_DISABLED);
	assert(m_autopilot_mode != AUTOPILOT_PLANNING);
	assert(delta_time >= 0.0);

	static const double PI = 3.1415926535897932384626433832795;

	const Fish& target = map.getFishSchool(m_autopilot_school).getFish(m_autopilot_fish);

	//
	//  Step 1: Choose current mode
	//

	double horizontal_distance = getPosition().getDistanceXZ(target.getPosition());
	if(horizontal_distance < AUTOPILOT_DIVE_DISTANCE)
		m_autopilot_mode = AUTOPILOT_DIVE;
	else if(getPosition().y > AUTOPILOT_SURFACE_Y)
		m_autopilot_mode = AUTOPILOT_ON_SURFACE;
	else
		m_autopilot_mode = AUTOPILOT_SWIM_UP;

	//
	//  Step 2: Choose desired velocity
	//

	Vector3 D;  // desired velocity
	if(m_autopilot_mode == AUTOPILOT_SWIM_UP)
	{
		// swim straight up
		D = Vector3(0.0, PLAYER_DESIRED_SPEED, 0.0);
	}
	else
	{
		// pursue
		double d = getPosition().getDistance(target.getPosition());  // distance (including vertical)
		double t = d * PURSUIT_TUNING_FACTOR;  // look ahead time
		Vector3 T = target.getPosition() + target.getVelocity() * t;  // future position
		Vector3 R = T - getPosition();  // from player to future position
		D = R.getCopyWithNormSafe(PLAYER_DESIRED_SPEED);

		if(m_autopilot_mode == AUTOPILOT_ON_SURFACE)
		{
			// pursue on horizontal plain
			D.y = 0;
			D.setNormSafe(PLAYER_DESIRED_SPEED);
		}
	}

	//
	//  Step 3: Steer to desired velocity
	//

	Vector3 S = D - getVelocity();  // steering vector
	Vector3 acceleration = S;
	acceleration.truncate(PLAYER_ACCELERATION);
	addVelocity(acceleration * delta_time);

	//
	//  Step 4: Rotate camera
	//    -> forward vector towards desired velocity
	//    -> up vector towards global up
	//

	m_look_smoothed += (D.getNormalized() - m_look_smoothed) * 0.02;
	rotateToVector(m_look_smoothed, PLAYER_TURN_RATE * delta_time * 0.5);
	double angle_from_vertical = getForward().getAngle(Vector3::UNIT_Y_PLUS);
	if(angle_from_vertical > 0.1 && angle_from_vertical < PI - 0.1)
		rotateToUpright(PLAYER_TURN_RATE * delta_time * 0.25);
}



bool Player :: isInvariantTrue () const
{
	if(m_autopilot_mode > AUTOPILOT_COUNT)
		return false;
	return true;
}


