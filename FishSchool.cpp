//
//  FishSchool.cpp
//

#include "FishSchool.h"

#include <cassert>
#include <climits>
#include <vector>

#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"

#include "Entity.h"
#include "Fish.h"
#include "Terrain.h"
#include "FixedEntity.h"
#include "Collision.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	//
	//  NEIGHBOUR_COUNT
	//
	//  The number of neighbours a fish uses for flocking.  This
	//    does not include the flock leader.
	//
	const unsigned int NEIGHBOUR_COUNT = 4;

	//
	//  NEIGHBOUR_UPDATE_INTERVAL
	//
	//  How many AI update cycles should pass befor a fish's
	//    neighbours are recalculated.
	//
	const unsigned int NEIGHBOUR_UPDATE_INTERVAL = 60;

	const double LEADER_SPEED_FRACTION        = 0.85;  // compared to fish
	const double LEADER_ACCELERATION_FRACTION = 0.7;   // compared to fish

	const double PURSUIT_TUNING_FACTOR     = 0.8;  // for fish following leader
	const double DESIRED_SEPARATION_FACTOR = 4.0;  // in fish radiuses

}  // end of anonymous namespace



FishSchool :: FishSchool ()
		: Entity(Vector3::ZERO, 1.0),
		  m_species(0),
		// mv_fish will be initialized to empty by the default constructor
		  m_update_neighbours_first(0),
		  m_explore_center(Vector3::ZERO),
		  m_explore_max_distance(0.0),
		  m_leader(Vector3::ZERO, 0.0),
		  m_leader_target(Vector3::ZERO)
{
	assert(isInvariantTrue());
}

FishSchool :: FishSchool (const ObjLibrary::Vector3& school_center,
                          double school_radius,
                          unsigned int fish_count,
                          unsigned int fish_species,
                          double explore_max_distance)
		: Entity(school_center, school_radius),
		  m_species(fish_species),
		// mv_fish will be initialized below
		  m_update_neighbours_first(rand() % NEIGHBOUR_UPDATE_INTERVAL),
		  m_explore_center(school_center),
		  m_explore_max_distance(explore_max_distance),
		  m_leader(school_center, 0.0),
		  m_leader_target(school_center)
{
	assert(Fish::isModelsLoaded());
	assert(school_radius > 0.0);
	assert(fish_species < Fish::SPECIES_COUNT);
	assert(explore_max_distance >= 0.0);

	double speed = Fish::getSpeed(fish_species);

	for(unsigned int i = 0; i < fish_count; i++)
	{
		Vector3 position = school_center + Vector3::getRandomSphereVector() * school_radius;
		Vector3 forward  = Vector3::getRandomUnitVector();
		Fish fish(position, forward, fish_species);
		fish.setVelocity(forward * speed);
		mv_fish.push_back(fish);
	}
	assert(mv_fish.size() == fish_count);

	assert(isInvariantTrue());
}



unsigned int FishSchool :: getSpecies () const
{
	assert(isInvariantTrue());

	return m_species;
}

unsigned int FishSchool :: getCount () const
{
	assert(isInvariantTrue());

	return mv_fish.size();
}

const Fish& FishSchool :: getFish (unsigned int fish) const
{
	assert(isInvariantTrue());
	assert(fish < getCount());

	return mv_fish[fish];
}

void FishSchool :: draw () const
{
	assert(isInvariantTrue());

	for(unsigned int i = 0; i < mv_fish.size(); i++)
		mv_fish[i].draw();
}

void FishSchool :: drawAllCoordinateSystems (double length) const
{
	assert(isInvariantTrue());
	assert(length > 0.0);

	for(unsigned int i = 0; i < mv_fish.size(); i++)
	{
		glPushMatrix();
			mv_fish[i].applyDrawTransformations();
			glBegin(GL_LINES);
				glColor3d(1.0, 0.0, 0.0);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(length, 0.0, 0.0);
				glColor3d(0.0, 1.0, 0.0);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(0.0, length, 0.0);
				glColor3d(0.0, 0.0, 1.0);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(0.0, 0.0, length);
			glEnd();
		glPopMatrix();
	}
}

void FishSchool :: drawAllCollisionSpheres () const
{
	assert(isInvariantTrue());

	for(unsigned int i = 0; i < mv_fish.size(); i++)
	{
		double radius = mv_fish[i].getRadius();
		glPushMatrix();
			mv_fish[i].applyDrawTransformations();
			glScaled(radius, radius, radius);
			glutWireIcosahedron();
			//glutWireSphere(1.0, 8, 6);
		glPopMatrix();
	}
}

void FishSchool :: drawNeighbours () const
{
	assert(isInvariantTrue());

	if(mv_fish.empty())
		return;  // no fish to draw neighbours for

	const Vector3& agent_position = mv_fish[0].getPosition();

	glBegin(GL_LINES);
		const Vector3& leader_position = m_leader.getPosition();
		glVertex3d( agent_position.x,  agent_position.y,  agent_position.z);
		glVertex3d(leader_position.x, leader_position.y, leader_position.z);

		const vector<unsigned int>& v_neighbours = mv_fish[0].getNeighbours();
		for(unsigned int i = 0; i < v_neighbours.size(); i++)
		{
			if(v_neighbours[i] >= mv_fish.size())
				continue;  // can happen if player catches fish

			assert(v_neighbours[i] < mv_fish.size());
			const Fish& neighbour = mv_fish[v_neighbours[i]];
			const Vector3& neighbour_position = neighbour.getPosition();
			glVertex3d(    agent_position.x,     agent_position.y,     agent_position.z);
			glVertex3d(neighbour_position.x, neighbour_position.y, neighbour_position.z);
		}
	glEnd();
}

void FishSchool :: drawLeader () const
{
	assert(isInvariantTrue());

	const Vector3& leader_position = m_leader.getPosition();
	glBegin(GL_LINES);
		glVertex3d(leader_position.x, leader_position.y, leader_position.z);
		glVertex3d(m_leader_target.x, m_leader_target.y, m_leader_target.z);
	glEnd();

	glPushMatrix();
		glTranslated(leader_position.x, leader_position.y, leader_position.z);
		glScaled(0.02, 0.02, 0.02);
		glutSolidIcosahedron();
	glPopMatrix();

	glPushMatrix();
		glTranslated(m_leader_target.x, m_leader_target.y, m_leader_target.z);
		glScaled(0.02, 0.02, 0.02);
		glutSolidIcosahedron();
	glPopMatrix();
}



void FishSchool :: setLeaderTarget (const ObjLibrary::Vector3& target)
{
	assert(isInvariantTrue());

	m_leader_target = target;

	assert(isInvariantTrue());
}

void FishSchool :: moveAllByVelocity (float delta_time)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	for(unsigned int i = 0; i < mv_fish.size(); i++)
		mv_fish[i].moveByVelocity(delta_time);
	m_leader.moveByVelocity(delta_time);

	assert(isInvariantTrue());
}

void FishSchool :: applyGravityAll (float delta_time)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	for(unsigned int i = 0; i < mv_fish.size(); i++)
		if(mv_fish[i].getPosition().y > 0)
			mv_fish[i].applyGravity(delta_time);

	assert(isInvariantTrue());
}

void FishSchool :: bounceAllInwards ()
{
	assert(isInvariantTrue());

	for(unsigned int i = 0; i < mv_fish.size(); i++)
	{
		Vector3 fish_pos = mv_fish[i].getPosition();
		if(fish_pos.isDistanceGreaterThan(getPosition(), getRadius()))
		{
			Vector3 normal = getPosition() - fish_pos;
			normal.normalize();
			mv_fish[i].bounce(normal);
		}
	}

	assert(isInvariantTrue());
}

void FishSchool :: checkCollisionAll (const Terrain& terrain)
{
	assert(isInvariantTrue());

	// no school-level check

	for(unsigned int i = 0; i < mv_fish.size(); i++)
		if(isCollision(mv_fish[i], terrain))
		{
			Vector3 surface_normal = terrain.getSurfaceNormal(mv_fish[i].getPosition());
			mv_fish[i].bounce(surface_normal);
		}

	assert(isInvariantTrue());
}

void FishSchool :: checkCollisionAll (const FixedEntity& entity)
{
	assert(isInvariantTrue());

	if(isCollision(*this, entity))
	{
		for(unsigned int i = 0; i < mv_fish.size(); i++)
		{
			Fish& r_fish = mv_fish[i];
			if(isCollision(entity, r_fish))
			{
				Vector3 surface_normal = entity.getSurfaceNormal(r_fish.getPosition());
				r_fish.bounce(surface_normal);
			}
		}
	}

	assert(isInvariantTrue());
}

unsigned int FishSchool :: checkPlayerCaughtFish (const Entity& player)
{
	assert(isInvariantTrue());

	unsigned int caught_count = 0;
	if(isCollision(*this, player))
	{
		for(unsigned int i = 0; i < mv_fish.size(); i++)
			if(isCollision(player, mv_fish[i]))
			{
				caught_count++;

				// remove fish from vector
				mv_fish[i] = mv_fish.back();
				mv_fish.pop_back();
				i--;  // don't skip new fish in this spot
			}
	}

	assert(isInvariantTrue());
	return caught_count;
}

void FishSchool :: updateLeader (float delta_time,
                                 const Terrain& terrain)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	if(mv_fish.empty())
		return;
	assert(0 < mv_fish.size());

	if(isLeaderNearTarget())
		m_leader_target = chooseNewLeaderTarget(terrain);
	leaderSteerToTarget(delta_time);

	assert(isInvariantTrue());
}

void FishSchool :: doFlockingAll (float delta_time)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	// update neighbours for a few fish
	for(unsigned int i = m_update_neighbours_first; i < mv_fish.size(); i += NEIGHBOUR_UPDATE_INTERVAL)
		updateNeighbours(i);
	m_update_neighbours_first++;
	m_update_neighbours_first %= NEIGHBOUR_UPDATE_INTERVAL;

	// do flocking for all fish
	for(unsigned int i = 0; i < mv_fish.size(); i++)
		if(mv_fish[i].isUnderwater())
			doFlocking(i, delta_time);

	assert(isInvariantTrue());
}

void FishSchool :: updateBoundingSphere ()
{
	assert(isInvariantTrue());

	if(mv_fish.empty())
		return;

	Vector3 center;
	for(unsigned int i = 0; i < mv_fish.size(); i++)
		center += mv_fish[i].getPosition();
	assert(mv_fish.size() > 0);
	center /= mv_fish.size();
	setPosition(center);

	double max_distance = 0.0;
	for(unsigned int i = 0; i < mv_fish.size(); i++)
	{
		double distance = center.getDistance(mv_fish[i].getPosition());
		if(max_distance < distance)
			max_distance = distance;
	}
	setRadius(max_distance + mv_fish[0].getRadius());

	assert(isInvariantTrue());
}

void FishSchool :: updateOrientationAll ()
{
	assert(isInvariantTrue());

	for(unsigned int i = 0; i < mv_fish.size(); i++)
		mv_fish[i].setOrientation(mv_fish[i].getVelocity().getNormalized());

	assert(isInvariantTrue());
}



bool FishSchool :: isLeaderNearTarget () const
{
	double max_speed = Fish::getSpeed(getSpecies());
	double current_speed = m_leader.getVelocity().getNorm();
	double max_distance = current_speed * 0.1;
	const Vector3& leader_position = m_leader.getPosition();

	if(leader_position.isDistanceLessThan(m_leader_target, max_distance))
		return true;
	else
		return false;
}

ObjLibrary::Vector3 FishSchool :: chooseNewLeaderTarget (const Terrain& terrain)
{
	for(unsigned int sanity = 0; sanity < 1000; sanity++)
	{
		Vector3 new_position = m_explore_center + Vector3::getRandomSphereVector() * m_explore_max_distance;

		if(!terrain.isInside(new_position))
			continue;  // position is outside map; try again
		double terrain_height = terrain.getHeight(new_position);
		if(terrain_height >= 0.0)
			continue;  // position is on land; try again

		//double padding = getRadius() * 0.5;  // half of current fisch school size
		double padding = Fish::getRadiusBySpecies(m_species);
		double y_min = terrain_height + padding;
		double y_max = 0.0 - padding;  // 0.0 is water surface
		if(new_position.y < y_min)
			new_position.y = y_min;
		if(new_position.y > y_max)
			new_position.y = y_max;
		return new_position;
	}

	return m_leader_target;
}

void FishSchool :: leaderSteerToTarget (float delta_time)
{
	assert(delta_time >= 0.0);

	double speed        = Fish::getSpeed       (getSpecies()) * LEADER_SPEED_FRACTION;
	double acceleration = Fish::getAcceleration(getSpecies()) * LEADER_ACCELERATION_FRACTION;

	const Vector3& leader_position = m_leader.getPosition();
	const Vector3& leader_velocity = m_leader.getVelocity();

	Vector3 desired_direction = m_leader_target - leader_position;
	desired_direction.normalizeSafe();
	Vector3 desired_velocity = desired_direction * speed;

	Vector3 steering = desired_velocity - m_leader.getVelocity();
	steering.truncate(acceleration * delta_time);

	m_leader.addVelocity(steering);
}

void FishSchool :: updateNeighbours (unsigned int agent_index)
{
	assert(agent_index < getCount());

	static const unsigned int NO_NEIGHBOUR = UINT_MAX;

	assert(NEIGHBOUR_COUNT > 0);
	vector<unsigned int> v_neighbours(NEIGHBOUR_COUNT, NO_NEIGHBOUR);
	vector<double>       v_distances (NEIGHBOUR_COUNT, 1.0e40);

	Vector3 agent_position = mv_fish[agent_index].getPosition();

	for(unsigned int i = 0; i < mv_fish.size(); i++)
	{
		if(i == agent_index)
			continue;  // cannot be own neighbour

		double i_distance = agent_position.getDistance(mv_fish[i].getPosition());
		assert(!v_distances.empty());
		if(i_distance >= v_distances.back())
			continue;  // didn't get into list

		// find correct spot in list (as in insertion sort)
		//   -> bump old neighbour backwards as needed
		//   -> old neighbours at the back disappear
		unsigned int n = v_neighbours.size() - 1;
		while(n >= 1 && i_distance < v_distances[n - 1])
		{
			v_neighbours[n] = v_neighbours[n - 1];
			v_distances [n] = v_distances [n - 1];
			n--;
		}

		// add new neighbour to list
		v_neighbours[n] = i;
		v_distances [n] = i_distance;
	}

	// remove non-neighbours from list
	while(!v_neighbours.empty() &&
	       v_neighbours.back() == NO_NEIGHBOUR)
	{
		v_neighbours.pop_back();
	}

	for(unsigned int i = 0; i < v_neighbours.size(); i++)
		assert(v_neighbours[i] < mv_fish.size());

	mv_fish[agent_index].setNeighbours(v_neighbours);
}

void FishSchool :: doFlocking (unsigned int agent_index,
                               float delta_time)
{
	assert(agent_index < getCount());
	assert(delta_time >= 0.0);

	Vector3 following  = calculateLeaderFollowing(agent_index);
	Vector3 separation = calculateSeparationAll  (agent_index);
	Vector3 alignment  = calculateAlignment      (agent_index);
	Vector3 desired_velocity = combineFlockingForces(following, separation, alignment);

	double acceleration = Fish::getAcceleration(getSpecies());
	Vector3 old_velocity = mv_fish[agent_index].getVelocity();
	Vector3 steering = desired_velocity - old_velocity;
	steering.truncate(acceleration * delta_time);
	mv_fish[agent_index].addVelocity(steering);
}

ObjLibrary::Vector3 FishSchool :: calculateLeaderFollowing (unsigned int agent_index)
{
	assert(agent_index < getCount());

	// pursuit part
	assert(agent_index < mv_fish.size());
	Vector3 P1 = mv_fish[agent_index].getPosition();  // agent position
	Vector3 P2 = m_leader.getPosition();              // current target position
	Vector3 V2 = m_leader.getVelocity();              // target velocity
	double  d = P1.getDistance(P2);                   // current distance
	double  t = d * PURSUIT_TUNING_FACTOR;            // look ahead time
	Vector3 T = P2 + V2 * t;                          // future target position

	// arrival part
	double  s_max     = Fish::getSpeed       (getSpecies());
	double  a         = Fish::getAcceleration(getSpecies());
	double  d_slow    = 0.5 * (s_max * s_max) / a;  // slowing distance
	double  d2        = P1.getDistance(T);          // distance to future target position
	double  s_ramped  = s_max * (d / d_slow);
	double  s_clipped = s_max;                      // desired speed
	if(s_ramped < s_clipped)
		s_clipped = s_max;
	Vector3 R = T - P1;                             // desired direction

	// simple method - gives an elongated school
	Vector3 D = R.getCopyWithNormSafe(s_clipped);   // desired velocity
	return D;

	// relative to target - gives a spherical school
	//Vector3 D = R.getCopyWithNormSafe(s_ramped);   // desired velocity
	//Vector3 desired = D;// + V2;
	//desired.truncate(s_max);
	//return desired;
}

ObjLibrary::Vector3 FishSchool :: calculateSeparationAll (unsigned int agent_index)
{
	assert(agent_index < getCount());

	assert(agent_index < mv_fish.size());
	Vector3 agent_position = mv_fish[agent_index].getPosition();
	const vector<unsigned int>& v_neighbours = mv_fish[agent_index].getNeighbours();

	Vector3 separation = calculateSeparation(agent_position, m_leader.getPosition());
	for(unsigned int i = 0; i < v_neighbours.size(); i++)
	{
		if(v_neighbours[i] >= mv_fish.size())
			continue;  // can happen if player catches fish
		assert(v_neighbours[i] < mv_fish.size());

		const Fish& neighbour = mv_fish[v_neighbours[i]];
		const Vector3& neighbour_position = neighbour.getPosition();
		separation += calculateSeparation(agent_position, neighbour_position);
	}

	return separation;
}

ObjLibrary::Vector3 FishSchool :: calculateSeparation (const ObjLibrary::Vector3& agent_position,
                                                       const ObjLibrary::Vector3& neighbour_position)
{
	assert(!mv_fish.empty());
	double desired_separation = mv_fish[0].getRadius() * DESIRED_SEPARATION_FACTOR;
	assert(desired_separation >= 0.0);

	double distance = agent_position.getDistance(neighbour_position);
	if(distance < 0.001)
		return Vector3::ZERO;  // too close for distance to be meaningful

	assert(distance > 0.0);
	if(distance >= desired_separation)
		return Vector3::ZERO;

	Vector3 direction = agent_position - neighbour_position;
	direction.normalizeSafe();

	double speed_max = Fish::getSpeed(getSpecies());

	// using 1 - d / d_slow
	//   result is 0 at edge, 1 at same place
	//   then square the result
	// 1/r is not as good
	//   unit-dependant
	//   scaling r to interval [0, 1] makes numbers always larger than max speed

	double relative_distance = 1.0 - distance / desired_separation;
	assert(relative_distance >= 0.0);
	assert(relative_distance <= 1.0);
	return direction * relative_distance * relative_distance * speed_max;
}

ObjLibrary::Vector3 FishSchool :: calculateAlignment (unsigned int agent_index)
{
	assert(agent_index < getCount());

	assert(agent_index < mv_fish.size());
	const vector<unsigned int>& v_neighbours = mv_fish[agent_index].getNeighbours();

	Vector3 alignment = m_leader.getVelocity();
	double neighbour_count = v_neighbours.size() + 1;
	if(neighbour_count >= 1)
	{
		for(unsigned int i = 0; i < v_neighbours.size(); i++)
		{
			if(v_neighbours[i] >= mv_fish.size())
				continue;  // can happen if player catches fish
			assert(v_neighbours[i] < mv_fish.size());

			const Fish& neighbour = mv_fish[v_neighbours[i]];
			alignment += neighbour.getVelocity();
		}
		alignment /= neighbour_count;
	}

	return alignment;
}

ObjLibrary::Vector3 FishSchool :: combineFlockingForces (const ObjLibrary::Vector3& following,
                                                         const ObjLibrary::Vector3& separation,
                                                         const ObjLibrary::Vector3& alignment)
{
	double speed_max = Fish::getSpeed(getSpecies());

	Vector3 desired_velocity = following  * 1.0 +
	                           separation * 3.0 +
	                           alignment  * 0.0;
	desired_velocity.truncate(speed_max);
	return desired_velocity;
}

bool FishSchool :: isInvariantTrue () const
{
	if(m_species >= Fish::SPECIES_COUNT)
		return false;
	if(m_explore_max_distance < 0.0)
		return false;
	return true;
}


