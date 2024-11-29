//
//  FishSchool.h
//
//  A module to represent a school of fish.
//

#pragma once

#include <vector>

#include "ObjLibrary/Vector3.h"

#include "Entity.h"
#include "Fish.h"

class Terrain;
class FixedEntity;


//
//  FishSchool
//
//  A class to represent a school of fish.
//
//  Class Invariant:
//    <1> m_species < Fish::SPECIES_COUNT
//    <2> m_explore_max_distance >= 0.0
//
class FishSchool : public Entity
{
public:
//
//  Default Constructor
//
//  Purpose: To construct a FishSchool containing no fish.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A FishSchool is constructed at the origin,
//               containing 0 fish of species 0 in a sphere of
//               radius 1.0.
//
	FishSchool ();

//
//  Constructor
//
//  Purpose: To construct a FishSchool containing the specified
//           number of fish of the specified species.
//  Parameter(s):
//    <1> school_center: The center of the school
//    <2> school_radius: The radius of the school
//    <3> fish_count: The number of fish in the school
//    <4> fish_species: The fish species
//    <5> explore_max_distance: The maximum distance to range
//                              from the starting position
//  Precondition(s):
//    <1> Fish::isModelsLoaded()
//    <2> school_radius > 0.0
//    <3> fish_species < Fish::SPECIES_COUNT
//    <4> explore_max_distance >= 0.0
//  Returns: N/A
//  Side Effect: A FishSchool is constructed at position
//               school_center, containing fish_count fish of
//               species fish_species in a sphere of radius
//               school_radius.
//
	FishSchool (const ObjLibrary::Vector3& school_center,
	            double school_radius,
	            unsigned int fish_count,
	            unsigned int fish_species,
	            double explore_max_distance);

//
//  getSpecies
//
//  Purpose: To determine the species of the fish in this
//           FishSchool.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The fish species.  This value will always be
//           strictly less than Fish::SPECIES_COUNT.
//  Side Effect: N/A
//
	unsigned int getSpecies () const;

//
//  getCount
//
//  Purpose: To determine the number of fish in this FishSchool.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The number of fish.
//  Side Effect: N/A
//
	unsigned int getCount () const;

//
//  getFish
//
//  Purpose: To retreive a reference to the specified fish in
//           this FishSchool.
//  Parameter(s):
//    <1> fish: Which fish
//  Precondition(s):
//    <1> fish < getCount()
//  Returns: Fish fish in this FishSchool.
//  Side Effect: N/A
//
	const Fish& getFish (unsigned int fish) const;

//
//  draw
//
//  Purpose: To display the fish in this FishSchool.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: All fish in this FishSchool are displayed.
//
	void draw () const;

//
//  drawAllCoordinateSystems
//
//  Purpose: To display the local coordinate system for each
//           fish in this FishSchool.
//  Parameter(s):
//    <1> length: The length of the coordinate system axes
//  Precondition(s):
//    <1> length > 0.0
//  Returns: N/A
//  Side Effect: The local coordinate system for each fish in
//               this FishSchool are displayed.  Each axis has
//               length length.
//
	void drawAllCoordinateSystems (double length) const;

//
//  drawAllCollisionSpheres
//
//  Purpose: To display the collision sphere for each fish in
//           this FishSchool.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The collision sphere for each fish in this
//               FishSchool is displayed.
//
	void drawAllCollisionSpheres () const;

//
//  drawNeighbours
//
//  Purpose: To display the current neighbours for one fish in
//           this FishSchool.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The neighbours of a fish in this FishSchool are
//               displayed.  If this function is called
//               repeatedly, the same fish will always be
//               displayed.  If this FishSchool contains no
//               fish, this function has no effect.
//
	void drawNeighbours () const;

//
//  drawLeader
//
//  Purpose: To display the flock leader for this FishSchool and
//           its current explore target.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A line is drawn between the flock leader and
//               the current explore target.
//
	void drawLeader () const;

//
//  setLeaderTarget
//
//  Purpose: To change the exploration target for the flock
//           leader.
//  Parameter(s):
//    <1> target: The new exploration target
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The exploration target for the flock leader is
//               set to target.
//
	void setLeaderTarget (const ObjLibrary::Vector3& target);

//
//  moveAllByVelocity
//
//  Purpose: To move all the fish in this FishSchool according
//           to their velocities.  This function also moves the
//           flock leader.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: The position of each fish in this FishSchool is
//               updated for moving at its current velocity for
//               a duration of delta_time.
//
	void moveAllByVelocity (float delta_time);

//
//  applyGravityAll
//
//  Purpose: To apply gravity to all the fish in this FishSchool
//           that are currently above water.
//  Parameter(s):
//    <1> delta_time: The duration to apply gravity for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: Each fish in this FishSchool is checked against
//               the surface of the water.  If it above, gravity
//               is applied.
//
	void applyGravityAll (float delta_time);

//
//  bounceAllInwards
//
//  Purpose: To make all the fish in this FishSchool bounce off
//           the inside of this FishSchool's bounding sphere.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: Each fish in this FishSchool is checked against
//               the school's bounding sphere.  If a fish is
//               outside the bounding sphere, it is bounced to
//               face inwards.
//
	void bounceAllInwards ();

//
//  checkCollisionAll
//
//  Purpose: To handle collisions between all fish in this
//           FishSchool and the specified Terrain.
//            Heirarchical collision checking is NOT used.
//  Parameter(s):
//    <1> terrain: The Terrain to check for collisions
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: Each fish in this FishSchool is checked for a
//               collision with terrain.  Each fish that
//               collides with terrain bounces off it.
//
	void checkCollisionAll (const Terrain& terrain);

//
//  checkCollisionAll
//
//  Purpose: To handle collisions between all fish in this
//           FishSchool and the specified FixedEntity.
//  Parameter(s):
//    <1> entity: The FixedEntity to check for collisions
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This FishSchool is checked for a collision with
//               entity.  If there is one, each fish in this
//               FishSchool is also checked for a collision.
//               Each fish that collides with entity bounces off
//               it.
//
	void checkCollisionAll (const FixedEntity& entity);

//
//  checkPlayerCaughtFish
//
//  Purpose: To handle the player catching fish in this
//           FishSchool.
//  Parameter(s):
//    <1> player: The player Entity
//  Precondition(s): N/A
//  Returns: The number of fish caught.
//  Side Effect: This FishSchool is checked for a collision with
//               player.  If there is one, each fish in this
//               FishSchool is also checked for a collision.
//               Each fish that collides with player is removed.
//
	unsigned int checkPlayerCaughtFish (const Entity& player);

//
//  updateLeader
//
//  Purpose: To handle the AI updates for the flock leader in
//           this FishSchool.  The flock leader will follow the
//           explore steering beahviour.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//    <2> terrain: The map terrain
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: If the flock leader is near the current explore
//               target, a new target is chosen.  In either
//               case, the flock leader steers towards the
//               current target.
//
	void updateLeader (float delta_time,
	                   const Terrain& terrain);

//
//  doFlockingAll
//
//  Purpose: To handle the flocking AI for the fish in this
//           FishSchool.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: Flocking behaviour is applied to the fish in
//               this FishSchool.  The neighbours for each fish
//               are preiodically updated.
//
	void doFlockingAll (float delta_time);

//
//  updateBoundingSphere
//
//  Purpose: To recalculate the bounding sphere for this fish
//           school.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The center of this fish school is calculated as
//               the average position of the fish in it.  The
//               radius is calculated a the distance from the
//               center to the outside of the farthest-out fish.
//               If there are no fish in this school, there is
//               no effect.
//
	void updateBoundingSphere ();

//
//  updateOrientationAll
//
//  Purpose: To adjust the orientation for all fish so they look
//           like they are swimming in the direction they are
//           moving.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The orientation of each fish in this FishSchool
//               is recalculated.
//
	void updateOrientationAll ();

private:
//
//  isLeaderNearTarget
//
//  Purpose: To determine if the flock leader for this school is
//           near its target.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the flock leader is within 1 second's
//           travel from its target.
//  Side Effect: N/A
//
	bool isLeaderNearTarget () const;

//
//  chooseNewLeaderTarget
//
//  Purpose: To choose a new exploration target for the flock
//           leader in this FishSchool.
//  Parameter(s):
//    <1> terrain: The map terrain
//  Precondition(s): N/A
//  Returns: A new explore target for the flock leader.  It will
//           be underwater according to terrain.  If no new
//           valid exploration target is found, the current
//           target is returned.
//  Side Effect: N/A
//
	ObjLibrary::Vector3 chooseNewLeaderTarget (
	                                    const Terrain& terrain);

//
//  leaderSteerToTarget
//
//  Purpose: To make the flock leader for this FishSchool steer
//           towards the current explore target.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: The flock leader turns towards the current
//               exploration target and accelerates to maximum
//               swim speed.
//
	void leaderSteerToTarget (float delta_time);

//
//  updateNeighbours
//
//  Purpose: To recalculate the neighbours for the specified
//           fish.
//  Parameter(s):
//    <1> agent_index: Which fish
//  Precondition(s):
//    <1> agent_index < getCount()
//  Returns: N/A
//  Side Effect: The neighbours for fish agent_index are
//               recalculated. The flock leader is never
//               included in the neighbours list.
//
	void updateNeighbours (unsigned int agent_index);

//
//  doFlocking
//
//  Purpose: To apply flocking forces to the specified fish.
//  Parameter(s):
//    <1> agent_index: Which fish
//    <2> delta_time: The duration to move for
//  Precondition(s):
//    <1> agent_index < getCount()
//    <2> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: Cohesion, seperation, and alignment forces are
//               applied to fish agent_index.  The flock leader
//               is treated as a neighbour.
//
	void doFlocking (unsigned int agent_index,
	                 float delta_time);

	ObjLibrary::Vector3 calculateLeaderFollowing (
	                                  unsigned int agent_index);

	ObjLibrary::Vector3 calculateSeparationAll (
	                                  unsigned int agent_index);

	ObjLibrary::Vector3 calculateSeparation (
	             const ObjLibrary::Vector3& agent_position,
	             const ObjLibrary::Vector3& neighbour_position);

	ObjLibrary::Vector3 calculateAlignment (
	                                  unsigned int agent_index);

	ObjLibrary::Vector3 combineFlockingForces (
	                      const ObjLibrary::Vector3& following,
	                      const ObjLibrary::Vector3& separation,
	                      const ObjLibrary::Vector3& alignment);

//
//  isInvariantTrue
//
//  Purpose: To determine if the class invariant is true.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the class invariant is true.
//  Side Effect: N/A
//
	bool isInvariantTrue () const;

private:
	unsigned int m_species;
	std::vector<Fish> mv_fish;
	unsigned int m_update_neighbours_first;
	ObjLibrary::Vector3 m_explore_center;
	double m_explore_max_distance;
	Entity m_leader;
	ObjLibrary::Vector3 m_leader_target;
};




