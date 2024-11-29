//
//  Entity.h
//
//  A module to store an entity on the map.
//

#pragma once

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "CoordinateSystem.h"



//
//  Entity
//
//  A class to store an entity on the map.  An Entity has a
//    local coordinate system and a radius.
//
//  Class Invariant:
//    <1> m_radius >= 0.0
//
class Entity : public CoordinateSystem
{
public:
//
//  Default Constructor
//
//  Purpose: To construct a default Entity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: An Entity with radius 0.0 is constructed at the
//               origin.
//
	Entity ();

//
//  Position Constructor
//
//  Purpose: To construct an Entity with the specified center
//           and radius.
//  Parameter(s):
//    <1> center: The Entity center
//    <2> radius: The Entity radius
//  Precondition(s):
//    <1> radius >= 0.0
//  Returns: N/A
//  Side Effect: An Entity with radius radius is constructed at
//               position center.
//
	Entity (const ObjLibrary::Vector3& center,
	        double radius);

//
//  Coordinate Constructor
//
//  Purpose: To construct an Entity with the specified local
//           coordinate system and radius.
//  Parameter(s):
//    <1> coords: The local coordinate system for the Entity
//    <2> radius: The radius of the Entity
//  Precondition(s):
//    <1> radius >= 0.0
//  Returns: N/A
//  Side Effect: An Entity with radius radius is constructed
//               with local coordinate system coords.
//
	Entity (const CoordinateSystem& coords,
	        double radius);

//
//  getRadius
//
//  Purpose: To determine the radius of this Entity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The Entity radius.  This value will always be
//           non-negative.
//  Side Effect: N/A
//
	double getRadius () const;

//
//  getVelocity
//
//  Purpose: To determine the current velocity of this Entity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The velocity of this Entity.
//  Side Effect: N/A
//
	const ObjLibrary::Vector3& getVelocity () const;

//
//  isUnderwater
//
//  Purpose: To determine if this Entity is currently
//           underwater.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the Y coordinate of this Entity is less
//           than 0.0.
//  Side Effect: N/A
//
	bool isUnderwater () const;

//
//  setCoordinateSystem
//
//  Purpose: To change the coordinate system for this Entity.
//  Parameter(s):
//    <1> coords: The new coordinate system
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This Entity is set to have coordinate system
//               coords.
//
	void setCoordinateSystem (const CoordinateSystem& coords);

//
//  setRadius
//
//  Purpose: To change the radius of this Entity.
//  Parameter(s):
//    <1> radius: The new radius
//  Precondition(s):
//    <1> radius >= 0.0
//  Returns: N/A
//  Side Effect: This Entity is set to have a radius of radius.
//
	void setRadius (double radius);

//
//  setVelocity
//
//  Purpose: To change the velocity of this Entity.
//  Parameter(s):
//    <1> velocity: The new velocity
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: This Entity is set to have velocity velocity.
//
	void setVelocity (const ObjLibrary::Vector3& velocity);

//
//  addVelocity
//
//  Purpose: To increase the velocity of this Entity.
//  Parameter(s):
//    <1> delta: The change to the velocity
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The velocity of this Entity is increased by
//               delta.
//
	void addVelocity (const ObjLibrary::Vector3& delta);

//
//  moveByVelocity
//
//  Purpose: To move this Entity according to its velocity.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: The position of this Entity is updated for
//               moving at its current velocity for a duration
//               of delta_time.
//
	void moveByVelocity (float delta_time);

//
//  applyGravity
//
//  Purpose: To apply gravity to this Entity.
//  Parameter(s):
//    <1> delta_time: The duration to move for
//  Precondition(s):
//    <1> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: A downward acceleration of 9.8 m/s^2 is
//               applied to the Entity.
//
	void applyGravity (float delta_time);

//
//  bounce
//
//  Purpose: To make this Entity bounce elastically off a
//           surface with the specified surface normal.  This is
//           an elastic bounce.
//  Parameter(s):
//    <1> normal: The surface normal
//  Precondition(s):
//    <1> normal.isUnit()
//  Returns: Whether this Entity bounced.
//  Side Effect: If this Entity is moving in the opposite
//               direction as normal (dot product < 0), the
//               current velocity is changed to the reflected
//               angle.  The projection of the velocity onto
//               normal is inverted, while the rejection is
//               unchanged.  If this Entity is stopped or
//               already moving in the same direction as normal,
//               there is no effect.
//
	bool bounce (const ObjLibrary::Vector3& normal);

private:
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
	double m_radius;
	ObjLibrary::Vector3 m_velocity;
};




