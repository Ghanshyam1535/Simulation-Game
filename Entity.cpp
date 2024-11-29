//
//  Entity.cpp
//

#include "Entity.h"

#include <cassert>
#include <cmath>

#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

using namespace std;
using namespace ObjLibrary;
namespace std
{
	const Vector3 GRAVITY(0.0, -9.8, 0.0);
}



Entity :: Entity ()
		: CoordinateSystem(Vector3(0.0, 0.0, 0.0)),
		  m_radius(0.0)
		// m_velocity will be initialized by its own default constructor
{
	assert(isInvariantTrue());
}

Entity :: Entity (const ObjLibrary::Vector3& center,
                  double radius)
		: CoordinateSystem(center),
		  m_radius(radius)
		// m_velocity will be initialized by its own default constructor
{
	assert(radius >= 0.0);

	assert(isInvariantTrue());
}

Entity :: Entity (const CoordinateSystem& coords,
                  double radius)
		: CoordinateSystem(coords),
		  m_radius(radius)
		// m_velocity will be initialized by its own default constructor
{
	assert(radius >= 0.0);

	assert(isInvariantTrue());
}



double Entity :: getRadius () const
{
	assert(isInvariantTrue());

	return m_radius;
}

const Vector3& Entity :: getVelocity () const
{
	assert(isInvariantTrue());

	return m_velocity;
}

bool  Entity :: isUnderwater () const
{
	assert(isInvariantTrue());

	return getPosition().y < 0.0;
}



void Entity :: setCoordinateSystem (const CoordinateSystem& coords)
{
	assert(isInvariantTrue());

	CoordinateSystem::operator=(coords);

	assert(isInvariantTrue());
}

void Entity :: setRadius (double radius)
{
	assert(isInvariantTrue());
	assert(radius >= 0.0);

	m_radius = radius;

	assert(isInvariantTrue());
}

void Entity :: setVelocity (const ObjLibrary::Vector3& velocity)
{
	assert(isInvariantTrue());

	m_velocity = velocity;

	assert(isInvariantTrue());
}

void Entity :: addVelocity (const ObjLibrary::Vector3& delta)
{
	assert(isInvariantTrue());

	m_velocity += delta;

	assert(isInvariantTrue());
}

void Entity :: moveByVelocity (float delta_time)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	Vector3 new_position = getPosition() + m_velocity * delta_time;
	setPosition(new_position);

	assert(isInvariantTrue());
}

void Entity :: applyGravity (float delta_time)
{
	assert(isInvariantTrue());
	assert(delta_time >= 0.0);

	addVelocity(GRAVITY * delta_time);

	assert(isInvariantTrue());
}

bool Entity :: bounce (const ObjLibrary::Vector3& normal)
{
	assert(isInvariantTrue());

	bool is_bounce = false;
	if(!m_velocity.isSameHemisphere(normal))
	{
		Vector3 projection = m_velocity.getProjection(normal);
		Vector3  rejection = m_velocity.getRejection (normal);
		m_velocity = rejection - projection;
		is_bounce = true;
	}

	assert(isInvariantTrue());
	return is_bounce;
}


	
bool Entity :: isInvariantTrue () const
{
	if(m_radius < 0.0)
		return false;
	return true;
}

