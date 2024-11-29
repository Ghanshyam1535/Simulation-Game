//
//  FixedEntity.cpp
//

#include "FixedEntity.h"

#include <cassert>
#include <cmath>

#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "SurfaceNormal.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	const double NORMAL_LENGTH = 0.5;
}



FixedEntity :: FixedEntity ()
		: Entity(),
		// m_display_list will be initialized by its own default constructor
		// m_normals_list will be initialized by its own default constructor
		  m_is_sphere(true)
		// m_end1 will be initialized by its own default constructor
		// m_end2 will be initialized by its own default constructor
{
	assert(isInvariantTrue());
}

FixedEntity :: FixedEntity (const ObjLibrary::Vector3& center,
                            double radius,
                            const ObjLibrary::DisplayList& display_list)
		: Entity(center, radius),
		  m_display_list(display_list),
		// m_normals_list will be initialized by initNormalsList
		  m_is_sphere(true)
		// m_end1 will be initialized by its own default constructor
		// m_end2 will be initialized by its own default constructor
{
	assert(radius >= 0.0);
	assert(display_list.isReady());

	initSurfaceNormalsList();

	assert(isInvariantTrue());
}

FixedEntity :: FixedEntity (const ObjLibrary::Vector3& end1,
                            const ObjLibrary::Vector3& end2,
                            double radius,
                            const ObjLibrary::DisplayList& display_list)
		: Entity((end1 + end2) * 0.5, radius),
		  m_display_list(display_list),
		// m_normals_list will be initialized by initNormalsList
		  m_is_sphere(false),
		  m_end1(end1),
		  m_end2(end2)
{
	assert(end1 != end2);
	assert(radius >= 0.0);
	assert(display_list.isReady());

	initSurfaceNormalsList();

	assert(isInvariantTrue());
}



bool FixedEntity :: isSphere () const
{
	assert(isInvariantTrue());

	return m_is_sphere;
}

bool FixedEntity :: isCylinder () const
{
	assert(isInvariantTrue());

	return !m_is_sphere;
}

const ObjLibrary::Vector3& FixedEntity :: getEnd1 () const
{
	assert(isInvariantTrue());
	assert(isCylinder());

	return m_end1;
}

const ObjLibrary::Vector3& FixedEntity :: getEnd2 () const
{
	assert(isInvariantTrue());
	assert(isCylinder());

	return m_end2;
}

ObjLibrary::Vector3 FixedEntity :: getDirection () const
{
	assert(isInvariantTrue());
	assert(isCylinder());

	assert(m_end1 != m_end2);
	Vector3 e2_to_e1 = m_end2 - m_end1;
	assert(!e2_to_e1.isZero());
	e2_to_e1.normalize();
	return e2_to_e1;
}

double FixedEntity :: getLength () const
{
	assert(isInvariantTrue());
	assert(isCylinder());

	return m_end2.getDistance(m_end1);
}

ObjLibrary::Vector3 FixedEntity :: getSurfaceNormal (const ObjLibrary::Vector3& query_pos) const
{
	if(isSphere())
		return getSurfaceNormalSphere(query_pos, getPosition());
	else
		return getSurfaceNormalOrientedCylinder(query_pos, getEnd1(), getEnd2());
}

bool FixedEntity :: isDrawable () const
{
	assert(isInvariantTrue());

	assert(m_display_list.isReady() == m_surface_normals_list.isReady());
	return m_display_list.isReady();
}

void FixedEntity :: draw () const
{
	assert(isInvariantTrue());
	assert(isDrawable());

	static const double RADIANS_TO_DEGREES = 180.0 / 3.1415926535897932384626433832795;

	if(m_display_list.isReady())
	{
		Vector3 center = getPosition();
		double  radius = getRadius();

		if(isSphere())
		{
			glPushMatrix();
				glTranslated(center.x, center.y, center.z);
				glScaled(radius, radius, radius);
				m_display_list.draw();
			glPopMatrix();
		}
		else
		{
			Vector3 direction = getDirection();
			double  length    = getLength();

			assert(!direction.isZero());
			Vector3 axis = Vector3::UNIT_X_PLUS.crossProduct(direction);
			axis.normalizeSafe();
			double radians = Vector3::UNIT_X_PLUS.getAngleSafe(direction);
			double degrees = radians * RADIANS_TO_DEGREES;

			glPushMatrix();
				glTranslated(center.x, center.y, center.z);
				glRotated(degrees, axis.x, axis.y, axis.z);
				glScaled(length * 0.5, radius, radius);
				m_display_list.draw();
			glPopMatrix();
		}
	}
}

void FixedEntity :: drawSurfaceNormals () const
{
	assert(isInvariantTrue());
	assert(isDrawable());

	m_surface_normals_list.draw();
}


	
void FixedEntity :: initSurfaceNormalsList ()
{
	m_surface_normals_list.begin();
	glColor3d(1.0, 1.0, 0.0);  // yellow
		glBegin(GL_LINES);
			for(unsigned int i = 0; i < 100; i++)
			{
				Vector3 query_pos = getRandomSurfacePoint();
				Vector3 normal    = getSurfaceNormal(query_pos);
				Vector3 end_at    = query_pos + normal * NORMAL_LENGTH;
				glVertex3d(query_pos.x, query_pos.y, query_pos.z);
				glVertex3d(   end_at.x,    end_at.y,    end_at.z);
			}
		glEnd();
	m_surface_normals_list.end();
}

ObjLibrary::Vector3 FixedEntity :: getRandomSurfacePoint () const
{
	static const double RADIANS_TO_DEGREES = 180.0 / 3.1415926535897932384626433832795;

	Vector3 center = getPosition();
	double  radius = getRadius();

	if(isSphere())
	{
		Vector3 direction = Vector3::getRandomUnitVector();
		return center + direction * radius;
	}
	else
	{
		// generate cylinder aligned with the X-axis
		//  -> for X, we generate 3 random numbers and throw 2 away
		Vector3 unrotated = Vector3::getRandomUnitVectorYZ() * radius;
		unrotated.x = getLength() * Vector3::getRandomInRange(-0.5, 0.5).x;

		// calculate rotation values
		Vector3 cylinder_direction = getDirection();
		assert(!cylinder_direction.isZero());
		Vector3 axis = Vector3::UNIT_X_PLUS.crossProduct(cylinder_direction);
		axis.normalizeSafe();
		double radians = Vector3::UNIT_X_PLUS.getAngleSafe(cylinder_direction);
		double degrees = radians * RADIANS_TO_DEGREES;

		// rotate cylinder
		Vector3 rotated = unrotated.getRotatedArbitraryNormal(axis, radians);
		return center + rotated;
	}
}

bool FixedEntity :: isInvariantTrue () const
{
	if(m_is_sphere)
		return true;

	if(m_end1 == m_end2)
		return false;
	if(getPosition() != (m_end1 + m_end2) * 0.5)
		return false;
	return true;
}

