//
//  CoordinateSystem.cpp
//

#include <cstdlib>

#include "CoordinateSystem.h"

#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"

using namespace ObjLibrary;



CoordinateSystem :: CoordinateSystem ()
		: m_position(0, 0, 0)
		, forward_vec(1, 0, 0)
		, up_vec     (0, 1, 0)
		, right_vec  (0, 0, 1)
{
}

CoordinateSystem :: CoordinateSystem (const ObjLibrary::Vector3& position)
		: m_position(position)
		, forward_vec(1, 0, 0)
		, up_vec     (0, 1, 0)
		, right_vec  (0, 0, 1)
{
}

CoordinateSystem :: CoordinateSystem (const ObjLibrary::Vector3& position,
                                      const ObjLibrary::Vector3& forward)
		: m_position(position)
		, forward_vec(forward)
		, up_vec     (calculateUpVector(forward))
		// right_vec is set below
{
	right_vec = forward.crossProduct(up_vec);
}

CoordinateSystem :: CoordinateSystem (const ObjLibrary::Vector3& position,
                                      const ObjLibrary::Vector3& forward,
                                      const ObjLibrary::Vector3& up)
		: m_position(position)
		, forward_vec(forward)
		, up_vec     (up)
		, right_vec  (forward.crossProduct(up))
{
}

const ObjLibrary::Vector3& CoordinateSystem :: getPosition () const
{
	return m_position;
}

const ObjLibrary::Vector3& CoordinateSystem :: getForward () const
{
	return forward_vec;
}

const ObjLibrary::Vector3& CoordinateSystem :: getUp () const
{
	return up_vec;
}

const ObjLibrary::Vector3& CoordinateSystem :: getRight () const
{
	return right_vec;
}

void CoordinateSystem :: setupCamera () const
{
	Vector3 look_at = m_position + forward_vec;
	gluLookAt(m_position.x, m_position.y, m_position.z,
	          look_at.x,    look_at.y,    look_at.z,
	          up_vec.x,       up_vec.y,       up_vec.z);
}

void CoordinateSystem :: applyDrawTransformations () const
{
	// code for translation will go here
	glTranslated(m_position.x, m_position.y, m_position.z);

	// code for rotation will go here
	double a_matrix[16];
	calculateOrientationMatrix(a_matrix);
	glMultMatrixd(a_matrix);
}

void CoordinateSystem :: calculateOrientationMatrix (double a_matrix[]) const
{
	a_matrix[ 0] = forward_vec.x;
	a_matrix[ 1] = forward_vec.y;
	a_matrix[ 2] = forward_vec.z;
	a_matrix[ 3] = 0.0;
	a_matrix[ 4] = up_vec.x;
	a_matrix[ 5] = up_vec.y;
	a_matrix[ 6] = up_vec.z;
	a_matrix[ 7] = 0.0;
	a_matrix[ 8] = right_vec.x;
	a_matrix[ 9] = right_vec.y;
	a_matrix[10] = right_vec.z;
	a_matrix[11] = 0.0;
	a_matrix[12] = 0.0;
	a_matrix[13] = 0.0;
	a_matrix[14] = 0.0;
	a_matrix[15] = 1.0;
}



void CoordinateSystem :: setPosition (const ObjLibrary::Vector3& position)
{
	m_position = position;
}

void CoordinateSystem :: setOrientation (const ObjLibrary::Vector3& forward,
                                         const ObjLibrary::Vector3& up)
{
	forward_vec = forward;
	up_vec      = up;
	right_vec   = forward.crossProduct(up_vec);
}

void CoordinateSystem :: setOrientation (const ObjLibrary::Vector3& forward)
{
	forward_vec = forward;
	up_vec      = calculateUpVector(forward);
	right_vec   = forward.crossProduct(up_vec);
}

void CoordinateSystem :: randomizeUp ()
{
	static const double TWO_PI = 6.283185307179586476925286766559;

	double random0to1 = rand() / (RAND_MAX + 1.0);
	double random_radians = random0to1 * TWO_PI;
	rotateAroundForward(random_radians);
}

void CoordinateSystem :: randomizeOrientation ()
{
	setOrientation(Vector3::getRandomUnitVector());
	randomizeUp();
}

void CoordinateSystem :: moveForward (double distance)
{
	m_position += forward_vec * distance;
}

void CoordinateSystem :: moveUp (double distance)
{
	m_position += up_vec * distance;
}

void CoordinateSystem :: moveRight (double distance)
{
	m_position += right_vec * distance;
}

void CoordinateSystem :: rotateAroundForward (double radians)
{
	up_vec   .rotateArbitrary(forward_vec, radians);
	right_vec.rotateArbitrary(forward_vec, radians);
}

void CoordinateSystem :: rotateAroundUp (double radians)
{
	forward_vec.rotateArbitrary(up_vec, radians);
	right_vec  .rotateArbitrary(up_vec, radians);
}

void CoordinateSystem :: rotateAroundRight (double radians)
{
	forward_vec.rotateArbitrary(right_vec, radians);
	up_vec     .rotateArbitrary(right_vec, radians);
}

void CoordinateSystem :: rotateAroundArbitrary (ObjLibrary::Vector3 axis,
                                                double radians)
{
	if(axis.isZero())
		return;

	axis.normalize();
	forward_vec.rotateArbitrary(axis, radians);
	up_vec     .rotateArbitrary(axis, radians);
	right_vec  .rotateArbitrary(axis, radians);
}

void CoordinateSystem :: rotateToVector (const ObjLibrary::Vector3& desired_forward,
                                         double max_radians)
{
	if(desired_forward.isZero())
		return;

	Vector3 axis = forward_vec.crossProduct(desired_forward);
	if(axis.isZero())
		axis = up_vec;
	else
		axis.normalize();

	double radians = forward_vec.getAngleSafe(desired_forward);
	if(radians > max_radians)
		radians = max_radians;
	forward_vec.rotateArbitrary(axis, radians);
	up_vec     .rotateArbitrary(axis, radians);
	right_vec  .rotateArbitrary(axis, radians);
}

void CoordinateSystem :: rotateToUpright (double max_radians)
{
	Vector3 desired_up = calculateUpVector(forward_vec);

	Vector3 axis = up_vec.crossProduct(desired_up);
	if(axis.isZero())
		axis = forward_vec;
	else
		axis.normalize();

	double radians = up_vec.getAngleSafe(desired_up);
	if(radians > max_radians)
		radians = max_radians;

	up_vec   .rotateArbitrary(axis, radians);
	right_vec.rotateArbitrary(axis, radians);
}



Vector3 CoordinateSystem :: calculateUpVector (const Vector3& local_forward) const
{
	static const Vector3 IDEAL_UP_VECTOR(0.0, 1.0, 0.0);
	static const double HALF_PI = 1.5707963267948966;

	if(local_forward.isZero())
		return IDEAL_UP_VECTOR;
	assert(!local_forward.isZero());

	Vector3 axis = local_forward.crossProduct(IDEAL_UP_VECTOR);
	if(axis.isZero())
		return Vector3(1.0, 0.0, 0.0);
	else
	{
		axis.normalize();
		Vector3 local_up = local_forward.getRotatedArbitrary(axis, HALF_PI);
		return local_up;
	}
}

