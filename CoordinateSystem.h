//
//  CoordinateSystem.h
//

#pragma once

#include "ObjLibrary/Vector3.h"



class CoordinateSystem
{
public:
	CoordinateSystem ();
	CoordinateSystem (const ObjLibrary::Vector3& position);
	CoordinateSystem (const ObjLibrary::Vector3& position,
	                  const ObjLibrary::Vector3& forward);
	CoordinateSystem (const ObjLibrary::Vector3& position,
	                  const ObjLibrary::Vector3& forward,
	                  const ObjLibrary::Vector3& up);

	const ObjLibrary::Vector3& getPosition () const;
	const ObjLibrary::Vector3& getForward () const;
	const ObjLibrary::Vector3& getUp () const;
	const ObjLibrary::Vector3& getRight () const;
	void setupCamera () const;  // calls gluLookAt
	void applyDrawTransformations () const;
	void calculateOrientationMatrix (double a_matrix[]) const;

	void setPosition (const ObjLibrary::Vector3& position);
	void setOrientation (const ObjLibrary::Vector3& local_forward);
	void setOrientation (const ObjLibrary::Vector3& forward,
	                     const ObjLibrary::Vector3& up);
	void randomizeUp ();
	void randomizeOrientation ();
	void moveForward (double distance);
	void moveUp (double distance);
	void moveRight (double distance);
	void rotateAroundForward (double radians);
	void rotateAroundUp (double radians);
	void rotateAroundRight (double radians);
	void rotateAroundArbitrary (ObjLibrary::Vector3 axis,
	                            double radians);
	void rotateToVector (const ObjLibrary::Vector3& desired_forward,
	                     double max_radians);
	void rotateToUpright (double max_radians);

private:
	ObjLibrary::Vector3 calculateUpVector (const ObjLibrary::Vector3& local_forward) const;

private:
	ObjLibrary::Vector3 m_position;
	ObjLibrary::Vector3 forward_vec; 
	ObjLibrary::Vector3 up_vec;
	ObjLibrary::Vector3 right_vec;
};


