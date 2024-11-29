//
//  SurfaceNormal.cpp
//

#include "SurfaceNormal.h"

#include "ObjLibrary/Vector3.h"

using namespace ObjLibrary;



ObjLibrary::Vector3 getSurfaceNormalGroundPlain (
        const ObjLibrary::Vector3& query_pos)
{
	return Vector3(0.0, 1.0, 0.0);
}

ObjLibrary::Vector3 getSurfaceNormalSphere (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center)
{
	Vector3 surface_normal;
	surface_normal = query_pos - center;
	surface_normal.normalizeSafe();
	return surface_normal;
}

ObjLibrary::Vector3 getSurfaceNormalVerticalCylinder (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center)
{
	Vector3 surface_normal;
	surface_normal = query_pos - center;
	surface_normal.y = 0;
	surface_normal.normalizeSafe();
	return surface_normal;
}

ObjLibrary::Vector3 getSurfaceNormalOrientedCylinder (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& end1,
        const ObjLibrary::Vector3& end2)
{
	Vector3 surface_normal;
	Vector3 cylinder_direction = end2 - end1;
	Vector3 to_query = query_pos - end1;
	surface_normal = to_query.getRejectionSafe(cylinder_direction);
	surface_normal.normalizeSafe();
	return surface_normal;
}

ObjLibrary::Vector3 getSurfaceNormalCuboid (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center,
        const ObjLibrary::Vector3& half_size)
{
	Vector3 signed_dist   = query_pos - center;
	Vector3 unsigned_dist = fabs(signed_dist) - half_size;

	if(unsigned_dist.x > unsigned_dist.y &&
		unsigned_dist.x > unsigned_dist.z)
	{
		// X axis is longest
		if(signed_dist.x > 0.0)
			return Vector3( 1.0, 0.0, 0.0);  // +X axis
		else
			return Vector3(-1.0, 0.0, 0.0);  // -X axis
	}
	else if(unsigned_dist.y > unsigned_dist.z)
	{
		// Y axis is longest
		if(signed_dist.y > 0.0)
			return Vector3(0.0,  1.0, 0.0);  // +Y axis
		else
			return Vector3(0.0, -1.0, 0.0);  // -Y axis
	}
	else
	{
		// Z axis is longest
		if(signed_dist.z > 0.0)
			return Vector3(0.0, 0.0,  1.0);  // +Z axis
		else
			return Vector3(0.0, 0.0, -1.0);  // -Z axis
	}
}

