//
//  Collision.cpp
//

#include "Collision.h"

#include "ObjLibrary/Vector3.h"

#include "Entity.h"
#include "FixedEntity.h"
#include "Terrain.h"

using namespace ObjLibrary;



bool isCollision (const Entity& entity1,
                  const Entity& entity2)
{
	const Vector3& p1 = entity1.getPosition();
	const Vector3& p2 = entity2.getPosition();
	double distance = p1.getDistance(p2);

	double radius_sum = entity1.getRadius() + entity2.getRadius();
	if(distance < radius_sum)
		return true;
	else
		return false;
}

bool isCollision (const Entity& entity1,
                  const FixedEntity& entity2_fixed)
{
	if(entity2_fixed.isSphere())
	{
		//
		// spheres should be easy
		//  -> we just call the Entity-Entity function
		//
		// but the compiler will want to call this function
		//  -> it sees that entity2 has type FixedEntity
		//  -> so we lie to it and say entity2 is of type Entity
		//  -> this is safe because EntityFixed is derived from Entity
		//  -> the compiler would stop us if the type conversions weren't valid
		//

		const Entity& entity2_unfixed = entity2_fixed;  // treat entity2_fixed as Entity type
		return isCollision(entity1, entity2_unfixed);   // 2 Entitys: calls Entity-Entity function
	}
	else
	{
		assert(entity2_fixed.isCylinder());

		Vector3 end1_to_center = entity1.getPosition() - entity2_fixed.getEnd1();
		Vector3 direction2     = entity2_fixed.getDirection();

		Vector3 rejection  = end1_to_center.getRejection(direction2);
		double  radius_sum = entity1.getRadius() + entity2_fixed.getRadius();
		if(rejection.getNorm() > radius_sum)
			return false;  // too far from cylinder

		Vector3 projection = end1_to_center.getProjection(direction2);
		if(!projection.isSameDirection(direction2))
			return false;  // off end1 end
		double length2 = entity2_fixed.getLength();
		if(projection.getNorm() > length2)
			return false;  // off end2 end

		return true;  // must be a collision
	}
}

bool isCollision (const FixedEntity& entity1_fixed,
                  const Entity& entity2)
{
	//
	// just call the other function
	//  -> we don't want to duplicate the code
	//

	return isCollision(entity2, entity1_fixed);
}

bool isCollision (const Entity& entity,
                  const Terrain& terrain)
{
	Vector3 position = entity.getPosition();
	return terrain.getHeight(position) + entity.getRadius() > position.y;
}




