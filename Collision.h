//
//  Collision.h
//
//  A module to detect collisions between Entities.
//

#pragma once

#include "ObjLibrary/Vector3.h"

class Entity;
class FixedEntity;
class Terrain;



bool isCollision (const Entity& entity1,
                  const Entity& entity2);
bool isCollision (const Entity& entity1,
                  const FixedEntity& entity2_fixed);
bool isCollision (const FixedEntity& entity1_fixed,
                  const Entity& entity2);
bool isCollision (const Entity& entity,
                  const Terrain& terrain);
