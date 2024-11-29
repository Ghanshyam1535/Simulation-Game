//
//  SurfaceNormal.h
//
#pragma once

#include "ObjLibrary/Vector3.h"



ObjLibrary::Vector3 getSurfaceNormalGroundPlain (
        const ObjLibrary::Vector3& query_pos);
ObjLibrary::Vector3 getSurfaceNormalSphere (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center);
ObjLibrary::Vector3 getSurfaceNormalVerticalCylinder (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center);
ObjLibrary::Vector3 getSurfaceNormalOrientedCylinder (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& end1,
        const ObjLibrary::Vector3& end2);
ObjLibrary::Vector3 getSurfaceNormalCuboid (
        const ObjLibrary::Vector3& query_pos,
        const ObjLibrary::Vector3& center,
        const ObjLibrary::Vector3& half_size);
