//
//  FixedEntity.h
//
//  A module to store a fixed entity on the map.
//

#pragma once

#include "Entity.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "CoordinateSystem.h"



//
//  FixedEntity
//
//  A class to store a fixed entity on the map, meaning an
//    entity that never changes.  An FixedEntity can be a sphere
//    or an oriented cylinder.
//
//  Class Invariant:
//    <1> m_is_sphere || m_end1 != m_end2
//    <2> m_is_sphere || getPosition() == (m_end1 + m_end2) * 0.5
//
class FixedEntity : public Entity
{
public:
//
//  Default Constructor
//
//  Purpose: To construct a default FixedEntity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A spherical FixedEntity with radius 0.0 is
//               constructed at the origin.  It will not be
//               displayed.
//
	FixedEntity ();

//
//  Sphere Constructor
//
//  Purpose: To construct an spherical FixedEntity.
//  Parameter(s):
//    <1> center: The center of the sphere
//    <2> radius: The radius of the sphere
//    <3> display_list: The DisplayList to use to display this
//                      FixedEntity
//  Precondition(s):
//    <1> radius >= 0.0
//    <2> display_list.isReady()
//  Returns: N/A
//  Side Effect: A spherical FixedEntity with radius radius is
//               constructed at position center.  It will be
//               displayed using display_list.
//
	FixedEntity (const ObjLibrary::Vector3& center,
	             double radius,
	             const ObjLibrary::DisplayList& display_list);

//
//  Cylinder Constructor
//
//  Purpose: To construct an cylindrical FixedEntity.
//  Parameter(s):
//    <1> end1: The center of one end of the cylinder
//    <2> end2: The center of the other end of the cylinder
//    <3> radius: The radius of the cylinder
//    <4> display_list: The DisplayList to use to display this
//                      FixedEntity
//  Precondition(s):
//    <1> end1 != end2
//    <2> radius >= 0.0
//    <3> display_list.isReady()
//  Returns: N/A
//  Side Effect: A cylindrical FixedEntity with radius radius is
//               constructed stretching between positions end1
//               and end2.  It will be displayed using
//               display_list.
//
	FixedEntity (const ObjLibrary::Vector3& end1,
	             const ObjLibrary::Vector3& end2,
	             double radius,
	             const ObjLibrary::DisplayList& display_list);

//
//  isSphere
//
//  Purpose: To determine if this FixedEntity is a sphere.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether this FixedEntity is a sphere.  If not, it is a
//           cylinder.
//  Side Effect: N/A
//
	bool isSphere () const;

//
//  isCylinder
//
//  Purpose: To determine if this FixedEntity is a cylinder.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether this FixedEntity is a cylinder.  If not, it is a
//           sphere.
//  Side Effect: N/A
//
	bool isCylinder () const;

//
//  getEnd1
//
//  Purpose: To determine the center of the first end of this
//           cylinder FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isCylinder()
//  Returns: The center of the the first end of this cylinder.
//  Side Effect: N/A
//
	const ObjLibrary::Vector3& getEnd1 () const;

//
//  getEnd2
//
//  Purpose: To determine the center of the second end of this
//           cylinder FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isCylinder()
//  Returns: The center of the the second end of this cylinder.
//  Side Effect: N/A
//
	const ObjLibrary::Vector3& getEnd2 () const;

//
//  getDirection
//
//  Purpose: To determine the direction along this cylinder
//           FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isCylinder()
//  Returns: A direction vector from first end to the second end
//           of this FixedEntity.  The result is a normal
//           vector.
//  Side Effect: N/A
//
	ObjLibrary::Vector3 getDirection () const;

//
//  getLength
//
//  Purpose: To determine the length of this cylinder
//           FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isCylinder()
//  Returns: The distance between the two ends of this
//           FixedEntity.
//  Side Effect: N/A
//
	double getLength () const;

//
//  getSurfaceNormal
//
//  Purpose: To determine the surface normal of this FixedEntity
//           at the specified position.
//  Parameter(s): N/A
//    <1> query_pos: The position in world coordinates to query
//                   the normal at
//  Precondition(s): N/A
//  Returns: The surface normal vector for this FixedEntity at
//           position query_pos.  Note that surface normals are
//           defined for all positions, not just on the surface.
//  Side Effect: N/A
//
	ObjLibrary::Vector3 getSurfaceNormal (
	                const ObjLibrary::Vector3& query_pos) const;

//
//  isDrawable
//
//  Purpose: To determine if this Entity can be displayed.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether a DisplayList is set for this Entity.
//  Side Effect: N/A
//
	bool isDrawable () const;

//
//  draw
//
//  Purpose: To display this FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isDrawable()
//  Returns: N/A
//  Side Effect: This FixedEntity is displayed.  If no DisplayList
//               has been set, there is no effect.
//
	void draw () const;

//
//  drawSurfaceNormals
//
//  Purpose: To display the surface normals for this
//           FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isDrawable()
//  Returns: N/A
//  Side Effect: Surface normals for this FixedEntity are
//               displayed.
//
	void drawSurfaceNormals () const;

private:
//
//  initSurfaceNormalsList
//
//  Purpose: To initialize the surface normal display list for
//           this FixedEntity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The surface normals display list for this
//               FixedEntity is initialized.  It includes the
//               world transformations and the colour.
//
	void initSurfaceNormalsList ();

//
//  getRandomSurfacePoint
//
//  Purpose: To calculate a random point on the surface of this
//           FixedEntity.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: A random position in world coordinates that lies on
//           the surface of this FoxedEntity.  This function
//           works for spheres and cylinders.
//  Side Effect: N/A
//
	ObjLibrary::Vector3 getRandomSurfacePoint () const;

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
	ObjLibrary::DisplayList m_display_list;
	ObjLibrary::DisplayList m_surface_normals_list;
	bool m_is_sphere;
	ObjLibrary::Vector3 m_end1;
	ObjLibrary::Vector3 m_end2;
};




