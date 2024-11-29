//
//  Terrain.h
//
//  A module to represent the ground terrain for the map.
//

#pragma once

#include <string>

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/TextureBmp.h"
#include "ObjLibrary/DisplayList.h"

#include "Heightmap.h"



//
//  Terrain.h
//
//  A class to represent the ground terrain for the map.  This
//    includes both above water and underwater.  This class
//    handles offsets and scaling.
//
//  Class Invariant:
//    <1> m_scale.isAllComponentsPositive()
//
class Terrain
{
public:
//
//  isPlantLoaded
//
//  Purpose: To determine if the plant OBJ model has been
//           loaded.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the plant OBJ model has been loaded.
//  Side Effect: N/A
//
	static bool isPlantLoaded ();

//
//  loadPlant
//
//  Purpose: To load the plant OBJ model.  This function should
//           be called once at game start after OpenGL is
//           initialized but before the map is displayed.
//  Parameter(s):
//    <1> resource_path: The file path to prepend to the
//                       filenames
//  Precondition(s):
//    <1> !isPlantLoaded()
//  Returns: N/A
//  Side Effect: The plant OpenGL model is loaded and converted
//               to a display list.  After this function has
//               been called, isPlantLoaded will return true.
//
	static void loadPlant (const std::string& resource_path);

public:
//
//  Default Constructor
//
//  Purpose: To create an invalid Terrain.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A Terrain is created that does not contain any
//               heightmaps or plants.  This terrain cannot be
//               displayed.
//
	Terrain ();

//
//  Constructor
//
//  Purpose: To create a Terrain based on an image file.
//  Parameter(s):
//    <1> resource_path: The file path to prepend to the
//                       filenames
//    <2> heights_texture: The name of the image file to load
//                         the heightmap heights from
//    <3> underwater_texture: The texture to display the
//                            underwater terrain with
//    <4> above_water_texture: The texture to display the
//                             above-water terrain with
//    <5> offset: The XYZ offset to display the heightmap at
//    <6> size: The XYZ dimensions of the heightmap when
//              displayed
//  Precondition(s):
//    <1> isPlantLoaded()
//    <2> heights_texture != ""
//    <3> underwater_texture != ""
//    <4> above_water_texture != ""
//    <5> size.isAllComponentsPositive()
//  Returns: N/A
//  Side Effect: A Terrain is created based on heights_texture.
//               It can be displayed.
//
	Terrain (const std::string& resource_path,
	         const std::string& heights_texture,
	         const std::string& underwater_texture,
	         const std::string& above_water_texture,
	         const ObjLibrary::Vector3& offset,
	         const ObjLibrary::Vector3& size);

//
//  isReadyToDraw
//
//  Purpose: Whether this Terrain can be dispayed.
//  Parameter(s):
//    <1> check_at: The position to test
//  Precondition(s): N/A
//  Returns: Whether this Terrain was correctly initialized and
//           is now ready to draw.
//  Side Effect: N/A
//
	bool isReadyToDraw () const;

//
//  isInside
//
//  Purpose: To determine whether a horizontal position is
//           within the area covered by this Terrain.
//  Parameter(s):
//    <1> check_at: The position to test
//  Precondition(s): N/A
//  Returns: Whether the XZ components of check_at are within
//           the bounds of this Terrain.
//  Side Effect: N/A
//
	bool isInside (const ObjLibrary::Vector3& check_at) const;

//
//  getHeight
//
//  Purpose: To determine the height of this Terrain at the
//           specified horizontal position.
//  Parameter(s):
//    <1> check_at: The position to test
//  Precondition(s): N/A
//  Returns: If the XZ components of check_at are within the
//           bounds of this Terrain, the terrrain height at that
//           XZ position is returned.  Otherwise, 0.0 is
//           returned.
//  Side Effect: N/A
//
	double getHeight (
	                 const ObjLibrary::Vector3& check_at) const;

//
//  getSurfaceNormal
//
//  Purpose: To determine the surface normal of this Terrain at
//           the specified horizontal position.
//  Parameter(s):
//    <1> check_at: The position to test
//  Precondition(s): N/A
//  Returns: If the XZ components of check_at are within the
//           bounds of this Terrain, the surface normal at that
//           XZ position is returned.  Otherwise, (0, 1, 0) is
//           returned.
//  Side Effect: N/A
//
	ObjLibrary::Vector3 getSurfaceNormal (
	                 const ObjLibrary::Vector3& check_at) const;

//
//  draw
//
//  Purpose: To display this Terrain.
//  Parameter(s):
//    <1> is_underwater: Whether the camera is underwater
//  Precondition(s):
//    <1> isReadyToDraw()
//  Returns: N/A
//  Side Effect: The underwater portion of this Terrain is
//               displayed.  The plants are displayed.  If
//               is_underwater == false, the above-water portion
//               of this Terrain is also displayed.
//
	void draw (bool is_underwater) const;

//
//  draw
//
//  Purpose: To display the surface normals for this Terrain.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isReadyToDraw()
//  Returns: N/A
//  Side Effect: The surface normals portion of this Terrain are
//               displayed.  They are yellow and each has a
//               length of 0.5.
//
	void drawSurfaceNormals () const;

private:
//
//  initAllPlantsList
//
//  Purpose: To initialize the combined display list for all the
//           plants on this Terrain.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isPlantLoaded()
//  Returns: N/A
//  Side Effect: The plants display list is initialized.
//
	void initAllPlantsList ();

//
//  initSurfaceNormalsList
//
//  Purpose: To initialize the display list for the surface
//           normals for this Terrain.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The surface normals display list is
//               initialized.
//
	void initSurfaceNormalsList ();

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
	ObjLibrary::TextureBmp m_heights_texture;
	Heightmap m_underwater;
	Heightmap m_above_water;
	ObjLibrary::Vector3 m_offset;
	ObjLibrary::Vector3 m_scale;
	ObjLibrary::DisplayList m_all_plants_list;
	ObjLibrary::DisplayList m_surface_normals_list;
};



