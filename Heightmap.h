//
//  Heightmap.h
//

#pragma once

#include <string>
#include <vector>

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/TextureBmp.h"
#include "ObjLibrary/DisplayList.h"



class Heightmap
{
public:
	Heightmap ();  // default constructor
	Heightmap (unsigned int size_cells_x_in,
	           unsigned int size_cells_z_in);
	Heightmap (const ObjLibrary::TextureBmp& heights_image);

	unsigned int getSizeCellsX () const;
	unsigned int getSizeCellsZ () const;
	bool isInside (unsigned int x_query,
	               unsigned int z_query) const;
	bool isInside (float x, float z) const;
	float getHeight (unsigned int x_query,
	                 unsigned int z_query) const;
	float getHeight (float x, float z) const;
	ObjLibrary::Vector3 getSurfaceNormal (float x, float z) const;
	void draw () const;

	void initDisplayList (const std::string& texture_filename,
	                      float texture_offset_u,
	                      float texture_offset_v,
	                      float texture_repeat_u,
	                      float texture_repeat_v);

private:
	unsigned int size_cells_x;
	unsigned int size_cells_z;
	std::vector<std::vector<float> > heights;
	ObjLibrary::DisplayList display_list;
};


