//
//  Heightmap.cpp
//

#include "Heightmap.h"

#include <string>
#include <cassert>
#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/TextureBmp.h"
#include "ObjLibrary/TextureManager.h"
#include "ObjLibrary/DisplayList.h"

using namespace std;
using namespace ObjLibrary;



Heightmap :: Heightmap ()
{
	size_cells_x = 1;
	size_cells_z = 1;
	heights.assign(size_cells_x + 1, vector<float>(size_cells_z + 1, 0.0f));
}

Heightmap :: Heightmap (unsigned int size_cells_x_in,
                        unsigned int size_cells_z_in)
{
	size_cells_x = size_cells_x_in;
	size_cells_z = size_cells_z_in;
	heights.assign(size_cells_x + 1, vector<float>(size_cells_z + 1, 0.0f));

	for(unsigned int i = 0; i <= size_cells_x; i++)  // x
	{
		for(unsigned int k = 0; k <= size_cells_z; k++)  // z
		{
			heights[i][k] = (i % 2) * 0.5f -
			                 k * k  * 0.05f;
		}
	}
}

Heightmap :: Heightmap (const ObjLibrary::TextureBmp& heights_image)
{
	size_cells_x = heights_image.getWidth() - 1;
	size_cells_z = heights_image.getHeight() - 1;
	heights.assign(size_cells_x + 1, vector<float>(size_cells_z + 1, 0.0f));

	for(unsigned int i = 0; i <= size_cells_x; i++)  // x
	{
		for(unsigned int k = 0; k <= size_cells_z; k++)  // z
		{
            unsigned char red = heights_image.getRed(i, k);
            heights[i][k] = red / 255.0f;
		}
	}
}

unsigned int Heightmap :: getSizeCellsX () const
{
  return size_cells_x;
}

unsigned int Heightmap :: getSizeCellsZ () const
{
  return size_cells_z;
}

bool Heightmap :: isInside (unsigned int x_query,
                            unsigned int z_query) const
{
	if(x_query > size_cells_x)
		return false;
	if(z_query > size_cells_z)
		return false;
	return true;
}

bool Heightmap :: isInside (float x, float z) const
{
	if(x < 0.0f)
		return false;
	if(x > getSizeCellsX())
		return false;
	if(z < 0.0f)
		return false;
	if(z > getSizeCellsZ())
		return false;
	return true;
}

float Heightmap :: getHeight (unsigned int x_query,
                              unsigned int z_query) const
{
	return heights[x_query][z_query];
}

float Heightmap :: getHeight (float x, float z) const
{
	assert(isInside(x, z));

	unsigned int i0 = (unsigned int)(x);
	unsigned int k0 = (unsigned int)(z);
	unsigned int i1 = i0 + 1;
	unsigned int k1 = k0 + 1;
	float i_frac = x - i0;
	float k_frac = z - k0;

	if(i_frac > k_frac)
	{
		float weight00 = 1.0f - i_frac;
		float weight11 = k_frac;
		float weight10 = 1.0f - weight00 - weight11;

		float height00 = getHeight(i0, k0);
		float height11 = getHeight(i1, k1);
		float height10 = getHeight(i1, k0);

		return weight00 * height00 +
		       weight11 * height11 +
		       weight10 * height10;
	}
	else
	{
		float weight00 = 1.0f - k_frac;
		float weight11 = i_frac;
		float weight01 = 1.0f - weight00 - weight11;
	   
		float height00 = getHeight(i0, k0);
		float height11 = getHeight(i1, k1);
		float height01 = getHeight(i0, k1);
	   
		return weight00 * height00 +
		       weight11 * height11 +
		       weight01 * height01;
	}
}

ObjLibrary::Vector3 Heightmap::getSurfaceNormal (float x, float z) const
{
	assert(isInside(x, z));

	unsigned int i0 = (unsigned int)(x);
	unsigned int k0 = (unsigned int)(z);
	unsigned int i1 = i0 + 1;
	unsigned int k1 = k0 + 1;
	float i_frac = x - i0;
	float k_frac = z - k0;

	if(i_frac > k_frac)
	{
		// in pink triangle

		float height00 = getHeight(i0, k0);
		float height11 = getHeight(i1, k1);
		float height10 = getHeight(i1, k0);
	   
		Vector3 v00(i0, height00, k0);
		Vector3 v11(i1, height11, k1);
		Vector3 v10(i1, height10, k0);

		Vector3 v10_to_v00 = v00 - v10;
		Vector3 v10_to_v11 = v11 - v10;
		Vector3 up = v10_to_v00.crossProduct(v10_to_v11);

		up.normalizeSafe();
		return up;
	}
	else
	{
		// in green triangle

		float height00 = getHeight(i0, k0);
		float height11 = getHeight(i1, k1);
		float height01 = getHeight(i0, k1);
	   
		Vector3 v00(i0, height00, k0);
		Vector3 v11(i1, height11, k1);
		Vector3 v01(i0, height01, k1);
	   
		Vector3 v01_to_v00 = v00 - v01;
		Vector3 v01_to_v11 = v11 - v01;
		Vector3 up = v01_to_v11.crossProduct(v01_to_v00);
	   
		up.normalizeSafe();
		return up;
	}
}

void Heightmap :: draw () const
{
	display_list.draw();
}

void Heightmap :: initDisplayList (const std::string& texture_filename,
                                   float texture_offset_u,
                                   float texture_offset_v,
                                   float texture_repeat_u,
                                   float texture_repeat_v)
{
	TextureManager::activate(texture_filename);
	display_list.begin();
		glEnable(GL_TEXTURE_2D);
		TextureManager::activate(texture_filename);
		glColor3d(1.0, 1.0, 1.0);
		for(unsigned int i0 = 0; i0 < size_cells_x; i0++)  // x
		{
			unsigned int i1 = i0 + 1;
			float tex_i0 = texture_repeat_u * (float)(i0) / size_cells_x + texture_offset_u;
			float tex_i1 = texture_repeat_u * (float)(i1) / size_cells_x + texture_offset_u;
			glBegin(GL_TRIANGLE_STRIP);
				for(unsigned int k = 0; k <= size_cells_z; k++)  // z
				{
					float tex_k = texture_repeat_v * (float)(k) / size_cells_z + texture_offset_v;
					glTexCoord2d(tex_i1, tex_k);
					glVertex3d(i1, heights[i1][k], k);
					glTexCoord2d(tex_i0, tex_k);
					glVertex3d(i0, heights[i0][k], k);
				}
			glEnd();
		}
		glDisable(GL_TEXTURE_2D);
	display_list.end();
}
