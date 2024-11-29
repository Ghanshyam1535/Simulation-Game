//
//  Terrain.cpp
//

#include "Terrain.h"

#include <cassert>
#include <string>

#include "GetGlut.h"

#include "ObjLibrary/Vector2.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"
#include "ObjLibrary/TextureBmp.h"
#include "ObjLibrary/TextureManager.h"

#include "Heightmap.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	DisplayList plant_list;

	const double NORMAL_LENGTH = 0.5;

}  // end of anonymous namespace



bool Terrain :: isPlantLoaded ()
{
	return plant_list.isReady();
}

void Terrain :: loadPlant (const std::string& resource_path)
{
	assert(!isPlantLoaded());

	TextureManager::load(resource_path + "green_algae2.bmp",
	                     GL_CLAMP, GL_CLAMP,
	                     GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST,
	                     Vector3(0.0, 0.0, 0.0));
	plant_list = ObjModel(resource_path + "algae.obj").getDisplayList();

	assert(isPlantLoaded());
}



Terrain :: Terrain ()
		: m_offset(0.0, 0.0, 0.0),
		  m_scale(1.0, 1.0, 1.0)
{
	assert(!isReadyToDraw());
	assert(isInvariantTrue());
}

Terrain :: Terrain (const std::string& resource_path,
                    const std::string& heights_texture,
                    const std::string& underwater_texture,
                    const std::string& above_water_texture,
                    const ObjLibrary::Vector3& offset,
                    const ObjLibrary::Vector3& size)
		: m_heights_texture(resource_path + heights_texture)
{
	assert(isPlantLoaded());
	assert(heights_texture != "");
	assert(underwater_texture != "");
	assert(above_water_texture != "");
	assert(size.isAllComponentsPositive());

	m_underwater = Heightmap(m_heights_texture);
	m_underwater.initDisplayList(resource_path + underwater_texture, 0.0f, 0.0f, 15.0f, 15.0f);

	m_above_water = m_underwater;
	m_above_water.initDisplayList(resource_path + above_water_texture, 0.0f, 0.0f, 40.0f, 40.0f);

	m_offset = offset;

	m_scale.x = size.x / m_underwater.getSizeCellsX();
	m_scale.y = size.y;
	m_scale.z = size.z / m_underwater.getSizeCellsZ();

	initAllPlantsList();
	initSurfaceNormalsList();

	assert(isReadyToDraw());
	assert(isInvariantTrue());
}



bool Terrain :: isReadyToDraw () const
{
	assert(isInvariantTrue());

	assert(m_surface_normals_list.isReady() == m_all_plants_list.isReady());
	return m_all_plants_list.isReady();
}

bool Terrain :: isInside (const ObjLibrary::Vector3& check_at) const
{
	assert(isInvariantTrue());

	Vector3 local_pos = check_at - m_offset;
	local_pos = local_pos.getComponentRatioSafe(m_scale);

	float float_x = (float)(local_pos.x);
	float float_z = (float)(local_pos.z);
	return m_underwater.isInside(float_x, float_z);
}

double Terrain :: getHeight (const ObjLibrary::Vector3& check_at) const
{
	assert(isInvariantTrue());

	Vector3 local_pos = check_at - m_offset;
	local_pos = local_pos.getComponentRatioSafe(m_scale);

	float float_x = (float)(local_pos.x);
	float float_z = (float)(local_pos.z);
	if(m_underwater.isInside(float_x, float_z))
	{
		float raw_height = m_underwater.getHeight(float_x, float_z);
		return raw_height * m_scale.y + m_offset.y;
	}
	else
		return 0.0;
}

ObjLibrary::Vector3 Terrain :: getSurfaceNormal (const ObjLibrary::Vector3& check_at) const
{
	assert(isInvariantTrue());

	Vector3 local_pos = check_at - m_offset;
	local_pos = local_pos.getComponentRatioSafe(m_scale);

	float float_x = (float)(local_pos.x);
	float float_z = (float)(local_pos.z);
	if(m_underwater.isInside(float_x, float_z))
	{
		Vector3 normal = m_underwater.getSurfaceNormal(float_x, float_z);
		normal = normal.getComponentRatio(m_scale);
		normal.normalize();
		return normal;
	}
	else
		return Vector3::UNIT_Y_PLUS;
}

void Terrain :: draw (bool is_underwater) const
{
	assert(isInvariantTrue());
	assert(isReadyToDraw());

	glPushMatrix();
		glTranslated(m_offset.x, m_offset.y, m_offset.z);
		glScaled(m_scale.x, m_scale.y, m_scale.z);
		m_underwater.draw();
	glPopMatrix();

	if(!is_underwater)
	{
		glDisable(GL_FOG);
		glPushMatrix();
			glTranslated(m_offset.x, m_offset.y * 1.1, m_offset.z);
			glScaled(m_scale.x, m_scale.y * 1.1, m_scale.z);
			m_above_water.draw();
		glPopMatrix();
		glEnable(GL_FOG);
	}

	m_all_plants_list.draw();
}

void Terrain :: drawSurfaceNormals () const
{
	assert(isInvariantTrue());
	assert(isReadyToDraw());

	m_surface_normals_list.draw();
}



void Terrain :: initAllPlantsList ()
{
	assert(isPlantLoaded());

	m_all_plants_list.begin();
		for(unsigned int x = 0; x < m_underwater.getSizeCellsX(); x++)
			for(unsigned int z = 0; z < m_underwater.getSizeCellsZ(); z++)
				if(m_heights_texture.getGreen(x, z) >= 64)
				{
					double y = m_underwater.getHeight(x, z);
					Vector3 position = m_offset + Vector3(x, y, z).getComponentProduct(m_scale);
					glPushMatrix();
						glTranslated(position.x, position.y, position.z);
						plant_list.draw();
					glPopMatrix();
				}
	m_all_plants_list.end();
}

void Terrain :: initSurfaceNormalsList ()
{
	m_surface_normals_list.begin();
		glColor3ub(255, 255, 0);  // yellow
		glBegin(GL_LINES);
			for(float x = 0.25f; x < m_underwater.getSizeCellsX(); x += 0.5f)
				for(float z = 0.25f; z < m_underwater.getSizeCellsZ(); z += 0.5f)
				{
					float x_frac = fmod(x, 1.0f);
					float z_frac = fmod(z, 1.0f);
					if(fabs(x_frac - z_frac) < 0.0001f)
						continue;

					float y = m_underwater.getHeight(x, z);

					Vector3 pos(x, y, z);
					pos = pos.getComponentProduct(m_scale);
					pos += m_offset;
					glVertex3d(pos.x, pos.y, pos.z);

					Vector3 dir = m_underwater.getSurfaceNormal(x, z);
					dir = dir.getComponentRatioSafe(m_scale);
					dir.normalize();
					Vector3 end = pos + dir * 0.5;
					glVertex3d(end.x, end.y, end.z);
				}
		glEnd();
	m_surface_normals_list.end();
}

bool Terrain :: isInvariantTrue () const
{
	if(!m_scale.isAllComponentsPositive())
		return false;
	return true;
}


