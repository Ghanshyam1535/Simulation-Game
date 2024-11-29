//
//  Map.cpp

#include "Map.h"

#include <cassert>
#include <cstdlib>
#include <climits>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>

#include "GetGlut.h"

#include "ObjLibrary/Vector2.h"
#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"
#include "ObjLibrary/TextureManager.h"

#include "TimeManager.h"
#include "Terrain.h"
#include "CoordinateSystem.h"
#include "Entity.h"
#include "FixedEntity.h"
#include "Fish.h"
#include "FishSchool.h"
#include "Collision.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	DisplayList skybox_list;
	DisplayList surface_list;

	//
	//  DisplayListMap
	//
	//  A type to represent an unordered mapping of DisplayLists
	//    using OBJ model file names as key values.
	//
	//  When you need a display list, first check if it is in
	//    the mapping.  If not, load the model and add the
	//    display list to the mapping.  Then, in either case,
	//    use the display list from the mapping.
	//
	using DisplayListMap = std::unordered_map<std::string, DisplayList>;

	DisplayListMap sphere_lists;
	DisplayListMap cylinder_lists;

	const double MAX_DEPTH = 30.0;
	const double PLAYER_DRAG = 0.6;

}  // end of anonymous namespace



bool Map :: isModelsLoaded ()
{
	return skybox_list.isReady();
}

void Map :: loadModels (const std::string& resource_path)
{
	assert(!isModelsLoaded());

	skybox_list  = ObjModel(resource_path + "Skybox.obj").getDisplayList();
	surface_list = ObjModel(resource_path + "surface.obj").getDisplayList();

	TextureManager::load(resource_path + "anemone.bmp",
	                     GL_CLAMP, GL_CLAMP,
	                     GL_NEAREST, GL_NEAREST_MIPMAP_NEAREST,
	                     Vector3(0.0, 0.0, 0.0));

	assert(!Terrain::isPlantLoaded());
	Terrain::loadPlant(resource_path);
	assert(Terrain::isPlantLoaded());

	assert(!Fish::isModelsLoaded());
	Fish::loadModels(resource_path);
	assert(Fish::isModelsLoaded());

	assert(isModelsLoaded());
}



Map :: Map ()
		: m_player(),
		  m_fish_caught_count(0)
{
	resetPlayer();
}

Map :: Map (const std::string& resource_path,
            const std::string& filename)
		: m_player(),
		  m_fish_caught_count(0)
{
	assert(isModelsLoaded());
	assert(Fish::isModelsLoaded());

	loadEntities(resource_path, filename);
	resetPlayer();
}



const ObjLibrary::Vector3& Map :: getPlayerPosition () const
{
	return m_player.getPosition();
}

const CoordinateSystem& Map :: getPlayerCoords () const
{
	return m_player;  // implict typecast to base type CoordinateSystem
}

const ObjLibrary::Vector3& Map :: getPlayerVelocity () const
{
	return m_player.getVelocity();
}

std::string Map :: getPlayerAutopilotName () const
{
	return m_player.getAutopilotName();
}

unsigned int Map :: getFishCaughtCount () const
{
	return m_fish_caught_count;
}

bool Map :: isPlayerUnderwater () const
{
	return m_player.isUnderwater();
}

bool Map :: isCameraUnderwater () const
{
	return m_player.isUnderwater();
}

double Map :: getSeafloorDepth () const
{
	return m_terrain.getHeight(m_player.getPosition());
}

unsigned int Map :: getFixedEntityCount () const
{
	return mv_fixed_entities.size();
}

unsigned int Map :: getFishSchoolCount () const
{
	return mv_fish_schools.size();
}

const FishSchool& Map :: getFishSchool (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	return mv_fish_schools[fish_school_index];
}

unsigned int Map :: findNearestFixedEntity (const Vector3& search_from) const
{
	unsigned int nearest_entity = NOT_FOUND;
	double nearest_distance = 1.0e40;  // very far away

	for(unsigned int i = 0; i < mv_fixed_entities.size(); i++)
	{
		double current_distance = search_from.getDistance(mv_fixed_entities[i].getPosition());
		if(current_distance < nearest_distance)
		{
			nearest_entity   = i;
			nearest_distance = current_distance;
		}
	}

	return nearest_entity;
}

unsigned int Map :: findNearestSchool (const Vector3& search_from,
                                       bool exclude_empty) const
{
	unsigned int nearest_school = NOT_FOUND;
	double nearest_distance = 1.0e40;  // very far away

	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
	{
		// optionally skip schools containing no fish
		if(exclude_empty)
			if(mv_fish_schools[i].getCount() == 0)
				continue;

		double current_distance = search_from.getDistance(mv_fish_schools[i].getPosition());
		if(current_distance < nearest_distance)
		{
			nearest_school   = i;
			nearest_distance = current_distance;
		}
	}

	return nearest_school;
}

void Map :: updateFog () const
{
	double player_y = m_player.getPosition().y;
	if(!isCameraUnderwater())
		player_y = 0.0;

	double depth_fraction = -player_y / MAX_DEPTH;
	double green = 0.75 - 0.7 * depth_fraction;
	double blue  = 1.0  - 0.5 * depth_fraction * depth_fraction;

	float fog_color[4] = {	0.0f, (float)(green), (float)(blue), 0.0f	};
	glFogfv(GL_FOG_COLOR, fog_color);
	glClearColor(fog_color[0], fog_color[1], fog_color[2], fog_color[3]);
}

void Map :: draw () const
{
	glLoadIdentity();
	m_player.setupCamera();
	// camera is now set up - any drawing before here will display incorrectly

	// draw skybox - must be first of 3D drawing
	if(!isCameraUnderwater())
		drawSkybox();

	// display positive X, Y, and Z axes near origin
	//drawAxes();

	m_terrain.draw(isCameraUnderwater());
	drawEntites();

	// must be last of 3D drawing
	drawSurface();
}

void Map :: drawTerrainSurfaceNormals () const
{
	m_terrain.drawSurfaceNormals();
}

void Map :: drawFixedEntitySurfaceNormals (unsigned int fixed_entity_index) const
{
	assert(fixed_entity_index < getFixedEntityCount());

	mv_fixed_entities[fixed_entity_index].drawSurfaceNormals();
}

void Map :: drawFishSchoolSphere (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	const FishSchool& school = mv_fish_schools[fish_school_index];
	Vector3 position = school.getPosition();
	double  radius   = school.getRadius();

	if(isCollision(m_player, school))
		glColor3d(1.0, 1.0, 1.0);
	else
		glColor3d(1.0, 0.0, 1.0);

	glPushMatrix();
		glTranslated(position.x, position.y, position.z);
		glutWireSphere(radius, 12, 8);
	glPopMatrix();
}

void Map :: drawFishCoords (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	mv_fish_schools[fish_school_index].drawAllCoordinateSystems(0.5);
}

void Map :: drawFishSpheres (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	glColor3d(0.0, 1.0, 1.0);
	mv_fish_schools[fish_school_index].drawAllCollisionSpheres();
}

void Map :: drawFishNeighbours (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	glColor3d(0.0, 1.0, 0.0);
	mv_fish_schools[fish_school_index].drawNeighbours();
}

void Map :: drawFishSchoolLeader (unsigned int fish_school_index) const
{
	assert(fish_school_index < getFishSchoolCount());

	glColor3d(1.0, 0.0, 0.0);
	mv_fish_schools[fish_school_index].drawLeader();
}



void Map :: resetPlayer ()
{
	m_player.setCoordinateSystem(m_player_start);
	m_player.disableAutopilot();
}

void Map :: addPlayerVelocity (const ObjLibrary::Vector3& delta)
{
	m_player.addVelocity(delta);
	m_player.disableAutopilot();
}

void Map :: rotatePlayerAroundForward (double radians)
{
	m_player.rotateAroundForward(radians);
	m_player.disableAutopilot();
}

void Map :: rotatePlayerAroundUp (double radians)
{
	m_player.rotateAroundUp(radians);
	m_player.disableAutopilot();
}

void Map :: rotatePlayerAroundRight (double radians)
{
	m_player.rotateAroundRight(radians);
	m_player.disableAutopilot();
}

void Map :: rotatePlayerToDirection (const ObjLibrary::Vector3& desired_forward,
                                     double radians)
{
	assert(radians >= 0.0);

	m_player.rotateToVector(desired_forward, radians);
	m_player.disableAutopilot();
}

void Map :: enablePlayerAutopilot ()
{
	m_player.enableAutopilot();
}

void Map :: updatePhysicsAll (float delta_time)
{
	assert(delta_time >= 0.0f);

	// apply gravity and drag

	if(isPlayerUnderwater())
	{
		// underwater: apply drag
		double factor = pow(PLAYER_DRAG, delta_time);
		m_player.setVelocity(m_player.getVelocity() * factor);
	}
	else
	{
		// above water: apply gravity
		m_player.applyGravity(delta_time);
	}

	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
		mv_fish_schools[i].applyGravityAll(delta_time);

	// check collisions

	// player and fish vs. heightmap
	if(isCollision(m_player, m_terrain))
	{
		Vector3 surface_normal = m_terrain.getSurfaceNormal(getPlayerPosition());
		m_player.bounce(surface_normal);
	}
	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
		mv_fish_schools[i].checkCollisionAll(m_terrain);

	// player and fish vs. fixed entities
	for(unsigned int i = 0; i < mv_fixed_entities.size(); i++)
	{
		const FixedEntity& entity = mv_fixed_entities[i];
		if(isCollision(m_player, entity))
		{
			Vector3 surface_normal = entity.getSurfaceNormal(getPlayerPosition());
			m_player.bounce(surface_normal);
		}

		for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
			mv_fish_schools[i].checkCollisionAll(entity);
	}

	// player vs. fish
	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
	{
		unsigned int fresh_caught = mv_fish_schools[i].checkPlayerCaughtFish(m_player);
		m_fish_caught_count += fresh_caught;

		if(fresh_caught > 0)
			if(m_player.isAutopilot())
				m_player.enableAutopilot();  // reset autopilot
	}

	// move all

	m_player.moveByVelocity(delta_time);
	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
		mv_fish_schools[i].moveAllByVelocity(delta_time);

	// cleanup

	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
	{
		mv_fish_schools[i].updateBoundingSphere();
		mv_fish_schools[i].updateOrientationAll();
	}
}

void Map :: updateAiAll (float delta_time)
{
	assert(delta_time >= 0.0f);

	// update player
	if(m_player.isAutopilot())
		m_player.updateAutopilot(*this, delta_time);

	// update fish
	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
	{
		mv_fish_schools[i].updateLeader(delta_time, m_terrain);
		mv_fish_schools[i].doFlockingAll(delta_time);
	}
}

void Map :: setLeaderTargetToPlayer (unsigned int fish_school_index)
{
	assert(fish_school_index < getFishSchoolCount());

	mv_fish_schools[fish_school_index].setLeaderTarget(m_player.getPosition());
}



//
//  Helper functions for loading map
//

void Map :: loadEntities (const std::string& resource_path,
                          const std::string& filename)
{
	assert(isModelsLoaded());
	assert(Fish::isModelsLoaded());

	ifstream fin(resource_path + filename);

	// read first line
	string line;
	getline(fin, line);
	while(fin)
	{
		// skip blank lines
		if(line == "")
		{
			getline(fin, line);
			continue;
		}

		// process line
		switch(line[0])
		{
		case 't':
			readTerrain(resource_path, line);
			break;
		case 'p':
			readPlayerStart(resource_path, line);
			break;
		case 's':
			readSphere(resource_path, line);
			break;
		case 'c':
			readCylinder(resource_path, line);
			break;
		case 'f':
			readSchool(resource_path, line);
			break;
		}

		// read next line
		getline(fin, line);
	}
}

void Map :: readTerrain (const std::string& resource_path,
                         const std::string& line)
{
	istringstream ss(line);

	char junk;
	ss >> junk;

	Vector3 offset;
	ss >> offset;

	Vector3 size;
	ss >> size;
	if(size.x <= 0.0)
	{
		cerr << "Error: Invalid terrain X size " << size.x << endl;
		exit(1);
	}
	if(size.y <= 0.0)
	{
		cerr << "Error: Invalid terrain Y size " << size.y << endl;
		exit(1);
	}
	if(size.z <= 0.0)
	{
		cerr << "Error: Invalid terrain Z size " << size.z << endl;
		exit(1);
	}

	string texture_name;
	ss >> texture_name;

	m_terrain = Terrain(resource_path, texture_name,
	                    "dirt2.bmp", "grass1.bmp",
	                    offset, size);
}

void Map :: readPlayerStart (const std::string& resource_path,
                             const std::string& line)
{
	istringstream ss(line);

	char junk;
	ss >> junk;

	Vector3 position;
	ss >> position;

	Vector3 forward;
	ss >> forward;
	if(!forward.isUnit())
	{
		cerr << "Error: Player forward vector is not a unit vector " << forward << endl;
		exit(1);
	}

	Vector3 up;
	ss >> up;
	if(!up.isUnit())
	{
		cerr << "Error: Player up vector is not a unit vector " << up << endl;
		exit(1);
	}

	m_player_start = CoordinateSystem(position, forward, up);
}

void Map :: readSphere (const std::string& resource_path,
                        const std::string& line)
{
	istringstream ss(line);

	char junk;
	ss >> junk;

	Vector3 center;
	ss >> center;

	double radius;
	ss >> radius;
	if(radius < 0.0)
	{
		cerr << "Error: Invalid sphere radius " << radius << endl;
		exit(1);
	}

	string model_name;
	ss >> model_name;

	if(sphere_lists.count(model_name) == 0)
	{
		// key is not in map
		ObjModel model(resource_path + model_name);
		if(model.isLoadedSuccessfully())
			sphere_lists[model_name] = model.getDisplayList();
		else
		{
			cerr << "Error: Could not load model \"" << model_name << "\"" << endl;
			exit(1);
		}
	}
	assert(sphere_lists.count(model_name) != 0);

	mv_fixed_entities.push_back(FixedEntity(center, radius, sphere_lists[model_name]));
}

void Map :: readCylinder (const std::string& resource_path,
                          const std::string& line)
{
	istringstream ss(line);

	char junk;
	ss >> junk;

	Vector3 end1;
	ss >> end1;

	Vector3 end2;
	ss >> end2;

	double radius;
	ss >> radius;
	if(radius < 0.0)
	{
		cerr << "Error: Invalid cylinder radius " << radius << endl;
		exit(1);
	}

	string model_name;
	ss >> model_name;

	if(cylinder_lists.count(model_name) == 0)
	{
		// key is not in map
		ObjModel model(resource_path + model_name);
		if(model.isLoadedSuccessfully())
			cylinder_lists[model_name] = model.getDisplayList();
		else
		{
			cerr << "Error: Could not load model \"" << model_name << "\"" << endl;
			exit(1);
		}
	}
	assert(cylinder_lists.count(model_name) != 0);

	mv_fixed_entities.push_back(FixedEntity(end1, end2, radius, cylinder_lists[model_name]));
}

void Map :: readSchool (const std::string& resource_path,
                        const std::string& line)
{
	istringstream ss(line);

	char junk;
	ss >> junk;

	Vector3 center;
	ss >> center;

	double radius;
	ss >> radius;
	if(radius <= 0.0)
	{
		cerr << "Error: Invalid fish school radius " << radius << endl;
		exit(1);
	}

	int count;
	ss >> count;
	if(count < 1)
	{
		cerr << "Error: Invalid fish count in school " << count << endl;
		exit(1);
	}

	double explore_max_distance;
	ss >> explore_max_distance;
	if(explore_max_distance < 0.0)
	{
		cerr << "Error: Invalid maximum school explore distance " << explore_max_distance << endl;
		exit(1);
	}

	string model_name;
	ss >> model_name;
	unsigned int species = Fish::getSpeciesForFilename(model_name);
	if(species == Fish::SPECIES_COUNT)
	{
		cerr << "Error: Unrecognized fish model \"" << model_name << "\"" << endl;
		exit(1);
	}

	mv_fish_schools.push_back(FishSchool(center, radius, count, species, explore_max_distance));
}



//
//  Helper functions for displaying map
//

void Map :: drawAxes () const
{
	glBegin(GL_LINES);
		glColor3d(1.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(2.0, 0.0, 0.0);
		glColor3d(0.0, 1.0, 0.0);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 2.0, 0.0);
		glColor3d(0.0, 0.0, 1.0);
		glVertex3d(0.0, 0.0, 0.0);
		glVertex3d(0.0, 0.0, 2.0);
	glEnd();
}

void Map :: drawSkybox () const
{
	glPushMatrix();
		glTranslated(m_player.getPosition().x,
		             m_player.getPosition().y,
		             m_player.getPosition().z);
		glDisable(GL_FOG);
		glDepthMask(GL_FALSE);
		skybox_list.draw();
		glDepthMask(GL_TRUE);
		glEnable(GL_FOG);
	glPopMatrix();
}

void Map :: drawEntites () const
{
	for(unsigned int i = 0; i < mv_fixed_entities.size(); i++)
		mv_fixed_entities[i].draw();

	for(unsigned int i = 0; i < mv_fish_schools.size(); i++)
		mv_fish_schools[i].draw();
}

void Map :: drawSurface () const
{
	if(isCameraUnderwater())
		surface_list.draw();
	else
	{
		glDisable(GL_FOG);
		surface_list.draw();
		glEnable(GL_FOG);
	}
}




