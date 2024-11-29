//
//  Map.h
//
//  A module to represent the current game map.
//

#pragma once

#include <climits>
#include <string>
#include <vector>

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"
#include "ObjLibrary/TextureBmp.h"

#include "Heightmap.h"
#include "Terrain.h"

#include "CoordinateSystem.h"
#include "Entity.h"
#include "Player.h"
#include "FixedEntity.h"
#include "FishSchool.h"



//
//  Map
//
//  A class to represent the current game map.
//
class Map
{
public:
//
//  NOT_FOUND
//
//  A specifal value returned when there are no qualifying
//    entities and thus there is no nearest one.
//
	static const unsigned int NOT_FOUND = UINT_MAX;

//
//  isModelsLoaded
//
//  Purpose: To determine if the general OBJ models for the map
//           have been loaded.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the OBJ models have been loaded.
//  Side Effect: N/A
//
	static bool isModelsLoaded ();

//
//  loadModels
//
//  Purpose: To load the general OBJ models for the map.  These
//           are the models that don't fit anywhere else
//           (e.g. the skybox).  This function should be called
//           once at game start after OpenGL is initialized but
//           before the map is displayed.
//  Parameter(s):
//    <1> resource_path: The file path to prepend to the
//                       filenames
//  Precondition(s):
//    <1> !isModelsLoaded()
//  Returns: N/A
//  Side Effect: The general OpenGL models are loaded and
//               converted to display lists.  After this
//               function has been called, isModelsLoaded will
//               return true.
//
	static void loadModels (const std::string& resource_path);

public:
	Map ();
	Map (const std::string& resource_path,
	     const std::string& filename);

	const ObjLibrary::Vector3& getPlayerPosition () const;
	const CoordinateSystem& getPlayerCoords () const;
	const ObjLibrary::Vector3& getPlayerVelocity () const;
	std::string getPlayerAutopilotName () const;
	unsigned int getFishCaughtCount () const;
	bool isPlayerUnderwater () const;
	bool isCameraUnderwater () const;

	double getSeafloorDepth () const;

	unsigned int getFixedEntityCount () const;
	unsigned int getFishSchoolCount () const;
	const FishSchool& getFishSchool (unsigned int fish_school_index) const;
	unsigned int findNearestFixedEntity (const ObjLibrary::Vector3& search_from) const;
	unsigned int findNearestSchool (const ObjLibrary::Vector3& search_from,
	                                bool exclude_empty) const;

	void updateFog () const;
	void draw () const;
	void drawTerrainSurfaceNormals () const;
	void drawFixedEntitySurfaceNormals (unsigned int fixed_entity_index) const;
	void drawFishSchoolSphere (unsigned int fish_school_index) const;
	void drawFishCoords (unsigned int fish_school_index) const;
	void drawFishSpheres (unsigned int fish_school_index) const;
	void drawFishNeighbours (unsigned int fish_school_index) const;
	void drawFishSchoolLeader (unsigned int fish_school_index) const;
	     
	void resetPlayer ();
	void addPlayerVelocity (const ObjLibrary::Vector3& delta);
	void rotatePlayerAroundForward (double radians);
	void rotatePlayerAroundUp (double radians);
	void rotatePlayerAroundRight (double radians);
	void rotatePlayerToDirection (const ObjLibrary::Vector3& desired_forward,
	                              double radians);
	void enablePlayerAutopilot ();
	void updatePhysicsAll (float delta_time);
	void updateAiAll (float delta_time);
	void setLeaderTargetToPlayer (unsigned int fish_school_index);

private:
	void loadEntities (const std::string& resource_path,
	                   const std::string& filename);
	void readTerrain (const std::string& resource_path,
	                  const std::string& line);
	void readPlayerStart (const std::string& resource_path,
	                      const std::string& line);
	void readSphere (const std::string& resource_path,
	                 const std::string& line);
	void readCylinder (const std::string& resource_path,
	                   const std::string& line);
	void readSchool (const std::string& resource_path,
	                 const std::string& line);

	void drawAxes () const;
	void drawSkybox () const;
	void drawEntites () const;
	void drawSurface () const;

private:
	CoordinateSystem m_player_start;
	Player m_player;
	unsigned int m_fish_caught_count;

	Terrain m_terrain;
	std::vector<FixedEntity> mv_fixed_entities;
	std::vector<FishSchool> mv_fish_schools;
};

