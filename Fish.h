//
//  Fish.h
//
//  A module to represent a single fish in a school.
//

#pragma once

#include <vector>

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/DisplayList.h"

#include "Entity.h"



//
//  Fish
//
//  A class to represent a single fish in a school.  For
//    collision checking, a fish is a sphere with a radius
//    determined by its species.
//
//  Class Invariant:
//    <1> m_species < SPECIES_COUNT
//
class Fish : public Entity
{
public:
//
//  SPECIES_COUNT
//
//  The number of fish species.
//
	static const unsigned int SPECIES_COUNT = 9;

//
//  getRadiusBySpecies
//
//  Purpose: To determine the radius for a species of fish.
//  Parameter(s):
//    <1> species: The fish species to check
//  Precondition(s):
//    <1> species < SPECIES_COUNT
//  Returns: The radius of fish species species.
//  Side Effect: N/A
//
	static double getRadiusBySpecies (unsigned int species);

//
//  getSpeed
//
//  Purpose: To determine the speed for a species of fish.
//  Parameter(s):
//    <1> species: The fish species to check
//  Precondition(s):
//    <1> species < SPECIES_COUNT
//  Returns: The swimming speed of fish species species.
//  Side Effect: N/A
//
	static double getSpeed (unsigned int species);

//
//  getAcceleration
//
//  Purpose: To determine the maximum acceleration for a species
//           of fish.
//  Parameter(s):
//    <1> species: The fish species to check
//  Precondition(s):
//    <1> species < SPECIES_COUNT
//  Returns: The maximum swimming acceleration of fish species
//           species.
//  Side Effect: N/A
//
	static double getAcceleration (unsigned int species);

//
//  isModelsLoaded
//
//  Purpose: To determine if the OBJ models for fish have been
//           loaded.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the OBJ models have been loaded.
//  Side Effect: N/A
//
	static bool isModelsLoaded ();

//
//  getSpeciesForFilename
//
//  Purpose: To determine the species that is displayed with the
//           specified OBJ model.
//  Parameter(s):
//    <1> filename: The file name of the OBJ model
//  Precondition(s):
//    <1> filename != ""
//  Returns: The fish species that is dispayed with OBJ model
//           filename.  If there is no such species,
//           SPECIES_COUNT is returned.
//  Side Effect: N/A
//
	static unsigned int getSpeciesForFilename (
	                               const std::string& filename);

//
//  loadModels
//
//  Purpose: To load the OBJ models for fish.  This function
//           should be called once at game start after OpenGL
//           is initialized but before any fish are created.
//  Parameter(s):
//    <1> resource_path: The file path to prepend to the
//                       filenames
//  Precondition(s):
//    <1> !isModelsLoaded()
//  Returns: N/A
//  Side Effect: OpenGL models for all fish species are loaded
//               and converted to display lists.  After this
//               function has been called, isModelsLoaded will
//               return true.
//
	static void loadModels (const std::string& resource_path);

public:
//
//  Default Constructor
//
//  Purpose: To construct an invalid Fish.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: An invalid Fish is constructed at the origin
//               with radius 0.0.  It has species 0 and cannot
//               be displayed.
//
	Fish ();

//
//  Constructor
//
//  Purpose: To construct a fish of the specified species.
//  Parameter(s):
//    <1> position: The position of the fish
//    <2> forward: The forward vector for the fish
//    <3> species: The fish species
//  Precondition(s):
//    <1> isModelsLoaded()
//    <2> forward.isUnit()
//    <3> species < SPECIES_COUNT
//  Returns: N/A
//  Side Effect: An invalid Fish is constructed at position
//               center and species species.
//
	Fish (const ObjLibrary::Vector3& position,
	      const ObjLibrary::Vector3& forward,
	      unsigned int species);

//
//  getSpecies
//
//  Purpose: To determine the species of this fish.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The fish species.  This value will always be
//           strictly less than SPECIES_COUNT.
//  Side Effect: N/A
//
	unsigned int getSpecies () const;

//
//  getNeighbours
//
//  Purpose: To determine the neighbours of this fish in its
//           school.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The indexes of the neighbouring fish to consider
//           when flocking.  This does not include the pointman.
//  Side Effect: N/A
//
	std::vector<unsigned int> getNeighbours () const;

//
//  draw
//
//  Purpose: To display this FixedEntity.
//  Parameter(s): N/A
//  Precondition(s):
//    <1> isModelsLoaded()
//  Returns: N/A
//  Side Effect: This FixedEntity is displayed.  If no DisplayList
//               has been set, there is no effect.
//
	void draw () const;

//
//  setNeighbours
//
//  Purpose: To change the neighbours of this fish in its
//           school.  These neighbours are used for flocking.
//  Parameter(s):
//    <1> v_new_neighbours: The new neighbours list
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The neighbours for this Fish are set to
//               v_new_neighbours.  The pointman is always
//               implictly a neighbour.
//
	void setNeighbours (
	          const std::vector<unsigned int> v_new_neighbours);

private:
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
	unsigned int m_species;
	std::vector<unsigned int> mv_neighbours;
};




