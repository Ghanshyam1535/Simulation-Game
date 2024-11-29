//
//  Fish.cpp
//

#include "Fish.h"

#include <cassert>
#include <string>

#include "GetGlut.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/ObjModel.h"
#include "ObjLibrary/DisplayList.h"

#include "Entity.h"

using namespace std;
using namespace ObjLibrary;
namespace
{
	const string FISH_FILENAMES[Fish::SPECIES_COUNT] =
	{
		"moonfish.obj",
		"sturgeon.obj",
		"dolphinfish.obj",
		"salmon.obj",
		"freshwater-drum.obj",
		"rainbow_trout.obj",
		"yellow-tang.obj",
		"anchovy.obj",
		"clownfish.obj",
	};

	const double FISH_RADIUSES[Fish::SPECIES_COUNT] =
	{
		// m - 1/4 the maximum length

		0.7,	// moonfish (1/3 length, disk-like from side)
		0.4,	// sturgeon (1/6 length, long and thin)
		0.25,	// dolphinfish
		0.18,	// salmon
		0.12,	// freshwater-drum
		0.10,	// rainbow trout
		0.08,	// yellow-tang (1/3 length, disk-like from side)
		0.04,	// anchovy (1/6 length, long and thin)
		0.03,	// clownfish
	};

	const double FISH_SPEED[Fish::SPECIES_COUNT] =
	{
		//  m / s

		4.0,	// moonfish
		0.3,	// sturgeon
		18.0,	// dolphinfish
		3.0,	// salmon           - maximum
		0.75,	// freshwater-drum  - really red drum
		1.0,	// rainbow trout
		0.07,	// yellow-tang
		0.6,	// anchovy
		1.7,	// clownfish        - fast or average?
	};

	const double FISH_ACCELERATION[Fish::SPECIES_COUNT] =
	{
		//  m / s^2  -- I made these numbers up

		1.0,	// moonfish
		0.5,	// sturgeon
		15.0,	// dolphinfish
		1.5,	// salmon
		0.8,	// freshwater-drum
		1.2,	// rainbow trout
		0.1,	// yellow-tang
		0.4,	// anchovy
		2.5,	// clownfish
	};

	DisplayList fish_lists[Fish::SPECIES_COUNT];

}  // end of anonymous namespace



double Fish :: getRadiusBySpecies (unsigned int species)
{
	assert(species < SPECIES_COUNT);

	return FISH_RADIUSES[species];
}

double Fish :: getSpeed (unsigned int species)
{
	assert(species < SPECIES_COUNT);

	return FISH_SPEED[species];
}

double Fish :: getAcceleration (unsigned int species)
{
	assert(species < SPECIES_COUNT);

	return FISH_ACCELERATION[species];
}

bool Fish :: isModelsLoaded ()
{
	assert(0 < SPECIES_COUNT);
	return fish_lists[0].isReady();
}

unsigned int Fish :: getSpeciesForFilename (const std::string& filename)
{
	assert(filename != "");

	for(unsigned int i = 0; i < SPECIES_COUNT; i++)
		if(filename == FISH_FILENAMES[i])
			return i;
	return SPECIES_COUNT;
}

void Fish :: loadModels (const std::string& resource_path)
{
	assert(!isModelsLoaded());

	for(unsigned int i = 0; i < SPECIES_COUNT; i++)
		fish_lists[i] = ObjModel(resource_path + FISH_FILENAMES[i]).getDisplayList();

	assert(isModelsLoaded());
}



Fish :: Fish ()
		: Entity(),
		  m_species(0),
		  mv_neighbours()
{
	assert(isInvariantTrue());
}

Fish :: Fish (const ObjLibrary::Vector3& position,
              const ObjLibrary::Vector3& forward,
              unsigned int species)
		: Entity(CoordinateSystem(position, forward),
		         FISH_RADIUSES[species]),
		  m_species(species),
		  mv_neighbours()
{
	assert(isModelsLoaded());
	assert(species < SPECIES_COUNT);

	assert(isInvariantTrue());
}

unsigned int Fish :: getSpecies () const
{
	assert(isInvariantTrue());

	return m_species;
}

std::vector<unsigned int> Fish :: getNeighbours () const
{
	assert(isInvariantTrue());

	return mv_neighbours;
}

void Fish :: draw () const
{
	assert(isInvariantTrue());
	assert(isModelsLoaded());

	Vector3 center = getPosition();
	double  radius = getRadius();

	glPushMatrix();
		applyDrawTransformations();
		glScaled(radius, radius, radius);
		assert(fish_lists[m_species].isReady());
		fish_lists[m_species].draw();
	glPopMatrix();
}



void Fish :: setNeighbours (const std::vector<unsigned int> v_new_neighbours)
{
	assert(isInvariantTrue());

	mv_neighbours = v_new_neighbours;

	assert(isInvariantTrue());
}



bool Fish :: isInvariantTrue () const
{
	if(m_species >= SPECIES_COUNT)
		return false;
	return true;
}



