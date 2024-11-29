//
//  Player.h
//
//  A module to represent the player.
//

#pragma once

#include <string>

#include "ObjLibrary/Vector3.h"

#include "CoordinateSystem.h"
#include "Entity.h"

class Map;



const double PLAYER_ACCELERATION = 3.0;  // m/s^2
const double PLAYER_TURN_RATE    = 3.0;  // radians/s



//
//  Player
//
//  A class to represent the player.  The player has an
//    autopilot that will chase fish.
//
//  Class Invariant:
//    <1> m_autopilot_mode < AUTOPILOT_COUNT
//
class Player : public Entity
{
public:
//
//  AUTOPILOT_COUNT
//
//  The number of autopilot modes.
//
	static const unsigned int AUTOPILOT_COUNT = 5;

//
//  AUTOPILOT_DISABLED
//
//  A special value indicating that the player autopilot is
//    disabled.
//
	static const unsigned int AUTOPILOT_DISABLED = 0;

//
//  AUTOPILOT_PLANNING
//
//  The autopilot mode where the autopilot chooses a fish to
//    catch.
//
	static const unsigned int AUTOPILOT_PLANNING = 1;

//
//  AUTOPILOT_SWIM_UP
//
//  The autopilot mode where the player swims to the surface.
//
	static const unsigned int AUTOPILOT_SWIM_UP = 2;

//
//  AUTOPILOT_ON_SURFACE
//
//  The autopilot mode where the player swims along the water
//    surface.
//
	static const unsigned int AUTOPILOT_ON_SURFACE = 3;

//
//  AUTOPILOT_DIVE
//
//  The autopilot mode where the player dives down to catch the
//    fish.
//
	static const unsigned int AUTOPILOT_DIVE = 4;

public:
//
//  Default Constructor
//
//  Purpose: To construct an default Player.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: A Player is constructed at the origin.  Its
//               forward direction is the +X axis and its up
//               direction is the +Y axis.
//
	Player ();

//
//  Constructor
//
//  Purpose: To construct a Player with the specified coordinate
//           system.
//  Parameter(s):
//    <1> coords: The coordinate system
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: An invalid Player is constructed with
//               coordinate system coords.
//
	Player (const CoordinateSystem& coords);

//
//  isAutopilot
//
//  Purpose: To determine if the player autopilot is currently
//           active.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: Whether the player autopilot is active.
//  Side Effect: N/A
//
	bool isAutopilot () const;

//
//  getAutopilotName
//
//  Purpose: To determine the current player autopilot mode as a
//           string.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: The name for the player autopilot mode.  This will
//           be a short string.
//  Side Effect: N/A
//
	std::string getAutopilotName () const;

//
//  enableAutopilot
//
//  Purpose: To start the player autopilot.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The player autopilot is initialized and marked
//               as active.  If it was already active, the old
//               state is lost.
//
	void enableAutopilot ();

//
//  disableAutopilot
//
//  Purpose: To stop the player autopilot.
//  Parameter(s): N/A
//  Precondition(s): N/A
//  Returns: N/A
//  Side Effect: The player autopilot is marked as inactive.
//               The current state is lost.  If the autopilot
//               was already disabled, there is no effect.
//
	void disableAutopilot ();

//
//  updateAutopilot
//
//  Purpose: To run the player autopilot for one update cycle.
//  Parameter(s):
//    <1> map: The current map state
//    <2> delta_time: The duration for the update cycle
//  Precondition(s):
//    <1> isAutopilot()
//    <2> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: The player autopilot is updated for one update
//               cycle.
//
	void updateAutopilot (const Map& map,
	                      float delta_time);

private:
//
//  autopilotPlan
//
//  Purpose: To handle the autopilot choosing which fish to
//           chase.
//  Parameter(s):
//    <1> map: The current map state
//  Precondition(s):
//    <1> isAutopilot()
//    <2> m_autopilot_mode == AUTOPILOT_PLANNING
//  Returns: N/A
//  Side Effect: The player autopilot attempts to chooses a fish
//               to chase.  If the autopilot succeeds, it leaves
//               planning mode.
//
	void autopilotPlan (const Map& map);

//
//  autopilotChase
//
//  Purpose: To handle the autopilot chasing a fish.
//  Parameter(s):
//    <1> map: The current map state
//    <2> delta_time: The duration for the update cycle
//  Precondition(s):
//    <1> isAutopilot()
//    <2> m_autopilot_mode != AUTOPILOT_DISABLED
//    <3> m_autopilot_mode != AUTOPILOT_PLANNING
//    <4> delta_time >= 0.0
//  Returns: N/A
//  Side Effect: The player autopilot chases a fish for one
//               update cycle.  This will change the velocity
//               but not the position.  The camera orientation
//               and autopilot mode are updated.
//
	void autopilotChase (const Map& map,
	                     float delta_time);

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
	unsigned int m_autopilot_mode;
	unsigned int m_autopilot_school;
	unsigned int m_autopilot_fish;
	ObjLibrary::Vector3 m_look_smoothed;
};




