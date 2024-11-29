//
//  main.cpp
//
//  Solution for Assignment 5 of CS 409, 202430.
//

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "GetGlut.h"
#include "Sleep.h"

#include "ObjLibrary/Vector3.h"
#include "ObjLibrary/SpriteFont.h"
#include "ObjLibrary/TextureBmp.h"

#include "TimeManager.h"
#include "CoordinateSystem.h"
#include "Map.h"
#include "Player.h"

using namespace std;
using namespace ObjLibrary;

void init ();
void initDisplay ();

void keyboard (unsigned char key, int x, int y);
void keyboardUp (unsigned char key, int x, int y);
void special (int special_key, int x, int y);
void specialUp (int special_key, int x, int y);

void update ();
void doGameUpdates ();
void updateForKeyboard ();

void reshape (int w, int h);

void display ();
void drawHUD ();
void drawFrameRateDebugging ();
void drawKeyboardInput ();

const unsigned int KEY_UP_ARROW    = 256;
const unsigned int KEY_DOWN_ARROW  = 257;
const unsigned int KEY_LEFT_ARROW  = 258;
const unsigned int KEY_RIGHT_ARROW = 259;
const unsigned int KEY_F1          = 260;
const unsigned int KEY_COUNT       = 261;
bool key_pressed[KEY_COUNT];

int window_width  = 1024;
int window_height = 768;
SpriteFont font;

TimeManager time_manager;

const string RESOURCE_PATH = "Resources/";
Map map;
bool is_paused = false;

bool display_frame_rate              = false;
bool display_nearby_fish             = false;
bool display_nearby_surface_normals  = false;
bool display_terrain_surface_normals = false;
bool display_keyboard_input          = false;


int main (int argc, char* argv[])
{
	glutInitWindowSize(window_width, window_height);
	glutInitWindowPosition(0, 0);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
	glutCreateWindow("Assignment 5 Solution");
	glutKeyboardFunc(keyboard);
	glutKeyboardUpFunc(keyboardUp);
	glutSpecialFunc(special);
	glutSpecialUpFunc(specialUp);
	glutIdleFunc(update);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);

	init();

	glutMainLoop();  // contains an infinite loop, so it never returns
	return 1;
}

void init ()
{
	// seed random number generator
	srand((unsigned int)time(NULL));

	initDisplay();

	font.load(RESOURCE_PATH + "Font.bmp");
	Map::loadModels(RESOURCE_PATH);

	map = Map(RESOURCE_PATH, "map.txt");

	time_manager = TimeManager(60, 10);
}

void initDisplay ()
{
	glClearColor(0.5, 0.5, 0.5, 0.0);
	glColor3f(0.0, 0.0, 0.0);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	glEnable(GL_FOG);
	glFogi (GL_FOG_MODE, GL_EXP);
	glFogf (GL_FOG_DENSITY, 0.15f);

	glutPostRedisplay();
}



void keyboard (unsigned char key, int x, int y)
{
	key = toupper(key);

	switch (key)
	{
	case 27: // on [ESC]
		exit(0); // normal exit
		break;
	case 'P':
		if(!key_pressed['P'])
			is_paused = !is_paused;
		break;
	case 'R':
		if(!key_pressed['R'])
			map.resetPlayer();
		break;
	case 'Z':
		if(!key_pressed['Z'])
			map.enablePlayerAutopilot();
		break;
	case '1':
		if(!key_pressed['1'])
			display_frame_rate = !display_frame_rate;
		break;
	case '2':
		if(!key_pressed['2'])
			display_nearby_fish = !display_nearby_fish;
		break;
	case '3':
		if(!key_pressed['3'])
			display_nearby_surface_normals = !display_nearby_surface_normals;
		break;
	case '4':
		if(!key_pressed['4'])
			display_terrain_surface_normals = !display_terrain_surface_normals;
		break;
	case '5':
		if(!key_pressed['5'])
		{
			unsigned int school = map.findNearestSchool(map.getPlayerPosition(), false);
			map.setLeaderTargetToPlayer(school);
		}
		break;
	}

	key_pressed[key] = true;
}

void keyboardUp (unsigned char key, int x, int y)
{
	key = toupper(key);
	key_pressed[key] = false;
}

void special (int special_key, int x, int y)
{
	switch(special_key)
	{
	case GLUT_KEY_UP:
		key_pressed[KEY_UP_ARROW] = true;
		break;
	case GLUT_KEY_DOWN:
		key_pressed[KEY_DOWN_ARROW] = true;
		break;
	case GLUT_KEY_LEFT:
		key_pressed[KEY_LEFT_ARROW] = true;
		break;
	case GLUT_KEY_RIGHT:
		key_pressed[KEY_RIGHT_ARROW] = true;
		break;
	case GLUT_KEY_F1:
		if(!key_pressed[KEY_F1])
			display_keyboard_input = !display_keyboard_input;
		key_pressed[KEY_F1] = true;
		break;
	}
}

void specialUp (int special_key, int x, int y)
{
	switch(special_key)
	{
	case GLUT_KEY_UP:
		key_pressed[KEY_UP_ARROW] = false;
		break;
	case GLUT_KEY_DOWN:
		key_pressed[KEY_DOWN_ARROW] = false;
		break;
	case GLUT_KEY_LEFT:
		key_pressed[KEY_LEFT_ARROW] = false;
		break;
	case GLUT_KEY_RIGHT:
		key_pressed[KEY_RIGHT_ARROW] = false;
		break;
	case GLUT_KEY_F1:
		key_pressed[KEY_F1] = false;
		break;
	}
}



void update ()
{
	for(unsigned int i = 0; i < time_manager.getMaxUpdatesPerFrame() &&
	                        time_manager.isUpdateWaiting(); i++)
	{
		doGameUpdates();
		time_manager.markNextUpdate();
	}

	time_manager.sleepUntilNextUpdate();
	glutPostRedisplay();
}

void doGameUpdates ()
{
	if(!is_paused)
	{
		updateForKeyboard();
		map.updatePhysicsAll(time_manager.getUpdateDeltaTime());
		map.updateAiAll(time_manager.getUpdateDeltaTime());
	}

	if(key_pressed['U'])
		sleep(0.05);
}

void updateForKeyboard ()
{
	float delta_time = time_manager.getUpdateDeltaTime();
	const CoordinateSystem& player_coords = map.getPlayerCoords();

	// player movement

	if(map.isPlayerUnderwater())
	{
		Vector3 acceleration;
		if(key_pressed[' '])
			acceleration += player_coords.getForward();
		if(key_pressed['/'])
			acceleration -= player_coords.getForward();
		if(key_pressed['D'])
			acceleration += player_coords.getRight();
		if(key_pressed['A'])
			acceleration -= player_coords.getRight();
		if(key_pressed['W'])
			acceleration += player_coords.getUp();
		if(key_pressed['S'])
			acceleration -= player_coords.getUp();
		acceleration.truncate(1.0);  // don't go faster by holding multiple keys
		if(!acceleration.isZero())
			map.addPlayerVelocity(acceleration * PLAYER_ACCELERATION * delta_time);
	}

	// player rotation

	if(key_pressed[KEY_LEFT_ARROW])
		map.rotatePlayerAroundUp(PLAYER_TURN_RATE * delta_time);
	if(key_pressed[KEY_RIGHT_ARROW])
		map.rotatePlayerAroundUp(-PLAYER_TURN_RATE * delta_time);
	if(key_pressed[KEY_UP_ARROW])
		map.rotatePlayerAroundRight(PLAYER_TURN_RATE * delta_time);
	if(key_pressed[KEY_DOWN_ARROW])
		map.rotatePlayerAroundRight(-PLAYER_TURN_RATE * delta_time);
	if(key_pressed[','])
		map.rotatePlayerAroundForward(-PLAYER_TURN_RATE * delta_time);
	if(key_pressed['.'])
		map.rotatePlayerAroundForward(PLAYER_TURN_RATE * delta_time);
	if(key_pressed['H'])
	{
		Vector3 origin = Vector3::ZERO;
		Vector3 direction_to_origin = origin - map.getPlayerPosition();
		map.rotatePlayerToDirection(direction_to_origin, PLAYER_TURN_RATE * delta_time);
	}
}



void reshape (int w, int h)
{
	glViewport (0, 0, w, h);

	window_width  = w;
	window_height = h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / (GLdouble)h, 0.1, 1000.0);
	glMatrixMode(GL_MODELVIEW);

	glutPostRedisplay();
}

void display ()
{
	time_manager.markNextFrame();
	if(key_pressed['Y'])
		sleep(0.05);

	map.updateFog();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// clear the screen - any drawing before here will not display at all

	map.draw();

	if(display_nearby_fish)
	{
		unsigned int school = map.findNearestSchool(map.getPlayerPosition(), false);
		map.drawFishSchoolSphere(school);
		//map.drawFishCoords(school);
		map.drawFishSpheres(school);
		map.drawFishNeighbours(school);
		map.drawFishSchoolLeader(school);
	}

	if(display_nearby_surface_normals)
	{
		unsigned int fixed = map.findNearestFixedEntity(map.getPlayerPosition());
		map.drawFixedEntitySurfaceNormals(fixed);
	}

	if(display_terrain_surface_normals)
		map.drawTerrainSurfaceNormals();

	drawHUD();

	// send the current image to the screen - any drawing after here will not display
	glutSwapBuffers();
}

void drawHUD ()
{
	SpriteFont::setUp2dView(window_width, window_height);

	stringstream caught_ss;
	caught_ss << "Caught: " << map.getFishCaughtCount();
	font.draw(caught_ss.str(), 16, window_height - 40);

	if(map.isCameraUnderwater())
	{
		stringstream depth_ss;
		double player_depth = -map.getPlayerPosition().y;
		depth_ss << "Depth: " << std::setprecision(3) << player_depth << "m";
		font.draw(depth_ss.str(), 16, window_height - 72);

		stringstream seafloor_ss;
		double seafloor_depth = -map.getSeafloorDepth();
		seafloor_ss << "Seafloor: " << std::setprecision(3) << seafloor_depth << "m";
		font.draw(seafloor_ss.str(), 16, window_height - 104);

		stringstream autopilot_ss;
		autopilot_ss << "Autopilot: " << map.getPlayerAutopilotName();
		font.draw(autopilot_ss.str(), 16, window_height - 136);
	}

	if(is_paused)
	{
		string message = "GAME PAUSED";
		int draw_x = (window_width - font.getWidth(message)) / 2;
		font.draw(message, draw_x, window_height / 2);
	}

	if(display_frame_rate)
		drawFrameRateDebugging();

	drawKeyboardInput();

	SpriteFont::unsetUp2dView();
}

void drawFrameRateDebugging ()
{
	stringstream game_time_ss;
	game_time_ss << "Game running time: " << time_manager.getGameDuration();
	font.draw(game_time_ss.str(), 16, 16);

	// frame rate

	stringstream frame_count_ss;
	frame_count_ss << "Frame count: " << time_manager.getFrameCount();
	font.draw(frame_count_ss.str(), 16, 48);

	stringstream average_frame_rate_ss;
	average_frame_rate_ss << "Average frame rate: " << time_manager.getFrameRateAverage();
	font.draw(average_frame_rate_ss.str(), 16, 72);

	stringstream instantaneous_frame_rate_ss;
	instantaneous_frame_rate_ss << "Instantaneous frame rate: " << time_manager.getFrameRateInstantaneous();
	font.draw(instantaneous_frame_rate_ss.str(), 16, 96);

	stringstream smoothed_frame_rate_ss;
	smoothed_frame_rate_ss << "Smoothed frame rate: " << time_manager.getFrameRateSmoothed();
	font.draw(smoothed_frame_rate_ss.str(), 16, 120);

	// update rate

	stringstream update_count_ss;
	update_count_ss << "Update count: " << time_manager.getUpdateCount();
	font.draw(update_count_ss.str(), 16, 152);

	stringstream average_update_rate_ss;
	average_update_rate_ss << "Average update rate: " << time_manager.getUpdateRateAverage();
	font.draw(average_update_rate_ss.str(), 16, 176);

	stringstream instantaneous_update_rate_ss;
	instantaneous_update_rate_ss << "Instantaneous update rate: " << time_manager.getUpdateRateInstantaneous();
	font.draw(instantaneous_update_rate_ss.str(), 16, 200);

	stringstream smoothed_update_rate_ss;
	smoothed_update_rate_ss << "Smoothed update rate: " << time_manager.getUpdateRateSmoothed();
	font.draw(smoothed_update_rate_ss.str(), 16, 224);
}

void drawKeyboardInput ()
{
	int key_x  = window_width - 384;
	int text_x = key_x + 96;

	if(display_keyboard_input)
	{
		font.draw("Press [F1] to hide keyboard input", key_x, 16);

		int base_y = 48;
		font.draw("[ESC]", key_x, base_y + 0);
		font.draw("Quit", text_x, base_y + 0);

		font.draw("[P]", key_x, base_y + 24);
		font.draw("Pause/unpause", text_x, base_y + 24);

		base_y += 56;
		font.draw("[SPACE]", key_x, base_y + 0);
		font.draw("Go forwards", text_x, base_y + 0);

		font.draw("[/]", key_x, base_y + 24);
		font.draw("Go backwards", text_x, base_y + 24);

		base_y += 56;
		font.draw("[UP]", key_x, base_y + 0);
		font.draw("Turn upwards", text_x, base_y + 0);

		font.draw("[DOWN]", key_x, base_y + 24);
		font.draw("Turn downwards", text_x, base_y + 24);

		font.draw("[LEFT]", key_x, base_y + 48);
		font.draw("Turn left", text_x, base_y + 48);

		font.draw("[RIGHT]", key_x, base_y + 72);
		font.draw("Turn right", text_x, base_y + 72);

		base_y += 104;
		font.draw("[Z]", key_x, base_y + 0);
		font.draw("Enable autopilot", text_x, base_y + 0);

		base_y += 32;
		font.draw("[W]", key_x, base_y + 0);
		font.draw("Strafe upwards", text_x, base_y + 0);

		font.draw("[S]", key_x, base_y + 24);
		font.draw("Strafe downwards", text_x, base_y + 24);

		font.draw("[A]", key_x, base_y + 48);
		font.draw("Strafe left", text_x, base_y + 48);

		font.draw("[D]", key_x, base_y + 72);
		font.draw("Strafe right", text_x, base_y + 72);

		base_y += 104;
		font.draw("[1]", key_x, base_y + 0);
		font.draw("Display frame rate", text_x, base_y + 0);

		font.draw("[2]", key_x, base_y + 24);
		font.draw("Display fish school", text_x, base_y + 24);

		font.draw("[3]", key_x, base_y + 48);
		font.draw("Display fixed entity normals", text_x, base_y + 48);

		font.draw("[4]", key_x, base_y + 72);
		font.draw("Display heightmap normals", text_x, base_y + 72);

		font.draw("[5]", key_x, base_y + 96);
		font.draw("Call fish school to player", text_x, base_y + 96);
	}
	else
		font.draw("Press [F1] to show keyboard input", key_x, 16);
}
