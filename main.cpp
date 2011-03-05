#include <boost/shared_ptr.hpp>
using namespace boost;
#include <ode/ode.h>
#include <GL/glut.h>
#include <iostream>
#include <windows.h>
using namespace std;

#include <XInput.h>

#pragma comment(lib, "XInput.lib")
const int MAX_CONTROLLERS = 4;
const int LEFT_JOYSTICK = 0;
const int RIGHT_JOYSTICK = 1;
const int JOYSTICK_X = 0;
const int JOYSTICK_Y = 1;
XINPUT_STATE controllerStates[MAX_CONTROLLERS];
float joysticks[MAX_CONTROLLERS][2][2];
bool playerOneSwitch = false;
int playerOneControlling = 0;

void updateControllerStates()
{
    bool prevSwitchState = (controllerStates[0].Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;

	for(int i = 0; i < MAX_CONTROLLERS; ++i)
	{
		if(XInputGetState(i, &controllerStates[i]) != ERROR_SUCCESS)
		{
			joysticks[i][LEFT_JOYSTICK][JOYSTICK_X] = 0;
			joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y] = 0;
			joysticks[i][RIGHT_JOYSTICK][JOYSTICK_X] = 0;
			joysticks[i][RIGHT_JOYSTICK][JOYSTICK_Y] = 0;
			continue;
		}

		// zero value if thumbsticks are within the dead zone 
		if( ( controllerStates[i].Gamepad.sThumbLX < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE && 
			controllerStates[i].Gamepad.sThumbLX > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE ) && 
			( controllerStates[i].Gamepad.sThumbLY < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE && 
			controllerStates[i].Gamepad.sThumbLY > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE ) )
		{	
		   controllerStates[i].Gamepad.sThumbLX = 0;

		   controllerStates[i].Gamepad.sThumbLY = 0;
		}

		if( ( controllerStates[i].Gamepad.sThumbRX < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE && 
			controllerStates[i].Gamepad.sThumbRX > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE ) && 
			( controllerStates[i].Gamepad.sThumbRY < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE && 
			controllerStates[i].Gamepad.sThumbRY > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE ) ) 
		{
		   controllerStates[i].Gamepad.sThumbRX = 0;

		   controllerStates[i].Gamepad.sThumbRY = 0;
		}

		joysticks[i][LEFT_JOYSTICK][JOYSTICK_X] = (float) controllerStates[i].Gamepad.sThumbLX / 32767.0f;
		joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y] = (float) controllerStates[i].Gamepad.sThumbLY / 32767.0f;
		joysticks[i][RIGHT_JOYSTICK][JOYSTICK_X] = (float) controllerStates[i].Gamepad.sThumbRX / 32767.0f;
		joysticks[i][RIGHT_JOYSTICK][JOYSTICK_Y] = (float) controllerStates[i].Gamepad.sThumbRY / 32767.0f;
	}

    bool curSwitchState = (controllerStates[0].Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
    if(curSwitchState && curSwitchState != prevSwitchState)
    {
        playerOneSwitch = true;
        playerOneControlling += 1;
    }
}

const dReal PI = 3.141592654;
const dReal EPSILON = 0.01;
const dReal EPSILON_BIG = 5.0 * PI / 180.0;

void drawPanel()
{
    glBegin(GL_TRIANGLES);
        //looking from +z axis
        glVertex3f(-0.5,-0.5,0);
        glVertex3f(-0.5,0.5,0);
        glVertex3f(0.5,-0.5,0);

        glVertex3f(-0.5,0.5,0);
        glVertex3f(0.5,0.5,0);
        glVertex3f(0.5,-0.5,0);
    glEnd();
}

void drawSquare(float r, float g, float b)
{
    glLineWidth(2);
    glColor3f(1,0.5,0);
    glBegin(GL_LINE_STRIP);
        glVertex3f(-0.5,-0.5,0.5);
        glVertex3f(-0.5,0.5,0.5);

        glVertex3f(0.5,0.5,0.5);
        glVertex3f(0.5,-0.5,0.5);

        glVertex3f(-0.5,-0.5,0.5);
    glEnd();

    glBegin(GL_LINE_STRIP);
        glVertex3f(-0.5,-0.5,-0.5);
        glVertex3f(-0.5,0.5,-0.5);

        glVertex3f(0.5,0.5,-0.5);
        glVertex3f(0.5,-0.5,-0.5);

        glVertex3f(-0.5,-0.5,-0.5);
    glEnd();

    glBegin(GL_LINES);
        glVertex3f(-0.5,-0.5,0.5);
        glVertex3f(-0.5,-0.5,-0.5);

        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(-0.5,0.5,-0.5);

        glVertex3f(0.5,0.5,0.5);
        glVertex3f(0.5,0.5,-0.5);

        glVertex3f(0.5,-0.5,0.5);
        glVertex3f(0.5,-0.5,-0.5);
    glEnd();

    glColor3f(r,g,b);
    glBegin(GL_TRIANGLES);
        //looking from +z axis
        glVertex3f(-0.5,-0.5,0.5);
        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(0.5,-0.5,0.5);

        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(0.5,0.5,0.5);
        glVertex3f(0.5,-0.5,0.5);

        //looking from -z axis
        glVertex3f(0.5,-0.5,0.5);
        glVertex3f(0.5,0.5,0.5);
        glVertex3f(-0.5,-0.5,0.5);

        glVertex3f(0.5,0.5,0.5);
        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(-0.5,-0.5,0.5);

        //looking from +x axis
        glVertex3f(0.5,0.5,0.5);
        glVertex3f(0.5,0.5,-0.5);
        glVertex3f(0.5,-0.5,0.5);

        glVertex3f(0.5,-0.5,0.5);
        glVertex3f(0.5,0.5,-0.5);
        glVertex3f(0.5,-0.5,-0.5);

        //looking from -x axis
        glVertex3f(-0.5,0.5,-0.5);
        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(-0.5,-0.5,-0.5);

        glVertex3f(-0.5,-0.5,-0.5);
        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(-0.5,-0.5,0.5);

        //looking from +y axis
        glVertex3f(0.5,0.5,-0.5);
        glVertex3f(0.5,0.5,0.5);
        glVertex3f(-0.5,0.5,0.5);

        glVertex3f(-0.5,0.5,0.5);
        glVertex3f(-0.5,0.5,-0.5);
        glVertex3f(0.5,0.5,-0.5);

        //looking from -y axis
        glVertex3f(0.5,-0.5,0.5);
        glVertex3f(0.5,-0.5,-0.5);
        glVertex3f(-0.5,-0.5,-0.5);

        glVertex3f(-0.5,-0.5,-0.5);
        glVertex3f(-0.5,-0.5,0.5);
        glVertex3f(0.5,-0.5,0.5);
    glEnd();
}

const float CYLINDER_SIDES = 32.0f;
void drawCylinder(bool highlight)
{
    float r = 0.5;
    float g = 0;
    if(highlight)
    {
        r = 1;
        g = 1;
    }

    glBegin(GL_TRIANGLES);
        float inc = (2*PI) / CYLINDER_SIDES;
        float oldX = 0;
        float oldY = 0;
        int c = 0;
        for(float i = 2 * PI; i > -EPSILON; i -= inc)
        {
            ++c;
            const int mod = c % 6;
            if(mod < 3)
            {
                glColor3f(r, g, 0);
            }
            else
            {
                glColor3f(0,0.5,0.5);
            }

            if(c == 3)
            {
                glColor3f(1,1,1);
            }

            float x = cosf(i);
            float y = sinf(i);

            //top piece
            glVertex3f(oldX, oldY, 0.5);
            glVertex3f(x, y, 0.5);
            glVertex3f(0.0, 0.0, 0.5);

            //bottom piece
            glVertex3f(x, y, -0.5);
            glVertex3f(oldX, oldY, -0.5);
            glVertex3f(0.0, 0.0, -0.5);

            //side piece
            glVertex3f(x,y,0.5);
            glVertex3f(oldX,oldY,0.5);
            glVertex3f(x,y,-0.5);

            glVertex3f(x,y,-0.5);
            glVertex3f(oldX,oldY,0.5);
            glVertex3f(oldX,oldY,-0.5);

            oldX = x;
            oldY = y;
        }
    glEnd();
}

dWorldID world;
dSpaceID space;

void ODEtoOGL(float* M, const dReal* p, const dReal* R)
{
    M[0]  = R[0]; M[1]  = R[4]; M[2]  = R[8];  M[3]  = 0;
    M[4]  = R[1]; M[5]  = R[5]; M[6]  = R[9];  M[7]  = 0;
    M[8]  = R[2]; M[9]  = R[6]; M[10] = R[10]; M[11] = 0;
    M[12] = p[0]; M[13] = p[1]; M[14] = p[2];  M[15] = 1;
}

const dReal ROBOT_DIMENSION_X = 0.9652;
const dReal ROBOT_DIMENSION_Y = 0.7112;
const dReal WHEEL_RADIUS_SMALL = (3.0 * 2.5) / 100.0;
const dReal WHEEL_RADIUS_BIG = (4.0 * 2.5) / 100.0;//4
//const dReal ROBOT_DIMENSION_Z = WHEEL_RADIUS * 0.5;
const dReal WHEEL_THICKNESS = 0.15;
const dReal WHEEL_EDGE_X = 0;//WHEEL_RADIUS + 0.05;
const dReal WHEEL_EDGE_Y = 0;//WHEEL_THICKNESS + 0.05;
const dReal TRAILER_RADIUS = (14.0*2.5)/100.0;

enum
{
    LEFT_FRONT_WHEEL = 1,
    LEFT_BACK_WHEEL = 2,
    RIGHT_FRONT_WHEEL = 3,
    RIGHT_BACK_WHEEL = 4,
};

enum
{
    BOX_GEOM,
    WHEEL_GEOM,
    PLANE,
    WALL_GEOM,
    HITCH_GEOM,
};

const dReal ALL_DIR_FORCE = 10;
const dReal ALL_DIR_SPEED = 20;

const dReal MAX_SPEED = 40;
class Robot
{
public:
    Robot(dVector3 pos, dReal mass, dReal wheelMass, dReal wheelFMax, bool wide, dReal trailerWeight, bool steerable, bool sixWheels = false, bool allDirections = false, bool useWorld = false)
    : steerable(steerable), sixWheel(sixWheels), allDirections(allDirections), useWorld(useWorld)
    {
        const dReal WHEEL_RADIUS = sixWheel ? WHEEL_RADIUS_SMALL : WHEEL_RADIUS_BIG;
        const dReal ROBOT_DIMENSION_Z = WHEEL_RADIUS;
        const dReal ROBOT_WHEEL_OFFSET_Z = sixWheel ? ROBOT_DIMENSION_Z * 0.5 : ROBOT_DIMENSION_Z * 2;
        if(steerable && allDirections)
        {
            cout << "can't be steerable and all directions" << endl;
            throw runtime_error("can't be steerable and all directions");
        }

        for(int i = 0;i < 4; ++i)
        {
            curWheelSpeed[i] = 0;
        }

        myWidth = ROBOT_DIMENSION_Y;
        myHeight = ROBOT_DIMENSION_X;
        if(wide)
        {
            myWidth = ROBOT_DIMENSION_X;
            myHeight = ROBOT_DIMENSION_Y;
        }
        boxBody = dBodyCreate(world);
        dMassSetBoxTotal(&boxMass, mass, myWidth, myHeight, ROBOT_DIMENSION_Z);
        dBodySetMass(boxBody, &boxMass);
        dBodySetPosition(boxBody, pos[0], pos[1], pos[2]);
        boxGeom = dCreateBox(space, myWidth, myHeight, ROBOT_DIMENSION_Z);
        dGeomSetBody(boxGeom, boxBody);

        dMatrix3 wheelRot;//wheels are on z-axis - need to be on 9
        dRFromAxisAndAngle(wheelRot, 1, 0, 0, PI/2);

        dReal wheelMoveX = myWidth / 2;
        dReal wheelMoveY = myHeight / 2;
        if(!sixWheels)
        {
            wheelMoveX *= 0.66;
        }

        //back left wheel
        wheelBody1 = dBodyCreate(world);
        dMassSetCylinderTotal(&wheelMass1, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
        dBodySetMass(wheelBody1, &wheelMass1);
        dBodySetPosition(wheelBody1, pos[0] - wheelMoveX + WHEEL_EDGE_X, pos[1] + wheelMoveY - WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
        dBodySetRotation(wheelBody1, wheelRot);
        wheelGeom1 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
        dGeomSetBody(wheelGeom1, wheelBody1);

        //front left wheel
        wheelBody2 = dBodyCreate(world);
        dMassSetCylinderTotal(&wheelMass2, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
        dBodySetMass(wheelBody2, &wheelMass2);
        dBodySetPosition(wheelBody2, pos[0] + wheelMoveX - WHEEL_EDGE_X, pos[1] + wheelMoveY - WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
        dBodySetRotation(wheelBody2, wheelRot);
        wheelGeom2 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
        dGeomSetBody(wheelGeom2, wheelBody2);

        //back right wheel
        wheelBody3 = dBodyCreate(world);
        dMassSetCylinderTotal(&wheelMass3, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
        dBodySetMass(wheelBody3, &wheelMass3);
        dBodySetPosition(wheelBody3, pos[0] - wheelMoveX + WHEEL_EDGE_X, pos[1] - wheelMoveY + WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
        dBodySetRotation(wheelBody3, wheelRot);
        wheelGeom3 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
        dGeomSetBody(wheelGeom3, wheelBody3);

        //front right wheel
        wheelBody4 = dBodyCreate(world);
        dMassSetCylinderTotal(&wheelMass4, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
        dBodySetMass(wheelBody4, &wheelMass4);
        dBodySetPosition(wheelBody4, pos[0] + wheelMoveX - WHEEL_EDGE_X, pos[1] - wheelMoveY + WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
        dBodySetRotation(wheelBody4, wheelRot);
        wheelGeom4 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
        dGeomSetBody(wheelGeom4, wheelBody4);

        if(!steerable && !allDirections)
        {
            wheelJoint1 = dJointCreateHinge(world, 0);
            wheelJoint2 = dJointCreateHinge(world, 0);
            wheelJoint3 = dJointCreateHinge(world, 0);
            wheelJoint4 = dJointCreateHinge(world, 0);

            dJointAttach(wheelJoint1, boxBody, wheelBody1);
            dJointAttach(wheelJoint2, boxBody, wheelBody2);
            dJointAttach(wheelJoint3, boxBody, wheelBody3);
            dJointAttach(wheelJoint4, boxBody, wheelBody4);

            dJointSetHingeAnchor(wheelJoint1, dBodyGetPosition(wheelBody1)[0], dBodyGetPosition(wheelBody1)[1], dBodyGetPosition(wheelBody1)[2]);
            dJointSetHingeAnchor(wheelJoint2, dBodyGetPosition(wheelBody2)[0], dBodyGetPosition(wheelBody2)[1], dBodyGetPosition(wheelBody2)[2]);
            dJointSetHingeAnchor(wheelJoint3, dBodyGetPosition(wheelBody3)[0], dBodyGetPosition(wheelBody3)[1], dBodyGetPosition(wheelBody3)[2]);
            dJointSetHingeAnchor(wheelJoint4, dBodyGetPosition(wheelBody4)[0], dBodyGetPosition(wheelBody4)[1], dBodyGetPosition(wheelBody4)[2]);

            dJointSetHingeAxis(wheelJoint1, 0, -1, 0);
            dJointSetHingeAxis(wheelJoint2, 0, 1, 0);
            dJointSetHingeAxis(wheelJoint3, 0, -1, 0);
            dJointSetHingeAxis(wheelJoint4, 0, 1, 0);

            dJointSetHingeParam(wheelJoint1, dParamFMax, wheelFMax);
            dJointSetHingeParam(wheelJoint2, dParamFMax, wheelFMax);
            dJointSetHingeParam(wheelJoint3, dParamFMax, wheelFMax);
            dJointSetHingeParam(wheelJoint4, dParamFMax, wheelFMax);
        }
        else if(steerable)
        {
            wheelJoint1 = dJointCreateHinge(world, 0);
            wheelJoint2 = dJointCreateHinge2(world, 0);
            wheelJoint3 = dJointCreateHinge(world, 0);
            wheelJoint4 = dJointCreateHinge2(world, 0);

            dJointAttach(wheelJoint1, boxBody, wheelBody1);
            dJointAttach(wheelJoint2, boxBody, wheelBody2);
            dJointAttach(wheelJoint3, boxBody, wheelBody3);
            dJointAttach(wheelJoint4, boxBody, wheelBody4);

            dJointSetHingeAnchor(wheelJoint1, dBodyGetPosition(wheelBody1)[0], dBodyGetPosition(wheelBody1)[1], dBodyGetPosition(wheelBody1)[2]);
            dJointSetHinge2Anchor(wheelJoint2, dBodyGetPosition(wheelBody2)[0], dBodyGetPosition(wheelBody2)[1], dBodyGetPosition(wheelBody2)[2]);
            dJointSetHingeAnchor(wheelJoint3, dBodyGetPosition(wheelBody3)[0], dBodyGetPosition(wheelBody3)[1], dBodyGetPosition(wheelBody3)[2]);
            dJointSetHinge2Anchor(wheelJoint4, dBodyGetPosition(wheelBody4)[0], dBodyGetPosition(wheelBody4)[1], dBodyGetPosition(wheelBody4)[2]);

            dJointSetHingeAxis(wheelJoint1, 0, -1, 0);
            dJointSetHinge2Axis2(wheelJoint2, 0, 1, 0);
            dJointSetHinge2Axis1(wheelJoint2, 0, 0, 1);
            dJointSetHingeAxis(wheelJoint3, 0, -1, 0);
            dJointSetHinge2Axis2(wheelJoint4, 0, 1, 0);
            dJointSetHinge2Axis1(wheelJoint4, 0, 0, 1);

            dJointSetHingeParam(wheelJoint1, dParamFMax, wheelFMax);
            dJointSetHinge2Param(wheelJoint2, dParamFMax2, wheelFMax);
            dJointSetHinge2Param(wheelJoint2, dParamFMax, 10);
            dJointSetHingeParam(wheelJoint3, dParamFMax, wheelFMax);
            dJointSetHinge2Param(wheelJoint4, dParamFMax2, wheelFMax);
            dJointSetHinge2Param(wheelJoint4, dParamFMax, 10);

            dJointSetHinge2Param(wheelJoint2, dParamLoStop, -PI/4);
            dJointSetHinge2Param(wheelJoint4, dParamLoStop, -PI/4);
            dJointSetHinge2Param(wheelJoint2, dParamHiStop, PI/4);
            dJointSetHinge2Param(wheelJoint4, dParamHiStop, PI/4);
        }
        else if(allDirections)
        {
            wheelJoint1 = dJointCreateHinge2(world, 0);
            wheelJoint2 = dJointCreateHinge2(world, 0);
            wheelJoint3 = dJointCreateHinge2(world, 0);
            wheelJoint4 = dJointCreateHinge2(world, 0);

            dJointAttach(wheelJoint1, boxBody, wheelBody1);
            dJointAttach(wheelJoint2, boxBody, wheelBody2);
            dJointAttach(wheelJoint3, boxBody, wheelBody3);
            dJointAttach(wheelJoint4, boxBody, wheelBody4);

            dJointSetHinge2Anchor(wheelJoint1, dBodyGetPosition(wheelBody1)[0], dBodyGetPosition(wheelBody1)[1], dBodyGetPosition(wheelBody1)[2]);
            dJointSetHinge2Anchor(wheelJoint2, dBodyGetPosition(wheelBody2)[0], dBodyGetPosition(wheelBody2)[1], dBodyGetPosition(wheelBody2)[2]);
            dJointSetHinge2Anchor(wheelJoint3, dBodyGetPosition(wheelBody3)[0], dBodyGetPosition(wheelBody3)[1], dBodyGetPosition(wheelBody3)[2]);
            dJointSetHinge2Anchor(wheelJoint4, dBodyGetPosition(wheelBody4)[0], dBodyGetPosition(wheelBody4)[1], dBodyGetPosition(wheelBody4)[2]);

            dJointSetHinge2Axis2(wheelJoint1, 0, -1, 0);
            dJointSetHinge2Axis1(wheelJoint1, 0, 0, 1);
            dJointSetHinge2Axis2(wheelJoint2, 0, 1, 0);
            dJointSetHinge2Axis1(wheelJoint2, 0, 0, 1);
            dJointSetHinge2Axis2(wheelJoint3, 0, -1, 0);
            dJointSetHinge2Axis1(wheelJoint3, 0, 0, 1);
            dJointSetHinge2Axis2(wheelJoint4, 0, 1, 0);
            dJointSetHinge2Axis1(wheelJoint4, 0, 0, 1);

            dJointSetHinge2Param(wheelJoint1, dParamFMax, wheelFMax);
            dJointSetHinge2Param(wheelJoint1, dParamFMax2, ALL_DIR_FORCE);
            dJointSetHinge2Param(wheelJoint2, dParamFMax2, wheelFMax);
            dJointSetHinge2Param(wheelJoint2, dParamFMax, ALL_DIR_FORCE);
            dJointSetHinge2Param(wheelJoint3, dParamFMax, wheelFMax);
            dJointSetHinge2Param(wheelJoint3, dParamFMax2, ALL_DIR_FORCE);
            dJointSetHinge2Param(wheelJoint4, dParamFMax2, wheelFMax);
            dJointSetHinge2Param(wheelJoint4, dParamFMax, ALL_DIR_FORCE);
        }
        else
        {
            cout << "invalid robot type" << endl;
            throw std::runtime_error("invalid robot type");
        }

        dGeomSetData(boxGeom, (void*)BOX_GEOM);
        dGeomSetData(wheelGeom1, (void*)WHEEL_GEOM);
        dGeomSetData(wheelGeom2, (void*)WHEEL_GEOM);
        dGeomSetData(wheelGeom3, (void*)WHEEL_GEOM);
        dGeomSetData(wheelGeom4, (void*)WHEEL_GEOM);

        if(sixWheel)
        {
            //middle left wheel
            wheelBody5 = dBodyCreate(world);
            dMassSetCylinderTotal(&wheelMass5, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
            dBodySetMass(wheelBody5, &wheelMass5);
            dBodySetPosition(wheelBody5, pos[0], pos[1] + myHeight / 2 - WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
            dBodySetRotation(wheelBody5, wheelRot);
            wheelGeom5 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
            dGeomSetBody(wheelGeom5, wheelBody5);
            dGeomSetData(wheelGeom5, (void*)WHEEL_GEOM);

            //middle right wheel
            wheelBody6 = dBodyCreate(world);
            dMassSetCylinderTotal(&wheelMass6, wheelMass, 3, WHEEL_RADIUS, WHEEL_THICKNESS);
            dBodySetMass(wheelBody6, &wheelMass6);
            dBodySetPosition(wheelBody6, pos[0], pos[1] - myHeight / 2 + WHEEL_EDGE_Y, pos[2] - ROBOT_WHEEL_OFFSET_Z);
            dBodySetRotation(wheelBody6, wheelRot);
            wheelGeom6 = dCreateCCylinder(space, WHEEL_RADIUS, WHEEL_THICKNESS);
            dGeomSetBody(wheelGeom6, wheelBody6);
            dGeomSetData(wheelGeom6, (void*)WHEEL_GEOM);

            if(!allDirections)
            {
                wheelJoint5 = dJointCreateHinge(world, 0);
                wheelJoint6 = dJointCreateHinge(world, 0);

                dJointAttach(wheelJoint5, boxBody, wheelBody5);
                dJointAttach(wheelJoint6, boxBody, wheelBody6);

                dJointSetHingeAnchor(wheelJoint5, dBodyGetPosition(wheelBody5)[0], dBodyGetPosition(wheelBody5)[1], dBodyGetPosition(wheelBody5)[2]);
                dJointSetHingeAnchor(wheelJoint6, dBodyGetPosition(wheelBody6)[0], dBodyGetPosition(wheelBody6)[1], dBodyGetPosition(wheelBody6)[2]);

                dJointSetHingeAxis(wheelJoint5, 0, 1, 0);
                dJointSetHingeAxis(wheelJoint6, 0, 1, 0);

                dJointSetHingeParam(wheelJoint5, dParamFMax, wheelFMax);
                dJointSetHingeParam(wheelJoint6, dParamFMax, wheelFMax);
            }
            else
            {
                wheelJoint5 = dJointCreateHinge2(world, 0);
                wheelJoint6 = dJointCreateHinge2(world, 0);

                dJointAttach(wheelJoint5, boxBody, wheelBody5);
                dJointAttach(wheelJoint6, boxBody, wheelBody6);

                dJointSetHinge2Anchor(wheelJoint5, dBodyGetPosition(wheelBody5)[0], dBodyGetPosition(wheelBody5)[1], dBodyGetPosition(wheelBody5)[2]);
                dJointSetHinge2Anchor(wheelJoint6, dBodyGetPosition(wheelBody6)[0], dBodyGetPosition(wheelBody6)[1], dBodyGetPosition(wheelBody6)[2]);

                dJointSetHinge2Axis2(wheelJoint5, 0, 1, 0);
                dJointSetHinge2Axis1(wheelJoint5, 0, 0, 1);
                dJointSetHinge2Axis2(wheelJoint6, 0, 1, 0);
                dJointSetHinge2Axis1(wheelJoint6, 0, 0, 1);

                dJointSetHinge2Param(wheelJoint5, dParamFMax, wheelFMax);
                dJointSetHinge2Param(wheelJoint5, dParamFMax2, ALL_DIR_FORCE);
                dJointSetHinge2Param(wheelJoint6, dParamFMax2, wheelFMax);
                dJointSetHinge2Param(wheelJoint6, dParamFMax, ALL_DIR_FORCE);
            }
        }

        jointArray[0] = wheelJoint1;
        jointArray[1] = wheelJoint2;
        jointArray[2] = wheelJoint3;
        jointArray[3] = wheelJoint4;
        jointArray[4] = wheelJoint5;
        jointArray[5] = wheelJoint6;

        turnMult[0] = -1;
        turnMult[1] = 1;
        turnMult[2] = -1;
        turnMult[3] = 1;
        turnMult[4] = 1;
        turnMult[5] = 1;

        wheelBodies[0] = wheelBody1;
        wheelBodies[1] = wheelBody2;
        wheelBodies[2] = wheelBody3;
        wheelBodies[3] = wheelBody4;
        wheelBodies[4] = wheelBody5;
        wheelBodies[5] = wheelBody6;

        WHEEL_ONE_RAD_BONUS = atan((wheelMoveY) / (wheelMoveX));
    }

    virtual ~Robot()
    {
        dJointDestroy(wheelJoint1);
        dJointDestroy(wheelJoint2);
        dJointDestroy(wheelJoint3);
        dJointDestroy(wheelJoint4);
        dGeomDestroy(boxGeom);
        dBodyDestroy(boxBody);
        dGeomDestroy(wheelGeom1);
        dBodyDestroy(wheelBody1);
        dGeomDestroy(wheelGeom2);
        dBodyDestroy(wheelBody2);
        dGeomDestroy(wheelGeom3);
        dBodyDestroy(wheelBody3);
        dGeomDestroy(wheelGeom4);
        dBodyDestroy(wheelBody4);   

        if(sixWheel)
        {
            dJointDestroy(wheelJoint5);
            dGeomDestroy(wheelGeom5);
            dBodyDestroy(wheelBody5);
            dJointDestroy(wheelJoint6);
            dGeomDestroy(wheelGeom6);
            dBodyDestroy(wheelBody6);
        }  
    }

    void draw(float r, float g, float b)
    {
        const dReal WHEEL_RADIUS = sixWheel ? WHEEL_RADIUS_SMALL : WHEEL_RADIUS_BIG;
        const dReal ROBOT_DIMENSION_Z = WHEEL_RADIUS * 0.5;
        float transform[16];

        const dReal* pos = dBodyGetPosition(boxBody);
        const dReal* rot = dBodyGetRotation(boxBody);
        glPushMatrix();
            ODEtoOGL(transform, pos, rot);
            glMultMatrixf(transform);
            glScalef(myWidth, myHeight, ROBOT_DIMENSION_Z);
            drawSquare(r,g,b);
        glPopMatrix();

        pos = dBodyGetPosition(wheelBody1);
        rot = dBodyGetRotation(wheelBody1);
        glPushMatrix();
            ODEtoOGL(transform, pos, rot);
            glMultMatrixf(transform);
            glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
            drawCylinder(false);
        glPopMatrix();

        pos = dBodyGetPosition(wheelBody2);
        rot = dBodyGetRotation(wheelBody2);
        glPushMatrix();
            ODEtoOGL(transform, pos, rot);
            glMultMatrixf(transform);
            glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
            drawCylinder(false);
        glPopMatrix();

        pos = dBodyGetPosition(wheelBody3);
        rot = dBodyGetRotation(wheelBody3);
        glPushMatrix();
            ODEtoOGL(transform, pos, rot);
            glMultMatrixf(transform);
            glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
            drawCylinder(false);
        glPopMatrix();

        pos = dBodyGetPosition(wheelBody4);
        rot = dBodyGetRotation(wheelBody4);
        glPushMatrix();
            ODEtoOGL(transform, pos, rot);
            glMultMatrixf(transform);
            glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
            drawCylinder(false);
        glPopMatrix();

        if(sixWheel)
        {
            pos = dBodyGetPosition(wheelBody5);
            rot = dBodyGetRotation(wheelBody5);
            glPushMatrix();
                ODEtoOGL(transform, pos, rot);
                glMultMatrixf(transform);
                glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
                drawCylinder(false);
            glPopMatrix();

            pos = dBodyGetPosition(wheelBody6);
            rot = dBodyGetRotation(wheelBody6);
            glPushMatrix();
                ODEtoOGL(transform, pos, rot);
                glMultMatrixf(transform);
                glScalef(WHEEL_RADIUS, WHEEL_RADIUS, WHEEL_THICKNESS);
                drawCylinder(false);
            glPopMatrix();
        }
    }

    virtual void update()
    {
    }

    void setWheelSpeed(int num, dReal vel)
    {
        if(num > 4 && !sixWheel)
            return;

        dJointID theJoint;
        dReal mult = 1.0f;
        switch(num)
        {
        case 1:
            theJoint = wheelJoint1;
            break;
        case 2:
            theJoint = wheelJoint2;
            mult = -1.0f;
            break;
        case 3:
            theJoint = wheelJoint3;
            break;
        case 4:
            theJoint = wheelJoint4;
            mult = -1.0f;
            break;
        case 5:
            theJoint = wheelJoint5;
            mult = -1.0f;
            break;
        case 6:
            theJoint = wheelJoint6;
            mult = -1.0f;
            break;
        default:
            return;
        }
        if(!steerable && !allDirections)
        {
            dJointSetHingeParam(theJoint, dParamVel, vel * mult);
        }
        else if(steerable)
        {
            if(num == 1 || num == 3 || num == 5 || num == 6)
            {
                dJointSetHingeParam(theJoint, dParamVel, vel * mult);
            }
            else
            {
                dJointSetHinge2Param(theJoint, dParamVel2, vel * mult);
            }
        }
        else if(allDirections)
        {
            dJointSetHinge2Param(theJoint, dParamVel2, vel * mult);
        }
        else
        {
            cout << "setWheelSpeed -- doing nothing" << endl;
            throw std::runtime_error("setWheelSpeed -- doing nothing");
        }
    }

    void setWheelSpeedAll(dReal vel)
    {
        for(int i = 1; i <= 6; ++i)
        {
            setWheelSpeed(i, vel);
        }
    }

    void setWheelSpeedLeft(dReal vel)
    {
        setWheelSpeed(1, vel);
        setWheelSpeed(2, vel);
        setWheelSpeed(5, vel);
    }

    void setWheelSpeedRight(dReal vel)
    {
        setWheelSpeed(3, vel);
        setWheelSpeed(4, vel);
        setWheelSpeed(6, vel);
    }

    virtual void setThrottles(dReal timeDiff, dReal left, dReal right)
    {
        if(steerable || allDirections)
        {
            cout << "don't set throttles for steerable robots" <<  endl;
        }
        setWheelSpeedLeft(left*MAX_SPEED);
        setWheelSpeedRight(right*MAX_SPEED);
    }

    void setThrottle(dReal timeDiff, dReal speed)
    {
        setWheelSpeedAll(speed * MAX_SPEED);
    }

    dReal getWorldRot()
    {
        //take the dot product with x,y = 1, 0 (since 0 degree's points in +x direction)
        const dReal* bodyPos = dBodyGetPosition(boxBody);
        const dReal* wheel1Pos = dBodyGetPosition(wheelBody1);
        const dReal diff[2] = { wheel1Pos[0] - bodyPos[0], wheel1Pos[1] - bodyPos[1] };
        const dReal dotProd = diff[0];
        const dReal diffMag = sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
        dReal theta = acos(dotProd / diffMag);
        if(diff[1] > 0)
        {
            theta *= -1.0;
        }
        return theta - WHEEL_ONE_RAD_BONUS + PI;
    }

    void setSteering(dReal dir)
    {
        if(!steerable && !allDirections)
        {
            cout << "don't set steering for non-steerable robots" << endl;
        }

        if(steerable)
        {
		    if(dir < PI/8 && dir > -PI/8)
		    {
			    dir = 0;
		    }

            if(dir == 0)
            {
                dReal newSpeed = 0;
                dReal curAngle = dJointGetHinge2Angle1(wheelJoint2);
                if(curAngle > EPSILON || curAngle < -EPSILON)
                {
                    newSpeed = -10.0 * curAngle;
                }
                dJointSetHinge2Param(wheelJoint2, dParamVel1, newSpeed);

                curAngle = dJointGetHinge2Angle1(wheelJoint4);
                if(curAngle > EPSILON || curAngle < -EPSILON)
                {
                    newSpeed = -10.0 * curAngle;
                }
                dJointSetHinge2Param(wheelJoint4, dParamVel1, newSpeed);
            }
            else
            {
                dJointSetHinge2Param(wheelJoint2, dParamVel1, dir * 10.0);
                dJointSetHinge2Param(wheelJoint4, dParamVel1, dir * 10.0);
            }
        }
        else if(allDirections)
        {
            const dReal worldRot = getWorldRot();
            const int maxWheel = sixWheel ? 6 : 4;            
            for(int i = 0; i < maxWheel; ++i)
            {
                dReal curDir = dJointGetHinge2Angle1(jointArray[i]);
                if(useWorld)
                {
                    curDir += worldRot;
                }
                if(curDir > dir + EPSILON_BIG)
                {
                    dJointSetHinge2Param(jointArray[i], dParamVel1, -ALL_DIR_SPEED);// * turnMult[i]);
                }
                else if(curDir < dir - EPSILON_BIG)
                {
                    dJointSetHinge2Param(jointArray[i], dParamVel1, ALL_DIR_SPEED);// * turnMult[i]);
                }
                else
                {
                    dJointSetHinge2Param(jointArray[i], dParamVel1, 0);
                }
            }
        }
        else
        {
            cout << "setSteering -- doing nothing" << endl;
            throw std::runtime_error("setSteering -- doing nothing");
        }
    }

    bool isSteerable()
    {
        return steerable;
    }

    bool isAllDirections()
    {
        return allDirections;
    }

private:
    bool steerable;

    dGeomID boxGeom;
    dBodyID boxBody;
    dMass boxMass;

    dMass wheelMass1;
    dGeomID wheelGeom1;
    dBodyID wheelBody1;
    dJointID wheelJoint1;

    dMass wheelMass2;
    dGeomID wheelGeom2;
    dBodyID wheelBody2;
    dJointID wheelJoint2;

    dMass wheelMass3;
    dGeomID wheelGeom3;
    dBodyID wheelBody3;
    dJointID wheelJoint3;

    dMass wheelMass4;
    dGeomID wheelGeom4;
    dBodyID wheelBody4;
    dJointID wheelJoint4;

    dMass wheelMass5;
    dGeomID wheelGeom5;
    dBodyID wheelBody5;
    dJointID wheelJoint5;

    dMass wheelMass6;
    dGeomID wheelGeom6;
    dBodyID wheelBody6;
    dJointID wheelJoint6;

    dReal myWidth;
    dReal myHeight;

    dReal curWheelSpeed[4];
    bool sixWheel;
    bool allDirections;
    dJointID jointArray[6];
    double turnMult[6];
    dBodyID wheelBodies[6];
    dReal WHEEL_ONE_RAD_BONUS;
    bool useWorld;
};

const int MAX_CONTACTS = 256;
dJointGroupID contactGroup = 0;

const dReal FLOOR_WHEEL_FRICTION = 0.5;
const dReal STANDARD_FRICTION = 0.5;
const dReal BOUNCINESS = 0;
 
void doCollide(void *data, dGeomID o1, dGeomID o2)
{
    if((dGeomGetData(o1) == (void*)BOX_GEOM && dGeomGetData(o2) == (void*)WHEEL_GEOM)
        || (dGeomGetData(o1) == (void*)WHEEL_GEOM && dGeomGetData(o2) == (void*)BOX_GEOM)
        || (dGeomGetData(o1) == (void*)WHEEL_GEOM && dGeomGetData(o2) == (void*)WHEEL_GEOM))
    {
        return;
    }

    if(dGeomGetData(o1) == (void*)WALL_GEOM && dGeomGetData(o2) == (void*)WALL_GEOM)
    {
        return;
    }

    if((dGeomGetData(o1) == (void*)WALL_GEOM && dGeomGetData(o2) == (void*)PLANE)
       || (dGeomGetData(o1) == (void*)PLANE && dGeomGetData(o2) == (void*)WALL_GEOM))
    {
        return;
    }

    if(dGeomGetData(o1) == (void*)HITCH_GEOM || dGeomGetData(o2) == (void*)HITCH_GEOM)
    {
        return;
    }

    dBodyID body1 = dGeomGetBody(o1);
    dBodyID body2 = dGeomGetBody(o2);

    dContact contact[MAX_CONTACTS];


    dReal friction = STANDARD_FRICTION;
    if((dGeomGetData(o1) == (void*)PLANE && dGeomGetData(o2) == (void*)WHEEL_GEOM)
        || (dGeomGetData(o1) == (void*)WHEEL_GEOM && dGeomGetData(o2) == (void*)PLANE))
    {
        friction = FLOOR_WHEEL_FRICTION;
    }
 
    for (int i = 0; i < MAX_CONTACTS; i++)
    {
        contact[i].surface.mode = dContactBounce | dContactApprox1;
        contact[i].surface.bounce = BOUNCINESS;
        contact[i].surface.mu = friction;
    }
 
    int collisions = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    if (collisions)
    {
        for (int i = 0; i < collisions; ++i)
        {
            dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
            dJointAttach(c, body1, body2);
        }
    }
}

const int NUM_ROBOTS = 6;
shared_ptr<Robot> robots[NUM_ROBOTS];
float robotColours[NUM_ROBOTS][3] =
            {
                {0, 0, 1},
                {0, 1, 0},
                {0, 1, 1},
                {1, 0, 0},
                {1, 0, 1},
                {1, 1, 0},
            };

dReal fieldWidth = 16.4592;
dReal fieldHeight = 8.2296;
dReal FOV = PI/4;
dReal viewPointZ = (fieldWidth / tanf(FOV)) * 0.9;
dReal viewPointY = viewPointZ * tanf(PI/8.0);

dReal WALL_LENGTH = 50;
dReal WALL_THICKNESS = 1;

dReal BUMP_PART_WIDTH = 0.3048;
dReal BUMP_HALF_WIDTH = 0.5 * BUMP_PART_WIDTH;
dReal BUMP_ANGLE_PART_WIDTH = sqrt(2.0 * (BUMP_PART_WIDTH * BUMP_PART_WIDTH));

dReal wallPositions[4][3] =
            {
                {fieldWidth/2 + WALL_THICKNESS/2, 0, WALL_THICKNESS/2 + EPSILON},
                {-fieldWidth/2 - WALL_THICKNESS/2, 0, WALL_THICKNESS/2 + EPSILON},
                {0, fieldHeight/2 + WALL_THICKNESS/2, WALL_THICKNESS/2 + EPSILON},
                {0, -fieldHeight/2 - WALL_THICKNESS/2, WALL_THICKNESS/2 + EPSILON},
            };

dReal wallRotations[4][16];

dBodyID wallBodies[4];
dGeomID wallGeoms[4];
dMass wallMasses[4];
dJointID wallJoints[4];
float wallColour[4] = { 0.5, 0.75, 0.5, 0.75 };

dBodyID bumpBodies[6];
dGeomID bumpGeoms[6];
dMass bumpMasses[6];
dReal bumpRotations[6][16];
dJointID bumpJoints[6];

dReal bumpOffset = 0.5 * fieldWidth / 3.0;
dReal bumpPositions[6][3] =
            {
                {-bumpOffset - BUMP_HALF_WIDTH, 0, 0},
                {-bumpOffset, 0, BUMP_PART_WIDTH * 0.5},
                {-bumpOffset + BUMP_HALF_WIDTH, 0, 0},
                {bumpOffset - BUMP_HALF_WIDTH, 0, 0},
                {bumpOffset, 0, BUMP_PART_WIDTH * 0.5},
                {bumpOffset + BUMP_HALF_WIDTH, 0, 0},
            };

dReal bumpWidth[6] =
            {
                BUMP_ANGLE_PART_WIDTH,
                BUMP_PART_WIDTH,
                BUMP_ANGLE_PART_WIDTH,
                BUMP_ANGLE_PART_WIDTH,
                BUMP_PART_WIDTH,
                BUMP_ANGLE_PART_WIDTH,
            };

float bumpColour[6] = { 0.5, 0.75, 0.5, 0.5, 0.75, 0.5 };

dReal leftThrottle = 0;
dReal rightThrottle = 0;

dReal steerThrottle = 0;
dReal steerDir = 0;

dReal angle=0.0;
DWORD lastTime = timeGetTime();
void renderScene(void)
{
    DWORD curTime = timeGetTime();
    DWORD deltaTime = curTime - lastTime;
    dReal delta = ((dReal)deltaTime) / 1000.0;
    lastTime = curTime;

    dReal stepsize = delta;
    if(stepsize > 1.0/60.0)
    {
        stepsize = 1.0/60.0;
    }

    dJointGroupEmpty(contactGroup);
    dSpaceCollide(space, NULL, &doCollide);

    dWorldQuickStep(world, stepsize);

    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);
    glColor3f(0.25, 0.25, 0.25);
    glBegin(GL_TRIANGLE_STRIP);
        glVertex3f(-1000, -1000, 0);
        glVertex3f(-1000, 1000, 0);
        glVertex3f(1000, -1000, 0);
        glVertex3f(1000, 1000, 0);
    glEnd();

    glColor3f(0.05, 0.05, 0.05);
    for(dReal x = -20; x < 20 + EPSILON; x += 2)
    {
        glPushMatrix();
            glTranslatef(x, 0, 0);
            glScalef(1, 2000, 1);
            drawPanel();
        glPopMatrix();
    }

    for(dReal y = -20; y < 20 + EPSILON; y += 2)
    {
        glPushMatrix();
            glTranslatef(0, y, 0);
            glScalef(2000, 1, 1);
            drawPanel();
        glPopMatrix();
    }

    glEnable(GL_DEPTH_TEST);

    float transform[16];
    for(int i = 0; i < 4; ++i)
    {
        glPushMatrix();
            ODEtoOGL(transform, dBodyGetPosition(wallBodies[i]), dBodyGetRotation(wallBodies[i]));
            glMultMatrixf(transform);
            glScalef(WALL_LENGTH, WALL_THICKNESS, WALL_THICKNESS);
            drawSquare(0, wallColour[i], 0);
        glPopMatrix();
    }

    for(int i = 0; i < 6; ++i)
    {
        glPushMatrix();
            ODEtoOGL(transform, dBodyGetPosition(bumpBodies[i]), dBodyGetRotation(bumpBodies[i]));
            glMultMatrixf(transform);
            glScalef(WALL_LENGTH, bumpWidth[i], bumpWidth[i]);
            drawSquare(i >= 3 ? bumpColour[i] : 0, 0, i < 3 ? bumpColour[i] : 0);
        glPopMatrix();
    }

    for(int i = 0; i < NUM_ROBOTS; ++i)
    {
        if(robots[i])
        {
            glPushMatrix();
                robots[i]->draw(robotColours[i][0], robotColours[i][1], robotColours[i][2]);
                robots[i]->update();
            glPopMatrix();
        }
    }

	updateControllerStates();
    for(int i = 0; i < NUM_ROBOTS; ++i)
    {
		if(robots[i] && i < MAX_CONTROLLERS && !playerOneSwitch)
		{
			if(!robots[i]->isSteerable() && !robots[i]->isAllDirections())
			{
				robots[i]->setThrottles(delta, joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y], joysticks[i][RIGHT_JOYSTICK][JOYSTICK_Y]);
			}
			else if(robots[i]->isSteerable())
			{
				robots[i]->setThrottle(delta, joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y]);
				robots[i]->setSteering(joysticks[i][LEFT_JOYSTICK][JOYSTICK_X]);
			}
            else if(robots[i]->isAllDirections())
            {
                const dReal throttle = sqrt(joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y] * joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y]
                                                + joysticks[i][LEFT_JOYSTICK][JOYSTICK_X] * joysticks[i][LEFT_JOYSTICK][JOYSTICK_X]);
                const dReal dir = atan2(-joysticks[i][LEFT_JOYSTICK][JOYSTICK_Y], joysticks[i][LEFT_JOYSTICK][JOYSTICK_X]);
                robots[i]->setThrottle(delta, throttle);
				robots[i]->setSteering(dir);
            }
		}
    }

    if(playerOneSwitch)
    {
        if(playerOneControlling >= NUM_ROBOTS)
            playerOneControlling = 0;

        if(robots[playerOneControlling])
		{
			if(!robots[playerOneControlling]->isSteerable() && !robots[playerOneControlling]->isAllDirections())
			{
				robots[playerOneControlling]->setThrottles(delta, joysticks[0][LEFT_JOYSTICK][JOYSTICK_Y], joysticks[0][RIGHT_JOYSTICK][JOYSTICK_Y]);
			}
			else if(robots[playerOneControlling]->isSteerable())
			{
				robots[playerOneControlling]->setThrottle(delta, joysticks[0][LEFT_JOYSTICK][JOYSTICK_Y]);
				robots[playerOneControlling]->setSteering(joysticks[0][LEFT_JOYSTICK][JOYSTICK_X]);
			}
            else if(robots[playerOneControlling]->isAllDirections())
            {
                const dReal throttle = sqrt(joysticks[0][LEFT_JOYSTICK][JOYSTICK_Y] * joysticks[0][LEFT_JOYSTICK][JOYSTICK_Y]
                                                + joysticks[0][LEFT_JOYSTICK][JOYSTICK_X] * joysticks[0][LEFT_JOYSTICK][JOYSTICK_X]);
                const dReal dir = atan2(-joysticks[0][LEFT_JOYSTICK][JOYSTICK_Y], joysticks[0][LEFT_JOYSTICK][JOYSTICK_X]);
                robots[playerOneControlling]->setThrottle(delta, throttle);
				robots[playerOneControlling]->setSteering(dir);
            }
		}
    }

    glFlush();

    glutSwapBuffers();
    angle += 0.1;
}

void changeSize(int w, int h)
{
    if(h == 0)
        h = 1;

    dReal ratio = 1.0 * w / h;

    // Reset the coordinate system before modifying
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // Set the viewport to be the entire window
    glViewport(0, 0, w, h);

    // Set the correct perspective.
    gluPerspective(45,ratio,1,1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,-viewPointY,viewPointZ,
              0.0,0.0,0.0,
              0.0f,1.0f,0.0f);
}

void handleKeyDown(unsigned char key, int x, int y)
{
    switch(key)
    {
    case 'q':
        leftThrottle = 1;
        break;
    case 'w':
        rightThrottle = 1;
        break;
    case 'a':
        leftThrottle = -1;
        break;
    case 's':
        rightThrottle = -1;
        break;
    default:
        return;
    }
}

void handleKeyUp(unsigned char key, int x, int y)
{
    switch(key)
    {
    case 'q':
        leftThrottle = 0;
        break;
    case 'w':
        rightThrottle = 0;
        break;
    case 'a':
        leftThrottle = 0;
        break;
    case 's':
        rightThrottle = 0;
        break;
    default:
        return;
    }
}

void handleArrowsDown(int key, int x, int y)
{
    switch(key)
    {
    case GLUT_KEY_UP:
        steerThrottle = 1;
        break;
    case GLUT_KEY_DOWN:
        steerThrottle = -1;
        break;
    case GLUT_KEY_LEFT:
        steerDir = -1;
        break;
    case GLUT_KEY_RIGHT:
        steerDir = 1;
        break;
    default:
        return;
    }
}

void handleArrowsUp(int key, int x, int y)
{
    switch(key)
    {
    case GLUT_KEY_DOWN:
        steerThrottle = 0;
        break;
    case GLUT_KEY_UP:
        steerThrottle = 0;
        break;
    case GLUT_KEY_LEFT:
        steerDir = 0;
        break;
    case GLUT_KEY_RIGHT:
        steerDir = 0;
        break;
    default:
        return;
    }
}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(800,600);
    glutCreateWindow("runnymede robot sim");
    glutDisplayFunc(renderScene);
    glutReshapeFunc(changeSize);
    glutIdleFunc(renderScene);
    glutKeyboardFunc(handleKeyDown);
    glutKeyboardUpFunc(handleKeyUp);
    glutSpecialUpFunc(handleArrowsUp);
    glutSpecialFunc(handleArrowsDown);
    glEnable(GL_DEPTH_TEST);

    dInitODE();

    world = dWorldCreate();

    space = dSimpleSpaceCreate(NULL);

    contactGroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.81);

    dGeomID plane = dCreatePlane(space, 0, 0, 1, 0);//create xy-plane to drive on
    dGeomSetData(plane, (void*)PLANE);

    dVector3 robotPos[NUM_ROBOTS] =
        {
            {-6, 3, 1.5},
            {-6, 0, 1.5},
            {-6, -3, 1.5},
            {6, 3, 1.5},
            {6, 0, 1.5},
            {6, -3, 1.5},
        };

    const dReal ROBOT_MASS = 54.4310844;
    const dReal TRAILER_MASS = ROBOT_MASS / 3.0;
    const dReal HITCH_LENGTH = 0.25;

    robots[0] = shared_ptr<Robot>(new Robot(robotPos[0], ROBOT_MASS, 0.3, 10, true, TRAILER_MASS, false, true, false));//120 lbs)
//    robots[0]->setWheelSpeedAll(40);

    robots[1] = shared_ptr<Robot>(new Robot(robotPos[1], ROBOT_MASS, 0.3, 10, false, TRAILER_MASS, false, true, false));//120 lbs)
//    robots[1]->setWheelSpeedAll(45);

    robots[2] = shared_ptr<Robot>(new Robot(robotPos[2], ROBOT_MASS, 0.3, 10, true, TRAILER_MASS, false, false, true));//120 lbs)
//    robots[2]->setWheelSpeedAll(25);

    robots[3] = shared_ptr<Robot>(new Robot(robotPos[3], ROBOT_MASS, 0.3, 10, false, TRAILER_MASS, false, false, true));//120 lbs)
//    robots[3]->setWheelSpeedAll(1);

    robots[4] = shared_ptr<Robot>(new Robot(robotPos[4], ROBOT_MASS, 0.3, 10, true, TRAILER_MASS, false, true, true, true));//120 lbs)
//    robots[4]->setWheelSpeedAll(1);

    robots[5] = shared_ptr<Robot>(new Robot(robotPos[5], ROBOT_MASS, 0.3, 10, false, TRAILER_MASS, false, false, false));//120 lbs)

    dRFromAxisAndAngle(wallRotations[0], 0, 0, 1, PI/2);
    dRFromAxisAndAngle(wallRotations[1], 0, 0, 1, PI/2);
    dRFromAxisAndAngle(wallRotations[2], 0, 0, 1, 0);
    dRFromAxisAndAngle(wallRotations[3], 0, 0, 1, 0);

    for(int i = 0; i < 4; ++i)
    {
        wallBodies[i] = dBodyCreate(world);
        dMassSetBoxTotal(&wallMasses[i], 10000, WALL_LENGTH, WALL_THICKNESS, WALL_THICKNESS);
        dBodySetMass(wallBodies[i], &wallMasses[i]);
        dBodySetPosition(wallBodies[i], wallPositions[i][0], wallPositions[i][1], wallPositions[i][2]);
        dBodySetRotation(wallBodies[i], wallRotations[i]);
        wallGeoms[i] = dCreateBox(space, WALL_LENGTH, WALL_THICKNESS, WALL_THICKNESS);
        dGeomSetBody(wallGeoms[i], wallBodies[i]);
        dGeomSetData(wallGeoms[i], (void*)WALL_GEOM);
        wallJoints[i] = dJointCreateFixed(world, 0);
        dJointAttach(wallJoints[i], wallBodies[i], 0);
        dJointSetFixed(wallJoints[i]);
    }

    dQuaternion zRot;
    dQFromAxisAndAngle(zRot, 0, 0, 1, PI * 0.5);
    dQuaternion yRot;
    dQFromAxisAndAngle(yRot, 0, 1, 0, PI * 0.25);
    dQuaternion combinedRot;
    dQMultiply0(combinedRot, yRot, zRot);
    for(int i = 0; i < 6; ++i)
    {
        if(i != 1 && i != 4)
        {
            dQtoR(combinedRot, bumpRotations[i]);
        }
        else
        {
            dRFromAxisAndAngle(bumpRotations[i], 0, 0, 1, PI * 0.5);
        }

        bumpBodies[i] = dBodyCreate(world);
        dMassSetBoxTotal(&bumpMasses[i], 10000, WALL_LENGTH, bumpWidth[i], bumpWidth[i]);
        dBodySetMass(bumpBodies[i], &bumpMasses[i]);
        dBodySetPosition(bumpBodies[i], bumpPositions[i][0], bumpPositions[i][1], bumpPositions[i][2]);
        dBodySetRotation(bumpBodies[i], bumpRotations[i]);
        bumpGeoms[i] = dCreateBox(space, WALL_LENGTH, bumpWidth[i], bumpWidth[i]);
        dGeomSetBody(bumpGeoms[i], bumpBodies[i]);
        dGeomSetData(bumpGeoms[i], (void*)WALL_GEOM);
        bumpJoints[i] = dJointCreateFixed(world, 0);
        dJointAttach(bumpJoints[i], bumpBodies[i], 0);
        dJointSetFixed(bumpJoints[i]);
    }

    glutMainLoop();

    dJointGroupDestroy(contactGroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);

    dCloseODE();

    return 0;
}

//measured ~0.1 coeff of friction

