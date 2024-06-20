//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <chrono>

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#else
#include <signal.h>
#endif

#include "rec/robotino/api2/all.h"
#include "PID_control.h"

using namespace rec::robotino::api2;

bool _run = true;

#ifdef WIN32 
static BOOL WINAPI sigint_handler( DWORD fdwCtrlType ) 
{ 
	switch( fdwCtrlType ) 
	{  
	case CTRL_C_EVENT: // Handle the CTRL-C signal.
		_run = false;
		return TRUE;

	default: 
		return FALSE; 
	} 
} 
#else
void sigint_handler( int signum )
{
	_run = false;
}
#endif

class MyCom : public Com
{
public:
	MyCom()
		: Com( "example_circle" )
	{
	}

	void errorEvent( const char* errorString )
	{
		std::cerr << "Error: " << errorString << std::endl;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
	}

	void logEvent( const char* message, int level )
	{
		std::cout << message << std::endl;
	}

	void pingEvent( float timeMs )
	{
		std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

class MyBumper : public Bumper
{
public:
	MyBumper()
		: bumped( false )
	{
	}

	void bumperEvent( bool hasContact )
	{
		bumped |= hasContact;
		std::cout << "Bumper has " << ( hasContact ? "contact" : "no contact") << std::endl;
	}

	bool bumped;
};


MyCom com;
OmniDrive omniDrive;
MyBumper bumper;
Motor motor;

//rotate vector in by deg degrees and store the output in out
void rotate( const float* in, float* out, float deg )
{
	float rad = rec::robotino::api2::deg2rad( deg );

	out[0] = cos( rad ) * in[0] - sin( rad ) * in[1];
	out[1] = sin( rad ) * in[0] + cos( rad ) * in[1];
}

void init( const std::string& hostname )
{
	// Initialize the actors

	// Connect
	std::cout << "connecting to: " << hostname.c_str() << std::endl;
	
	com.setAddress( hostname.c_str() );

	com.connectToServer( true );

	if( false == com.isConnected() )
	{
		std::cout << std::endl << "Could not connect to " << com.address() << std::endl;
#ifdef WIN32
		std::cout << "Press any key to exit..." << std::endl;
		rec::robotino::api2::waitForKey();
#endif
		rec::robotino::api2::shutdown();
		exit( 1 );
	}
	else
	{
		std::cout << "success" << std::endl;
		omniDrive.setComId(com.id());
	}
}

void drive()
{
	const float startVector[2] = {0.2f, 0.0f};
	float dir[2];
	float a = 0.0f;

	while( com.isConnected() && false == bumper.value() && _run )
	{
		//rotate 360degrees in 5s
		rotate( startVector, dir, a );
		a = 360.0f * com.msecsElapsed() / 5000;
		omniDrive.setVelocity(dir[0], dir[1], 0);

		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}
}

void drive8()
{
	float radius = 0.3f;
	float linear_vel = 0.25f;
	float duration = (2 * rec::robotino::api2::PI * radius) / linear_vel / 2;

	float angular_vel = linear_vel / radius;
	auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<float>(duration);

	while( com.isConnected() && false == bumper.value() && _run )
	{
		if (std::chrono::steady_clock::now() < end_time) {
			omniDrive.setVelocity(linear_vel, 0.0f, angular_vel);
		}
		else {
			end_time = std::chrono::steady_clock::now() + std::chrono::duration<float>(duration);
			angular_vel = -angular_vel;
		}

		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}
}

void driveRot()
{
    while( com.isConnected() && false == bumper.value() && _run )
    {
        omniDrive.setVelocity(0.0f, 0.0f, -atan2(0.0, -2.0));

        com.processEvents();
        rec::robotino::api2::msleep( 100 );
    }
}

void drivePID() {
	float waypoints[2] = {0.0, 3.0};
	float kp = 0.2f;
	float ki = 0.1f;
	float kd = 0.1f;
	float dt = 0.1f;

	PID pid_linear(kp, ki, kd, dt); // 前进方向的PID控制器
	State state;

	float x, y, theta= 0; // world position
	while( com.isConnected() && false == bumper.value() && _run )
	{	
		if (abs(waypoints[1] - y) < 0.1f) {
			omniDrive.setVelocity(0.0f, 0.0f, 0.0f);
			break;
		}
		// get current world position
		state.getCurState(x, y, theta);
		// calculate speed
		float vel = pid_linear.calculate(waypoints[1], y, dt);
		// update world position at next timestep
		state.update(0.0, vel, 0.0f, dt);

		omniDrive.setVelocity(vel, 0.0f, 0.0f);

		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}
}

void driveMotor() {
	motor.setComId(com.id());
	motor.setMotorNumber(1);
	while( com.isConnected() && false == bumper.value() && _run )
    {
        motor.setSpeedSetPoint(0.05f);

        com.processEvents();
        rec::robotino::api2::msleep( 100 );
    }
}

void destroy()
{
	com.disconnectFromServer();
}

int main( int argc, char **argv )
{
	std::string hostname = "0.0.0.0";
	if( argc > 1 )
	{
		hostname = argv[1];
	}

#ifdef WIN32
	::SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sigint_handler, TRUE );
#else
	struct sigaction act;
	memset( &act, 0, sizeof( act ) );
	act.sa_handler = sigint_handler;
	sigaction( SIGINT, &act, NULL );
#endif

	try
	{
		init( hostname );
		drivePID();
		// driveMotor();
		destroy();
	}
	catch( const rec::robotino::api2::RobotinoException& e )
	{
		std::cerr << "Com Error: " << e.what() << std::endl;
	}
	catch( const std::exception& e )
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch( ... )
	{
		std::cerr << "Unknow Error" << std::endl;
	}

	rec::robotino::api2::shutdown();

#ifdef WIN32
	std::cout << "Press any key to exit..." << std::endl;
	rec::robotino::api2::waitForKey();
#endif
}
