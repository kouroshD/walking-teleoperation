/**
* @file VirtualizerModule.cpp
* @authors  Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
*           Giulio Romualdi <giulio.romualdi@iit.it>
* @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
*            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
* @date 2018
*/

// YARP

#define _USE_MATH_DEFINES


#include "yarp/os/LogStream.h"
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include "VirtualizerModule.hpp"
#include "Utils.hpp"


bool VirtualizerModule::configureVirtualizer()
{
	// try to connect to the virtualizer
	int maxAttempt = 5;
	for (int i = 0; i < maxAttempt; i++)
	{
		m_cvirtDeviceID = CVirt::FindDevice();
		if (m_cvirtDeviceID != nullptr)
		{
			if (!m_cvirtDeviceID->Open())
			{
				yError() << "[configureVirtualizer] Unable to open the device";
				return false;
			}
			return true;
		}
		// wait one millisecond
		yarp::os::Time::delay(0.001);
	}

	yError() << "[configureVirtualizer] I'm not able to configure the virtualizer";
	return false;
}

bool VirtualizerModule::configure(yarp::os::ResourceFinder &rf)
{
	yarp::os::Value* value;
	velocity_factor = 2;
	// check if the configuration file is empty
	if (rf.isNull())
	{
		yError() << "[configure] Empty configuration for the force torque sensors.";
		return false;
	}

	// get the period
	m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();

	// set the module name
	std::string name;
	if (!YarpHelper::getStringFromSearchable(rf, "name", name))
	{
		yError() << "[configure] Unable to get a string from a searchable";
		return false;
	}
	setName(name.c_str());

	// set deadzone
	if (!YarpHelper::getDoubleFromSearchable(rf, "deadzone", m_deadzone))
	{
		yError() << "[configure] Unable to get a double from a searchable";
		return false;
	}

	// open ports
	std::string portName;
	if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort_name", portName))
	{
		yError() << "[configure] Unable to get a string from a searchable";
		return false;
	}

	if (!m_playerOrientationPort.open("/" + getName() + portName))
	{
		yError() << "[configure] " << portName << " port already open.";
		return false;
	}

	if (!YarpHelper::getStringFromSearchable(rf, "robotOrientationPort_name", portName))
	{
		yError() << "[configure] Unable to get a string from a searchable";
		return false;
	}
	if (!m_robotOrientationPort.open("/" + getName() + portName))
	{
		yError() << "[configure] " << portName << " port already open.";
		return false;
	}

	if (!YarpHelper::getStringFromSearchable(rf, "rpcPort_name", portName))
	{
		yError() << "[configure] Unable to get a string from a searchable";
		return false;
	}
	if (!m_rpcPort.open("/" + getName() + portName))
	{
		yError() << "[configure] " << portName << " port already open.";
		return false;
	}


	if (!configureVirtualizer())
	{
		yError() << "[configure] Unable to configure the virtualizer";
		return false;
	}

	// remove me!!!
	// this is because the virtualizer is not ready 
	yarp::os::Time::delay(0.5);

	// reset some quanties
	m_robotYaw = 0;
	oldPlayerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());
	oldPlayerYaw *= 360.0f;
	oldPlayerYaw = oldPlayerYaw * M_PI / 180;
	oldPlayerYaw = Angles::normalizeAngle(oldPlayerYaw);

	m_offset = oldPlayerYaw;

	yInfo() << "First player yaw: " << oldPlayerYaw;
	return true;
}

double VirtualizerModule::getPeriod()
{
	return m_dT;
}

bool VirtualizerModule::close()
{
	// close the ports
	m_rpcPort.close();
	m_robotOrientationPort.close();
	m_playerOrientationPort.close();

	// deallocate memory
	delete m_cvirtDeviceID;

	return true;
}

bool VirtualizerModule::updateModule()
{
	// get data from virtualizer
	double playerYaw;
	playerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());

	playerYaw *= 360.0f;
	playerYaw = playerYaw * M_PI / 180;
	playerYaw = Angles::normalizeAngle(playerYaw);


	// get the robot orientation
	yarp::sig::Vector *tmp = m_robotOrientationPort.read(false);
	if (tmp != NULL)
	{
		auto vector = *tmp;
		m_robotYaw = -Angles::normalizeAngle(vector[0]);
	}
	if (fabs(Angles::shortestAngularDistance(playerYaw, oldPlayerYaw)) > 0.15)
	{
		yError() << "Virtualizer misscalibrated or disconnected";
		return false;
	}
	oldPlayerYaw = playerYaw;

	double playerYawAfterOffset = Angles::shortestAngularDistance(m_offset, playerYaw);
	// error between the robot orientation and the player orientation
	double angularError = threshold(Angles::shortestAngularDistance(m_robotYaw, playerYawAfterOffset));

	yInfo() << "Current player yaw: " << playerYaw << " after offset: " << playerYawAfterOffset;

	// get the player speed
	double speedData = (double)(m_cvirtDeviceID->GetMovementSpeed());

	double x = speedData * cos(angularError) * velocity_factor;
	double y = speedData * sin(angularError) * velocity_factor;

	// send data to the walking module
	yarp::os::Bottle cmd, outcome;
	cmd.addString("setGoal");
	cmd.addDouble(x);
	cmd.addDouble(-y);
	m_rpcPort.write(cmd, outcome);

	// send the orientation of the player
	yarp::sig::Vector& playerOrientationVector = m_playerOrientationPort.prepare();
	playerOrientationVector.clear();
	playerOrientationVector.push_back(playerYawAfterOffset);
	m_playerOrientationPort.write();

	return true;
}

double VirtualizerModule::threshold(const double &input)
{
	if (input >= 0)
	{
		if (input > m_deadzone)
			return input;
		else return 0.0;
	}
	else
	{
		if (input < -m_deadzone)
			return input;
		else return 0.0;
	}
}
