/**
 * @file HandRetargeting.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 *          Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <HandRetargeting.hpp>
#include <Utils.hpp>

bool HandRetargeting::configure(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if (config.isNull())
    {
        yError() << "[HandRetargeting::configure] Empty configuration for hand retargeting.";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(config, "scalingFactor", m_scalingFactor))
    {
        yError() << "[HandRetargeting::configure] Unable to find the hands smoothing time";
        return false;
    }

    // getting the mapping between the robot and the retargeting frames
    iDynTree::Rotation tempRotation;
    if (!iDynTree::parseRotationMatrix(config, "handOculusFrame_R_handRobotFrame", tempRotation))
    {
        yError() << "[HandRetargeting::configure] Unable to find the hands Oculus to hand robot "
                    "rotation matrix";
        return false;
    }

    m_handOculusFrame_T_handRobotFrame.setRotation(tempRotation);
    m_handOculusFrame_T_handRobotFrame.setPosition(iDynTree::Position::Zero());

    if (!iDynTree::parseRotationMatrix(
            config, "teleoperationRobotFrame_R_teleoperationFrame", tempRotation))
    {
        yError() << "[HandRetargeting::configure] Unable to find the teleoperation robot to "
                    "teleoperation rotation matrix";
        return false;
    }

    m_teleopRobotFrame_T_teleopFrame.setRotation(tempRotation);
    m_teleopRobotFrame_T_teleopFrame.setPosition(iDynTree::Position::Zero());

    m_oculusInertial_T_teleopFrame.setPosition(iDynTree::Position::Zero());

    return true;
}

void HandRetargeting::set_oculusInertial_T_teleopFrame(const double& playerOrientation)
{
    // notice the minus sign is not an error. Indeed the virtualizer angle is positive clockwise
    // we assume the human teleoperation frame (base frame) has same position as human head frame
    //    yInfo() << "--1--player orientation" << playerOrientation;
    m_oculusInertial_T_teleopFrame.setRotation(iDynTree::Rotation::RotZ(-playerOrientation));
    m_oculusInertial_T_teleopFrame.setPosition(iDynTree::Position::Zero());

    //    yInfo() << "--1-- m_oculusInertial_T_teleopFrame: \n"
    //            << m_oculusInertial_T_teleopFrame.toString();
}

void HandRetargeting::set_oculusInertial_T_teleopFrame(
    const double& neckLength,
    const yarp::sig::Matrix& oculusInertial_T_head,
    const yarp::sig::Matrix& head_T_chest)
{
    iDynTree::Transform chest_T_teleopFrame, tmp_oculusInertial_T_head, tmp_head_T_chest;
    iDynTree::toiDynTree(oculusInertial_T_head, tmp_oculusInertial_T_head);
    iDynTree::toiDynTree(head_T_chest, tmp_head_T_chest);
    chest_T_teleopFrame.setPosition(iDynTree::Position(0.0, 0.0, neckLength));
    chest_T_teleopFrame.setRotation(iDynTree::Rotation::Identity());
    //    yInfo() << "tmp_oculusInertial_T_head: \n" << tmp_oculusInertial_T_head.toString();
    //    yInfo() << "tmp_head_T_chest: \n" << tmp_head_T_chest.toString();
    //    yInfo() << "chest_T_teleopFrame: \n" << chest_T_teleopFrame.toString();

    m_oculusInertial_T_teleopFrame
        = tmp_oculusInertial_T_head * tmp_head_T_chest * chest_T_teleopFrame;
    //    yInfo() << "m_oculusInertial_T_teleopFrame: \n" <<
    //    m_oculusInertial_T_teleopFrame.toString();
}

void HandRetargeting::setHandTransform(const yarp::sig::Matrix& handTransformation)
{
    iDynTree::toiDynTree(handTransformation, m_oculusInertial_T_handOculusFrame);
}

void HandRetargeting::evaluateDesiredHandPose(yarp::sig::Vector& handPose,
                                              yarp::sig::Vector& handPoseInertial)
{
    //    yInfo() << "m_teleopRobotFrame_T_teleopFrame:\n "
    //            << m_teleopRobotFrame_T_teleopFrame.toString();
    //    yInfo() << "m_oculusInertial_T_teleopFrame.inverse(): \n"
    //            << (m_oculusInertial_T_teleopFrame.inverse()).toString();
    //    yInfo() << "m_oculusInertial_T_handOculusFrame: \n"
    //            << m_oculusInertial_T_handOculusFrame.toString();
    //    yInfo() << "m_handOculusFrame_T_handRobotFrame: \n"
    //            << m_handOculusFrame_T_handRobotFrame.toString();

    m_teleopRobotFrame_T_handRobotFrame
        = m_teleopRobotFrame_T_teleopFrame * m_oculusInertial_T_teleopFrame.inverse()
          * m_oculusInertial_T_handOculusFrame * m_handOculusFrame_T_handRobotFrame;
    //    yInfo() << "m_teleopRobotFrame_T_handRobotFrame: \n"
    //            << m_teleopRobotFrame_T_handRobotFrame.toString();

    // probably we should avoid to use roll pitch and yaw. A possible solution
    // is to use quaternion or directly SE(3).
    iDynTree::Vector3 handOrientation, handPosition;
    handOrientation = m_teleopRobotFrame_T_handRobotFrame.getRotation().asRPY();
    handPosition = m_teleopRobotFrame_T_handRobotFrame.getPosition();
    iDynTree::toEigen(handPosition) = m_scalingFactor * iDynTree::toEigen(handPosition);

    handPose.clear();
    handPose.push_back(handPosition(0));
    handPose.push_back(handPosition(1));
    handPose.push_back(handPosition(2));
    handPose.push_back(handOrientation(0));
    handPose.push_back(handOrientation(1));
    handPose.push_back(handOrientation(2));

    iDynTree::Vector3 handOrientationInertial, handPositionInertial;
    handOrientationInertial = m_oculusInertial_T_handOculusFrame.getRotation().asRPY();
    handPositionInertial = m_oculusInertial_T_handOculusFrame.getPosition();

    handPoseInertial.clear();

    handPoseInertial.push_back(handPositionInertial(0));
    handPoseInertial.push_back(handPositionInertial(1));
    handPoseInertial.push_back(handPositionInertial(2));
    handPoseInertial.push_back(handOrientationInertial(0));
    handPoseInertial.push_back(handOrientationInertial(1));
    handPoseInertial.push_back(handOrientationInertial(2));
}
