#include "RoveDifferentialJoint.h"

#include <cmath>


void RoveDifferentialJoint::twistAndTiltDecipercent_to_leftAndRightDecipercent(int twistDecipercent, int tiltDecipercent, int& leftDecipercent, int& rightDecipercent) {
    leftDecipercent = twistDecipercent + tiltDecipercent;
    rightDecipercent = twistDecipercent - tiltDecipercent;

    float scale = 1.0;
    if (std::abs(leftDecipercent) > 1000) scale = 1000.0 / std::abs(leftDecipercent);
    else if (std::abs(rightDecipercent) > 1000) scale = 1000.0 / std::abs(rightDecipercent);

    leftDecipercent *= scale;
    rightDecipercent *= scale;
}



void RoveDifferentialJoint::attachTwistPID(RovePIDController* twistPIDController) {
    m_twistPIDController = twistPIDController;
    m_hasTwistClosedLoop = true;
}

void RoveDifferentialJoint::attachTiltPID(RovePIDController* tiltPIDController) {
    m_tiltPIDController = tiltPIDController;
    m_hasTiltClosedLoop = true;
}

void RoveDifferentialJoint::attachTwistLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse) {
    m_forwardTwistLimitSwitch = forward;
    m_reverseTwistLimitSwitch = reverse;
    m_hasTwistHardLimits = true;
}

void RoveDifferentialJoint::attachTiltLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse) {
    m_forwardTiltLimitSwitch = forward;
    m_reverseTiltLimitSwitch = reverse;
    m_hasTiltHardLimits = true;
}



void RoveDifferentialJoint::configForwardTwistSoftLimit(float limitDegrees) {
    m_forwardTwistSoftLimitDegrees = limitDegrees;
    m_hasForwardTwistSoftLimit = true;
}

void RoveDifferentialJoint::configReverseTwistSoftLimit(float limitDegrees) {
    m_reverseTwistSoftLimitDegrees = limitDegrees;
    m_hasReverseTwistSoftLimit = true;
}

void RoveDifferentialJoint::configTwistSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees) {
    configForwardTwistSoftLimit(forwardLimitDegrees);
    configReverseTwistSoftLimit(reverseLimitDegrees);
}



void RoveDifferentialJoint::configForwardTiltSoftLimit(float limitDegrees) {
    m_forwardTiltSoftLimitDegrees = limitDegrees;
    m_hasForwardTiltSoftLimit = true;
}

void RoveDifferentialJoint::configReverseTiltSoftLimit(float limitDegrees) {
    m_reverseTiltSoftLimitDegrees = limitDegrees;
    m_hasReverseTiltSoftLimit = true;
}

void RoveDifferentialJoint::configTiltSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees) {
    configForwardTiltSoftLimit(forwardLimitDegrees);
    configReverseTiltSoftLimit(reverseLimitDegrees);
}



bool RoveDifferentialJoint::atForwardTwistSoftLimit() {
    if (m_hasForwardTwistSoftLimit) {
        return m_twistEncoder->readDegrees() >= m_forwardTwistSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atReverseTwistSoftLimit() {
    if (m_hasReverseTwistSoftLimit) {
        return m_twistEncoder->readDegrees() >= m_reverseTwistSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atForwardTwistHardLimit() {
    if (m_hasTwistHardLimits) {
        return m_forwardTwistLimitSwitch->read();
    }
    return false;
}

bool RoveDifferentialJoint::atReverseTwistHardLimit() {
    if (m_hasTwistHardLimits) {
        return m_reverseTwistLimitSwitch->read();
    }
    return false;
}



bool RoveDifferentialJoint::atForwardTiltSoftLimit() {
    if (m_hasForwardTiltSoftLimit) {
        return m_tiltEncoder->readDegrees() >= m_forwardTiltSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atReverseTiltSoftLimit() {
    if (m_hasReverseTiltSoftLimit) {
        return m_tiltEncoder->readDegrees() >= m_reverseTiltSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atForwardTiltHardLimit() {
    if (m_hasTiltHardLimits) {
        return m_forwardTiltLimitSwitch->read();
    }
    return false;
}

bool RoveDifferentialJoint::atReverseTiltHardLimit() {
    if (m_hasTiltHardLimits) {
        return m_reverseTiltLimitSwitch->read();
    }
    return false;
}



void RoveDifferentialJoint::drive(int twistDecipercent, int tiltDecipercent) {
    if (twistDecipercent > 0 && (atForwardTwistHardLimit() || atForwardTwistSoftLimit())) twistDecipercent = 0;
    else if (twistDecipercent < 0 && (atReverseTwistHardLimit() || atReverseTwistSoftLimit())) twistDecipercent = 0;
    
    if (tiltDecipercent > 0 && (atForwardTiltHardLimit() || atForwardTiltSoftLimit())) tiltDecipercent = 0;
    else if (tiltDecipercent < 0 && (atReverseTiltHardLimit() || atReverseTiltSoftLimit())) tiltDecipercent = 0;

    int leftDecipercent, rightDecipercent;
    twistAndTiltDecipercent_to_leftAndRightDecipercent(twistDecipercent, tiltDecipercent, leftDecipercent, rightDecipercent);
    
    m_leftMotor->drive(leftDecipercent);
    m_rightMotor->drive(rightDecipercent);
}

void RoveDifferentialJoint::setAngles(float twistTargetDegrees, float tiltTargetDegrees, float timestamp) {
    int twistDecipercent = (int) m_twistPIDController->calculate(twistTargetDegrees, m_twistEncoder->readDegrees(), timestamp);
    int tiltDecipercent = (int) m_tiltPIDController->calculate(tiltTargetDegrees, m_tiltEncoder->readDegrees(), timestamp);

    drive(twistDecipercent, tiltDecipercent);
}