#include "RoveJoint.h"


void RoveJoint::attachPID(RovePIDController* pidController) {
    m_pidController = pidController;
    m_hasClosedLoop = true;
}

void RoveJoint::attachLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse) {
    m_forwardLimitSwitch = forward;
    m_reverseLimitSwitch = reverse;
    m_hasHardLimits = true;
}



void RoveJoint::configForwardSoftLimit(float limitDegrees) {
    m_forwardSoftLimitDegrees = limitDegrees;
    m_hasForwardSoftLimit = true;
}

void RoveJoint::configReverseSoftLimit(float limitDegrees) {
    m_reverseSoftLimitDegrees = limitDegrees;
    m_hasReverseSoftLimit = true;
}

void RoveJoint::configSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees) {
    configForwardSoftLimit(forwardLimitDegrees);
    configReverseSoftLimit(reverseLimitDegrees);
}



bool RoveJoint::atForwardSoftLimit() {
    if (m_hasForwardSoftLimit) {
        return m_encoder->readDegrees() >= m_forwardSoftLimitDegrees;
    }
    return false;
}

bool RoveJoint::atReverseSoftLimit() {
    if (m_hasReverseSoftLimit) {
        return m_encoder->readDegrees() <= m_reverseSoftLimitDegrees;
    }
    return false;
}

bool RoveJoint::atForwardHardLimit() {
    if (m_hasHardLimits) {
        return m_forwardLimitSwitch->read();
    }
    return false;
}

bool RoveJoint::atReverseHardLimit() {
    if (m_hasHardLimits) {
        return m_reverseLimitSwitch->read();
    }
    return false;
}



void RoveJoint::drive(int decipercent) {
    if (decipercent > 0 && (atForwardSoftLimit() || atForwardHardLimit())) decipercent = 0;
    else if (decipercent < 0 && (atReverseSoftLimit() || atReverseHardLimit())) decipercent = 0;
    
    m_motor->drive(decipercent);
}

void RoveJoint::setAngle(float targetDegrees, float timestamp) {
    int decipercent = (int) m_pidController->calculate(targetDegrees, m_encoder->readDegrees(), timestamp);
    drive(decipercent);
}