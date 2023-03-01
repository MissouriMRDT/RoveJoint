#include "RoveJoint.h"


bool RoveJoint::closedLoopTargetValid(const float& targetDegrees) {
    if(m_hasForwardSoftLimit && targetDegrees > m_forwardSoftLimitDegrees) return false;
    if(m_hasReverseSoftLimit && targetDegrees < m_reverseSoftLimitDegrees) return false;

    return true;
}


void RoveJoint::attachEncoder(RoveEncoder* encoder) {
    m_encoder = encoder;
    m_hasEncoder = true;
}

void RoveJoint::attachPID(RovePIDController* pidController) {
    m_pidController = pidController;
    m_hasClosedLoop = true;
}



void RoveJoint::attachForwardHardLimit(RoveSwitch* hardLimit) {
    m_forwardHardLimit = hardLimit;
    m_hasForwardHardLimit = true;
}

void RoveJoint::attachReverseHardLimit(RoveSwitch* hardLimit) {
    m_reverseHardLimit = hardLimit;
    m_hasReverseHardLimit = true;
}

void RoveJoint::attachHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit) {
    attachForwardHardLimit(forwardHardLimit);
    attachReverseHardLimit(reverseHardLimit);
}



void RoveJoint::configForwardSoftLimit(const float& limitDegrees) {
    m_forwardSoftLimitDegrees = limitDegrees;
    m_hasForwardSoftLimit = true;
}

void RoveJoint::configReverseSoftLimit(const float& limitDegrees) {
    m_reverseSoftLimitDegrees = limitDegrees;
    m_hasReverseSoftLimit = true;
}

void RoveJoint::configSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees) {
    configForwardSoftLimit(forwardLimitDegrees);
    configReverseSoftLimit(reverseLimitDegrees);
}



void RoveJoint::overrideForwardHardLimit(bool disable) {
    m_forwardHardLimitDisabled = disable;
}

void RoveJoint::overrideReverseHardLimit(bool disable) {
    m_reverseHardLimitDisabled = disable;
}



bool RoveJoint::atForwardSoftLimit() {
    if (m_hasEncoder && m_hasForwardSoftLimit) {
        return m_encoder->readDegrees() >= m_forwardSoftLimitDegrees;
    }
    return false;
}

bool RoveJoint::atReverseSoftLimit() {
    if (m_hasEncoder && m_hasReverseSoftLimit) {
        return m_encoder->readDegrees() <= m_reverseSoftLimitDegrees;
    }
    return false;
}



bool RoveJoint::atForwardHardLimit() {
    if (m_hasForwardHardLimit) {
        return m_forwardHardLimit->read();
    }
    return false;
}

bool RoveJoint::atReverseHardLimit() {
    if (m_hasReverseHardLimit) {
        return m_reverseHardLimit->read();
    }
    return false;
}



void RoveJoint::drive(int16_t decipercent) {
    if (decipercent > 0 && (atForwardSoftLimit() || (atForwardHardLimit() && !m_forwardHardLimitDisabled))) decipercent = 0;
    else if (decipercent < 0 && (atReverseSoftLimit() || (atReverseHardLimit() && !m_reverseHardLimitDisabled))) decipercent = 0;
    
    m_motor->drive(decipercent);
}

void RoveJoint::setAngle(const float& targetDegrees, const float& timestamp) {
    int16_t decipercent = 0;
    if (m_hasEncoder && m_hasClosedLoop && closedLoopTargetValid(targetDegrees)) {
        decipercent = (int16_t) m_pidController->calculate(targetDegrees, m_encoder->readDegrees(), timestamp);
    }
    
    drive(decipercent);
}