#include "RoveJoint.h"


bool RoveJoint::atForwardSoftLimit(float degrees) const {
    if (m_hasForwardSoftLimit) {
        if (m_hasReverseSoftLimit && (m_reverseSoftLimitDegrees > m_forwardSoftLimitDegrees)) {
            return (degrees >= m_forwardSoftLimitDegrees) && (degrees <= (m_reverseSoftLimitDegrees + m_forwardSoftLimitDegrees)/2);
        }
        return degrees >= m_forwardSoftLimitDegrees;
    }
    return false;
}

bool RoveJoint::atReverseSoftLimit(float degrees) const {
    if (m_hasReverseSoftLimit) {
        if (m_hasForwardSoftLimit && (m_reverseSoftLimitDegrees > m_forwardSoftLimitDegrees)) {
            return (degrees <= m_reverseSoftLimitDegrees) && (degrees >= (m_reverseSoftLimitDegrees + m_forwardSoftLimitDegrees)/2);
        }
        return degrees <= m_reverseSoftLimitDegrees;
    }
    return false;
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

void RoveJoint::attachHardLimits(RoveSwitch* reverseHardLimit, RoveSwitch* forwardHardLimit) {
    attachReverseHardLimit(reverseHardLimit);
    attachForwardHardLimit(forwardHardLimit);
}



void RoveJoint::configForwardSoftLimit(float limitDegrees) {
    m_forwardSoftLimitDegrees = limitDegrees;
    m_hasForwardSoftLimit = true;
}

void RoveJoint::configReverseSoftLimit(float limitDegrees) {
    m_reverseSoftLimitDegrees = limitDegrees;
    m_hasReverseSoftLimit = true;
}

void RoveJoint::configSoftLimits(float reverseLimitDegrees, float forwardLimitDegrees) {
    configReverseSoftLimit(reverseLimitDegrees);
    configForwardSoftLimit(forwardLimitDegrees);
}



void RoveJoint::overrideForwardHardLimit(bool disable) {
    m_forwardHardLimitDisabled = disable;
}

void RoveJoint::overrideReverseHardLimit(bool disable) {
    m_reverseHardLimitDisabled = disable;
}

void RoveJoint::overrideForwardSoftLimit(bool disable) {
    m_forwardSoftLimitDisabled = disable;
}

void RoveJoint::overrideReverseSoftLimit(bool disable) {
    m_reverseSoftLimitDisabled = disable;
}



bool RoveJoint::atForwardSoftLimit() const {
    return m_hasEncoder && atForwardSoftLimit(m_encoder->readDegrees());
}

bool RoveJoint::atReverseSoftLimit() const {
    return m_hasEncoder && atReverseSoftLimit(m_encoder->readDegrees());
}



bool RoveJoint::atForwardHardLimit() const {
    return m_hasForwardHardLimit && m_forwardHardLimit->read();
}

bool RoveJoint::atReverseHardLimit() const {
    return m_hasReverseHardLimit && m_reverseHardLimit->read();
}



void RoveJoint::drive(int16_t decipercent) const {
    if (decipercent > 0
        && ((!m_forwardSoftLimitDisabled && atForwardSoftLimit()) 
            || (!m_forwardHardLimitDisabled && atForwardHardLimit()))) {
        decipercent = 0;
    }
    else if (decipercent < 0 
            && ((!m_reverseSoftLimitDisabled && atReverseSoftLimit()) 
                || (!m_reverseHardLimitDisabled && atReverseHardLimit()))) {
        decipercent = 0;
    }
    
    m_motor->drive(decipercent);
}

void RoveJoint::setAngle(float targetDegrees) const {
    
    if ((!m_forwardSoftLimitDisabled && atForwardSoftLimit()) 
        || (!m_forwardHardLimitDisabled && atForwardHardLimit())) {
            targetDegrees = m_forwardSoftLimitDegrees;
    }
    else if ((!m_reverseSoftLimitDisabled && atReverseSoftLimit()) 
        || (!m_reverseHardLimitDisabled && atReverseHardLimit())) {
            targetDegrees = m_reverseSoftLimitDegrees;
    }

    int16_t decipercent = 0;
    if (m_hasEncoder && m_hasClosedLoop) {
        decipercent = (int16_t) m_pidController->calculate(targetDegrees, m_encoder->readDegrees());
    }
    
    m_motor->drive(decipercent);
}