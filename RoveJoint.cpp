#include "RoveJoint.h"


bool RoveJoint::atForwardSoftLimit(const float& degrees) const {
    if (m_hasForwardSoftLimit) {
        if (m_hasReverseSoftLimit && (m_reverseSoftLimitDegrees > m_forwardSoftLimitDegrees)) {
            return (degrees >= m_forwardSoftLimitDegrees) && (degrees <= (m_reverseSoftLimitDegrees + m_forwardSoftLimitDegrees)/2);
        }
        return degrees >= m_forwardSoftLimitDegrees;
    }
    return false;
}

bool RoveJoint::atReverseSoftLimit(const float& degrees) const {
    if (m_hasReverseSoftLimit) {
        if (m_hasForwardSoftLimit && (m_reverseSoftLimitDegrees > m_forwardSoftLimitDegrees)) {
            return (degrees <= m_reverseSoftLimitDegrees) && (degrees >= (m_reverseSoftLimitDegrees + m_forwardSoftLimitDegrees)/2);
        }
        return degrees <= m_reverseSoftLimitDegrees;
    }
    return false;
}



void RoveJoint::attachEncoder(const RoveEncoder* encoder) {
    m_encoder = encoder;
    m_hasEncoder = true;
}

void RoveJoint::attachPID(const RovePIDController* pidController) {
    m_pidController = pidController;
    m_hasClosedLoop = true;
}



void RoveJoint::attachForwardHardLimit(const RoveSwitch* hardLimit) {
    m_forwardHardLimit = hardLimit;
    m_hasForwardHardLimit = true;
}

void RoveJoint::attachReverseHardLimit(const RoveSwitch* hardLimit) {
    m_reverseHardLimit = hardLimit;
    m_hasReverseHardLimit = true;
}

void RoveJoint::attachHardLimits(const RoveSwitch* reverseHardLimit, const RoveSwitch* forwardHardLimit) {
    attachReverseHardLimit(reverseHardLimit);
    attachForwardHardLimit(forwardHardLimit);
}



void RoveJoint::configForwardSoftLimit(const float& limitDegrees) {
    m_forwardSoftLimitDegrees = limitDegrees;
    m_hasForwardSoftLimit = true;
}

void RoveJoint::configReverseSoftLimit(const float& limitDegrees) {
    m_reverseSoftLimitDegrees = limitDegrees;
    m_hasReverseSoftLimit = true;
}

void RoveJoint::configSoftLimits(const float& reverseLimitDegrees, const float& forwardLimitDegrees) {
    configReverseSoftLimit(reverseLimitDegrees);
    configForwardSoftLimit(forwardLimitDegrees);
}



void RoveJoint::overrideForwardHardLimit(bool disable) {
    m_forwardHardLimitDisabled = disable;
}

void RoveJoint::overrideReverseHardLimit(bool disable) {
    m_reverseHardLimitDisabled = disable;
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
    if (decipercent > 0 && (atForwardSoftLimit() || (!m_forwardHardLimitDisabled && atForwardHardLimit()))) decipercent = 0;
    else if (decipercent < 0 && (atReverseSoftLimit() || (!m_reverseHardLimitDisabled && atReverseHardLimit()))) decipercent = 0;
    
    m_motor->drive(decipercent);
}

void RoveJoint::setAngle(const float& targetDegrees) const {
    int16_t decipercent = 0;
    if (m_hasEncoder && m_hasClosedLoop && !atForwardSoftLimit(targetDegrees) && !atReverseSoftLimit(targetDegrees)) {
        decipercent = (int16_t) m_pidController->calculate(targetDegrees, m_encoder->readDegrees());
    }
    
    drive(decipercent);
}