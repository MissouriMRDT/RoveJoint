#include "RoveDifferentialJoint.h"

#include <cmath>


void RoveDifferentialJoint::twistAndTiltDecipercent_to_leftAndRightDecipercent(const int16_t& twistDecipercent, const int16_t& tiltDecipercent, int16_t& leftDecipercent, int16_t& rightDecipercent) {
    leftDecipercent = twistDecipercent + tiltDecipercent;
    rightDecipercent = twistDecipercent - tiltDecipercent;

    float scale = 1.0;
    if (std::abs(leftDecipercent) > 1000) scale = 1000.0 / std::abs(leftDecipercent);
    else if (std::abs(rightDecipercent) > 1000) scale = 1000.0 / std::abs(rightDecipercent);

    leftDecipercent *= scale;
    rightDecipercent *= scale;
}

bool RoveDifferentialJoint::twistClosedLoopTargetValid(const float& targetDegrees) {
    if(m_hasTwistForwardSoftLimit && targetDegrees > m_twistForwardSoftLimitDegrees) return false;
    if(m_hasTwistReverseSoftLimit && targetDegrees < m_twistReverseSoftLimitDegrees) return false;

    return true;
}

bool RoveDifferentialJoint::tiltClosedLoopTargetValid(const float& targetDegrees) {
    if(m_hasTiltForwardSoftLimit && targetDegrees > m_tiltForwardSoftLimitDegrees) return false;
    if(m_hasTiltReverseSoftLimit && targetDegrees < m_tiltReverseSoftLimitDegrees) return false;

    return true;
}



void RoveDifferentialJoint::attachTwistEncoder(RoveEncoder* twistEncoder) {
    m_twistEncoder = twistEncoder;
    m_hasTwistEncoder = true;
}

void RoveDifferentialJoint::attachTiltEncoder(RoveEncoder* tiltEncoder) {
    m_tiltEncoder = tiltEncoder;
    m_hasTiltEncoder = true;
}

void RoveDifferentialJoint::attachTwistPID(RovePIDController* twistPIDController) {
    m_twistPIDController = twistPIDController;
    m_hasTwistClosedLoop = true;
}

void RoveDifferentialJoint::attachTiltPID(RovePIDController* tiltPIDController) {
    m_tiltPIDController = tiltPIDController;
    m_hasTiltClosedLoop = true;
}



void RoveDifferentialJoint::attachTwistForwardHardLimit(RoveSwitch* hardLimit) {
    m_twistForwardHardLimit = hardLimit;
    m_hasTwistForwardHardLimit = true;
}

void RoveDifferentialJoint::attachTwistReverseHardLimit(RoveSwitch* hardLimit) {
    m_twistReverseHardLimit = hardLimit;
    m_hasTwistReverseHardLimit = true;
}

void RoveDifferentialJoint::attachTwistHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit) {
    attachTwistForwardHardLimit(forwardHardLimit);
    attachTwistReverseHardLimit(reverseHardLimit);
}



void RoveDifferentialJoint::attachTiltForwardHardLimit(RoveSwitch* hardLimit) {
    m_tiltForwardHardLimit = hardLimit;
    m_hasTiltForwardHardLimit = true;
}

void RoveDifferentialJoint::attachTiltReverseHardLimit(RoveSwitch* hardLimit) {
    m_tiltReverseHardLimit = hardLimit;
    m_hasTiltReverseHardLimit = true;
}

void RoveDifferentialJoint::attachTiltHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit) {
    attachTiltForwardHardLimit(forwardHardLimit);
    attachTiltReverseHardLimit(reverseHardLimit);
}



void RoveDifferentialJoint::configTwistForwardSoftLimit(const float& limitDegrees) {
    m_twistForwardSoftLimitDegrees = limitDegrees;
    m_hasTwistForwardSoftLimit = true;
}

void RoveDifferentialJoint::configTwistReverseSoftLimit(const float& limitDegrees) {
    m_twistReverseSoftLimitDegrees = limitDegrees;
    m_hasTwistReverseSoftLimit = true;
}

void RoveDifferentialJoint::configTwistSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees) {
    configTwistForwardSoftLimit(forwardLimitDegrees);
    configTwistReverseSoftLimit(reverseLimitDegrees);
}



void RoveDifferentialJoint::configTiltForwardSoftLimit(const float& limitDegrees) {
    m_tiltForwardSoftLimitDegrees = limitDegrees;
    m_hasTiltForwardSoftLimit = true;
}

void RoveDifferentialJoint::configTiltReverseSoftLimit(const float& limitDegrees) {
    m_tiltReverseSoftLimitDegrees = limitDegrees;
    m_hasTiltReverseSoftLimit = true;
}

void RoveDifferentialJoint::configTiltSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees) {
    configTiltForwardSoftLimit(forwardLimitDegrees);
    configTiltReverseSoftLimit(reverseLimitDegrees);
}



void RoveDifferentialJoint::overrideTwistForwardHardLimit(bool disable) {
    m_twistForwardHardLimitDisabled = disable;
}

void RoveDifferentialJoint::overrideTwistReverseHardLimit(bool disable) {
    m_twistReverseHardLimitDisabled = disable;
}

void RoveDifferentialJoint::overrideTiltForwardHardLimit(bool disable) {
    m_tiltForwardHardLimitDisabled = disable;
}

void RoveDifferentialJoint::overrideTiltReverseHardLimit(bool disable) {
    m_tiltReverseHardLimitDisabled = disable;
}



bool RoveDifferentialJoint::atTwistForwardSoftLimit() {
    if (m_hasTwistEncoder && m_hasTwistForwardSoftLimit) {
        return m_twistEncoder->readDegrees() >= m_twistForwardSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTwistReverseSoftLimit() {
    if (m_hasTwistEncoder && m_hasTwistReverseSoftLimit) {
        return m_twistEncoder->readDegrees() >= m_twistReverseSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTiltForwardSoftLimit() {
    if (m_hasTiltEncoder && m_hasTiltForwardSoftLimit) {
        return m_tiltEncoder->readDegrees() >= m_tiltForwardSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTiltReverseSoftLimit() {
    if (m_hasTiltEncoder && m_hasTiltReverseSoftLimit) {
        return m_tiltEncoder->readDegrees() >= m_tiltReverseSoftLimitDegrees;
    }
    return false;
}



bool RoveDifferentialJoint::atTwistForwardHardLimit() {
    if (m_hasTwistForwardHardLimit) {
        return m_twistForwardHardLimit->read();
    }
    return false;
}

bool RoveDifferentialJoint::atTwistReverseHardLimit() {
    if (m_hasTwistReverseHardLimit) {
        return m_twistReverseHardLimit->read();
    }
    return false;
}

bool RoveDifferentialJoint::atTiltForwardHardLimit() {
    if (m_hasTiltForwardHardLimit) {
        return m_tiltForwardHardLimit->read();
    }
    return false;
}

bool RoveDifferentialJoint::atTiltReverseHardLimit() {
    if (m_hasTiltReverseHardLimit) {
        return m_tiltReverseHardLimit->read();
    }
    return false;
}



void RoveDifferentialJoint::drive(int16_t twistDecipercent, int16_t tiltDecipercent) {
    if (twistDecipercent > 0 && (atTwistForwardSoftLimit() || (atTwistForwardHardLimit() && !m_twistForwardHardLimitDisabled))) twistDecipercent = 0;
    else if (twistDecipercent < 0 && (atTwistReverseSoftLimit() || (atTwistReverseHardLimit() && !m_twistReverseHardLimitDisabled))) twistDecipercent = 0;
    
    if (tiltDecipercent > 0 && (atTiltForwardSoftLimit() || (atTiltForwardHardLimit() && !m_tiltForwardHardLimitDisabled))) tiltDecipercent = 0;
    else if (tiltDecipercent < 0 && (atTiltReverseSoftLimit() || (atTiltReverseHardLimit() && !m_tiltReverseHardLimitDisabled))) tiltDecipercent = 0;

    int16_t leftDecipercent, rightDecipercent;
    twistAndTiltDecipercent_to_leftAndRightDecipercent(twistDecipercent, tiltDecipercent, leftDecipercent, rightDecipercent);
    
    m_leftMotor->drive(leftDecipercent);
    m_rightMotor->drive(rightDecipercent);
}

void RoveDifferentialJoint::setAngles(const float& twistTargetDegrees, const float& tiltTargetDegrees, const float& timestamp) {
    int16_t twistDecipercent = 0;
    if (m_hasTwistEncoder && m_hasTwistClosedLoop && twistClosedLoopTargetValid(twistTargetDegrees)) {
        twistDecipercent = (int16_t) m_twistPIDController->calculate(twistTargetDegrees, m_twistEncoder->readDegrees(), timestamp);
    }

    int16_t tiltDecipercent = 0;
    if (m_hasTiltEncoder && m_hasTiltClosedLoop && tiltClosedLoopTargetValid(tiltTargetDegrees)) {
        tiltDecipercent = (int16_t) m_tiltPIDController->calculate(tiltTargetDegrees, m_tiltEncoder->readDegrees(), timestamp);
    }

    drive(twistDecipercent, tiltDecipercent);
}