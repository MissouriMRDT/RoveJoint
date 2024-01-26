#include "RoveDifferentialJoint.h"

#include <cmath>


void RoveDifferentialJoint::twistAndTiltDecipercent_to_leftAndRightDecipercent(const int16_t& twistDecipercent, const int16_t& tiltDecipercent, int16_t& leftDecipercent, int16_t& rightDecipercent) const {
    leftDecipercent = twistDecipercent + tiltDecipercent;
    rightDecipercent = twistDecipercent - tiltDecipercent;

    float scale = 1.0;
    if (std::abs(leftDecipercent) > 1000) scale = 1000.0 / std::abs(leftDecipercent);
    else if (std::abs(rightDecipercent) > 1000) scale = 1000.0 / std::abs(rightDecipercent);

    leftDecipercent *= scale;
    rightDecipercent *= scale;
}

bool RoveDifferentialJoint::atTwistForwardSoftLimit(const float& degrees) const {
    if (m_hasTwistForwardSoftLimit) {
        if (m_hasTwistReverseSoftLimit && (m_twistReverseSoftLimitDegrees > m_twistForwardSoftLimitDegrees)) {
            return (degrees >= m_twistForwardSoftLimitDegrees) && (degrees <= (m_twistReverseSoftLimitDegrees + m_twistForwardSoftLimitDegrees)/2);
        }
        return degrees >= m_twistForwardSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTwistReverseSoftLimit(const float& degrees) const {
    if (m_hasTwistReverseSoftLimit) {
        if (m_hasTwistForwardSoftLimit && (m_twistReverseSoftLimitDegrees > m_twistForwardSoftLimitDegrees)) {
            return (degrees <= m_twistReverseSoftLimitDegrees) && (degrees >= (m_twistReverseSoftLimitDegrees + m_twistForwardSoftLimitDegrees)/2);
        }
        return degrees <= m_twistReverseSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTiltForwardSoftLimit(const float& degrees) const {
    if (m_hasTiltForwardSoftLimit) {
        if (m_hasTiltReverseSoftLimit && (m_tiltReverseSoftLimitDegrees > m_tiltForwardSoftLimitDegrees)) {
            return (degrees >= m_tiltForwardSoftLimitDegrees) && (degrees <= (m_tiltReverseSoftLimitDegrees + m_tiltForwardSoftLimitDegrees)/2);
        }
        return degrees >= m_tiltForwardSoftLimitDegrees;
    }
    return false;
}

bool RoveDifferentialJoint::atTiltReverseSoftLimit(const float& degrees) const {
    if (m_hasTiltReverseSoftLimit) {
        if (m_hasTiltForwardSoftLimit && (m_tiltReverseSoftLimitDegrees > m_tiltForwardSoftLimitDegrees)) {
            return (degrees <= m_tiltReverseSoftLimitDegrees) && (degrees >= (m_tiltReverseSoftLimitDegrees + m_tiltForwardSoftLimitDegrees)/2);
        }
        return degrees <= m_tiltReverseSoftLimitDegrees;
    }
    return false;
}



void RoveDifferentialJoint::attachTwistEncoder(const RoveEncoder* twistEncoder) {
    m_twistEncoder = twistEncoder;
    m_hasTwistEncoder = true;
}

void RoveDifferentialJoint::attachTiltEncoder(const RoveEncoder* tiltEncoder) {
    m_tiltEncoder = tiltEncoder;
    m_hasTiltEncoder = true;
}

void RoveDifferentialJoint::attachTwistPID(const RovePIDController* twistPIDController) {
    m_twistPIDController = twistPIDController;
    m_hasTwistClosedLoop = true;
}

void RoveDifferentialJoint::attachTiltPID(const RovePIDController* tiltPIDController) {
    m_tiltPIDController = tiltPIDController;
    m_hasTiltClosedLoop = true;
}



void RoveDifferentialJoint::attachTwistForwardHardLimit(const RoveSwitch* hardLimit) {
    m_twistForwardHardLimit = hardLimit;
    m_hasTwistForwardHardLimit = true;
}

void RoveDifferentialJoint::attachTwistReverseHardLimit(const RoveSwitch* hardLimit) {
    m_twistReverseHardLimit = hardLimit;
    m_hasTwistReverseHardLimit = true;
}

void RoveDifferentialJoint::attachTwistHardLimits(const RoveSwitch* reverseHardLimit, const RoveSwitch* forwardHardLimit) {
    attachTwistReverseHardLimit(reverseHardLimit);
    attachTwistForwardHardLimit(forwardHardLimit);
}



void RoveDifferentialJoint::attachTiltForwardHardLimit(const RoveSwitch* hardLimit) {
    m_tiltForwardHardLimit = hardLimit;
    m_hasTiltForwardHardLimit = true;
}

void RoveDifferentialJoint::attachTiltReverseHardLimit(const RoveSwitch* hardLimit) {
    m_tiltReverseHardLimit = hardLimit;
    m_hasTiltReverseHardLimit = true;
}

void RoveDifferentialJoint::attachTiltHardLimits(const RoveSwitch* reverseHardLimit, const RoveSwitch* forwardHardLimit) {
    attachTiltReverseHardLimit(reverseHardLimit);
    attachTiltForwardHardLimit(forwardHardLimit);
}



void RoveDifferentialJoint::configTwistForwardSoftLimit(const float& limitDegrees) {
    m_twistForwardSoftLimitDegrees = limitDegrees;
    m_hasTwistForwardSoftLimit = true;
}

void RoveDifferentialJoint::configTwistReverseSoftLimit(const float& limitDegrees) {
    m_twistReverseSoftLimitDegrees = limitDegrees;
    m_hasTwistReverseSoftLimit = true;
}

void RoveDifferentialJoint::configTwistSoftLimits(const float& reverseLimitDegrees, const float& forwardLimitDegrees) {
    configTwistReverseSoftLimit(reverseLimitDegrees);
    configTwistForwardSoftLimit(forwardLimitDegrees);
}



void RoveDifferentialJoint::configTiltForwardSoftLimit(const float& limitDegrees) {
    m_tiltForwardSoftLimitDegrees = limitDegrees;
    m_hasTiltForwardSoftLimit = true;
}

void RoveDifferentialJoint::configTiltReverseSoftLimit(const float& limitDegrees) {
    m_tiltReverseSoftLimitDegrees = limitDegrees;
    m_hasTiltReverseSoftLimit = true;
}

void RoveDifferentialJoint::configTiltSoftLimits(const float& reverseLimitDegrees, const float& forwardLimitDegrees) {
    configTiltReverseSoftLimit(reverseLimitDegrees);
    configTiltForwardSoftLimit(forwardLimitDegrees);
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



bool RoveDifferentialJoint::atTwistForwardSoftLimit() const {
    return m_hasTwistEncoder && atTwistForwardSoftLimit(m_twistEncoder->readDegrees());
}

bool RoveDifferentialJoint::atTwistReverseSoftLimit() const {
    return m_hasTwistEncoder && atTwistReverseSoftLimit(m_twistEncoder->readDegrees());
}

bool RoveDifferentialJoint::atTiltForwardSoftLimit() const {
    return m_hasTiltEncoder && atTiltForwardSoftLimit(m_tiltEncoder->readDegrees());
}

bool RoveDifferentialJoint::atTiltReverseSoftLimit() const {
    return m_hasTiltEncoder && atTiltReverseSoftLimit(m_tiltEncoder->readDegrees());
}



bool RoveDifferentialJoint::atTwistForwardHardLimit() const {
    return m_hasTwistForwardHardLimit && m_twistForwardHardLimit->read();
}

bool RoveDifferentialJoint::atTwistReverseHardLimit() const {
    return m_hasTwistReverseHardLimit && m_twistReverseHardLimit->read();
}

bool RoveDifferentialJoint::atTiltForwardHardLimit() const {
    return m_hasTiltForwardHardLimit && m_tiltForwardHardLimit->read();
}

bool RoveDifferentialJoint::atTiltReverseHardLimit() const {
    return m_hasTiltReverseHardLimit && m_tiltReverseHardLimit->read();
}



void RoveDifferentialJoint::drive(int16_t twistDecipercent, int16_t tiltDecipercent) const {
    if (twistDecipercent > 0 && (atTwistForwardSoftLimit() || (!m_twistForwardHardLimitDisabled && atTwistForwardHardLimit()))) twistDecipercent = 0;
    else if (twistDecipercent < 0 && (atTwistReverseSoftLimit() || (!m_twistReverseHardLimitDisabled && atTwistReverseHardLimit()))) twistDecipercent = 0;
    
    if (tiltDecipercent > 0 && (atTiltForwardSoftLimit() || (!m_tiltForwardHardLimitDisabled && atTiltForwardHardLimit()))) tiltDecipercent = 0;
    else if (tiltDecipercent < 0 && (atTiltReverseSoftLimit() || (!m_tiltReverseHardLimitDisabled && atTiltReverseHardLimit()))) tiltDecipercent = 0;

    int16_t leftDecipercent, rightDecipercent;
    twistAndTiltDecipercent_to_leftAndRightDecipercent(twistDecipercent, tiltDecipercent, leftDecipercent, rightDecipercent);
    
    m_leftMotor->drive(leftDecipercent);
    m_rightMotor->drive(rightDecipercent);
}

void RoveDifferentialJoint::setAngles(const float& twistTargetDegrees, const float& tiltTargetDegrees) const {
    int16_t twistDecipercent = 0;
    if (m_hasTwistEncoder && m_hasTwistClosedLoop && !atTwistForwardSoftLimit(twistTargetDegrees) && !atTwistReverseSoftLimit(twistTargetDegrees)) {
        twistDecipercent = (int16_t) m_twistPIDController->calculate(twistTargetDegrees, m_twistEncoder->readDegrees());
    }

    int16_t tiltDecipercent = 0;
    if (m_hasTiltEncoder && m_hasTiltClosedLoop && !atTiltForwardSoftLimit(tiltTargetDegrees) && !atTiltReverseSoftLimit(tiltTargetDegrees)) {
        tiltDecipercent = (int16_t) m_tiltPIDController->calculate(tiltTargetDegrees, m_tiltEncoder->readDegrees());
    }

    drive(twistDecipercent, tiltDecipercent);
}