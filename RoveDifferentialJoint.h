#ifndef ROVEDIFFERENTIALJOINT_H
#define ROVEDIFFERENTIALJOINT_H

#include <RoveMotor.h>
#include <RoveEncoder.h>
#include <RoveSwitch.h>
#include <RovePIDController.h>

class RoveDifferentialJoint {

private:

    RoveMotor* m_leftMotor;
    RoveMotor* m_rightMotor;
    RoveEncoder* m_twistEncoder;
    RoveEncoder* m_tiltEncoder;

    bool m_hasTwistClosedLoop = false, m_hasTiltClosedLoop = false;
    RovePIDController* m_twistPIDController = nullptr;
    RovePIDController* m_tiltPIDController = nullptr;

    bool m_hasTwistHardLimits = false, m_hasTiltHardLimits = false;
    RoveSwitch* m_forwardTwistLimitSwitch = nullptr;
    RoveSwitch* m_reverseTwistLimitSwitch = nullptr;
    RoveSwitch* m_forwardTiltLimitSwitch = nullptr;
    RoveSwitch* m_reverseTiltLimitSwitch = nullptr;

    bool m_hasForwardTwistSoftLimit = false, m_hasReverseTwistSoftLimit = false, m_hasForwardTiltSoftLimit = false, m_hasReverseTiltSoftLimit = false;
    float m_forwardTwistSoftLimitDegrees, m_reverseTwistSoftLimitDegrees, m_forwardTiltSoftLimitDegrees, m_reverseTiltSoftLimitDegrees;

    void twistAndTiltDecipercent_to_leftAndRightDecipercent(int twistDecipercent, int tiltDecipercent, int& leftDecipercent, int& rightDecipercent);

public:

    RoveDifferentialJoint(RoveMotor* leftMotor, RoveMotor* rightMotor, RoveEncoder* twistEncoder, RoveEncoder* tiltEncoder) : m_leftMotor(leftMotor), m_rightMotor(rightMotor), m_twistEncoder(twistEncoder), m_tiltEncoder(tiltEncoder) {}

    void attachTwistPID(RovePIDController* twistPIDController);
    void attachTiltPID(RovePIDController* tiltPIDController);
    void attachTwistLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse);
    void attachTiltLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse);

    void configForwardTwistSoftLimit(float limitDegrees);
    void configReverseTwistSoftLimit(float limitDegrees);
    void configTwistSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees);

    void configForwardTiltSoftLimit(float limitDegrees);
    void configReverseTiltSoftLimit(float limitDegrees);
    void configTiltSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees);

    bool atForwardTwistSoftLimit();
    bool atReverseTwistSoftLimit();
    bool atForwardTwistHardLimit();
    bool atReverseTwistHardLimit();

    bool atForwardTiltSoftLimit();
    bool atReverseTiltSoftLimit();
    bool atForwardTiltHardLimit();
    bool atReverseTiltHardLimit();

    void drive(int twistDecipercent, int tiltDecipercent);
    void setAngles(float twistTargetDegrees, float tiltTargetDegrees, float timestamp);
    
};

#endif