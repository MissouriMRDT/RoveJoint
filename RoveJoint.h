#ifndef ROVEJOINT_H
#define ROVEJOINT_H

#include <RoveMotor.h>
#include <RoveEncoder.h>
#include <RoveSwitch.h>
#include <RovePIDController.h>

class RoveJoint {

private:

    RoveMotor* m_motor;
    RoveEncoder* m_encoder;

    bool m_hasClosedLoop = false;
    RovePIDController* m_pidController = nullptr;

    bool m_hasHardLimits = false;
    RoveSwitch* m_forwardLimitSwitch = nullptr;
    RoveSwitch* m_reverseLimitSwitch = nullptr;

    bool m_hasForwardSoftLimit = false, m_hasReverseSoftLimit = false;
    float m_forwardSoftLimitDegrees, m_reverseSoftLimitDegrees;

public:

    RoveJoint(RoveMotor* motor, RoveEncoder* encoder) : m_motor(motor), m_encoder(encoder) {}

    void attachPID(RovePIDController* pidController);
    void attachLimitSwitches(RoveSwitch* forward, RoveSwitch* reverse);

    void configForwardSoftLimit(float limitDegrees);
    void configReverseSoftLimit(float limitDegrees);
    void configSoftLimits(float forwardLimitDegrees, float reverseLimitDegrees);

    bool atForwardSoftLimit();
    bool atReverseSoftLimit();
    bool atForwardHardLimit();
    bool atReverseHardLimit();

    void drive(int decipercent);
    void setAngle(float targetDegrees, float timestamp);

};

#endif