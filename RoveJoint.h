#ifndef ROVEJOINT_H
#define ROVEJOINT_H

#include <RoveMotor.h>
#include <RoveEncoder.h>
#include <RoveSwitch.h>
#include <RovePIDController.h>

#include <cstdint>


class RoveJoint {

private:
    
    RoveMotor* m_motor;

    bool m_hasEncoder = false;
    RoveEncoder* m_encoder = nullptr;

    bool m_hasClosedLoop = false;
    RovePIDController* m_pidController = nullptr;

    bool m_hasForwardHardLimit = false, m_hasReverseHardLimit = false;
    RoveSwitch* m_forwardHardLimit = nullptr;
    RoveSwitch* m_reverseHardLimit = nullptr;

    bool m_hasForwardSoftLimit = false, m_hasReverseSoftLimit = false;
    float m_forwardSoftLimitDegrees, m_reverseSoftLimitDegrees;

    /**
     * @brief Check if the closed loop target is within the range of the configured soft limits.
     * 
     * @param targetDegrees Closed loop target, in degrees.
     * @return False if targetDegrees is greater than the configured forward soft limit or less than the configured reverse soft limit.
     * @return True if no soft limits have been configured. 
     */
    bool closedLoopTargetValid(const float& targetDegrees);

public:

    /**
     * @brief Construct a new RoveJoint object.
     * 
     * @param motor Pointer to an already configured RoveMotor.
     */
    RoveJoint(RoveMotor* motor) : m_motor(motor) {}


    /**
     * @brief Attach an encoder to the joint.
     * 
     * @param encoder Pointer to an already configured RoveEncoder.
     */
    void attachEncoder(RoveEncoder* encoder);

    /**
     * @brief Attach a PID controller to the joint.
     * 
     * @param pidController Pointer to an already configured RovePIDController.
     */
    void attachPID(RovePIDController* pidController);


    /**
     * @brief Attach a forward hard limit to the joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachForwardHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach a reverse hard limit to the joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachReverseHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach both forward and reverse hard limits to the joint.
     * 
     * @param forwardHardLimit Pointer to an already configured RoveSwitch.
     * @param reverseHardLimit Pointer to an already configured RoveSwitch.
     */
    void attachHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit);


    /**
     * @brief Configure the forward soft limit.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the positive direction, in degrees.
     */
    void configForwardSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure the reverse soft limit.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the negative direction, in degrees.
     */
    void configReverseSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure both the forward and reverse soft limits.
     * 
     * @param forwardLimitDegrees Encoder value that is not to be exceeded in the positive direction, in degrees.
     * @param reverseLimitDegrees Encoder value that is not to be exceeded in the negative direction, in degrees.
     */
    void configSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees);


    /**
     * @brief Check if the joint is at its forward soft limit.
     * 
     * @return True if the encoder value is greater than the configured forward soft limit.
     * @return False if an encoder has not been attached or a forward soft limit has not been configured.
     */
    bool atForwardSoftLimit();

    /**
     * @brief Check if the joint is at its reverse soft limit.
     * 
     * @return True if the encoder value is less than the configured reverse soft limit.
     * @return False if an encoder has not been attached or a reverse soft limit has not been configured.
     */
    bool atReverseSoftLimit();



    /**
     * @brief Check if the joint is at its forward hard limit.
     * 
     * @return True if the forward hard limit is tripped.
     * @return False if a forward hard limit has not been attached. 
     */
    bool atForwardHardLimit();

    /**
     * @brief Check if the joint is at its reverse hard limit.
     * 
     * @return True if the reverse hard limit is tripped.
     * @return False if a reverse hard limit has not been attached. 
     */
    bool atReverseHardLimit();


    /**
     * @brief Write the provided drive signal to the joint.
     * 
     * @param decipercent Motor output [-1000, 1000].
     */
    void drive(int16_t decipercent);

    /**
     * @brief Set the joint in closed loop control towards the target angle.
     * 
     * @param targetDegrees Closed loop target, in degrees.
     * @param timestamp Current timestamp.
     */
    void setAngle(const float& targetDegrees, const float& timestamp);

};

#endif