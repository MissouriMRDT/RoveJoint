#ifndef ROVEDIFFERENTIALJOINT_H
#define ROVEDIFFERENTIALJOINT_H

#include <RoveMotor.h>
#include <RoveEncoder.h>
#include <RoveSwitch.h>
#include <RovePIDController.h>

#include <cstdint>


class RoveDifferentialJoint {

private:

    RoveMotor* m_leftMotor;
    RoveMotor* m_rightMotor;

    bool m_hasTwistEncoder = false, m_hasTiltEncoder = false;
    RoveEncoder* m_twistEncoder = nullptr;
    RoveEncoder* m_tiltEncoder = nullptr;

    bool m_hasTwistClosedLoop = false, m_hasTiltClosedLoop = false;
    RovePIDController* m_twistPIDController = nullptr;
    RovePIDController* m_tiltPIDController = nullptr;

    bool m_hasTwistForwardHardLimit = false, m_hasTwistReverseHardLimit = false, m_hasTiltForwardHardLimit = false, m_hasTiltReverseHardLimit = false;
    RoveSwitch* m_twistForwardHardLimit = nullptr;
    RoveSwitch* m_twistReverseHardLimit = nullptr;
    RoveSwitch* m_tiltForwardHardLimit = nullptr;
    RoveSwitch* m_tiltReverseHardLimit = nullptr;

    bool m_hasTwistForwardSoftLimit = false, m_hasTwistReverseSoftLimit = false, m_hasTiltForwardSoftLimit = false, m_hasTiltReverseSoftLimit = false;
    float m_twistForwardSoftLimitDegrees, m_twistReverseSoftLimitDegrees, m_tiltForwardSoftLimitDegrees, m_tiltReverseSoftLimitDegrees;

    /**
     * @brief Calculate the necessary left and right decipercents to move the joint at the desired twist and tilt decipercents.
     * 
     * @param twistDecipercent Desired twist decipercent.
     * @param tiltDecipercent Desired tilt decipercent.
     * @param leftDecipercent Output left decipercent.
     * @param rightDecipercent Output right decipercent.
     */
    void twistAndTiltDecipercent_to_leftAndRightDecipercent(const int16_t& twistDecipercent, const int16_t& tiltDecipercent, int16_t& leftDecipercent, int16_t& rightDecipercent);

    /**
     * @brief Check if the closed loop target is within the range of the configured twist soft limits.
     * 
     * @param targetDegrees Closed loop target, in degrees.
     * @return False if targetDegrees is greater than the configured twist forward soft limit or less than the configured twist reverse soft limit.
     * @return True if no twist soft limits have been configured. 
     */
    bool twistClosedLoopTargetValid(const float& targetDegrees);

    /**
     * @brief Check if the closed loop target is within the range of the configured tilt soft limits.
     * 
     * @param targetDegrees Closed loop target, in degrees.
     * @return False if targetDegrees is greater than the configured tilt forward soft limit or less than the configured tilt reverse soft limit.
     * @return True if no tilt soft limits have been configured. 
     */
    bool tiltClosedLoopTargetValid(const float& targetDegrees);

public:

    /**
     * @brief Construct a new RoveDifferentialJoint object.
     * 
     * @param leftMotor Pointer to an already configured RoveMotor.
     * @param rightMotor Pointer to an already configured RoveMotor.
     */
    RoveDifferentialJoint(RoveMotor* leftMotor, RoveMotor* rightMotor) : m_leftMotor(leftMotor), m_rightMotor(rightMotor) {}


    /**
     * @brief Attach an encoder to the twist axis of the differential joint.
     * 
     * @param twistEncoder Pointer to an already configured RoveEncoder.
     */
    void attachTwistEncoder(RoveEncoder* twistEncoder);

    /**
     * @brief Attach an encoder to the tilt axis of the differential joint.
     * 
     * @param tiltEncoder Pointer to an already configured RoveEncoder.
     */
    void attachTiltEncoder(RoveEncoder* tiltEncoder);

    /**
     * @brief Attach a PID controller to the twist axis of the differential joint.
     * 
     * @param twistPIDController Pointer to an already configured RovePIDController.
     */
    void attachTwistPID(RovePIDController* twistPIDController);

    /**
     * @brief Attach a PID controller to the tilt axis of the differential joint.
     * 
     * @param tiltPIDController Pointer to an already configured RovePIDController.
     */
    void attachTiltPID(RovePIDController* tiltPIDController);


    /**
     * @brief Attach a forward hard limit to the twist axis of the differential joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTwistForwardHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach a reverse hard limit to the twist axis of the differential joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTwistReverseHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach both forward and reverse hard limits to the twist axis of the differential joint.
     * 
     * @param forwardHardLimit Pointer to an already configured RoveSwitch.
     * @param reverseHardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTwistHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit);


    /**
     * @brief Attach a forward hard limit to the tilt axis of the differential joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTiltForwardHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach a reverse hard limit to the tilt axis of the differential joint.
     * 
     * @param hardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTiltReverseHardLimit(RoveSwitch* hardLimit);

    /**
     * @brief Attach both forward and reverse hard limits to the tilt axis of the differential joint.
     * 
     * @param forwardHardLimit Pointer to an already configured RoveSwitch.
     * @param reverseHardLimit Pointer to an already configured RoveSwitch.
     */
    void attachTiltHardLimits(RoveSwitch* forwardHardLimit, RoveSwitch* reverseHardLimit);


    /**
     * @brief Configure the forward soft limit of the twist axis.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the positive twist direction, in degrees.
     */
    void configTwistForwardSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure the reverse soft limit of the twist axis.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the negative twist direction, in degrees.
     */
    void configTwistReverseSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure both the forward and reverse soft limits of the twist axis.
     * 
     * @param forwardLimitDegrees Encoder value that is not to be exceeded in the positive twist direction, in degrees.
     * @param reverseLimitDegrees Encoder value that is not to be exceeded in the negative twist direction, in degrees.
     */
    void configTwistSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees);


    /**
     * @brief Configure the forward soft limit of the tilt axis.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the positive tilt direction, in degrees.
     */
    void configTiltForwardSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure the reverse soft limit of the tilt axis.
     * 
     * @param limitDegrees Encoder value that is not to be exceeded in the negative tilt direction, in degrees.
     */
    void configTiltReverseSoftLimit(const float& limitDegrees);

    /**
     * @brief Configure both the forward and reverse soft limits of the tilt axis.
     * 
     * @param forwardLimitDegrees Encoder value that is not to be exceeded in the positive tilt direction, in degrees.
     * @param reverseLimitDegrees Encoder value that is not to be exceeded in the negative tilt direction, in degrees.
     */
    void configTiltSoftLimits(const float& forwardLimitDegrees, const float& reverseLimitDegrees);


    /**
     * @brief Check if the twist axis of the differential joint is at its forward soft limit.
     * 
     * @return True if the twist encoder value is greater than the configured twist forward soft limit.
     * @return False if a twist encoder has not been attached or a twist forward soft limit has not been configured.
     */
    bool atTwistForwardSoftLimit();

    /**
     * @brief Check if the twist axis of the differential joint is at its reverse soft limit.
     * 
     * @return True if the twist encoder value is less than the configured twist reverse soft limit.
     * @return False if a twist encoder has not been attached or a twist reverse soft limit has not been configured.
     */
    bool atTwistReverseSoftLimit();

    /**
     * @brief Check if the tilt axis of the differential joint is at its forward soft limit.
     * 
     * @return True if the tilt encoder value is greater than the configured tilt forward soft limit.
     * @return False if a tilt encoder has not been attached or a tilt forward soft limit has not been configured.
     */
    bool atTiltForwardSoftLimit();

    /**
     * @brief Check if the tilt axis of the differential joint is at its reverse soft limit.
     * 
     * @return True if the tilt encoder value is less than the configured tilt reverse soft limit.
     * @return False if a tilt encoder has not been attached or a tilt reverse soft limit has not been configured.
     */
    bool atTiltReverseSoftLimit();


    /**
     * @brief Check if the twist axis of the differential joint is at its forward hard limit.
     * 
     * @return True if the twist forward hard limit is tripped.
     * @return False if a twist forward hard limit has not been attached. 
     */
    bool atTwistForwardHardLimit();

    /**
     * @brief Check if the twist axis of the differential joint is at its reverse hard limit.
     * 
     * @return True if the twist reverse hard limit is tripped.
     * @return False if a twist reverse hard limit has not been attached. 
     */
    bool atTwistReverseHardLimit();


    /**
     * @brief Check if the tilt axis of the differential joint is at its forward hard limit.
     * 
     * @return True if the tilt forward hard limit is tripped.
     * @return False if a tilt forward hard limit has not been attached. 
     */
    bool atTiltForwardHardLimit();

    /**
     * @brief Check if the tilt axis of the differential joint is at its reverse hard limit.
     * 
     * @return True if the tilt reverse hard limit is tripped.
     * @return False if a tilt reverse hard limit has not been attached. 
     */
    bool atTiltReverseHardLimit();


    /**
     * @brief Write the provided drive signal to the differential joint.
     * 
     * @param twistDecipercent Twist output [-1000, 1000].
     * @param tiltDecipercent Tilt output [-1000, 1000].
     */
    void drive(int16_t twistDecipercent, int16_t tiltDecipercent);

    /**
     * @brief Set the differential joint in closed loop control towards the target angles.
     * 
     * @param twistTargetDegrees Twist closed loop target, in degrees.
     * @param tiltTargetDegrees Tilt closed loop target, in degrees.
     * @param timestamp Current timestamp.
     */
    void setAngles(const float& twistTargetDegrees, const float& tiltTargetDegrees, const float& timestamp);
    
};

#endif