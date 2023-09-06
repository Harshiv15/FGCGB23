package org.firstinspires.ftc.teamcode.util.extractions

import android.annotation.SuppressLint
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile

class ProfiledPIDFController(Kp: Double, Ki: Double, Kd: Double, Kf: Double,
                             constraints: TrapezoidProfile.Constraints?) {

    private lateinit var m_controller: PIDFController
    private var m_goal = TrapezoidProfile.State()
    private var m_setpoint = TrapezoidProfile.State()
    private var m_constraints: TrapezoidProfile.Constraints? = null

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param Kf          The feedforward coefficient
     * @param constraints Velocity and acceleration constraints for goal.
     */
    init {
        m_controller = PIDFController(Kp, Ki, Kd, Kf)
        m_constraints = constraints
    }

    /**
     * Sets the PIDFController gain parameters.
     *
     *
     * Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Differential coefficient
     */
    fun setPIDF(Kp: Double, Ki: Double, Kd: Double, Kf: Double) {
        m_controller.setPIDF(Kp, Ki, Kd, Kf)
    }

    /**
     * Sets the proportional coefficient of the PIDF controller gain.
     *
     * @param Kp proportional coefficient
     */
    fun setP(Kp: Double) {
        m_controller.setP(Kp)
    }

    /**
     * Sets the integral coefficient of the PIDF controller gain.
     *
     * @param Ki integral coefficient
     */
    fun setI(Ki: Double) {
        m_controller.setI(Ki)
    }

    /**
     * Sets the differential coefficient of the PIDF controller gain.
     *
     * @param Kd differential coefficient
     */
    fun setD(Kd: Double) {
        m_controller.setD(Kd)
    }

    /**
     * Sets the feedforward coefficient of the PIDF controller gain.
     *
     * @param Kf feedforward coefficient
     */
    fun setF(Kf: Double) {
        m_controller.setF(Kf)
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    fun getP(): Double {
        return m_controller.getP()
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    fun getI(): Double {
        return m_controller.getI()
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    fun getD(): Double {
        return m_controller.getD()
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    fun getPeriod(): Double {
        return m_controller.getPeriod()
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    fun setGoal(goal: TrapezoidProfile.State) {
        m_goal = goal
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    fun setGoal(goal: Double) {
        m_goal = TrapezoidProfile.State(goal, 0.0)
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    fun setGoal(goal: Int) {
        m_goal = TrapezoidProfile.State(goal.toDouble(), 0.0)
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    fun getGoal(): TrapezoidProfile.State? {
        return m_goal
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     *
     * This will return false until at least one input value has been computed.
     */
    fun atGoal(): Boolean {
        return atSetpoint() && m_goal == m_setpoint
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    fun setConstraints(constraints: TrapezoidProfile.Constraints?) {
        m_constraints = constraints
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    fun getSetpoint(): TrapezoidProfile.State? {
        return m_setpoint
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     *
     * This will return false until at least one input value has been computed.
     */
    fun atSetpoint(): Boolean {
        return m_controller.atSetPoint()
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY)
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
        m_controller.setTolerance(positionTolerance, velocityTolerance)
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    fun getPositionError(): Double {
        return m_controller.getPositionError()
    }

    /**
     * Returns the change in error per second.
     */
    fun getVelocityError(): Double {
        return m_controller.getVelocityError()
    }

    /**
     * Returns the next output of the PIDFcontroller.
     *
     * @param measurement The current measurement of the process variable.
     */
    fun calculate(measurement: Double): Double {
        val profile = TrapezoidProfile(m_constraints, m_goal, m_setpoint)
        m_setpoint = profile.calculate(getPeriod())
        return m_controller.calculate(measurement, m_setpoint.position)
    }

    /**
     * Returns the next output of the PIDFcontroller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     */
    fun calculate(measurement: Double, goal: TrapezoidProfile.State): Double {
        setGoal(goal)
        return calculate(measurement)
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     */
    fun calculate(measurement: Double, goal: Double): Double {
        setGoal(goal)
        return calculate(measurement)
    }

    /**
     * Returns the next output of the PIDFcontroller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    fun calculate(measurement: Double, goal: TrapezoidProfile.State,
                  constraints: TrapezoidProfile.Constraints?): Double {
        setConstraints(constraints)
        return calculate(measurement, goal)
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    fun reset() {
        m_controller.reset()
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    fun reset(measurement: TrapezoidProfile.State) {
        m_controller.reset()
        m_setpoint = measurement
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    fun reset(measuredPosition: Double, measuredVelocity: Double) {
        reset(TrapezoidProfile.State(measuredPosition, measuredVelocity))
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is
     * assumed to be zero.
     */
    fun reset(measuredPosition: Double) {
        reset(measuredPosition, 0.0)
    }

}