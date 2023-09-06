package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.util.extractions.ProfiledPIDFController
import java.util.function.DoubleSupplier

class LiftSys (left: MotorEx, right: MotorEx) : SubsystemBase() {
    private val left: MotorEx
    private val right: MotorEx

    private val leftPIDF: ProfiledPIDFController
    private val rightPIDF: ProfiledPIDFController

    companion object {
        @JvmStatic
        var kP = 0.0

        @JvmStatic
        var kI = 0.0

        @JvmStatic
        var kD = 0.0

        @JvmStatic
        var kF = 0.0


        @JvmStatic
        var maxVel = 200.0

        @JvmStatic
        var maxAccel = 200.0

        @JvmStatic
        var threshold = 10

        @JvmStatic
        var currentHeight = 0
    }

    init {
        this.left = left
        this.right = right
        leftPIDF = ProfiledPIDFController(kP, kI, kD, kF,
                TrapezoidProfile.Constraints(maxVel, maxAccel))
        rightPIDF = ProfiledPIDFController(kP, kI, kD, kF,
                TrapezoidProfile.Constraints(maxVel, maxAccel))
    }

    fun setPower(power: DoubleSupplier) =
    RunCommand({
        left.set(power.asDouble)
        right.set(power.asDouble)
        leftPIDF.setGoal(left.currentPosition)
        rightPIDF.setGoal(right.currentPosition)
    })

    private fun setHeight(tick: Int) {
        currentHeight = tick
        leftPIDF.setGoal(tick)
        rightPIDF.setGoal(tick)
    }

    fun atTarget() = left.currentPosition < currentHeight + threshold
            && left.currentPosition > currentHeight - threshold

}