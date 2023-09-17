package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.Height
import org.firstinspires.ftc.teamcode.util.extractions.ProfiledPIDFController

@Config
class LiftSys (left: DcMotorEx, right: DcMotorEx) : SubsystemBase() {
    private val left: DcMotorEx
    private val right: DcMotorEx

    private val leftPIDF: ProfiledPIDFController
    private val rightPIDF: ProfiledPIDFController

    // private val ticksInDegrees = 1680 / 360.0

    companion object {
        @JvmField
        var kP = 0.0075

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0

        @JvmField
        var kF = 0.0


        @JvmField
        var maxVel = 200.0

        @JvmField
        var maxAccel = 200.0

        @JvmField
        var threshold = 10

        @JvmField
        var currentHeight = 0
    }

    fun getCurrentPosition() = (left.currentPosition + right.currentPosition)/2

    init {
        this.left = left
        this.right = right

        leftPIDF = ProfiledPIDFController(kP, kI, kD, kF,
                TrapezoidProfile.Constraints(maxVel, maxAccel))
        rightPIDF = ProfiledPIDFController(kP, kI, kD, kF,
                TrapezoidProfile.Constraints(maxVel, maxAccel))
    }

    fun setPower(power: Double) = RunCommand({
        left.power = power
        right.power = power
        leftPIDF.setGoal(left.currentPosition)
        rightPIDF.setGoal(right.currentPosition)
    }, this)

    private fun setHeight(tick: Int) {
        currentHeight = tick
        leftPIDF.setGoal(tick)
        rightPIDF.setGoal(tick)
    }

    fun goTo(tick: Int) = InstantCommand({
        setHeight(tick)
    }).andThen(WaitUntilCommand(this::atTarget))

    fun goTo(height: Height) = InstantCommand({
        setHeight(height.getHeight())
    }).andThen(WaitUntilCommand(this::atTarget))

    override fun periodic() {
        val leftPower = leftPIDF.calculate(left.currentPosition.toDouble())
        val rightPower = rightPIDF.calculate(right.currentPosition.toDouble())
        left.power = leftPower
        right.power = rightPower
    }

    fun atTarget() = left.currentPosition < currentHeight + threshold
            && left.currentPosition > currentHeight - threshold

}