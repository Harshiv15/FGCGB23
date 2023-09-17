package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotorEx

@Config
class IntakeSys(left: DcMotorEx, right: DcMotorEx): SubsystemBase() {
    private val left: DcMotorEx
    private val right: DcMotorEx

    private val leftPID: PIDController
    private val rightPID: PIDController

    companion object {
        @JvmField
        var kP = 0.0

        @JvmField
        var kI = 0.0

        @JvmField
        var kD = 0.0
    }

    init {
        this.left = left
        this.right = right
        this.leftPID = PIDController(kP, kI, kD)
        this.rightPID = PIDController(kP, kI, kD)
    }

    fun setPower(power: Double) = RunCommand({
        leftPID.setPoint = power
        leftPID.setPoint = power
    })

    /*fun intake() = InstantCommand({
        leftPID.setPoint = 0.8
        rightPID.setPoint = 0.8
    })

    fun outtake() = InstantCommand({66
        leftPID.setPoint = -0.8
        rightPID.setPoint = -0.8
    })*/

    override fun periodic() {
        left.power = leftPID.calculate(left.power)
        left.power = leftPID.calculate(left.power)
    }
}