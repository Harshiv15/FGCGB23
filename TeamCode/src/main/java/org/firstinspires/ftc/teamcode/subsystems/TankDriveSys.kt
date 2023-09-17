package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.Constants.joystickTransformFactor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign


class TankDriveSys (leftMotor: DcMotorEx, rightMotor: DcMotorEx) : SubsystemBase() {
    private val leftMotor: DcMotorEx
    private val rightMotor: DcMotorEx

    init {
        this.leftMotor = leftMotor
        this.rightMotor = rightMotor
    }

    // choose between differential or arcade based on preferences (arcade recommended)

    fun driveDifferential (leftPow: Double, rightPow: Double) =
        RunCommand({
            leftMotor.power = joystickTransform(leftPow)
            rightMotor.power = joystickTransform(rightPow)
        }, this)

    fun driveArcade (forwardPow: Double, turnPow: Double) =
        RunCommand({
            leftMotor.power = joystickTransform(forwardPow) + joystickTransform(turnPow)
            rightMotor.power = joystickTransform(forwardPow) - joystickTransform(turnPow)
        }, this)

    private fun joystickTransform (input: Double) =
        (1.0 / (joystickTransformFactor - 1)
            * sign(input)
            * (joystickTransformFactor.pow(abs(input)) - 1))

    override fun periodic() {
        super.periodic()

        leftMotor.power = leftMotor.power/max(leftMotor.power, 1.0)
        rightMotor.power = rightMotor.power/max(rightMotor.power, 1.0)
    }
}