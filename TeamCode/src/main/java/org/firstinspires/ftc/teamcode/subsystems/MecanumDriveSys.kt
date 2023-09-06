package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.slowFactor
import java.util.function.DoubleSupplier

class MecanumDriveSys(fL: MotorEx, fR: MotorEx, bL: MotorEx, bR: MotorEx) : SubsystemBase() {
    private val drive: MecanumDrive

    init {
        drive = MecanumDrive(false, fL, fR, bL,  bR)
    }

    fun fieldCentric(strafeSpeed: DoubleSupplier, forwardSpeed: DoubleSupplier,
                     turnSpeed: DoubleSupplier, gyroAngle: DoubleSupplier) : Command {
        return RunCommand({
            drive.driveFieldCentric(
                    strafeSpeed.asDouble, forwardSpeed.asDouble, turnSpeed.asDouble, gyroAngle.asDouble
            )
        }, this)
    }

    fun robotCentric(strafeSpeed: DoubleSupplier, forwardSpeed: DoubleSupplier,
                     turnSpeed: DoubleSupplier) : Command {
        return RunCommand({
            drive.driveRobotCentric(
                    strafeSpeed.asDouble, forwardSpeed.asDouble, turnSpeed.asDouble
            )
        }, this)
    }

    fun slowMode(strafeSpeed: DoubleSupplier, forwardSpeed: DoubleSupplier,
                 turnSpeed: DoubleSupplier) : Command {
        return RunCommand({
            drive.driveRobotCentric(
                    strafeSpeed.asDouble / slowFactor, forwardSpeed.asDouble / slowFactor, turnSpeed.asDouble / slowFactor
            )
        }, this)
    }
}