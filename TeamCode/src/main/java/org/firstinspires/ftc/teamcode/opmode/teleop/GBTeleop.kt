package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode
import org.firstinspires.ftc.teamcode.subsystems.LiftSys
import org.firstinspires.ftc.teamcode.util.Height
import kotlin.math.abs

@Config
@TeleOp (name = "GBTeleop")
class GBTeleop : BaseOpMode() {
    override fun initialize() {
        super.initialize()

        gb2(Y).whenActive(liftSys.goTo(Height.TANK))
        gb2(X).whenActive(liftSys.goTo(Height.ELECTROLYZER))
        gb2(B).whenActive(liftSys.goTo(Height.CLIMB))
        gb2(A).whenActive(liftSys.goTo(Height.ZERO))

        register(driveSys, liftSys, intakeSys)

        driveSys.defaultCommand = driveSys.driveArcade(gamepadEx1.leftY, gamepadEx1.rightX)
        liftSys.defaultCommand = liftSys.setPower(gamepadEx2.getTrigger(RIGHT_TRIGGER)-gamepadEx2.getTrigger(LEFT_TRIGGER))
        intakeSys.defaultCommand = intakeSys.setPower(
                // if(gb2(LEFT_BUMPER).get()) -0.8 else if(gb2(RIGHT_BUMPER).get()) 0.8 else 0.0
                gb2t(RIGHT_TRIGGER) - gb2t(LEFT_TRIGGER)
        )
    }

    override fun run() {
        super.run()

        tad("goal", LiftSys.currentHeight)
        tad("ref (avg)", liftSys.getCurrentPosition())
        tad("error", abs(LiftSys.currentHeight - liftSys.getCurrentPosition()))
        telemetry.update()
    }
}