package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Main", group = "Main")
class TestOpMode : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        telemetry.addData(">", "Running!")
        telemetry.update()
    }
}