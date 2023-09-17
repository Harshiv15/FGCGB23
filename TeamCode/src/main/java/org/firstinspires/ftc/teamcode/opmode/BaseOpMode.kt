package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys
import org.firstinspires.ftc.teamcode.subsystems.LiftSys
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSys
import org.firstinspires.ftc.teamcode.util.extractions.GamepadTrigger
import org.firstinspires.ftc.teamcode.util.extractions.TriggerGamepadEx
import java.math.BigDecimal
import java.math.RoundingMode

open class BaseOpMode : CommandOpMode() {
    // drive
    private lateinit var tankLeft: DcMotorEx
    private lateinit var tankRight: DcMotorEx

    // elev (integrated score + climb)
    private lateinit var elevLeft: DcMotorEx
    private lateinit var elevRight: DcMotorEx

    // intake
    private lateinit var intakeLeft: DcMotorEx
    private lateinit var intakeRight: DcMotorEx

    // gamepad joysticks + buttons
    protected lateinit var gamepadEx1 : GamepadEx
    protected lateinit var gamepadEx2 : GamepadEx

    // gamepad triggers
    protected lateinit var triggerGamepadEx1: TriggerGamepadEx
    protected lateinit var triggerGamepadEx2: TriggerGamepadEx

    // subsystems
    protected lateinit var driveSys: TankDriveSys
    protected lateinit var liftSys: LiftSys
    protected lateinit var intakeSys: IntakeSys

    protected fun round(value: Double, places: Int): Double {
        require(places >= 0)
        return BigDecimal(value.toString()).setScale(places, RoundingMode.HALF_UP).toDouble()
    }

    protected fun round(value: Double): Double {
        return round(value, 4)
    }

    protected fun gb1(button: GamepadKeys.Button?): GamepadButton {
        return gamepadEx1.getGamepadButton(button)
    }

    protected fun gb2(button: GamepadKeys.Button?): GamepadButton {
        return gamepadEx2.getGamepadButton(button)
    }

    protected fun gb1(trigger: Trigger) = triggerGamepadEx1.getGamepadTrigger(trigger)

    protected fun gb1t(trigger: Trigger) = gamepadEx2.getTrigger(trigger)

    protected fun gb2(trigger: Trigger) = triggerGamepadEx2.getGamepadTrigger(trigger)

    protected fun gb2t(trigger: Trigger) = gamepadEx2.getTrigger(trigger)

    // telemetry add data = tad
    protected fun tad(caption: String, value: Any) {
        telemetry.addData(caption, value)
    }
    override fun initialize() {

        gamepadEx1 = GamepadEx(gamepad1)
        gamepadEx2 = GamepadEx(gamepad2)

        initializeHardware()

        setupHardware()

        driveSys = TankDriveSys(tankLeft, tankRight)

        liftSys = LiftSys(elevLeft, elevRight)

        intakeSys = IntakeSys(intakeLeft, intakeRight)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    private fun initializeHardware() {
        tankLeft = hardwareMap.get(DcMotorEx::class.java, "leftDrive")
        tankRight = hardwareMap.get(DcMotorEx::class.java, "rightDrive")

        elevLeft = hardwareMap.get(DcMotorEx::class.java, "elevLeft")
        elevRight = hardwareMap.get(DcMotorEx::class.java, "elevRight")

        intakeLeft = hardwareMap.get(DcMotorEx::class.java, "intakeLeft")
        intakeRight = hardwareMap.get(DcMotorEx::class.java, "intakeRight")
    }

    private fun setupHardware() {
        tankLeft.direction = DcMotorSimple.Direction.REVERSE
    }
}