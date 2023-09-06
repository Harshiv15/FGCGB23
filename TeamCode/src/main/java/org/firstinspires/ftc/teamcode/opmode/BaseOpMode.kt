package org.firstinspires.ftc.teamcode.opmode

import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSys
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes
import org.firstinspires.ftc.teamcode.util.extractions.GamepadTrigger
import org.firstinspires.ftc.teamcode.util.extractions.TriggerGamepadEx
import java.math.BigDecimal
import java.math.RoundingMode


open class BaseOpMode : CommandOpMode() {
    private lateinit var tankLeft: DcMotorEx
    private lateinit var tankRight: DcMotorEx

    protected lateinit var gamepadEx1 : GamepadEx
    protected lateinit var gamepadEx2 : GamepadEx

    protected lateinit var triggerGamepadEx1: TriggerGamepadEx
    protected lateinit var triggerGamepadEx2: TriggerGamepadEx

    protected lateinit var driveSys: TankDriveSys

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

    protected fun gb1(trigger: GamepadKeys.Trigger): GamepadTrigger? {
        return triggerGamepadEx1.getGamepadTrigger(trigger)
    }

    protected fun gb2(trigger: GamepadKeys.Trigger): GamepadTrigger? {
        return triggerGamepadEx2.getGamepadTrigger(trigger)
    }

    // telemetry add data = tad
    protected fun tad(caption: String, value: Any) {
        BlocksOpModeCompanion.telemetry.addData(caption, value)
    }
    override fun initialize() {
        gamepadEx1 = GamepadEx(gamepad1)
        gamepadEx2 = GamepadEx(gamepad2)

        tankLeft = hardwareMap.get(DcMotorEx::class.java, "leftDrive")
        tankRight = hardwareMap.get(DcMotorEx::class.java, "rightDrive")

        if (TuningOpModes.DRIVE_CLASS == TankDrive::class.java) {
            driveSys = TankDriveSys(tankLeft, tankRight)
        }
    }

    companion object {

    }
}