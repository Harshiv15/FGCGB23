package org.firstinspires.ftc.teamcode.util.extractions

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.qualcomm.robotcore.hardware.Gamepad

class TriggerGamepadEx(gamepad : Gamepad, ex: GamepadEx) {
    lateinit var gamepad : Gamepad

    private lateinit var triggerReaders : HashMap<GamepadKeys.Trigger, TriggerReader>

    private lateinit var gamepadTriggers : HashMap<GamepadKeys.Trigger, GamepadTrigger>

    private val triggers = arrayOf(GamepadKeys.Trigger.LEFT_TRIGGER, GamepadKeys.Trigger.RIGHT_TRIGGER)

    init {
        this.gamepad = gamepad
        triggerReaders = HashMap()
        gamepadTriggers = HashMap()
        for(trigger : GamepadKeys.Trigger in triggers) {
            triggerReaders.put(trigger, TriggerReader(ex, trigger))
            gamepadTriggers.put(trigger, GamepadTrigger(ex, trigger))
        }
    }

    fun getTrigger(trigger: GamepadKeys.Trigger): Double {
        var triggerValue = 0.0
        when (trigger) {
            GamepadKeys.Trigger.LEFT_TRIGGER -> triggerValue = gamepad.left_trigger.toDouble()
            GamepadKeys.Trigger.RIGHT_TRIGGER -> triggerValue = gamepad.right_trigger.toDouble()
            else -> {}
        }
        return triggerValue
    }

    fun wasJustPressed(trigger: GamepadKeys.Trigger): Boolean {
        return triggerReaders[trigger]!!.wasJustPressed()
    }


    fun wasJustReleased(trigger: GamepadKeys.Trigger): Boolean {
        return triggerReaders[trigger]!!.wasJustReleased()
    }


    fun readTriggers() {
        for (trigger in triggers) {
            triggerReaders[trigger]!!.readValue()
        }
    }


    fun isDown(trigger: GamepadKeys.Trigger): Boolean {
        return triggerReaders[trigger]!!.isDown
    }


    fun stateJustChanged(trigger: GamepadKeys.Trigger): Boolean {
        return triggerReaders[trigger]!!.stateJustChanged()
    }


    fun getGamepadTrigger(trigger: GamepadKeys.Trigger): GamepadTrigger? {
        return gamepadTriggers[trigger]
    }
}