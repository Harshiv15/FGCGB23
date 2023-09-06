package org.firstinspires.ftc.teamcode.util.extractions

import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys

class GamepadTrigger(gamepad: GamepadEx, vararg triggers: GamepadKeys.Trigger) : Trigger() {
    private val m_gamepad: GamepadEx
    private val m_triggers: Array<out GamepadKeys.Trigger>

    init {
        m_gamepad = gamepad
        m_triggers = triggers
    }

    override fun get(): Boolean {
        var res = true
        for(trigger : GamepadKeys.Trigger in m_triggers) {
            res = res && m_gamepad.getTrigger(trigger) > 0.5
        }
        return res
    }
}