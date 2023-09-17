package org.firstinspires.ftc.teamcode.util

enum class Height(height: Int) {
    // TODO - tune electrolyzer, tank, & climb heights
    ELECTROLYZER(5000),
    TANK(10000),
    CLIMB(7500),
    ZERO(0);

    // height in encoder ticks
    private val height: Int

    init {
        this.height = height
    }

    open fun getHeight(): Int {
        return height
    }
}