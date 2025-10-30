package com.robgulley.vector

import com.robgulley.format.*
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.withSign

@kotlinx.serialization.Serializable
data class Quaternion(val w: Double, val x: Double, val y: Double, val z: Double) {
    constructor(array: DoubleArray) : this(array[0], array[1], array[2], array[3])
    constructor(other: Quaternion) : this(w = other.w, x = other.x, y = other.y, z = other.z)

    val list = listOf(w, x, y, z)
    val array = doubleArrayOf(w, x, y, z)

    companion object {
        // identity Quaternion
        val qId = Quaternion(1.0, 0.0, 0.0, 0.0)
    }

    override fun toString(): String {
        return "Q(w= ${w.format(3)} x= ${x.format(3)} y= ${y.format(3)} z= ${z.format(3)})"
    }

    val eulerAngle: Vector
        get() {
            // roll (x-axis rotation)
            val sinrCosp: Double = +2.0 * (w * x + y * z)
            val cosrCosp: Double = +1.0 - 2.0 * (x * x + y * y)
            val roll: Double = atan2(sinrCosp, cosrCosp)

            // pitch (y-axis rotation)
            val sinp: Double = +2.0 * (w * y - z * x)
            val pitch: Double = if (abs(sinp) >= 1.0)
                (PI / 2.0).withSign(sinp) // use 90 degrees if out of range
            else
                asin(sinp)

            // yaw (z-axis rotation)
            val sinyCosp: Double = +2.0 * (w * z + x * y)
            val cosyCosp: Double = +1.0 - 2.0 * (y * y + z * z)
            val yaw: Double = atan2(sinyCosp, cosyCosp)

            return Vector(roll, pitch, yaw)
        }

    infix operator fun get(i: Int): Double =
        when (i) {
            0 -> this.w
            1 -> this.x
            2 -> this.y
            3 -> this.z
            else -> throw IndexOutOfBoundsException()
        }
}