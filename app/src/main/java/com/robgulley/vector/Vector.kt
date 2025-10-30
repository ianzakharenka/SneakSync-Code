package com.robgulley.vector

import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

@kotlinx.serialization.Serializable
data class Vector(val x: Double, val y: Double, val z: Double)  {
    constructor(list: List<Double>) : this(list[0], list[1], list[2])
    constructor(array: DoubleArray) : this(array[0], array[1], array[2])

    val list = listOf(x, y, z)
    val array = doubleArrayOf(x, y, z)

    companion object {
        val ZeroVector = Vector(0.0, 0.0, 0.0)
        val NonZeroVector = Vector(Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE)
    }

    fun toVectorInt() = list.map(Double::roundToInt).toVectorInt()

    override fun toString() = "x: $x\t\ty: $y\t\tz: $z"

    val quaternion: Quaternion
        get() {
            // yaw (Z), pitch (Y), roll (X)
            // Abbreviations for the various angular functions
            val cy = cos(z * 0.5)
            val sy = sin(z * 0.5)
            val cp = cos(y * 0.5)
            val sp = sin(y * 0.5)
            val cr = cos(x * 0.5)
            val sr = sin(x * 0.5)

            return Quaternion(
                w = cy * cp * cr + sy * sp * sr,
                x = cy * cp * sr - sy * sp * cr,
                y = sy * cp * sr + cy * sp * cr,
                z = sy * cp * cr - cy * sp * sr)
        }

    operator fun times(other: Double) = Vector(x * other, y * other, z * other)
    operator fun minus(other: Double) = Vector(x - other, y - other, z - other)
    operator fun plus(other: Double) = Vector(x + other, y + other, z + other)
    operator fun div(other: Double) = Vector(x / other, y / other, z / other)
    operator fun rem(other: Double) = Vector(x % other, y % other, z % other)

    operator fun times(other: Int) = other.toDouble().let { value -> Vector(x * value, y * value, z * value) }
    operator fun minus(other: Int) = other.toDouble().let { value -> Vector(x - value, y - value, z - value) }
    operator fun plus(other: Int) = other.toDouble().let { value -> Vector(x + value, y + value, z + value) }
    operator fun div(other: Int) = other.toDouble().let { value -> Vector(x / value, y / value, z / value) }
    operator fun rem(other: Int) = other.toDouble().let { value -> Vector(x % value, y % value, z % value) }

    operator fun times(other: Vector) = Vector(x * other.x, y * other.y, z * other.z)
    operator fun minus(other: Vector) = Vector(x - other.x, y - other.y, z - other.z)
    operator fun plus(other: Vector) = Vector(x + other.x, y + other.y, z + other.z)
    operator fun div(other: Vector) = Vector(x / other.x, y / other.y, z / other.z)
    operator fun rem(other: Vector) = Vector(x % other.x, y % other.y, z % other.z)

    fun scale(factor: Double) = this * factor //shorthand for times
    fun scale(vector: Vector) = this * vector //shorthand for times

    fun averageOfComponents() = (x + y + z) / 3
    fun unit() = this / sqrt(x * x + y * y + z * z)

    operator fun get(i: Int): Double =
        when (i) {
            0 -> this.x
            1 -> this.y
            2 -> this.z
            else -> throw IndexOutOfBoundsException()
        }

    fun transform(transform: (Double) -> Int) = VectorInt(transform(x), transform(y), transform(z))
    fun transform(transform: (Double) -> Double) = Vector(transform(x), transform(y), transform(z))

    fun any(predicate: (Double) -> Boolean) = list.any(predicate)
    fun all(predicate: (Double) -> Boolean) = list.all(predicate)
}