package com.robgulley.vector

import kotlin.math.sqrt

@kotlinx.serialization.Serializable
data class VectorInt(val x: Int, val y: Int, val z: Int) {
    constructor(list: List<Int>) : this(list[0], list[1], list[2])
    constructor(array: IntArray) : this(array[0], array[1], array[2])

    val list = listOf(x, y, z)
    val array = intArrayOf(x, y, z)

    companion object {
        val ZeroVector = VectorInt(0, 0, 0)
    }

    fun toVector() = list.map(Int::toDouble).toVector()

    override fun toString() = "x: $x\t\ty: $y\t\tz: $z"

    infix operator fun times(other: Int) = VectorInt(x * other, y * other, z * other)
    infix operator fun minus(other: Int) = VectorInt(x - other, y - other, z - other)
    infix operator fun plus(other: Int) = VectorInt(x + other, y + other, z + other)
    infix operator fun div(other: Int) = VectorInt(x / other, y / other, z / other)
    infix operator fun rem(other: Int) = VectorInt(x % other, y % other, z % other)

    infix operator fun times(other: Double) = Vector(x * other, y * other, z * other)
    infix operator fun minus(other: Double) = Vector(x - other, y - other, z - other)
    infix operator fun plus(other: Double) = Vector(x + other, y + other, z + other)
    infix operator fun div(other: Double) = Vector(x / other, y / other, z / other)
    infix operator fun rem(other: Double) = Vector(x % other, y % other, z % other)

    infix operator fun times(other: VectorInt) = VectorInt(x * other.x, y * other.y, z * other.z)
    infix operator fun minus(other: VectorInt) = VectorInt(x - other.x, y - other.y, z - other.z)
    infix operator fun plus(other: VectorInt) = VectorInt(x + other.x, y + other.y, z + other.z)
    infix operator fun div(other: VectorInt) = VectorInt(x / other.x, y / other.y, z / other.z)
    infix operator fun rem(other: VectorInt) = VectorInt(x % other.x, y % other.y, z % other.z)

    infix operator fun times(other: Vector) = Vector(x * other.x, y * other.y, z * other.z)
    infix operator fun minus(other: Vector) = Vector(x - other.x, y - other.y, z - other.z)
    infix operator fun plus(other: Vector) = Vector(x + other.x, y + other.y, z + other.z)
    infix operator fun div(other: Vector) = Vector(x / other.x, y / other.y, z / other.z)
    infix operator fun rem(other: Vector) = Vector(x % other.x, y % other.y, z % other.z)

    infix fun and(other: Int): VectorInt = VectorInt(x and other, y and other, z and other)
    infix fun or(other: Int): VectorInt = VectorInt(x or other, y or other, z or other)
    infix fun shr(amount: Int): VectorInt = VectorInt(x shr amount, y shr amount, z shr amount)
    infix fun shl(amount: Int): VectorInt = VectorInt(x shl amount, y shl amount, z shl amount)

    fun scale(factor: Int) = this * factor //shorthand for times
    fun scale(factor: Double) = this * factor //shorthand for times
    fun scale(vector: VectorInt) = this * vector //shorthand for times
    fun scale(vector: Vector) = this * vector //shorthand for times

    fun averageOfComponents() = (x + y + z) / 3
    fun unit() = this / sqrt((x * x + y * y + z * z).toDouble())

    infix operator fun get(i: Int): Int =
        when (i) {
            0 -> this.x
            1 -> this.y
            2 -> this.z
            else -> throw IndexOutOfBoundsException()
        }

    fun transform(transform: (Int) -> Int) = VectorInt(transform(x), transform(y), transform(z))
    fun transform(transform: (Int) -> Double) = Vector(transform(x), transform(y), transform(z))

    fun any(predicate: (Int) -> Boolean) = list.any(predicate)
    fun all(predicate: (Int) -> Boolean) = list.all(predicate)
}

fun Collection<VectorInt>.average(): Vector = this.map { it.toVector() }.average()