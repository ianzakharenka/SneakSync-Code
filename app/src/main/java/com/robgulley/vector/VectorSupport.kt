package com.robgulley.vector

//--vector double utils
fun DoubleArray.toVector() = Vector(this)
fun DoubleArray.toVector(startIndex: Int) = Vector(this.sliceArray(startIndex..startIndex + 2))
fun List<Double>.toVector() = Vector(this)

fun minOf(a: Vector, b: Vector): Vector = Vector(minOf(a.x, b.x), minOf(a.y, b.y), minOf(a.z, b.z))
fun maxOf(a: Vector, b: Vector) = Vector(maxOf(a.x, b.x), maxOf(a.y, b.y), maxOf(a.z, b.z))

fun Collection<Vector>.average(): Vector =
    this.fold(Vector(0.0, 0.0, 0.0)) { acc: Vector, v: Vector -> acc + v } / this.size.toDouble()

infix operator fun Double.times(other: Vector) = Vector(this * other.x, this * other.y, this * other.z)
infix operator fun Double.minus(other: Vector) = Vector(this - other.x, this - other.y, this - other.z)
infix operator fun Double.plus(other: Vector) = Vector(this + other.x, this + other.y, this + other.z)
infix operator fun Double.div(other: Vector) = Vector(this / other.x, this / other.y, this / other.z)
infix operator fun Double.rem(other: Vector) = Vector(this % other.x, this % other.y, this % other.z)

//--vector int utils
fun IntArray.toVectorInt() = VectorInt(this)
fun IntArray.toVectorInt(startIndex: Int) = VectorInt(this.sliceArray(startIndex..startIndex + 2))
fun List<Int>.toVectorInt() = VectorInt(this)

fun minOf(a: VectorInt, b: VectorInt): VectorInt = VectorInt(minOf(a.x, b.x), minOf(a.y, b.y), minOf(a.z, b.z))
fun maxOf(a: VectorInt, b: VectorInt) = VectorInt(maxOf(a.x, b.x), maxOf(a.y, b.y), maxOf(a.z, b.z))

infix operator fun Int.times(other: VectorInt) = VectorInt(this * other.x, this * other.y, this * other.z)
infix operator fun Int.minus(other: VectorInt) = VectorInt(this - other.x, this - other.y, this - other.z)
infix operator fun Int.plus(other: VectorInt) = VectorInt(this + other.x, this + other.y, this + other.z)
infix operator fun Int.div(other: VectorInt) = VectorInt(this / other.x, this / other.y, this / other.z)
infix operator fun Int.rem(other: VectorInt) = VectorInt(this % other.x, this % other.y, this % other.z)

//quaternion
fun DoubleArray.toQuaternion() = Quaternion(this)