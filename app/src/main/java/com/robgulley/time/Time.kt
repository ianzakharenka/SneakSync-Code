@file:Suppress("EXPERIMENTAL_IS_NOT_ENABLED")

package com.robgulley.time

import kotlin.time.Duration
import kotlin.time.DurationUnit
import kotlin.time.ExperimentalTime
import kotlin.time.toDuration

@OptIn(ExperimentalTime::class)
class Time private constructor(private val markNanos: Long) : Comparable<Time> {
    companion object {
        fun now(): Time = Time(System.nanoTime())

    }

    fun between(other: Time): Duration {
        val diffNanos = other.markNanos - this.markNanos
        return diffNanos.toDuration(DurationUnit.NANOSECONDS)
    }

    val epochMilli: Long
        get() = System.currentTimeMillis()

    val nanoTime: Long
        get() = this.markNanos

    fun minusMillis(millis: Long): Time {
        return Time(this.markNanos - millis.toDuration(DurationUnit.MILLISECONDS).inWholeNanoseconds)
    }

    fun plusMillis(millis: Long): Time {
        return Time(this.markNanos + millis.toDuration(DurationUnit.MILLISECONDS).inWholeNanoseconds)
    }

    fun isAfter(timeMark: Time): Boolean {
        return this.markNanos > timeMark.markNanos
    }

    fun isBefore(timeMark: Time): Boolean {
        return this.markNanos < timeMark.markNanos
    }

    fun elapsedThenToNow(): Duration {
        val nowNanos = System.nanoTime()
        return (nowNanos - this.markNanos).toDuration(DurationUnit.NANOSECONDS)
    }

    operator fun minus(other: Duration): Time {
        return Time(this.markNanos - other.inWholeNanoseconds)
    }

    operator fun plus(other: Duration): Time {
        return Time(this.markNanos + other.inWholeNanoseconds)
    }

    override fun toString(): String {
        // Simple representation, can be made more sophisticated if needed
        return "Time(markNanos=$markNanos)"
    }

    override fun compareTo(other: Time): Int {
        return this.markNanos.compareTo(other.markNanos)
    }
}