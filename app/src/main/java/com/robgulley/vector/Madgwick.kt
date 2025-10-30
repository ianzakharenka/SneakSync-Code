package com.robgulley.vector

import com.robgulley.hertz.Hz
import com.robgulley.time.Time
import kotlin.math.sqrt
import kotlin.time.Duration
import kotlin.time.DurationUnit
import kotlin.time.ExperimentalTime


@ExperimentalTime
class Madgwick(
    private val sampleFreq: Hz? = null,
    private val betas: List<Double> = listOf(0.1)
) // 2 * proportional gain (Kp)
{
    private val startTime = Time.now()
    private var lastTimestamp = Time.now()
    private var currentTimestamp = Time.now()
    private var currentBeta = 0
    private val beta
        get() = betas[currentBeta]

    //---------------------------------------------------------------------------------------------------
    // AHRS algorithm update
    fun update(quaternion: Quaternion, gyro: Vector, accel: Vector, mag: Vector): Quaternion {
        lastTimestamp = currentTimestamp
        currentTimestamp = Time.now()
        return update(quaternion, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag.x, mag.y, mag.z)
    }

    fun update(quaternion: Quaternion, gyro: Vector, accel: Vector, mag: Vector, timeStamp: Time): Quaternion {
        lastTimestamp = currentTimestamp
        currentTimestamp = timeStamp
        return update(quaternion, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag.x, mag.y, mag.z)
    }

    private fun update(
        inQ: Quaternion,
        gx: Double,
        gy: Double,
        gz: Double,
        ax_in: Double,
        ay_in: Double,
        az_in: Double,
        mx_in: Double,
        my_in: Double,
        mz_in: Double
    ): Quaternion {
        var ax = ax_in
        var ay = ay_in
        var az = az_in
        var mx = mx_in
        var my = my_in
        var mz = mz_in

        var recipNorm: Double
        var s0: Double
        var s1: Double
        var s2: Double
        var s3: Double

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if ((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
            return updateIMU(inQ, gx, gy, gz, ax, ay, az)
        }

        // Rate of change of quaternion from gyroscope
        var qDot1: Double = 0.5 * (-inQ.x * gx - inQ.y * gy - inQ.z * gz)
        var qDot2: Double = 0.5 * (inQ.w * gx + inQ.y * gz - inQ.z * gy)
        var qDot3: Double = 0.5 * (inQ.w * gy - inQ.x * gz + inQ.z * gx)
        var qDot4: Double = 0.5 * (inQ.w * gz + inQ.x * gy - inQ.y * gx)

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz)
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm

            // Auxiliary variables to avoid repeated arithmetic
            val aux2q0mx = 2.0 * inQ.w * mx
            val aux2q0my = 2.0 * inQ.w * my
            val aux2q0mz = 2.0 * inQ.w * mz
            val aux2q1mx = 2.0 * inQ.x * mx
            val aux2q0 = 2.0 * inQ.w
            val aux2q1 = 2.0 * inQ.x
            val aux2q2 = 2.0 * inQ.y
            val aux2q3 = 2.0 * inQ.z
            val aux2q0q2 = 2.0 * inQ.w * inQ.y
            val aux2q2q3 = 2.0 * inQ.y * inQ.z
            val q0q0 = inQ.w * inQ.w
            val q0q1 = inQ.w * inQ.x
            val q0q2 = inQ.w * inQ.y
            val q0q3 = inQ.w * inQ.z
            val q1q1 = inQ.x * inQ.x
            val q1q2 = inQ.x * inQ.y
            val q1q3 = inQ.x * inQ.z
            val q2q2 = inQ.y * inQ.y
            val q2q3 = inQ.y * inQ.z
            val q3q3 = inQ.z * inQ.z

            // Reference direction of Earth's magnetic field
            val hx = mx * q0q0 - aux2q0my * inQ.z + aux2q0mz * inQ.y + mx * q1q1 + aux2q1 * my * inQ.y + aux2q1 * mz * inQ.z - mx * q2q2 - mx * q3q3
            val hy = aux2q0mx * inQ.z + my * q0q0 - aux2q0mz * inQ.x + aux2q1mx * inQ.y - my * q1q1 + my * q2q2 + aux2q2 * mz * inQ.z - my * q3q3
            val aux2bx = sqrt(hx * hx + hy * hy)
            val aux2bz = -aux2q0mx * inQ.y + aux2q0my * inQ.x + mz * q0q0 + aux2q1mx * inQ.z - mz * q1q1 + aux2q2 * my * inQ.z - mz * q2q2 + mz * q3q3
            val aux4bx = 2.0 * aux2bx
            val aux4bz = 2.0 * aux2bz

            // Gradient decent algorithm corrective step
            s0 = -aux2q2 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q1 * (2.0 * q0q1 + aux2q2q3 - ay) - aux2bz * inQ.y * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (-aux2bx * inQ.z + aux2bz * inQ.x) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + aux2bx * inQ.y * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz)
            s1 = aux2q3 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q0 * (2.0 * q0q1 + aux2q2q3 - ay) - 4.0 * inQ.x * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + aux2bz * inQ.z * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (aux2bx * inQ.y + aux2bz * inQ.w) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + (aux2bx * inQ.z - aux4bz * inQ.x) * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz)
            s2 = -aux2q0 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q3 * (2.0 * q0q1 + aux2q2q3 - ay) - 4.0 * inQ.y * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-aux4bx * inQ.y - aux2bz * inQ.w) * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (aux2bx * inQ.x + aux2bz * inQ.z) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + (aux2bx * inQ.w - aux4bz * inQ.y) * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz)
            s3 = aux2q1 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q2 * (2.0 * q0q1 + aux2q2q3 - ay) + (-aux4bx * inQ.z + aux2bz * inQ.x) * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (-aux2bx * inQ.w + aux2bz * inQ.y) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + aux2bx * inQ.x * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz)
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
            s0 *= recipNorm
            s1 *= recipNorm
            s2 *= recipNorm
            s3 *= recipNorm

            // Apply feedback step
            qDot1 -= beta * s0
            qDot2 -= beta * s1
            qDot3 -= beta * s2
            qDot4 -= beta * s3
        }

        val elapsed = getElapsed()

        // Integrate rate of change of quaternion to yield quaternion
        var newQw = inQ.w + qDot1 * elapsed //(1.0 / sampleFreq)
        var newQx = inQ.x + qDot2 * elapsed //(1.0 / sampleFreq)
        var newQy = inQ.y + qDot3 * elapsed //(1.0 / sampleFreq)
        var newQz = inQ.z + qDot4 * elapsed //(1.0 / sampleFreq)

        // Normalise quaternion
        recipNorm = invSqrt(inQ.w * inQ.w + inQ.x * inQ.x + inQ.y * inQ.y + inQ.z * inQ.z)
        newQw *= recipNorm
        newQx *= recipNorm
        newQy *= recipNorm
        newQz *= recipNorm

        return Quaternion(newQw, newQx, newQy, newQz)
    }

    //---------------------------------------------------------------------------------------------------
    // IMU algorithm update
    private fun updateIMU(inQ: Quaternion, gx: Double, gy: Double, gz: Double, ax_in: Double, ay_in: Double, az_in: Double): Quaternion {
        var ax = ax_in
        var ay = ay_in
        var az = az_in
        var recipNorm: Double
        var s0: Double
        var s1: Double
        var s2: Double
        var s3: Double

        // Rate of change of quaternion from gyroscope
        var qDot1: Double = 0.5 * (-inQ.x * gx - inQ.y * gy - inQ.z * gz)
        var qDot2: Double = 0.5 * (inQ.w * gx + inQ.y * gz - inQ.z * gy)
        var qDot3: Double = 0.5 * (inQ.w * gy - inQ.x * gz + inQ.z * gx)
        var qDot4: Double = 0.5 * (inQ.w * gz + inQ.x * gy - inQ.y * gx)

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            // Auxiliary variables to avoid repeated arithmetic
            val aux2q0 = 2.0 * inQ.w
            val aux2q1 = 2.0 * inQ.x
            val aux2q2 = 2.0 * inQ.y
            val aux2q3 = 2.0 * inQ.z
            val aux4q0 = 4.0 * inQ.w
            val aux4q1 = 4.0 * inQ.x
            val aux4q2 = 4.0 * inQ.y
            val aux8q1 = 8.0 * inQ.x
            val aux8q2 = 8.0 * inQ.y
            val q0q0 = inQ.w * inQ.w
            val q1q1 = inQ.x * inQ.x
            val q2q2 = inQ.y * inQ.y
            val q3q3 = inQ.z * inQ.z

            // Gradient decent algorithm corrective step
            s0 = aux4q0 * q2q2 + aux2q2 * ax + aux4q0 * q1q1 - aux2q1 * ay
            s1 = aux4q1 * q3q3 - aux2q3 * ax + 4.0 * q0q0 * inQ.x - aux2q0 * ay - aux4q1 + aux8q1 * q1q1 + aux8q1 * q2q2 + aux4q1 * az
            s2 = 4.0 * q0q0 * inQ.y + aux2q0 * ax + aux4q2 * q3q3 - aux2q3 * ay - aux4q2 + aux8q2 * q1q1 + aux8q2 * q2q2 + aux4q2 * az
            s3 = 4.0 * q1q1 * inQ.z - aux2q1 * ax + 4.0 * q2q2 * inQ.z - aux2q2 * ay
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
            s0 *= recipNorm
            s1 *= recipNorm
            s2 *= recipNorm
            s3 *= recipNorm

            // Apply feedback step
            qDot1 -= beta * s0
            qDot2 -= beta * s1
            qDot3 -= beta * s2
            qDot4 -= beta * s3
        }

        val elapsed = getElapsed()

        // Integrate rate of change of quaternion to yield quaternion
        var newQw = inQ.w + qDot1 * elapsed //(1.0 / sampleFreq)
        var newQx = inQ.x + qDot2 * elapsed //(1.0 / sampleFreq)
        var newQy = inQ.y + qDot3 * elapsed //(1.0 / sampleFreq)
        var newQz = inQ.z + qDot4 * elapsed //(1.0 / sampleFreq)

        // Normalise quaternion
        recipNorm = invSqrt(inQ.w * inQ.w + inQ.x * inQ.x + inQ.y * inQ.y + inQ.z * inQ.z)
        newQw *= recipNorm
        newQx *= recipNorm
        newQy *= recipNorm
        newQz *= recipNorm

        return Quaternion(newQw, newQx, newQy, newQz)
    }

    private fun invSqrt(x: Double) = 1 / sqrt(x)

    private fun getElapsed(): Double {
        checkBetaCooldown()
        return sampleFreq?.wavelength
            ?: between(lastTimestamp, currentTimestamp).toDouble(DurationUnit.SECONDS)
    }


    private fun checkBetaCooldown() {
        val secondsSinceStart = between(startTime, Time.now()).inWholeSeconds.toInt()
        currentBeta = secondsSinceStart.coerceAtMost(betas.lastIndex)
    }

    private fun between(a: Time, b: Time): Duration = a.between(b)
}