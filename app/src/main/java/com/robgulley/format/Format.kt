package com.robgulley.format

// This file provides number formatting extension functions.

/**
 * Formats a Double to a string with a specific number of decimal digits.
 * Uses platform's String.format.
 *
 * @param digits The number of decimal digits to display.
 * @return The formatted string.
 */
fun Double.format(digits: Int): String {
    // In Kotlin/JVM (Android), this delegates to java.lang.String.format
    return String.format("%.${digits}f", this)
}

/**
 * Formats a Float to a string with a specific number of decimal digits.
 * Uses platform's String.format.
 *
 * @param digits The number of decimal digits to display.
 * @return The formatted string.
 */
fun Float.format(digits: Int): String {
    // In Kotlin/JVM (Android), this delegates to java.lang.String.format
    return String.format("%.${digits}f", this)
}