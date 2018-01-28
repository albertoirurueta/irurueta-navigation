/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.units;

/**
 * Enumerator containing recognized typical time units.
 */
public enum TimeUnit {
    /**
     * Nanosecond time unit. This belongs to the International System of units.
     */
    NANOSECOND,

    /**
     * Microsecond time unit. This belongs to the International System of units.
     */
    MICROSECOND,

    /**
     * Millisecond time unit. This belongs to the International System of units.
     */
    MILLISECOND,

    /**
     * Second time unit. This belongs to the International System of units.
     */
    SECOND,

    /**
     * Minute time unit.
     */
    MINUTE,

    /**
     * Hour time unit.
     */
    HOUR,

    /**
     * Day time unit.
     */
    DAY,

    /**
     * Week time unit.
     */
    WEEK,

    /**
     * Month time unit (considered as 30 days).
     */
    MONTH,

    /**
     * Year time unit (considered as 365 days).
     */
    YEAR,

    /**
     * Century time unit (considered as 100 years).
     */
    CENTURY;

    /**
     * Returns unit system for provided time unit.
     * @param unit time unit to be checked.
     * @return returns metric system only for units belonging to the International
     * System of units.
     * @throws IllegalArgumentException if unit is null or does not belong to the
     * international system of units.
     */
    public static UnitSystem getUnitSystem(TimeUnit unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        switch (unit) {
            case NANOSECOND:
            case MICROSECOND:
            case MILLISECOND:
            case SECOND:
                return UnitSystem.METRIC;
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Gets all supported metric time units.
     * @return all supported metric time units.
     */
    public static TimeUnit[] getMetricUnits() {
        return new TimeUnit[] {
                NANOSECOND,
                MICROSECOND,
                MILLISECOND,
                SECOND
        };
    }

    /**
     * Gets all supported units not belonging to the International System
     * of Units.
     * @return all units not belonging to the International System of Units.
     */
    public static TimeUnit[] getNonInternationalSystemUnits() {
        return new TimeUnit[] {
                MINUTE,
                HOUR,
                DAY,
                WEEK,
                MONTH,
                YEAR,
                CENTURY
        };
    }

    /**
     * Indicates whether provided unit belongs to the metric unit system.
     * @param unit time unit to be checked.
     * @return true if unit belongs to metric unit system.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isMetric(TimeUnit unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        try {
            return getUnitSystem(unit) == UnitSystem.METRIC;
        } catch (IllegalArgumentException e) {
            return false;
        }
    }

    /**
     * Indicates whether provided unit belongs to the International System of
     * units.
     * @param unit time unit to be checked.
     * @return true if unit does not belong to the International System of units,
     * false otherwise.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isNonInternationalSystem(TimeUnit unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        return !isMetric(unit);
    }
}
