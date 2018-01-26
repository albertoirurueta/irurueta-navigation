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
 * Enumerator containing recognized typical distance units.
 */
public enum DistanceUnit {
    /**
     * Millimeter distance unit.
     */
    MILLIMETER,

    /**
     * Centimeter distance unit.
     */
    CENTIMETER,

    /**
     * Meter distance unit.
     */
    METER,

    /**
     * Kilometer distance unit.
     */
    KILOMETER,

    /**
     * Inch distance unit.
     */
    INCH,

    /**
     * Foot distance unit.
     */
    FOOT,

    /**
     * Yard distance unit.
     */
    YARD,

    /**
     * Mile distance unit.
     */
    MILE;

    /**
     * Returns unit system for provided distance unit.
     * @param unit distance unit to be checked.
     * @return unit system (metric or imperial).
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static UnitSystem getUnitSystem(DistanceUnit unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        switch (unit) {
            case INCH:
            case FOOT:
            case YARD:
            case MILE:
                return UnitSystem.IMPERIAL;
            case MILLIMETER:
            case CENTIMETER:
            case METER:
            case KILOMETER:
            default:
                return UnitSystem.METRIC;
        }
    }

    /**
     * Gets all supported metric distance units.
     * @return all supported metric distance units.
     */
    public static DistanceUnit[] getMetricUnits() {
        return new DistanceUnit[] {
                MILLIMETER,
                CENTIMETER,
                METER,
                KILOMETER
        };
    }

    /**
     * Gets all supported imperial distance units.
     * @return all supported imperial distance units.
     */
    public static DistanceUnit[] getImperialUnits() {
        return new DistanceUnit[] {
                INCH,
                FOOT,
                YARD,
                MILE
        };
    }

    /**
     * Indicates whether provided unit belongs to the metric unit system.
     * @param unit distance unit to be checked.
     * @return true if unit belongs to metric unit system.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isMetric(DistanceUnit unit) throws IllegalArgumentException {
        return getUnitSystem(unit) == UnitSystem.METRIC;
    }

    /**
     * Indicates whether provided unit belongs to the imperial unit system.
     * @param unit distance unit to be checked.
     * @return true if unit belongs to imperial unit system.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isImperial(DistanceUnit unit) throws IllegalArgumentException {
        return getUnitSystem(unit) == UnitSystem.IMPERIAL;
    }
}
