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
 * Enumerator containing recognized surface units.
 */
public enum SurfaceUnit {
    /**
     * Square millimeter (mm²).
     */
    SQUARE_MILLIMETER,

    /**
     * Square centimeter (cm²).
     */
    SQUARE_CENTIMETER,

    /**
     * Square meter (m²).
     */
    SQUARE_METER,

    /**
     * Square kilometer (Km²).
     */
    SQUARE_KILOMETER,

    /**
     * Square inch (sq in).
     */
    SQUARE_INCH,

    /**
     * Square foot (sq ft).
     */
    SQUARE_FOOT,

    /**
     * Square yard (sq yd).
     */
    SQUARE_YARD,

    /**
     * Square mile (sq mi).
     */
    SQUARE_MILE,

    /**
     * Centiare (ca).
     */
    CENTIARE,

    /**
     * Are (a).
     */
    ARE,

    /**
     * Decare (daa).
     */
    DECARE,

    /**
     * Hectare (ha).
     */
    HECTARE,

    /**
     * Acre (acre).
     */
    ACRE;

    /**
     * Returns unit system for provided surface unit.
     * @param unit surface unit to be checked.
     * @return unit system (metric or imperial).
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static UnitSystem getUnitSystem(SurfaceUnit unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        switch (unit) {
            case SQUARE_INCH:
            case SQUARE_FOOT:
            case SQUARE_YARD:
            case SQUARE_MILE:
            case ACRE:
                return UnitSystem.IMPERIAL;
            case SQUARE_MILLIMETER:
            case SQUARE_CENTIMETER:
            case SQUARE_METER:
            case SQUARE_KILOMETER:
            case CENTIARE:
            case ARE:
            case DECARE:
            case HECTARE:
            default:
                return UnitSystem.METRIC;
        }
    }

    /**
     * Gets all supported metric surface units.
     * @return all supported metric surface units.
     */
    public static SurfaceUnit[] getMetricUnits() {
        return new SurfaceUnit[] {
                SQUARE_MILLIMETER,
                SQUARE_CENTIMETER,
                SQUARE_METER,
                SQUARE_KILOMETER,
                CENTIARE,
                ARE,
                DECARE,
                HECTARE
        };
    }

    /**
     * Gets all supported imperial distance units.
     * @return all supported imperial distance units.
     */
    public static SurfaceUnit[] getImperialUnits() {
        return new SurfaceUnit[] {
                SQUARE_INCH,
                SQUARE_FOOT,
                SQUARE_YARD,
                SQUARE_MILE,
                ACRE
        };
    }

    /**
     * Indicates whether provided unit belongs to the metric unit system.
     * @param unit distance unit to be checked.
     * @return true if unit belongs to metric unit system.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isMetric(SurfaceUnit unit) throws IllegalArgumentException {
        return getUnitSystem(unit) == UnitSystem.METRIC;
    }

    /**
     * Indicates whether provided unit belongs to the imperial unit system.
     * @param unit distance unit to be checked.
     * @return true if unit belongs to imperial unit system.
     * @throws IllegalArgumentException if unit is null or not supported.
     */
    public static boolean isImperial(SurfaceUnit unit) throws IllegalArgumentException {
        return getUnitSystem(unit) == UnitSystem.IMPERIAL;
    }
}
