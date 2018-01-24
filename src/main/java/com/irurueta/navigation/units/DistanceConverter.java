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

import java.math.BigDecimal;

/**
 * Does distance conversions to different units.
 * To prevent loss of accuracy, conversion should only be done as a final step
 * before displaying distance measurements.
 */
public class DistanceConverter {

    /**
     * Number of meters in 1 millimeter.
     */
    static final double METERS_PER_MILLIMETER = 0.001;

    /**
     * Number of meters in 1 centimeter.
     */
    static final double METERS_PER_CENTIMETER = 0.01;

    /**
     * Number of meters in 1 kilometer.
     */
    static final double METERS_PER_KILOMETER = 1000.0;

    /**
     * Number of meters in 1 inch.
     */
    static final double METERS_PER_INCH = 0.0254;

    /**
     * Number of meters in 1 foot.
     */
    static final double METERS_PER_FOOT = 0.3048;

    /**
     * Number of meters in 1 yard.
     */
    static final double METERS_PER_YARD = 0.9144;

    /**
     * Number of meters in 1 mile.
     */
    static final double METERS_PER_MILE = 1609.344;

    /**
     * Constructor.
     * Prevents instantiation of helper class.
     */
    DistanceConverter() { }

    /**
     * Converts a distance to provided output distance unit.
     * @param input input distance to be converted.
     * @param output output distance where result will be stored and
     *               containing output unit.
     */
    public static void convert(Distance input, Distance output) {
        convert(input, output.getUnit(), output);
    }

    /**
     * Converts a distance to requested output unit.
     * @param input input distance to be converted.
     * @param outputUnit requested output unit.
     * @return converted distance unit.
     */
    public static Distance convertAndReturnNew(Distance input, DistanceUnit outputUnit) {
        Distance result = new Distance();
        convert(input, outputUnit, result);
        return result;
    }

    /**
     * Converts and updates a distance to requested output unit.
     * @param distance input distance be converted and updated.
     * @param outputUnit requested output unit.
     */
    public static void convert(Distance distance, DistanceUnit outputUnit) {
        convert(distance, outputUnit, distance);
    }

    /**
     * Converts a distance to requested output unit.
     * @param input input distance to be converted.
     * @param outputUnit requested output unit.
     * @param result distance unit where result will be stored.
     */
    public static void convert(Distance input, DistanceUnit outputUnit, Distance result) {
        Number value = convert(input.getValue(), input.getUnit(), outputUnit);
        result.setValue(value);
        result.setUnit(outputUnit);
    }

    /**
     * Converts a distance value from input unit to provided output unit.
     * @param input distance value.
     * @param inputUnit input distance unit.
     * @param outputUnit output distance unit.
     * @return converted distance value.
     */
    public static Number convert(Number input, DistanceUnit inputUnit,
                                 DistanceUnit outputUnit) {
        return new BigDecimal(convert(input.doubleValue(), inputUnit, outputUnit));
    }

    /**
     * Converts a distance value from input unit to provided output unit.
     * @param input distance value.
     * @param inputUnit input distance unit.
     * @param outputUnit output distance unit.
     * @return converted distance value.
     */
    public static double convert(double input, DistanceUnit inputUnit,
                                 DistanceUnit outputUnit) {
        double meters;

        //convert to meters
        switch (inputUnit) {
            case MILLIMETER:
                meters = millimeterToMeter(input);
                break;
            case CENTIMETER:
                meters = centimeterToMeter(input);
                break;
            case KILOMETER:
                meters = kilometerToMeter(input);
                break;
            case INCH:
                meters = inchToMeter(input);
                break;
            case FOOT:
                meters = footToMeter(input);
                break;
            case YARD:
                meters = yardToMeter(input);
                break;
            case MILE:
                meters = mileToMeter(input);
                break;

            case METER:
            default:
                meters = input;
                break;
        }

        //convert from meter to required output unit
        switch (outputUnit) {
            case MILLIMETER:
                return meterToMillimeter(meters);
            case CENTIMETER:
                return meterToCentimeter(meters);
            case KILOMETER:
                return meterToKilometer(meters);
            case INCH:
                return meterToInch(meters);
            case FOOT:
                return meterToFoot(meters);
            case YARD:
                return meterToYard(meters);
            case MILE:
                return meterToMile(meters);

            case METER:
            default:
                return meters;

        }
    }

    /**
     * Converts provided meter value to milimeters.
     * @param meter meter value.
     * @return same distance converted to milimeters.
     */
    public static double meterToMillimeter(double meter) {
        return meter / METERS_PER_MILLIMETER;
    }

    /**
     * Converts provided milimeter value to meters.
     * @param millimeter milimeter value.
     * @return same distance conterted to meters.
     */
    public static double millimeterToMeter(double millimeter) {
        return millimeter * METERS_PER_MILLIMETER;
    }

    /**
     * Converts provided meter value to centimeters.
     * @param meter meter value.
     * @return same distance converted to centimeters.
     */
    public static double meterToCentimeter(double meter) {
        return meter / METERS_PER_CENTIMETER;
    }

    /**
     * Converts provided centimeter value to meters.
     * @param centimeter centimeter value.
     * @return same distance converted to meters.
     */
    public static double centimeterToMeter(double centimeter) {
        return centimeter * METERS_PER_CENTIMETER;
    }

    /**
     * Converts provided meter value to kilometers.
     * @param meter meter value.
     * @return same distance converted to kilometer.
     */
    public static double meterToKilometer(double meter) {
        return meter / METERS_PER_KILOMETER;
    }

    /**
     * Converts provided kilometer value to meters.
     * @param kilometer kilometer value.
     * @return same distance converted to meters.
     */
    public static double kilometerToMeter(double kilometer) {
        return kilometer * METERS_PER_KILOMETER;
    }

    /**
     * Converts provided meter value to inches.
     * @param meter meter value.
     * @return same distance converted to inches.
     */
    public static double meterToInch(double meter) {
        return meter / METERS_PER_INCH;
    }

    /**
     * Converts provided inch value to meters.
     * @param inch inch value.
     * @return same distance converted to meters.
     */
    public static double inchToMeter(double inch) {
        return inch * METERS_PER_INCH;
    }

    /**
     * Converts provided meter value to feet.
     * @param meter meter value.
     * @return same distance converted to feet.
     */
    public static double meterToFoot(double meter) {
        return meter / METERS_PER_FOOT;
    }

    /**
     * Converts provided foot value to meters.
     * @param foot foot value.
     * @return same distance converted to meters.
     */
    public static double footToMeter(double foot) {
        return foot * METERS_PER_FOOT;
    }

    /**
     * Converts provided meter value to yards.
     * @param meter meter value.
     * @return same distance converted to yards.
     */
    public static double meterToYard(double meter) {
        return meter / METERS_PER_YARD;
    }

    /**
     * Converts provided yard value to meters.
     * @param yard yard value.
     * @return same distance converted to meters.
     */
    public static double yardToMeter(double yard) {
        return yard * METERS_PER_YARD;
    }

    /**
     * Converts provided meter value to miles.
     * @param meter meter value.
     * @return same distance converted to miles.
     */
    public static double meterToMile(double meter) {
        return meter / METERS_PER_MILE;
    }

    /**
     * Converts provided mile value to meters.
     * @param mile mile value.
     * @return same distance converted to meters.
     */
    public static double mileToMeter(double mile) {
        return mile * METERS_PER_MILE;
    }
}
