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
 * Does surface conversions to different units.
 * To prevent loss of accuracy, conversion should only be done as a final step
 * before displaying surface measurements.
 */
public class SurfaceConverter {

    /**
     * Number of square meters in 1 square milimeter.
     */
    static final double SQUARE_METERS_PER_SQUARE_MILLIMETER =
            DistanceConverter.METERS_PER_MILLIMETER * DistanceConverter.METERS_PER_MILLIMETER;

    /**
     * Number of square meters in 1 square centimeter.
     */
    static final double SQUARE_METERS_PER_SQUARE_CENTIMETER =
            DistanceConverter.METERS_PER_CENTIMETER * DistanceConverter.METERS_PER_CENTIMETER;

    /**
     * Number of square meters in 1 square kilometer.
     */
    static final double SQUARE_METERS_PER_SQUARE_KILOMETER =
            DistanceConverter.METERS_PER_KILOMETER * DistanceConverter.METERS_PER_KILOMETER;

    /**
     * Number of square meters in 1 square inch.
     */
    static final double SQUARE_METERS_PER_SQUARE_INCH =
            DistanceConverter.METERS_PER_INCH * DistanceConverter.METERS_PER_INCH;

    /**
     * Number of square meters in 1 square foot.
     */
    static final double SQUARE_METERS_PER_SQUARE_FOOT =
            DistanceConverter.METERS_PER_FOOT * DistanceConverter.METERS_PER_FOOT;

    /**
     * Number of square meters in 1 square yard.
     */
    static final double SQUARE_METERS_PER_SQUARE_YARD =
            DistanceConverter.METERS_PER_YARD * DistanceConverter.METERS_PER_YARD;

    /**
     * Number of square meters in 1 square mile.
     */
    static final double SQUARE_METERS_PER_SQUARE_MILE =
            DistanceConverter.METERS_PER_MILE * DistanceConverter.METERS_PER_MILE;

    /**
     * Number of square meters in 1 centiare.
     */
    private static final double SQUARE_METERS_PER_CENTIARE = 1.0;

    /**
     * Number of square meters in 1 are.
     */
    private static final double SQUARE_METERS_PER_ARE = 100.0;

    /**
     * Number of square meters in 1 decare.
     */
    private static final double SQUARE_METERS_PER_DECARE = 1000.0;

    /**
     * Number of square meters in 1 hectare.
     */
    private static final double SQUARE_METERS_PER_HECTARE = 10000.0;

    /**
     * Number of square meters in 1 acre.
     */
    private static final double SQUARE_METERS_PER_ACRE = 4046.8564224;

    /**
     * Constructor.
     * Prevents instantiaton of helper class.
     */
    SurfaceConverter() { }

    /**
     * Converts a surface value from input unit to provided output unit.
     * @param input surface value.
     * @param inputUnit input surface unit.
     * @param outputUnit output surface unit.
     * @return converted surface value.
     */
    public static double convert(double input, SurfaceUnit inputUnit,
                                 SurfaceUnit outputUnit) {
        double squareMeters;

        //convert to square meters.
        switch (inputUnit) {
            case SQUARE_MILLIMETER:
                squareMeters = squareMillimeterToSquareMeter(input);
                break;
            case SQUARE_CENTIMETER:
                squareMeters = squareCentimeterToSquareMeter(input);
                break;
            case SQUARE_KILOMETER:
                squareMeters = squareKilometerToSquareMeter(input);
                break;
            case SQUARE_INCH:
                squareMeters = squareInchToSquareMeter(input);
                break;
            case SQUARE_FOOT:
                squareMeters = squareFootToSquareMeter(input);
                break;
            case SQUARE_YARD:
                squareMeters = squareYardToSquareMeter(input);
                break;
            case SQUARE_MILE:
                squareMeters = squareMileToSquareMeter(input);
                break;
            case CENTIARE:
                squareMeters = centiareToSquareMeter(input);
                break;
            case ARE:
                squareMeters = areToSquareMeter(input);
                break;
            case DECARE:
                squareMeters = decareToSquareMeter(input);
                break;
            case HECTARE:
                squareMeters = hectareToSquareMeter(input);
                break;
            case ACRE:
                squareMeters = acreToSquareMeter(input);
                break;

            case SQUARE_METER:
            default:
                squareMeters = input;
                break;
        }

        //convert from square meter to required output unit
        switch (outputUnit) {
            case SQUARE_MILLIMETER:
                return squareMeterToSquareMillimeter(squareMeters);
            case SQUARE_CENTIMETER:
                return squareMeterToSquareCentimeter(squareMeters);
            case SQUARE_KILOMETER:
                return squareMeterToSquareKilometer(squareMeters);
            case SQUARE_INCH:
                return squareMeterToSquareInch(squareMeters);
            case SQUARE_FOOT:
                return squareMeterToSquareFoot(squareMeters);
            case SQUARE_YARD:
                return squareMeterToSquareYard(squareMeters);
            case SQUARE_MILE:
                return squareMeterToSquareMile(squareMeters);
            case CENTIARE:
                return squareMeterToCentiare(squareMeters);
            case ARE:
                return squareMeterToAre(squareMeters);
            case DECARE:
                return squareMeterToDecare(squareMeters);
            case HECTARE:
                return squareMeterToHectare(squareMeters);
            case ACRE:
                return squareMeterToAcre(squareMeters);

            case SQUARE_METER:
            default:
                return squareMeters;
        }
    }

    /**
     * Converts provided square meter value to square millimeters.
     * @param squareMeter square meter value.
     * @return same surface converted to square millimeters.
     */
    public static double squareMeterToSquareMillimeter(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_MILLIMETER;
    }

    /**
     * Converts provided square millimeter value to square meters.
     * @param squareMillimeter square millimeter value.
     * @return same surface converted to square meters.
     */
    public static double squareMillimeterToSquareMeter(double squareMillimeter) {
        return squareMillimeter * SQUARE_METERS_PER_SQUARE_MILLIMETER;
    }

    /**
     * Converts provided square meter value to square centimeters.
     * @param squareMeter square meter value.
     * @return same surface converted to square centimeters.
     */
    public static double squareMeterToSquareCentimeter(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_CENTIMETER;
    }

    /**
     * Converts provided square centimeter value to square meters.
     * @param squareCentimeter square centimeter value.
     * @return same surface converted to square meters.
     */
    public static double squareCentimeterToSquareMeter(double squareCentimeter) {
        return squareCentimeter * SQUARE_METERS_PER_SQUARE_CENTIMETER;
    }

    /**
     * Converts provided square meter value to square kilometers.
     * @param squareMeter square meter value.
     * @return same surface converted to square kilometers.
     */
    public static double squareMeterToSquareKilometer(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_KILOMETER;
    }

    /**
     * Converts provided square kilometer value to square meters.
     * @param squareKilometer square kilometer value.
     * @return same surface converted to square meters.
     */
    public static double squareKilometerToSquareMeter(double squareKilometer) {
        return squareKilometer * SQUARE_METERS_PER_SQUARE_KILOMETER;
    }

    /**
     * Converts provided square meter value to square inches.
     * @param squareMeter square meter value.
     * @return same surface converted to square inches.
     */
    public static double squareMeterToSquareInch(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_INCH;
    }

    /**
     * Converts provided square inch value to square meters.
     * @param squareInch square inch value.
     * @return same surface converted to square meters.
     */
    public static double squareInchToSquareMeter(double squareInch) {
        return squareInch * SQUARE_METERS_PER_SQUARE_INCH;
    }

    /**
     * Converts provided square meter value to square feet.
     * @param squareMeter square meter value.
     * @return same surface converted to square feet.
     */
    public static double squareMeterToSquareFoot(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_FOOT;
    }

    /**
     * Converts provided square foot value to square meters.
     * @param squareFoot square foot value.
     * @return same surface converted to square meters.
     */
    public static double squareFootToSquareMeter(double squareFoot) {
        return squareFoot * SQUARE_METERS_PER_SQUARE_FOOT;
    }

    /**
     * Converts provided square meter value to square yards.
     * @param squareMeter square meter value.
     * @return same surface converted to square yards.
     */
    public static double squareMeterToSquareYard(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_YARD;
    }

    /**
     * Converts provided square yard value to square meters.
     * @param squareYard square yard value.
     * @return same surface converted to square meters.
     */
    public static double squareYardToSquareMeter(double squareYard) {
        return squareYard * SQUARE_METERS_PER_SQUARE_YARD;
    }

    /**
     * Converts provided square meter value to square miles.
     * @param squareMeter square meter value.
     * @return same surface converted to square miles.
     */
    public static double squareMeterToSquareMile(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_SQUARE_MILE;
    }

    /**
     * Converts provided square mile value to square meters.
     * @param squareMile square mile value.
     * @return same surface converted to square meters.
     */
    public static double squareMileToSquareMeter(double squareMile) {
        return squareMile * SQUARE_METERS_PER_SQUARE_MILE;
    }

    /**
     * Converts provided square meter value to centiares.
     * @param squareMeter square meter value.
     * @return same surface converted to centiares.
     */
    public static double squareMeterToCentiare(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_CENTIARE;
    }

    /**
     * Converts provided centiare value to square meters.
     * @param centiare centiare value.
     * @return same surface converted to square meters.
     */
    public static double centiareToSquareMeter(double centiare) {
        return centiare * SQUARE_METERS_PER_CENTIARE;
    }

    /**
     * Converts provided square meter value to ares.
     * @param squareMeter square meter value.
     * @return same surface converted to ares.
     */
    public static double squareMeterToAre(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_ARE;
    }

    /**
     * Converts provided are value to square meters.
     * @param are are value.
     * @return same surface converted to square meters.
     */
    public static double areToSquareMeter(double are) {
        return are * SQUARE_METERS_PER_ARE;
    }

    /**
     * Converts provided square meter value to decares.
     * @param squareMeter square meter value.
     * @return same surface converted to decares.
     */
    public static double squareMeterToDecare(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_DECARE;
    }

    /**
     * Converts provided decare value to square meters.
     * @param decare decare value.
     * @return same surface converted to square meters.
     */
    public static double decareToSquareMeter(double decare) {
        return decare * SQUARE_METERS_PER_DECARE;
    }

    /**
     * Converts provided square meter value to hectares.
     * @param squareMeter square meter value.
     * @return same surface converted to hectares.
     */
    public static double squareMeterToHectare(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_HECTARE;
    }

    /**
     * Converts provided hectare value to square meters.
     * @param hectare hectare value.
     * @return same surface converted to square meters.
     */
    public static double hectareToSquareMeter(double hectare) {
        return hectare * SQUARE_METERS_PER_HECTARE;
    }

    /**
     * Converts provided square meter value to acres.
     * @param squareMeter square meter value.
     * @return same surface converted ot acres.
     */
    public static double squareMeterToAcre(double squareMeter) {
        return squareMeter / SQUARE_METERS_PER_ACRE;
    }

    /**
     * Converts provided acre value to square meters.
     * @param acre acre value.
     * @return same surface converted to square meters.
     */
    public static double acreToSquareMeter(double acre) {
        return acre * SQUARE_METERS_PER_ACRE;
    }
}
