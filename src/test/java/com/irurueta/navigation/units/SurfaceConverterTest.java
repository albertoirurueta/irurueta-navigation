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

import org.junit.*;

import java.math.BigDecimal;
import java.util.Random;

import static org.junit.Assert.*;

public class SurfaceConverterTest {

    /**
     * Number of square meters in 1 square milimeter.
     */
    private static final double SQUARE_METERS_PER_SQUARE_MILLIMETER =
            DistanceConverter.METERS_PER_MILLIMETER * DistanceConverter.METERS_PER_MILLIMETER;

    /**
     * Number of square meters in 1 square centimeter.
     */
    private static final double SQUARE_METERS_PER_SQUARE_CENTIMETER =
            DistanceConverter.METERS_PER_CENTIMETER * DistanceConverter.METERS_PER_CENTIMETER;

    /**
     * Number of square meters in 1 square kilometer.
     */
    private static final double SQUARE_METERS_PER_SQUARE_KILOMETER =
            DistanceConverter.METERS_PER_KILOMETER * DistanceConverter.METERS_PER_KILOMETER;

    /**
     * Number of square meters in 1 square inch.
     */
    private static final double SQUARE_METERS_PER_SQUARE_INCH =
            DistanceConverter.METERS_PER_INCH * DistanceConverter.METERS_PER_INCH;

    /**
     * Number of square meters in 1 square foot.
     */
    private static final double SQUARE_METERS_PER_SQUARE_FOOT =
            DistanceConverter.METERS_PER_FOOT * DistanceConverter.METERS_PER_FOOT;

    /**
     * Number of square meters in 1 square yard.
     */
    private static final double SQUARE_METERS_PER_SQUARE_YARD =
            DistanceConverter.METERS_PER_YARD * DistanceConverter.METERS_PER_YARD;

    /**
     * Number of square meters in 1 square mile.
     */
    private static final double SQUARE_METERS_PER_SQUARE_MILE =
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

    private static final double ERROR = 1e-6;

    public SurfaceConverterTest() { }

    @BeforeClass
    public static void setUpClass() {}

    @AfterClass
    public static void tearDownClass() {}

    @Before
    public void setUp() {}

    @After
    public void tearDown() {}

    @Test
    public void testConstructor() {
        //noinspection all
        assertNotNull(new SurfaceConverter());
    }

    @Test
    public void testSquareMetersSquareMillimeters() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareMillimeter(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_MILLIMETER, ERROR);
        assertEquals(SurfaceConverter.squareMillimeterToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_MILLIMETER, ERROR);
    }

    @Test
    public void testSquareMetersSquareCentimeters() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareCentimeter(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_CENTIMETER, ERROR);
        assertEquals(SurfaceConverter.squareCentimeterToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_CENTIMETER, ERROR);
    }

    @Test
    public void testSquareMetersSquareKilometers() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareKilometer(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_KILOMETER, ERROR);
        assertEquals(SurfaceConverter.squareKilometerToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_KILOMETER, ERROR);
    }

    @Test
    public void testSquareMetersSquareInches() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareInch(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_INCH, ERROR);
        assertEquals(SurfaceConverter.squareInchToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_INCH, ERROR);
    }

    @Test
    public void testSquareMetersSquareFeet() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareFoot(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_FOOT, ERROR);
        assertEquals(SurfaceConverter.squareFootToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_FOOT, ERROR);
    }

    @Test
    public void testSquareMetersSquareYards() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareYard(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_YARD, ERROR);
        assertEquals(SurfaceConverter.squareYardToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_YARD, ERROR);
    }

    @Test
    public void testSquareMetersSquareMiles() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToSquareMile(inputValue),
                inputValue / SQUARE_METERS_PER_SQUARE_MILE, ERROR);
        assertEquals(SurfaceConverter.squareMileToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_SQUARE_MILE, ERROR);
    }

    @Test
    public void testSquareMetersCentiares() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToCentiare(inputValue),
                inputValue / SQUARE_METERS_PER_CENTIARE, ERROR);
        assertEquals(SurfaceConverter.centiareToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_CENTIARE, ERROR);
    }

    @Test
    public void testSquareMetersAres() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToAre(inputValue),
                inputValue / SQUARE_METERS_PER_ARE, ERROR);
        assertEquals(SurfaceConverter.areToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_ARE, ERROR);
    }

    @Test
    public void testSquareMetersDecares() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToDecare(inputValue),
                inputValue / SQUARE_METERS_PER_DECARE, ERROR);
        assertEquals(SurfaceConverter.decareToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_DECARE, ERROR);
    }

    @Test
    public void testSquareMetersHectares() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToHectare(inputValue),
                inputValue / SQUARE_METERS_PER_HECTARE, ERROR);
        assertEquals(SurfaceConverter.hectareToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_HECTARE, ERROR);
    }

    @Test
    public void testSquareMetersAcres() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.squareMeterToAcre(inputValue),
                inputValue / SQUARE_METERS_PER_ACRE, ERROR);
        assertEquals(SurfaceConverter.acreToSquareMeter(inputValue),
                inputValue * SQUARE_METERS_PER_ACRE, ERROR);
    }

    @Test
    public void testConvertDouble() {
        double inputValue = new Random().nextDouble();

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_MILLIMETER),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareMillimeterToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_CENTIMETER),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_CENTIMETER, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareCentimeterToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_METER),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(inputValue),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareKilometerToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_KILOMETER),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_KILOMETER, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareKilometerToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareInchToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_INCH),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_INCH, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareInchToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareFootToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_FOOT),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_FOOT, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareFootToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareYardToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_YARD),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_YARD, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareYardToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareMileToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_MILE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.squareMileToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.SQUARE_MILE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.squareMileToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.squareMeterToCentiare(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.centiareToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.squareMeterToCentiare(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.CENTIARE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.CENTIARE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.centiareToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.areToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.ARE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ARE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.areToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.decareToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.DECARE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.DECARE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.decareToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.hectareToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.HECTARE),
                inputValue, ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.HECTARE, SurfaceUnit.ACRE),
                SurfaceConverter.squareMeterToAcre(
                SurfaceConverter.hectareToSquareMeter(inputValue)),
                ERROR);

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceConverter.squareMeterToSquareMillimeter(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceConverter.squareMeterToSquareCentimeter(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_METER),
                SurfaceConverter.acreToSquareMeter(inputValue),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_KILOMETER),
                SurfaceConverter.squareMeterToSquareKilometer(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_INCH),
                SurfaceConverter.squareMeterToSquareInch(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_FOOT),
                SurfaceConverter.squareMeterToSquareFoot(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_YARD),
                SurfaceConverter.squareMeterToSquareYard(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.SQUARE_MILE),
                SurfaceConverter.squareMeterToSquareMile(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.CENTIARE),
                SurfaceConverter.squareMeterToCentiare(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.ARE),
                SurfaceConverter.squareMeterToAre(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.DECARE),
                SurfaceConverter.squareMeterToDecare(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.HECTARE),
                SurfaceConverter.squareMeterToHectare(
                SurfaceConverter.acreToSquareMeter(inputValue)),
                ERROR);
        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.ACRE, SurfaceUnit.ACRE),
                inputValue, ERROR);
    }

    @Test
    public void testConvertNumber() {
        BigDecimal inputValue = new BigDecimal(new Random().nextDouble());

        assertEquals(SurfaceConverter.convert(inputValue,
                SurfaceUnit.SQUARE_MILLIMETER,
                SurfaceUnit.SQUARE_MILLIMETER).doubleValue(),
                inputValue.doubleValue(), ERROR);

    }

    @Test
    public void testConvertSurface() {
        double value = new Random().nextDouble();
        Surface inputSurface = new Surface(value, SurfaceUnit.SQUARE_METER);

        Surface outputSurface = new Surface();
        SurfaceConverter.convert(inputSurface, SurfaceUnit.SQUARE_KILOMETER,
                outputSurface);

        //check
        assertEquals(inputSurface.getValue().doubleValue(), value, 0.0);
        assertEquals(inputSurface.getUnit(), SurfaceUnit.SQUARE_METER);

        assertEquals(outputSurface.getUnit(), SurfaceUnit.SQUARE_KILOMETER);
        assertEquals(outputSurface.getValue().doubleValue(),
                SurfaceConverter.convert(value, inputSurface.getUnit(),
                outputSurface.getUnit()), 0.0);
    }

    @Test
    public void testConvertAndUpdateSurface() {
        double value = new Random().nextDouble();
        Surface surface = new Surface(value, SurfaceUnit.SQUARE_METER);

        SurfaceConverter.convert(surface, SurfaceUnit.SQUARE_KILOMETER);

        //check
        assertEquals(surface.getUnit(), SurfaceUnit.SQUARE_KILOMETER);
        assertEquals(surface.getValue().doubleValue(),
                SurfaceConverter.convert(value,
                SurfaceUnit.SQUARE_METER, SurfaceUnit.SQUARE_KILOMETER),
                0.0);
    }

    @Test
    public void testConvertAndReturnNewSurface() {
        double value = new Random().nextDouble();
        Surface inputSurface = new Surface(value, SurfaceUnit.SQUARE_METER);

        Surface outputSurface = SurfaceConverter.convertAndReturnNew(
                inputSurface, SurfaceUnit.SQUARE_KILOMETER);

        //check
        assertEquals(inputSurface.getValue().doubleValue(), value, 0.0);
        assertEquals(inputSurface.getUnit(), SurfaceUnit.SQUARE_METER);

        assertEquals(outputSurface.getUnit(), SurfaceUnit.SQUARE_KILOMETER);
        assertEquals(outputSurface.getValue().doubleValue(),
                SurfaceConverter.convert(value, inputSurface.getUnit(),
                        outputSurface.getUnit()), 0.0);
    }

    @Test
    public void testConvertToOutputSurfaceUnit() {
        double value = new Random().nextDouble();
        Surface inputSurface = new Surface(value, SurfaceUnit.SQUARE_METER);

        Surface outputSurface = new Surface();
        outputSurface.setUnit(SurfaceUnit.SQUARE_KILOMETER);
        SurfaceConverter.convert(inputSurface, outputSurface);

        //check
        assertEquals(inputSurface.getValue().doubleValue(), value, 0.0);
        assertEquals(inputSurface.getUnit(), SurfaceUnit.SQUARE_METER);

        assertEquals(outputSurface.getUnit(), SurfaceUnit.SQUARE_KILOMETER);
        assertEquals(outputSurface.getValue().doubleValue(),
                SurfaceConverter.convert(value, inputSurface.getUnit(),
                        outputSurface.getUnit()), 0.0);
    }
}
