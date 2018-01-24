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

import java.util.Random;

import static org.junit.Assert.*;

public class SurfaceConverterTest {

    /**
     * Number of square meters in 1 square milimeter.
     */
    public static final double SQUARE_METERS_PER_SQUARE_MILLIMETER =
            DistanceConverter.METERS_PER_MILLIMETER * DistanceConverter.METERS_PER_MILLIMETER;

    /**
     * Number of square meters in 1 square centimeter.
     */
    public static final double SQUARE_METERS_PER_SQUARE_CENTIMETER =
            DistanceConverter.METERS_PER_CENTIMETER * DistanceConverter.METERS_PER_CENTIMETER;

    /**
     * Number of square meters in 1 square kilometer.
     */
    public static final double SQUARE_METERS_PER_SQUARE_KILOMETER =
            DistanceConverter.METERS_PER_KILOMETER * DistanceConverter.METERS_PER_KILOMETER;

    /**
     * Number of square meters in 1 square inch.
     */
    public static final double SQUARE_METERS_PER_SQUARE_INCH =
            DistanceConverter.METERS_PER_INCH * DistanceConverter.METERS_PER_INCH;

    /**
     * Number of square meters in 1 square foot.
     */
    public static final double SQUARE_METERS_PER_SQUARE_FOOT =
            DistanceConverter.METERS_PER_FOOT * DistanceConverter.METERS_PER_FOOT;

    /**
     * Number of square meters in 1 square yard.
     */
    public static final double SQUARE_METERS_PER_SQUARE_YARD =
            DistanceConverter.METERS_PER_YARD * DistanceConverter.METERS_PER_YARD;

    /**
     * Number of square meters in 1 square mile.
     */
    public static final double SQUARE_METERS_PER_SQUARE_MILE =
            DistanceConverter.METERS_PER_MILE * DistanceConverter.METERS_PER_MILE;

    /**
     * Number of square meters in 1 centiare.
     */
    public static final double SQUARE_METERS_PER_CENTIARE = 1.0;

    /**
     * Number of square meters in 1 are.
     */
    public static final double SQUARE_METERS_PER_ARE = 100.0;

    /**
     * Number of square meters in 1 decare.
     */
    public static final double SQUARE_METERS_PER_DECARE = 1000.0;

    /**
     * Number of square meters in 1 hectare.
     */
    public static final double SQUARE_METERS_PER_HECTARE = 10000.0;

    /**
     * Number of square meters in 1 acre.
     */
    public static final double SQUARE_METERS_PER_ACRE = 4046.8564224;

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
    public void testConvert() {
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
    }
}
