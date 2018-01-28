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

public class DistanceConverterTest {

    /**
     * Number of meters in 1 milimeter
     */
    private static final double METERS_PER_MILLIMETER = 0.001;

    /**
     * Number of meters in 1 centimeter
     */
    private static final double METERS_PER_CENTIMETER = 0.01;

    /**
     * Number of meters in 1 kilometer
     */
    private static final double METERS_PER_KILOMETER = 1000.0;

    /**
     * Number of meters in 1 inch
     */
    private static final double METERS_PER_INCH = 0.0254;

    /**
     * Number of meters in 1 foot
     */
    private static final double METERS_PER_FOOT = 0.3048;

    /**
     * Number of meters in 1 yard
     */
    private static final double METERS_PER_YARD = 0.9144;

    /**
     * Number of meters in 1 mile
     */
    private static final double METERS_PER_MILE = 1609.344;

    private static final double ERROR = 1e-6;


    public DistanceConverterTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        //noinspection all
        assertNotNull(new DistanceConverter());
    }

    @Test
    public void testMetersMillimeters() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToMillimeter(inputValue),
                inputValue / METERS_PER_MILLIMETER, ERROR);
        assertEquals(DistanceConverter.millimeterToMeter(inputValue),
                inputValue * METERS_PER_MILLIMETER, ERROR);
    }

    @Test
    public void testMetersCentimeters() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToCentimeter(inputValue),
                inputValue / METERS_PER_CENTIMETER, ERROR);
        assertEquals(DistanceConverter.centimeterToMeter(inputValue),
                inputValue * METERS_PER_CENTIMETER, ERROR);
    }

    @Test
    public void testMetersKilometers() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToKilometer(inputValue),
                inputValue / METERS_PER_KILOMETER, ERROR);
        assertEquals(DistanceConverter.kilometerToMeter(inputValue),
                inputValue * METERS_PER_KILOMETER, ERROR);
    }

    @Test
    public void testMetersInches() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToInch(inputValue),
                inputValue / METERS_PER_INCH, ERROR);
        assertEquals(DistanceConverter.inchToMeter(inputValue),
                inputValue * METERS_PER_INCH, ERROR);
    }

    @Test
    public void testMetersFeet() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToFoot(inputValue),
                inputValue / METERS_PER_FOOT, ERROR);
        assertEquals(DistanceConverter.footToMeter(inputValue),
                inputValue * METERS_PER_FOOT, ERROR);
    }

    @Test
    public void testMetersYards() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToYard(inputValue),
                inputValue / METERS_PER_YARD, ERROR);
        assertEquals(DistanceConverter.yardToMeter(inputValue),
                inputValue * METERS_PER_YARD, ERROR);
    }

    @Test
    public void testMetersMiles() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.meterToMile(inputValue),
                inputValue / METERS_PER_MILE, ERROR);
        assertEquals(DistanceConverter.mileToMeter(inputValue),
                inputValue * METERS_PER_MILE, ERROR);
    }

    @Test
    public void testConvertDouble() {
        double inputValue = new Random().nextDouble();

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.MILLIMETER),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.METER),
                DistanceConverter.millimeterToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.millimeterToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.CENTIMETER),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.METER),
                DistanceConverter.centimeterToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.CENTIMETER, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.centimeterToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.METER),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.INCH),
                DistanceConverter.meterToInch(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.YARD),
                DistanceConverter.meterToYard(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.METER, DistanceUnit.MILE),
                DistanceConverter.meterToMile(inputValue), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.METER),
                DistanceConverter.kilometerToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.KILOMETER),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.KILOMETER, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.kilometerToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.METER),
                DistanceConverter.inchToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.INCH),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.INCH, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.inchToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.footToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.footToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.METER),
                DistanceConverter.footToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.footToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.footToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.FOOT),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.footToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.FOOT, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.footToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.METER),
                DistanceConverter.yardToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.YARD),
                inputValue, ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.YARD, DistanceUnit.MILE),
                DistanceConverter.meterToMile(
                        DistanceConverter.yardToMeter(inputValue)), ERROR);

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.MILLIMETER),
                DistanceConverter.meterToMillimeter(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.CENTIMETER),
                DistanceConverter.meterToCentimeter(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.METER),
                DistanceConverter.mileToMeter(inputValue), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.KILOMETER),
                DistanceConverter.meterToKilometer(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.INCH),
                DistanceConverter.meterToInch(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.FOOT),
                DistanceConverter.meterToFoot(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.YARD),
                DistanceConverter.meterToYard(
                        DistanceConverter.mileToMeter(inputValue)), ERROR);
        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILE, DistanceUnit.MILE),
                inputValue, ERROR);
    }

    @Test
    public void testConvertNumber() {
        BigDecimal inputValue = new BigDecimal(new Random().nextDouble());

        assertEquals(DistanceConverter.convert(inputValue,
                DistanceUnit.MILLIMETER, DistanceUnit.MILLIMETER).doubleValue(),
                inputValue.doubleValue(), ERROR);
    }

    @Test
    public void testConvertDistance() {
        double value = new Random().nextDouble();
        Distance inputDistance = new Distance(value, DistanceUnit.METER);

        Distance outputDistance = new Distance();
        DistanceConverter.convert(inputDistance, DistanceUnit.KILOMETER,
                outputDistance);

        //check
        assertEquals(inputDistance.getValue().doubleValue(), value, 0.0);
        assertEquals(inputDistance.getUnit(), DistanceUnit.METER);

        assertEquals(outputDistance.getUnit(), DistanceUnit.KILOMETER);
        assertEquals(outputDistance.getValue().doubleValue(),
                DistanceConverter.convert(value, inputDistance.getUnit(),
                outputDistance.getUnit()), 0.0);
    }

    @Test
    public void testConvertAndUpdateDistance() {
        double value = new Random().nextDouble();
        Distance distance = new Distance(value, DistanceUnit.METER);

        DistanceConverter.convert(distance, DistanceUnit.KILOMETER);

        //check
        assertEquals(distance.getUnit(), DistanceUnit.KILOMETER);
        assertEquals(distance.getValue().doubleValue(),
                DistanceConverter.convert(value,
                DistanceUnit.METER, DistanceUnit.KILOMETER), 0.0);
    }

    @Test
    public void testConvertAndReturnNewDistance() {
        double value = new Random().nextDouble();
        Distance inputDistance = new Distance(value, DistanceUnit.METER);

        Distance outputDistance = DistanceConverter.convertAndReturnNew(
                inputDistance, DistanceUnit.KILOMETER);

        //check
        assertEquals(inputDistance.getValue().doubleValue(), value, 0.0);
        assertEquals(inputDistance.getUnit(), DistanceUnit.METER);

        assertEquals(outputDistance.getUnit(), DistanceUnit.KILOMETER);
        assertEquals(outputDistance.getValue().doubleValue(),
                DistanceConverter.convert(value, inputDistance.getUnit(),
                outputDistance.getUnit()), 0.0);
    }

    @Test
    public void testConvertToOutputDistanceUnit() {
        double value = new Random().nextDouble();
        Distance inputDistance = new Distance(value, DistanceUnit.METER);

        Distance outputDistance = new Distance();
        outputDistance.setUnit(DistanceUnit.KILOMETER);
        DistanceConverter.convert(inputDistance, outputDistance);

        //check
        assertEquals(inputDistance.getValue().doubleValue(), value, 0.0);
        assertEquals(inputDistance.getUnit(), DistanceUnit.METER);

        assertEquals(outputDistance.getUnit(), DistanceUnit.KILOMETER);
        assertEquals(outputDistance.getValue().doubleValue(),
                DistanceConverter.convert(value, inputDistance.getUnit(),
                        outputDistance.getUnit()), 0.0);
    }
}
