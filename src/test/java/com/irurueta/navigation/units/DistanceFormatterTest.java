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
import java.math.RoundingMode;
import java.text.FieldPosition;
import java.text.NumberFormat;
import java.text.ParseException;
import java.util.Locale;

import static org.junit.Assert.*;

public class DistanceFormatterTest {

    public DistanceFormatterTest() { }

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
        //test empty constructor
        DistanceFormatter formatter = new DistanceFormatter();

        //check
        assertEquals(formatter.getLocale(), Locale.getDefault());
        assertEquals(formatter.getMaximumFractionDigits(),
                NumberFormat.getInstance().getMaximumFractionDigits());
        assertEquals(formatter.getMaximumIntegerDigits(),
                NumberFormat.getInstance().getMaximumIntegerDigits());
        assertEquals(formatter.getMinimumFractionDigits(),
                NumberFormat.getInstance().getMinimumFractionDigits());
        assertEquals(formatter.getMinimumIntegerDigits(),
                NumberFormat.getInstance().getMinimumIntegerDigits());
        assertEquals(formatter.getRoundingMode(),
                NumberFormat.getInstance().getRoundingMode());
        assertEquals(formatter.getUnitSystem(), UnitLocale.getDefault());
        assertEquals(formatter.isGroupingUsed(),
                NumberFormat.getInstance().isGroupingUsed());
        assertEquals(formatter.isParseIntegerOnly(),
                NumberFormat.getInstance().isParseIntegerOnly());
        assertEquals(formatter.getValueAndUnitFormatPattern(),
                MeasureFormatter.DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN);

        //test constructor with locale
        Locale locale = new Locale("es", "ES");
        formatter = new DistanceFormatter(locale);

        //check
        assertEquals(formatter.getLocale(), locale);
        assertEquals(formatter.getMaximumFractionDigits(),
                NumberFormat.getInstance(locale).getMaximumFractionDigits());
        assertEquals(formatter.getMaximumIntegerDigits(),
                NumberFormat.getInstance(locale).getMaximumIntegerDigits());
        assertEquals(formatter.getMinimumFractionDigits(),
                NumberFormat.getInstance(locale).getMinimumFractionDigits());
        assertEquals(formatter.getMinimumIntegerDigits(),
                NumberFormat.getInstance(locale).getMinimumIntegerDigits());
        assertEquals(formatter.getRoundingMode(),
                NumberFormat.getInstance(locale).getRoundingMode());
        assertEquals(formatter.getUnitSystem(), UnitLocale.getFrom(locale));
        assertEquals(formatter.isGroupingUsed(),
                NumberFormat.getInstance(locale).isGroupingUsed());
        assertEquals(formatter.isParseIntegerOnly(),
                NumberFormat.getInstance(locale).isParseIntegerOnly());
        assertEquals(formatter.getValueAndUnitFormatPattern(),
                MeasureFormatter.DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN);


        //force IllegalArgumentException
        formatter = null;
        try {
            formatter = new DistanceFormatter(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(formatter);
    }

    @Test
    public void testClone() {
        DistanceFormatter formatter1 = new DistanceFormatter();
        DistanceFormatter formatter2 = (DistanceFormatter)formatter1.clone();

        //check
        assertNotSame(formatter1, formatter2);
        assertEquals(formatter1, formatter2);

        //test after initializing internal number format
        assertNotNull(formatter1.format(0.5, DistanceUnit.METER,
                new StringBuffer(), new FieldPosition(0)));
        DistanceFormatter formatter3 = (DistanceFormatter)formatter1.clone();

        assertNotSame(formatter1, formatter3);
        assertEquals(formatter1, formatter3);
    }

    @Test
    public void testEquals() {
        DistanceFormatter formatter1 = new DistanceFormatter(Locale.ENGLISH);
        DistanceFormatter formatter2 = new DistanceFormatter(Locale.ENGLISH);
        DistanceFormatter formatter3 = new DistanceFormatter(Locale.FRENCH);

        //check
        assertEquals(formatter1, formatter1);
        assertEquals(formatter1, formatter2);
        assertNotEquals(formatter1, formatter3);

        assertNotEquals(formatter1, new Object());

        //noinspection all
        assertFalse(formatter1.equals(null));
    }

    @Test
    public void testHashCode() {
        DistanceFormatter formatter1 = new DistanceFormatter(Locale.ENGLISH);
        DistanceFormatter formatter2 = new DistanceFormatter(Locale.ENGLISH);
        DistanceFormatter formatter3 = new DistanceFormatter(Locale.FRENCH);

        assertEquals(formatter1.hashCode(), formatter1.hashCode());
        assertEquals(formatter1.hashCode(), formatter2.hashCode());
        assertNotEquals(formatter1.hashCode(), formatter3.hashCode());
    }

    @Test
    public void testFormatNumber() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.MILLIMETER), "5,5 mm");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.CENTIMETER), "5,5 cm");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.METER), "5,5 m");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.KILOMETER), "5,5 Km");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.INCH), "5,5 in");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.FOOT), "5,5 ft");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.YARD), "5,5 yd");
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.MILE), "5,5 mi");
    }

    @Test
    public void testFormatNumberAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.MILLIMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 mm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.CENTIMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 cm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.METER, buffer,
                new FieldPosition(0)).toString(), "5,5 m");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.KILOMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 Km");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.INCH, buffer,
                new FieldPosition(0)).toString(), "5,5 in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.FOOT, buffer,
                new FieldPosition(0)).toString(), "5,5 ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.YARD, buffer,
                new FieldPosition(0)).toString(), "5,5 yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                DistanceUnit.MILE, buffer,
                new FieldPosition(0)).toString(), "5,5 mi");
    }

    @Test
    public void testFormatDouble() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        assertEquals(formatter.format(value, DistanceUnit.MILLIMETER),
                "5,5 mm");
        assertEquals(formatter.format(value, DistanceUnit.CENTIMETER),
                "5,5 cm");
        assertEquals(formatter.format(value, DistanceUnit.METER),
                "5,5 m");
        assertEquals(formatter.format(value, DistanceUnit.KILOMETER),
                "5,5 Km");
        assertEquals(formatter.format(value, DistanceUnit.INCH),
                "5,5 in");
        assertEquals(formatter.format(value, DistanceUnit.FOOT),
                "5,5 ft");
        assertEquals(formatter.format(value, DistanceUnit.YARD),
                "5,5 yd");
        assertEquals(formatter.format(value, DistanceUnit.MILE),
                "5,5 mi");
    }

    @Test
    public void testFormatDoubleAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.MILLIMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 mm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.CENTIMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 cm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.METER, buffer,
                new FieldPosition(0)).toString(), "5,5 m");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.KILOMETER, buffer,
                new FieldPosition(0)).toString(), "5,5 Km");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.INCH, buffer,
                new FieldPosition(0)).toString(), "5,5 in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.FOOT, buffer,
                new FieldPosition(0)).toString(), "5,5 ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.YARD, buffer,
                new FieldPosition(0)).toString(), "5,5 yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, DistanceUnit.MILE, buffer,
                new FieldPosition(0)).toString(), "5,5 mi");
    }

    @Test
    public void testFormatDistance() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.MILLIMETER)), "5,5 mm");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.CENTIMETER)), "5,5 cm");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.METER)), "5,5 m");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.KILOMETER)), "5,5 Km");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.INCH)), "5,5 in");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.FOOT)), "5,5 ft");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.YARD)), "5,5 yd");
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.MILE)), "5,5 mi");
    }

    @Test
    public void testFormatDistanceAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.MILLIMETER), buffer,
                new FieldPosition(0)).toString(), "5,5 mm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.CENTIMETER), buffer,
                new FieldPosition(0)).toString(), "5,5 cm");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.METER), buffer,
                new FieldPosition(0)).toString(), "5,5 m");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.KILOMETER), buffer,
                new FieldPosition(0)).toString(), "5,5 Km");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.INCH), buffer,
                new FieldPosition(0)).toString(), "5,5 in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.FOOT), buffer,
                new FieldPosition(0)).toString(), "5,5 ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.YARD), buffer,
                new FieldPosition(0)).toString(), "5,5 yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Distance(value, DistanceUnit.MILE), buffer,
                new FieldPosition(0)).toString(), "5,5 mi");
    }

    @Test
    public void testFormatAndConvertNumber() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                DistanceUnit.MILLIMETER), "5,5 mm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(50.50),
                DistanceUnit.MILLIMETER), "5,05 cm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000.50),
                DistanceUnit.MILLIMETER), "5 m");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000000.50),
                DistanceUnit.MILLIMETER), "5 Km");

        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.INCH), "2,54 cm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.FOOT), "30,48 cm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.YARD), "91,44 cm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.MILE), "1,61 Km");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                DistanceUnit.INCH), "5.5 in");
        assertEquals(formatter.formatAndConvert(new BigDecimal(18.0),
                DistanceUnit.INCH), "1.5 ft");
        assertEquals(formatter.formatAndConvert(new BigDecimal(16.50),
                DistanceUnit.FOOT), "5.5 yd");
        assertEquals(formatter.formatAndConvert(new BigDecimal(348480.0),
                DistanceUnit.INCH), "5.5 mi");
    }

    @Test
    public void testFormatAndConvertDouble() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                DistanceUnit.MILLIMETER), "5,5 mm");
        assertEquals(formatter.formatAndConvert(50.50,
                DistanceUnit.MILLIMETER), "5,05 cm");
        assertEquals(formatter.formatAndConvert(5000.50,
                DistanceUnit.MILLIMETER), "5 m");
        assertEquals(formatter.formatAndConvert(5000000.50,
                DistanceUnit.MILLIMETER), "5 Km");

        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.INCH), "2,54 cm");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.FOOT), "30,48 cm");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.YARD), "91,44 cm");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.MILE), "1,61 Km");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                DistanceUnit.INCH), "5.5 in");
        assertEquals(formatter.formatAndConvert(18.0,
                DistanceUnit.INCH), "1.5 ft");
        assertEquals(formatter.formatAndConvert(16.50,
                DistanceUnit.FOOT), "5.5 yd");
        assertEquals(formatter.formatAndConvert(348480.0,
                DistanceUnit.INCH), "5.5 mi");
    }

    @Test
    public void testFormatAndConvertDistance() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new Distance(5.50,
                DistanceUnit.MILLIMETER)), "5,5 mm");
        assertEquals(formatter.formatAndConvert(new Distance(50.50,
                DistanceUnit.MILLIMETER)), "5,05 cm");
        assertEquals(formatter.formatAndConvert(new Distance(5000.50,
                DistanceUnit.MILLIMETER)), "5 m");
        assertEquals(formatter.formatAndConvert(new Distance(5000000.50,
                DistanceUnit.MILLIMETER)), "5 Km");

        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.INCH)), "2,54 cm");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.FOOT)), "30,48 cm");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.YARD)), "91,44 cm");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.MILE)), "1,61 Km");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new Distance(5.50,
                DistanceUnit.INCH)), "5.5 in");
        assertEquals(formatter.formatAndConvert(new Distance(18.0,
                DistanceUnit.INCH)), "1.5 ft");
        assertEquals(formatter.formatAndConvert(new Distance(16.50,
                DistanceUnit.FOOT)), "5.5 yd");
        assertEquals(formatter.formatAndConvert(new Distance(348480.0,
                DistanceUnit.INCH)), "5.5 mi");
    }

    @Test
    public void testFormatAnConvertNumberAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5,5 mm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(50.50),
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5,05 cm");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000.50),
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5 m");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000000.50),
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5 Km");

        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.INCH, UnitSystem.IMPERIAL), "1 in");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.FOOT, UnitSystem.IMPERIAL), "1 ft");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.YARD, UnitSystem.IMPERIAL), "1 yd");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                DistanceUnit.MILE, UnitSystem.IMPERIAL), "1 mi");
    }

    @Test
    public void testFormatAndConvertDoubleAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5,5 mm");
        assertEquals(formatter.formatAndConvert(50.50,
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5,05 cm");
        assertEquals(formatter.formatAndConvert(5000.50,
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5 m");
        assertEquals(formatter.formatAndConvert(5000000.50,
                DistanceUnit.MILLIMETER, UnitSystem.METRIC), "5 Km");

        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.INCH, UnitSystem.IMPERIAL), "1 in");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.FOOT, UnitSystem.IMPERIAL), "1 ft");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.YARD, UnitSystem.IMPERIAL), "1 yd");
        assertEquals(formatter.formatAndConvert(1.0,
                DistanceUnit.MILE, UnitSystem.IMPERIAL), "1 mi");
    }

    @Test
    public void testFormatAndConvertDistanceAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new Distance(5.50,
                DistanceUnit.MILLIMETER), UnitSystem.METRIC), "5,5 mm");
        assertEquals(formatter.formatAndConvert(new Distance(50.50,
                DistanceUnit.MILLIMETER), UnitSystem.METRIC), "5,05 cm");
        assertEquals(formatter.formatAndConvert(new Distance(5000.50,
                DistanceUnit.MILLIMETER), UnitSystem.METRIC), "5 m");
        assertEquals(formatter.formatAndConvert(new Distance(5000000.50,
                DistanceUnit.MILLIMETER), UnitSystem.METRIC), "5 Km");

        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.INCH), UnitSystem.IMPERIAL), "1 in");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.FOOT), UnitSystem.IMPERIAL), "1 ft");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.YARD), UnitSystem.IMPERIAL), "1 yd");
        assertEquals(formatter.formatAndConvert(new Distance(1.0,
                DistanceUnit.MILE), UnitSystem.IMPERIAL), "1 mi");
    }

    @Test
    public void testFormatAndConvertMetric() {
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5.50),
                DistanceUnit.MILLIMETER), "5,5 mm");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(50.50),
                DistanceUnit.MILLIMETER), "5,05 cm");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5000.50),
                DistanceUnit.MILLIMETER), "5 m");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5000000.50),
                DistanceUnit.MILLIMETER), "5 Km");

        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                DistanceUnit.INCH), "2,54 cm");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                DistanceUnit.FOOT), "30,48 cm");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                DistanceUnit.YARD), "91,44 cm");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                DistanceUnit.MILE), "1,61 Km");
    }

    @Test
    public void testFormatAndConvertImperial() {
        Locale l = new Locale("es", "ES");

        DistanceFormatter formatter = new DistanceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(5.50),
                DistanceUnit.INCH), "5,5 in");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(18.0),
                DistanceUnit.INCH), "1,5 ft");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(16.50),
                DistanceUnit.FOOT), "5,5 yd");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(348480.0),
                DistanceUnit.INCH), "5,5 mi");
    }

    @Test
    public void testGetAvailableLocales() {
        Locale[] locales = DistanceFormatter.getAvailableLocales();
        assertArrayEquals(locales, NumberFormat.getAvailableLocales());
    }

    @Test
    public void testGetSetMaximumFractionDigits() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getMaximumFractionDigits(),
                NumberFormat.getInstance().getMaximumFractionDigits());

        //set new value
        formatter.setMaximumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMaximumIntegerDigits() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getMaximumIntegerDigits(),
                NumberFormat.getInstance().getMaximumIntegerDigits());

        //set new value
        formatter.setMaximumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetMinimumFractionDigits() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getMinimumFractionDigits(),
                NumberFormat.getInstance().getMinimumFractionDigits());

        //set new value
        formatter.setMinimumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMinimumIntegerDigits() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getMinimumIntegerDigits(),
                NumberFormat.getInstance().getMinimumIntegerDigits());

        //set new value
        formatter.setMinimumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetRoundingMode() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getRoundingMode(),
                NumberFormat.getInstance().getRoundingMode());

        //set new value
        formatter.setRoundingMode(RoundingMode.UNNECESSARY);

        //check correctness
        assertEquals(formatter.getRoundingMode(), RoundingMode.UNNECESSARY);
    }

    @Test
    public void testIsSetGroupingUsed() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.isGroupingUsed(),
                NumberFormat.getInstance().isGroupingUsed());

        //set new value
        formatter.setGroupingUsed(!formatter.isGroupingUsed());

        //check correctness
        assertEquals(formatter.isGroupingUsed(),
                !NumberFormat.getInstance().isGroupingUsed());
    }

    @Test
    public void testIsSetParseIntegerOnly() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.isParseIntegerOnly(),
                NumberFormat.getInstance().isParseIntegerOnly());

        //set new value
        formatter.setParseIntegerOnly(!formatter.isParseIntegerOnly());

        //check correctness
        assertEquals(formatter.isParseIntegerOnly(),
                !NumberFormat.getInstance().isParseIntegerOnly());
    }

    @Test
    public void testGetSetValueAndUnitFormatPattern() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getValueAndUnitFormatPattern(),
                MeasureFormatter.DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN);

        //new value
        formatter.setValueAndUnitFormatPattern("{0}{1}");

        //check correctness
        assertEquals(formatter.getValueAndUnitFormatPattern(), "{0}{1}");

        //force IllegalArgumentException
        try {
            formatter.setValueAndUnitFormatPattern(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetUnitSystem() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.METRIC);

        formatter = new DistanceFormatter(
                new Locale("en", "US"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.IMPERIAL);
    }

    @Test
    public void testIsValidUnit() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertTrue(formatter.isValidUnit("mm"));
        assertTrue(formatter.isValidUnit("mm "));

        assertTrue(formatter.isValidUnit("cm"));
        assertTrue(formatter.isValidUnit("cm "));

        assertTrue(formatter.isValidUnit("m"));
        assertTrue(formatter.isValidUnit("m "));

        assertTrue(formatter.isValidUnit("Km"));
        assertTrue(formatter.isValidUnit("Km "));

        assertTrue(formatter.isValidUnit("in"));
        assertTrue(formatter.isValidUnit("in "));

        assertTrue(formatter.isValidUnit("ft"));
        assertTrue(formatter.isValidUnit("ft "));

        assertTrue(formatter.isValidUnit("yd"));
        assertTrue(formatter.isValidUnit("yd "));

        assertTrue(formatter.isValidUnit("mi"));
        assertTrue(formatter.isValidUnit("mi "));

        assertFalse(formatter.isValidUnit("w"));
    }

    @Test
    public void testIsValidMeasurement() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 cm";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 m";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 Km";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 in";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 ft";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 yd";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 mi";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 s";
        assertFalse(formatter.isValidMeasurement(text));

        text = "m";
        assertFalse(formatter.isValidMeasurement(text));
    }

    @Test
    public void testIsMetricUnit() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 cm";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 m";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 Km";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 in";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 ft";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 yd";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 mi";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 s";
        assertFalse(formatter.isMetricUnit(text));
    }

    @Test
    public void testIsImperialUnit() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 cm";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 m";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 Km";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 in";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 ft";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 yd";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 mi";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 s";
        assertFalse(formatter.isImperialUnit(text));
    }

    @Test
    public void testGetUnitSystemFromSource() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 cm";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 m";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 Km";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 in";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 ft";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 yd";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 mi";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 s";
        assertNull(formatter.getUnitSystem(text));
    }

    @Test
    public void testParse() throws ParseException, UnknownUnitException {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        Distance d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.MILLIMETER);

        text = "5,5 cm";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.CENTIMETER);

        text = "5,5 m";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.METER);

        text = "5,5 Km";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.KILOMETER);

        text = "5,5 in";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.INCH);

        text = "5,5 ft";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.FOOT);

        text = "5,5 yd";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.YARD);

        text = "5,5 mi";
        d = formatter.parse(text);
        assertEquals(d.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(d.getUnit(), DistanceUnit.MILE);

        //Force UnknownUnitException
        text = "5,5 s";
        try {
            formatter.parse(text);
            fail("UnknownUnitException expected but not thrown");
        } catch (UnknownUnitException ignore) { }

        //Force ParseException
        try {
            formatter.parse("m");
            fail("ParseException expected but not thrown");
        } catch (ParseException ignore) { }
    }

    @Test
    public void testFindUnit() {
        DistanceFormatter formatter = new DistanceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm";
        assertEquals(formatter.findUnit(text), DistanceUnit.MILLIMETER);

        text = "5,5 cm";
        assertEquals(formatter.findUnit(text), DistanceUnit.CENTIMETER);

        text = "5,5 m";
        assertEquals(formatter.findUnit(text), DistanceUnit.METER);

        text = "5,5 Km";
        assertEquals(formatter.findUnit(text), DistanceUnit.KILOMETER);

        text = "5,5 in";
        assertEquals(formatter.findUnit(text), DistanceUnit.INCH);

        text = "5,5 ft";
        assertEquals(formatter.findUnit(text), DistanceUnit.FOOT);

        text = "5,5 yd";
        assertEquals(formatter.findUnit(text), DistanceUnit.YARD);

        text = "5,5 mi";
        assertEquals(formatter.findUnit(text), DistanceUnit.MILE);

        text = "5,5 s";
        assertNull(formatter.findUnit(text));
    }

    @Test
    public void testGetUnitSymbol() {
        DistanceFormatter formatter = new DistanceFormatter();

        assertEquals(formatter.getUnitSymbol(DistanceUnit.MILLIMETER),
                DistanceFormatter.MILLIMETER);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.CENTIMETER),
                DistanceFormatter.CENTIMETER);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.METER),
                DistanceFormatter.METER);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.KILOMETER),
                DistanceFormatter.KILOMETER);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.INCH),
                DistanceFormatter.INCH);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.FOOT),
                DistanceFormatter.FOOT);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.YARD),
                DistanceFormatter.YARD);
        assertEquals(formatter.getUnitSymbol(DistanceUnit.MILE),
                DistanceFormatter.MILE);
    }
}
