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

public class SurfaceFormatterTest {

    public SurfaceFormatterTest() { }

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
        SurfaceFormatter formatter = new SurfaceFormatter();

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
        formatter = new SurfaceFormatter(locale);

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
            formatter = new SurfaceFormatter(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(formatter);
    }

    @Test
    public void testClone() {
        SurfaceFormatter formatter1 = new SurfaceFormatter();
        SurfaceFormatter formatter2 = (SurfaceFormatter)formatter1.clone();

        //check
        assertNotSame(formatter1, formatter2);
        assertEquals(formatter1, formatter2);

        //test after initializing internal number format
        assertNotNull(formatter1.format(0.5, SurfaceUnit.SQUARE_METER,
                new StringBuffer(), new FieldPosition(0)));
        SurfaceFormatter formatter3 = (SurfaceFormatter)formatter1.clone();

        assertNotSame(formatter1, formatter3);
        assertEquals(formatter1, formatter3);
    }

    @Test
    public void testEquals() {
        SurfaceFormatter formatter1 = new SurfaceFormatter(Locale.ENGLISH);
        SurfaceFormatter formatter2 = new SurfaceFormatter(Locale.ENGLISH);
        SurfaceFormatter formatter3 = new SurfaceFormatter(Locale.FRENCH);

        //chekc
        assertEquals(formatter1, formatter1);
        assertEquals(formatter1, formatter2);
        assertNotEquals(formatter1, formatter3);

        assertNotEquals(formatter1, new Object());

        //noinspection all
        assertFalse(formatter1.equals(null));
    }

    @Test
    public void testHashCode() {
        SurfaceFormatter formatter1 = new SurfaceFormatter(Locale.ENGLISH);
        SurfaceFormatter formatter2 = new SurfaceFormatter(Locale.ENGLISH);
        SurfaceFormatter formatter3 = new SurfaceFormatter(Locale.FRENCH);

        assertEquals(formatter1.hashCode(), formatter1.hashCode());
        assertEquals(formatter1.hashCode(), formatter2.hashCode());
        assertNotEquals(formatter1.hashCode(), formatter3.hashCode());
    }

    @Test
    public void testFormatNumber() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_MILLIMETER), "5,5 mm²");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_CENTIMETER), "5,5 cm²");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_METER), "5,5 m²");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_KILOMETER), "5,5 Km²");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_INCH), "5,5 sq in");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_FOOT), "5,5 sq ft");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_YARD), "5,5 sq yd");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_MILE), "5,5 sq mi");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.CENTIARE), "5,5 ca");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.ARE), "5,5 a");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.DECARE), "5,5 daa");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.HECTARE), "5,5 ha");
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.ACRE), "5,5 acre");
    }

    @Test
    public void testFormatNumberAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_MILLIMETER,
                buffer, new FieldPosition(0)).toString(), "5,5 mm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_CENTIMETER,
                buffer, new FieldPosition(0)).toString(), "5,5 cm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_METER,
                buffer, new FieldPosition(0)).toString(), "5,5 m²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_KILOMETER,
                buffer, new FieldPosition(0)).toString(), "5,5 Km²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_INCH,
                buffer, new FieldPosition(0)).toString(), "5,5 sq in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_FOOT,
                buffer, new FieldPosition(0)).toString(), "5,5 sq ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_YARD,
                buffer, new FieldPosition(0)).toString(), "5,5 sq yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.SQUARE_MILE,
                buffer, new FieldPosition(0)).toString(), "5,5 sq mi");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.CENTIARE,
                buffer, new FieldPosition(0)).toString(), "5,5 ca");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.ARE,
                buffer, new FieldPosition(0)).toString(), "5,5 a");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.DECARE,
                buffer, new FieldPosition(0)).toString(), "5,5 daa");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.HECTARE,
                buffer, new FieldPosition(0)).toString(), "5,5 ha");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                SurfaceUnit.ACRE,
                buffer, new FieldPosition(0)).toString(), "5,5 acre");
    }

    @Test
    public void testFormatDouble() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_MILLIMETER),
                "5,5 mm²");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_CENTIMETER),
                "5,5 cm²");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_METER),
                "5,5 m²");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_KILOMETER),
                "5,5 Km²");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_INCH),
                "5,5 sq in");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_FOOT),
                "5,5 sq ft");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_YARD),
                "5,5 sq yd");
        assertEquals(formatter.format(value, SurfaceUnit.SQUARE_MILE),
                "5,5 sq mi");
        assertEquals(formatter.format(value, SurfaceUnit.CENTIARE),
                "5,5 ca");
        assertEquals(formatter.format(value, SurfaceUnit.ARE),
                "5,5 a");
        assertEquals(formatter.format(value, SurfaceUnit.DECARE),
                "5,5 daa");
        assertEquals(formatter.format(value, SurfaceUnit.HECTARE),
                "5,5 ha");
        assertEquals(formatter.format(value, SurfaceUnit.ACRE),
                "5,5 acre");
    }

    @Test
    public void testFormatDoubleAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_MILLIMETER,
                buffer, new FieldPosition(0)).toString(),
                "5,5 mm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_CENTIMETER,
                buffer, new FieldPosition(0)).toString(),
                "5,5 cm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_METER,
                buffer, new FieldPosition(0)).toString(),
                "5,5 m²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_KILOMETER,
                buffer, new FieldPosition(0)).toString(),
                "5,5 Km²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_INCH,
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_FOOT,
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_YARD,
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.SQUARE_MILE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq mi");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.CENTIARE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 ca");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.ARE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 a");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.DECARE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 daa");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.HECTARE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 ha");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value,
                SurfaceUnit.ACRE,
                buffer, new FieldPosition(0)).toString(),
                "5,5 acre");
    }

    @Test
    public void testFormatSurface() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_MILLIMETER)),
                "5,5 mm²");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_CENTIMETER)),
                "5,5 cm²");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_METER)),
                "5,5 m²");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_KILOMETER)),
                "5,5 Km²");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_INCH)),
                "5,5 sq in");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_FOOT)),
                "5,5 sq ft");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_YARD)),
                "5,5 sq yd");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_MILE)),
                "5,5 sq mi");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.CENTIARE)),
                "5,5 ca");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.ARE)),
                "5,5 a");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.DECARE)),
                "5,5 daa");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.HECTARE)),
                "5,5 ha");
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.ACRE)),
                "5,5 acre");
    }

    @Test
    public void testFormatSurfaceAndStringBuffer() {
        double value = 5.50;
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_MILLIMETER),
                buffer, new FieldPosition(0)).toString(),
                "5,5 mm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_CENTIMETER),
                buffer, new FieldPosition(0)).toString(),
                "5,5 cm²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_METER),
                buffer, new FieldPosition(0)).toString(),
                "5,5 m²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_KILOMETER),
                buffer, new FieldPosition(0)).toString(),
                "5,5 Km²");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_INCH),
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq in");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_FOOT),
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq ft");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_YARD),
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq yd");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.SQUARE_MILE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 sq mi");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.CENTIARE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 ca");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.ARE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 a");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.DECARE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 daa");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.HECTARE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 ha");

        buffer = new StringBuffer();
        assertEquals(formatter.format(
                new Surface(value, SurfaceUnit.ACRE),
                buffer, new FieldPosition(0)).toString(),
                "5,5 acre");
    }

    @Test
    public void testFormatAndConvertNumber() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                SurfaceUnit.SQUARE_MILLIMETER), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(505.00),
                SurfaceUnit.SQUARE_MILLIMETER), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000005.00),
                SurfaceUnit.SQUARE_MILLIMETER), "5 m²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5000000000000.50),
                SurfaceUnit.SQUARE_MILLIMETER), "5 Km²");

        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_INCH), "6,45 cm²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_FOOT), "929,03 cm²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_YARD), "8.361,27 cm²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_MILE), "2,59 Km²");

        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.CENTIARE), "1 m²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.ARE), "100 m²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.DECARE), "1.000 m²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.HECTARE), "10.000 m²");
        assertEquals(formatter.formatAndConvert(new BigDecimal(1.0),
                SurfaceUnit.ACRE), "4.046,86 m²");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                SurfaceUnit.SQUARE_INCH), "5.5 sq in");
        assertEquals(formatter.formatAndConvert(new BigDecimal(180.0),
                SurfaceUnit.SQUARE_INCH), "1.25 sq ft");
        assertEquals(formatter.formatAndConvert(new BigDecimal(16.50),
                SurfaceUnit.SQUARE_FOOT), "1.83 sq yd");
        assertEquals(formatter.formatAndConvert(new BigDecimal(3520.0),
                SurfaceUnit.ACRE), "5.5 sq mi");
    }

    @Test
    public void testFormatAndConvertDouble() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                SurfaceUnit.SQUARE_MILLIMETER), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(505.00,
                SurfaceUnit.SQUARE_MILLIMETER), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(5000005.00,
                SurfaceUnit.SQUARE_MILLIMETER), "5 m²");
        assertEquals(formatter.formatAndConvert(5000000000000.50,
                SurfaceUnit.SQUARE_MILLIMETER), "5 Km²");

        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.SQUARE_INCH), "6,45 cm²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.SQUARE_FOOT), "929,03 cm²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.SQUARE_YARD), "8.361,27 cm²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.SQUARE_MILE), "2,59 Km²");

        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.CENTIARE), "1 m²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.ARE), "100 m²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.DECARE), "1.000 m²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.HECTARE), "10.000 m²");
        assertEquals(formatter.formatAndConvert(1.0,
                SurfaceUnit.ACRE), "4.046,86 m²");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                SurfaceUnit.SQUARE_INCH), "5.5 sq in");
        assertEquals(formatter.formatAndConvert(180.0,
                SurfaceUnit.SQUARE_INCH), "1.25 sq ft");
        assertEquals(formatter.formatAndConvert(16.50,
                SurfaceUnit.SQUARE_FOOT), "1.83 sq yd");
        assertEquals(formatter.formatAndConvert(3520.0,
                SurfaceUnit.ACRE), "5.5 sq mi");
    }

    @Test
    public void testFormatAndConvertSurface() {
        //test for metric system
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(
                new Surface(5.50,
                SurfaceUnit.SQUARE_MILLIMETER)), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(505.00,
                SurfaceUnit.SQUARE_MILLIMETER)), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(5000005.00,
                SurfaceUnit.SQUARE_MILLIMETER)), "5 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(5000000000000.50,
                SurfaceUnit.SQUARE_MILLIMETER)), "5 Km²");

        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.SQUARE_INCH)), "6,45 cm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.SQUARE_FOOT)), "929,03 cm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.SQUARE_YARD)), "8.361,27 cm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.SQUARE_MILE)), "2,59 Km²");

        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.CENTIARE)), "1 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.ARE)), "100 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.DECARE)), "1.000 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.HECTARE)), "10.000 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0,
                SurfaceUnit.ACRE)), "4.046,86 m²");

        //test for imperial system
        l = new Locale("en", "US");

        formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(
                new Surface(5.50,
                SurfaceUnit.SQUARE_INCH)), "5.5 sq in");
        assertEquals(formatter.formatAndConvert(
                new Surface(180.0,
                SurfaceUnit.SQUARE_INCH)), "1.25 sq ft");
        assertEquals(formatter.formatAndConvert(
                new Surface(16.50,
                SurfaceUnit.SQUARE_FOOT)), "1.83 sq yd");
        assertEquals(formatter.formatAndConvert(
                new Surface(3520.0,
                SurfaceUnit.ACRE)), "5.5 sq mi");
    }

    @Test
    public void testFormatAndConvertNumberAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(
                new BigDecimal(5.50), SurfaceUnit.SQUARE_MILLIMETER,
                UnitSystem.METRIC), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(505.00), SurfaceUnit.SQUARE_MILLIMETER,
                UnitSystem.METRIC), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(5000005.00),
                SurfaceUnit.SQUARE_MILLIMETER, UnitSystem.METRIC), "5 m²");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(5000000000000.50),
                SurfaceUnit.SQUARE_MILLIMETER, UnitSystem.METRIC), "5 Km²");

        assertEquals(formatter.formatAndConvert(
                new BigDecimal(1.0),
                SurfaceUnit.SQUARE_INCH, UnitSystem.IMPERIAL), "1 sq in");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(1.0),
                SurfaceUnit.SQUARE_FOOT, UnitSystem.IMPERIAL), "1 sq ft");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(1.0),
                SurfaceUnit.SQUARE_YARD, UnitSystem.IMPERIAL), "1 sq yd");
        assertEquals(formatter.formatAndConvert(
                new BigDecimal(1.0),
                SurfaceUnit.SQUARE_MILE, UnitSystem.IMPERIAL), "1 sq mi");
    }

    @Test
    public void testFormatAndConvertDoubleAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(
                5.50, SurfaceUnit.SQUARE_MILLIMETER,
                UnitSystem.METRIC), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(
                505.00, SurfaceUnit.SQUARE_MILLIMETER,
                UnitSystem.METRIC), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(
                5000005.00,
                SurfaceUnit.SQUARE_MILLIMETER, UnitSystem.METRIC), "5 m²");
        assertEquals(formatter.formatAndConvert(
                5000000000000.50,
                SurfaceUnit.SQUARE_MILLIMETER, UnitSystem.METRIC), "5 Km²");

        assertEquals(formatter.formatAndConvert(
                1.0,
                SurfaceUnit.SQUARE_INCH, UnitSystem.IMPERIAL), "1 sq in");
        assertEquals(formatter.formatAndConvert(
                1.0,
                SurfaceUnit.SQUARE_FOOT, UnitSystem.IMPERIAL), "1 sq ft");
        assertEquals(formatter.formatAndConvert(
                1.0,
                SurfaceUnit.SQUARE_YARD, UnitSystem.IMPERIAL), "1 sq yd");
        assertEquals(formatter.formatAndConvert(
                1.0,
                SurfaceUnit.SQUARE_MILE, UnitSystem.IMPERIAL), "1 sq mi");
    }

    @Test
    public void testFormatAndConvertSurfaceAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(
                new Surface(5.50, SurfaceUnit.SQUARE_MILLIMETER),
                UnitSystem.METRIC), "5,5 mm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(505.00, SurfaceUnit.SQUARE_MILLIMETER),
                UnitSystem.METRIC), "5,05 cm²");
        assertEquals(formatter.formatAndConvert(
                new Surface(5000005.00, SurfaceUnit.SQUARE_MILLIMETER),
                UnitSystem.METRIC), "5 m²");
        assertEquals(formatter.formatAndConvert(
                new Surface(5000000000000.50, SurfaceUnit.SQUARE_MILLIMETER),
                UnitSystem.METRIC), "5 Km²");

        assertEquals(formatter.formatAndConvert(
                new Surface(1.0, SurfaceUnit.SQUARE_INCH),
                UnitSystem.IMPERIAL), "1 sq in");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0, SurfaceUnit.SQUARE_FOOT),
                UnitSystem.IMPERIAL), "1 sq ft");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0, SurfaceUnit.SQUARE_YARD),
                UnitSystem.IMPERIAL), "1 sq yd");
        assertEquals(formatter.formatAndConvert(
                new Surface(1.0, SurfaceUnit.SQUARE_MILE),
                UnitSystem.IMPERIAL), "1 sq mi");
    }

    @Test
    public void testFormatAndConvertMetric() {
        Locale l = new Locale("es", "ES");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5.50),
                SurfaceUnit.SQUARE_MILLIMETER), "5,5 mm²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(505.00),
                SurfaceUnit.SQUARE_MILLIMETER), "5,05 cm²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5000005.00),
                SurfaceUnit.SQUARE_MILLIMETER), "5 m²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(5000000000000.50),
                SurfaceUnit.SQUARE_MILLIMETER), "5 Km²");

        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_INCH), "6,45 cm²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_FOOT), "929,03 cm²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_YARD), "8.361,27 cm²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.SQUARE_MILE), "2,59 Km²");

        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.CENTIARE), "1 m²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.ARE), "100 m²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.DECARE), "1.000 m²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.HECTARE), "10.000 m²");
        assertEquals(formatter.formatAndConvertMetric(new BigDecimal(1.0),
                SurfaceUnit.ACRE), "4.046,86 m²");
    }

    @Test
    public void testFormatAndConvertImperial() {
        Locale l = new Locale("en", "US");

        SurfaceFormatter formatter = new SurfaceFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(5.50),
                SurfaceUnit.SQUARE_INCH), "5.5 sq in");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(180.0),
                SurfaceUnit.SQUARE_INCH), "1.25 sq ft");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(16.50),
                SurfaceUnit.SQUARE_FOOT), "1.83 sq yd");
        assertEquals(formatter.formatAndConvertImperial(new BigDecimal(3520.0),
                SurfaceUnit.ACRE), "5.5 sq mi");
    }

    @Test
    public void testGetAvailableLocales() {
        Locale[] locales = SurfaceFormatter.getAvailableLocales();
        assertArrayEquals(locales, NumberFormat.getAvailableLocales());
    }

    @Test
    public void testGetSetMaximumFractionDigits() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertEquals(formatter.getMaximumFractionDigits(),
                NumberFormat.getInstance().getMaximumFractionDigits());

        //set new value
        formatter.setMaximumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMaximumIntegerDigits() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertEquals(formatter.getMaximumIntegerDigits(),
                NumberFormat.getInstance().getMaximumIntegerDigits());

        //set new value
        formatter.setMaximumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetMinimumFractionDigits() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertEquals(formatter.getMinimumFractionDigits(),
                NumberFormat.getInstance().getMinimumFractionDigits());

        //set new value
        formatter.setMinimumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMinimumIntegerDigits() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertEquals(formatter.getMinimumIntegerDigits(),
                NumberFormat.getInstance().getMinimumIntegerDigits());

        //set new value
        formatter.setMinimumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetRoundingMode() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertEquals(formatter.getRoundingMode(),
                NumberFormat.getInstance().getRoundingMode());

        //set new value
        formatter.setRoundingMode(RoundingMode.UNNECESSARY);

        //check correctness
        assertEquals(formatter.getRoundingMode(), RoundingMode.UNNECESSARY);
    }

    @Test
    public void testIsSetGroupingUsed() {
        SurfaceFormatter formatter = new SurfaceFormatter();

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
        SurfaceFormatter formatter = new SurfaceFormatter();

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
        SurfaceFormatter formatter = new SurfaceFormatter();

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
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.METRIC);

        formatter = new SurfaceFormatter(
                new Locale("en", "US"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.IMPERIAL);
    }

    @Test
    public void testIsValidUnit() {
        SurfaceFormatter formatter = new SurfaceFormatter();

        assertTrue(formatter.isValidUnit("mm²"));
        assertTrue(formatter.isValidUnit("mm² "));

        assertTrue(formatter.isValidUnit("cm²"));
        assertTrue(formatter.isValidUnit("cm² "));

        assertTrue(formatter.isValidUnit("m²"));
        assertTrue(formatter.isValidUnit("m² "));

        assertTrue(formatter.isValidUnit("Km²"));
        assertTrue(formatter.isValidUnit("Km² "));

        assertTrue(formatter.isValidUnit("sq ft"));
        assertTrue(formatter.isValidUnit("sq ft "));

        assertTrue(formatter.isValidUnit("sq mi"));
        assertTrue(formatter.isValidUnit("sq mi "));

        assertTrue(formatter.isValidUnit("ca"));
        assertTrue(formatter.isValidUnit("ca "));

        assertTrue(formatter.isValidUnit("a"));
        assertTrue(formatter.isValidUnit("a "));

        assertTrue(formatter.isValidUnit("daa"));
        assertTrue(formatter.isValidUnit("daa "));

        assertTrue(formatter.isValidUnit("ha"));
        assertTrue(formatter.isValidUnit("ha "));

        assertTrue(formatter.isValidUnit("acre"));
        assertTrue(formatter.isValidUnit("acre "));

        assertFalse(formatter.isValidUnit("w"));
    }

    @Test
    public void testIsValidMeasurement() {

        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 cm²";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 m²";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 Km²";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 sq in";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 sq ft";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 sq yd";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 sq mi";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 ca";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 a";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 daa";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 ha";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 acre";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5,5 s";
        assertFalse(formatter.isValidMeasurement(text));

        text = "m";
        assertFalse(formatter.isValidMeasurement(text));
    }

    @Test
    public void testIsMetricUnit() {
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 cm²";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 m²";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 Km²";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 sq in";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 sq ft";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 sq yd";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 sq mi";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 ca";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 a";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 daa";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 ha";
        assertTrue(formatter.isMetricUnit(text));

        text = "5,5 acre";
        assertFalse(formatter.isMetricUnit(text));

        text = "5,5 s";
        assertFalse(formatter.isMetricUnit(text));
    }

    @Test
    public void testIsImperialUnit() {
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 cm²";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 m²";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 Km²";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 sq in";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 sq ft";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 sq yd";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 sq mi";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 ca";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 a";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 daa";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 ha";
        assertFalse(formatter.isImperialUnit(text));

        text = "5,5 acre";
        assertTrue(formatter.isImperialUnit(text));

        text = "5,5 s";
        assertFalse(formatter.isImperialUnit(text));
    }

    @Test
    public void testGetUnitSystemFromSource() {
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 cm²";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 m²";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 Km²";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 sq in";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 sq ft";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 sq yd";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 sq mi";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 ca";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 a";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 daa";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 ha";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5,5 acre";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.IMPERIAL);

        text = "5,5 s";
        assertNull(formatter.getUnitSystem(text));
    }

    @Test
    public void testParse() throws ParseException, UnknownUnitException {
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        Surface s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_MILLIMETER);

        text = "5,5 cm²";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_CENTIMETER);

        text = "5,5 m²";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_METER);

        text = "5,5 Km²";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_KILOMETER);

        text = "5,5 sq in";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_INCH);

        text = "5,5 sq ft";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_FOOT);

        text = "5,5 sq yd";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_YARD);

        text = "5,5 sq mi";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.SQUARE_MILE);

        text = "5,5 ca";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.CENTIARE);

        text = "5,5 a";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.ARE);

        text = "5,5 daa";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.DECARE);

        text = "5,5 ha";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.HECTARE);

        text = "5,5 acre";
        s = formatter.parse(text);
        assertEquals(s.getValue().doubleValue(), 5.5, 0.0);
        assertEquals(s.getUnit(), SurfaceUnit.ACRE);

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
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        String text = "5,5 mm²";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_MILLIMETER);

        text = "5,5 cm²";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_CENTIMETER);

        text = "5,5 m²";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_METER);

        text = "5,5 Km²";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_KILOMETER);

        text = "5,5 sq in";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_INCH);

        text = "5,5 sq ft";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_FOOT);

        text = "5,5 sq yd";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_YARD);

        text = "5,5 sq mi";
        assertEquals(formatter.findUnit(text), SurfaceUnit.SQUARE_MILE);

        text = "5,5 ca";
        assertEquals(formatter.findUnit(text), SurfaceUnit.CENTIARE);

        text = "5,5 a";
        assertEquals(formatter.findUnit(text), SurfaceUnit.ARE);

        text = "5,5 daa";
        assertEquals(formatter.findUnit(text), SurfaceUnit.DECARE);

        text = "5,5 ha";
        assertEquals(formatter.findUnit(text), SurfaceUnit.HECTARE);

        text = "5,5 acre";
        assertEquals(formatter.findUnit(text), SurfaceUnit.ACRE);

        text = "5,5 s";
        assertNull(formatter.findUnit(text));
    }

    @Test
    public void testGetUnitSymbol() {
        SurfaceFormatter formatter = new SurfaceFormatter(
                new Locale("es", "ES"));

        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_MILLIMETER),
                SurfaceFormatter.SQUARE_MILLIMETER);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_CENTIMETER),
                SurfaceFormatter.SQUARE_CENTIMETER);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_METER),
                SurfaceFormatter.SQUARE_METER);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_KILOMETER),
                SurfaceFormatter.SQUARE_KILOMETER);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_INCH),
                SurfaceFormatter.SQUARE_INCH);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_FOOT),
                SurfaceFormatter.SQUARE_FOOT);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_YARD),
                SurfaceFormatter.SQUARE_YARD);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.SQUARE_MILE),
                SurfaceFormatter.SQUARE_MILE);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.CENTIARE),
                SurfaceFormatter.CENTIARE);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.ARE),
                SurfaceFormatter.ARE);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.DECARE),
                SurfaceFormatter.DECARE);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.HECTARE),
                SurfaceFormatter.HECTARE);
        assertEquals(formatter.getUnitSymbol(SurfaceUnit.ACRE),
                SurfaceFormatter.ACRE);
    }
}
