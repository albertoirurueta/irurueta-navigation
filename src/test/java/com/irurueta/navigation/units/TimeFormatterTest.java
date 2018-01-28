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

public class TimeFormatterTest {

    public TimeFormatterTest() { }

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
        TimeFormatter formatter = new TimeFormatter();

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
        formatter = new TimeFormatter(locale);

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
            formatter = new TimeFormatter(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(formatter);
    }

    @Test
    public void testClone() {
        TimeFormatter formatter1 = new TimeFormatter();
        TimeFormatter formatter2 = new TimeFormatter();

        //check
        assertNotSame(formatter1, formatter2);
        assertEquals(formatter1, formatter2);

        //test after initializing internal number format
        assertNotNull(formatter1.format(0.5, TimeUnit.SECOND,
                new StringBuffer(), new FieldPosition(0)));
        TimeFormatter formatter3 = (TimeFormatter)formatter1.clone();

        assertNotSame(formatter1, formatter3);
        assertEquals(formatter1, formatter3);
    }

    @Test
    public void testEquals() {
        TimeFormatter formatter1 = new TimeFormatter(Locale.ENGLISH);
        TimeFormatter formatter2 = new TimeFormatter(Locale.ENGLISH);
        TimeFormatter formatter3 = new TimeFormatter(Locale.FRENCH);

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
        TimeFormatter formatter1 = new TimeFormatter(Locale.ENGLISH);
        TimeFormatter formatter2 = new TimeFormatter(Locale.ENGLISH);
        TimeFormatter formatter3 = new TimeFormatter(Locale.FRENCH);

        assertEquals(formatter1.hashCode(), formatter1.hashCode());
        assertEquals(formatter1.hashCode(), formatter2.hashCode());
        assertNotEquals(formatter1.hashCode(), formatter3.hashCode());
    }

    @Test
    public void testFormatNumber() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.NANOSECOND), "3 ns");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MICROSECOND), "3 µs");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MILLISECOND), "3 ms");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.SECOND), "3 s");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MINUTE), "3 min");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.HOUR), "3 h");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.DAY), "3 d");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.WEEK), "3 wk");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MONTH), "3 mon");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.YEAR), "3 yr");
        assertEquals(formatter.format(new BigDecimal(1.0),
                TimeUnit.CENTURY), "1st c.");
        assertEquals(formatter.format(new BigDecimal(2.0),
                TimeUnit.CENTURY), "2nd c.");
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.CENTURY), "3th c.");
    }

    @Test
    public void testFormatNumberAndStringBuffer() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.NANOSECOND, buffer,
                new FieldPosition(0)).toString(), "3 ns");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MICROSECOND, buffer,
                new FieldPosition(0)).toString(), "3 µs");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MILLISECOND, buffer,
                new FieldPosition(0)).toString(), "3 ms");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.SECOND, buffer,
                new FieldPosition(0)).toString(), "3 s");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MINUTE, buffer,
                new FieldPosition(0)).toString(), "3 min");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.HOUR, buffer,
                new FieldPosition(0)).toString(), "3 h");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.DAY, buffer,
                new FieldPosition(0)).toString(), "3 d");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.WEEK, buffer,
                new FieldPosition(0)).toString(), "3 wk");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.MONTH, buffer,
                new FieldPosition(0)).toString(), "3 mon");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.YEAR, buffer,
                new FieldPosition(0)).toString(), "3 yr");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(1.0),
                TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "1st c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(2.0),
                TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "2nd c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new BigDecimal(value),
                TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "3th c.");
    }

    @Test
    public void testFormatDouble() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        assertEquals(formatter.format(value, TimeUnit.NANOSECOND),
                "3 ns");
        assertEquals(formatter.format(value, TimeUnit.MICROSECOND),
                "3 µs");
        assertEquals(formatter.format(value, TimeUnit.MILLISECOND),
                "3 ms");
        assertEquals(formatter.format(value, TimeUnit.SECOND),
                "3 s");
        assertEquals(formatter.format(value, TimeUnit.MINUTE),
                "3 min");
        assertEquals(formatter.format(value, TimeUnit.HOUR),
                "3 h");
        assertEquals(formatter.format(value, TimeUnit.DAY),
                "3 d");
        assertEquals(formatter.format(value, TimeUnit.WEEK),
                "3 wk");
        assertEquals(formatter.format(value, TimeUnit.MONTH),
                "3 mon");
        assertEquals(formatter.format(value, TimeUnit.YEAR),
                "3 yr");
        assertEquals(formatter.format(1.0, TimeUnit.CENTURY),
                "1st c.");
        assertEquals(formatter.format(2.0, TimeUnit.CENTURY),
                "2nd c.");
        assertEquals(formatter.format(value, TimeUnit.CENTURY),
                "3th c.");
    }

    @Test
    public void testFormatDoubleAndStringBuffer() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.NANOSECOND, buffer,
                new FieldPosition(0)).toString(), "3 ns");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.MICROSECOND, buffer,
                new FieldPosition(0)).toString(), "3 µs");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.MILLISECOND, buffer,
                new FieldPosition(0)).toString(), "3 ms");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.SECOND, buffer,
                new FieldPosition(0)).toString(), "3 s");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.MINUTE, buffer,
                new FieldPosition(0)).toString(), "3 min");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.HOUR, buffer,
                new FieldPosition(0)).toString(), "3 h");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.DAY, buffer,
                new FieldPosition(0)).toString(), "3 d");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.WEEK, buffer,
                new FieldPosition(0)).toString(), "3 wk");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.MONTH, buffer,
                new FieldPosition(0)).toString(), "3 mon");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.YEAR, buffer,
                new FieldPosition(0)).toString(), "3 yr");

        buffer = new StringBuffer();
        assertEquals(formatter.format(1.0, TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "1st c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(2.0, TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "2nd c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(value, TimeUnit.CENTURY, buffer,
                new FieldPosition(0)).toString(), "3th c.");
    }

    @Test
    public void testFormatTime() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        assertEquals(formatter.format(new Time(value, TimeUnit.NANOSECOND)),
                "3 ns");
        assertEquals(formatter.format(new Time(value, TimeUnit.MICROSECOND)),
                "3 µs");
        assertEquals(formatter.format(new Time(value, TimeUnit.MILLISECOND)),
                "3 ms");
        assertEquals(formatter.format(new Time(value, TimeUnit.SECOND)),
                "3 s");
        assertEquals(formatter.format(new Time(value, TimeUnit.MINUTE)),
                "3 min");
        assertEquals(formatter.format(new Time(value, TimeUnit.HOUR)),
                "3 h");
        assertEquals(formatter.format(new Time(value, TimeUnit.DAY)),
                "3 d");
        assertEquals(formatter.format(new Time(value, TimeUnit.WEEK)),
                "3 wk");
        assertEquals(formatter.format(new Time(value, TimeUnit.MONTH)),
                "3 mon");
        assertEquals(formatter.format(new Time(value, TimeUnit.YEAR)),
                "3 yr");
        assertEquals(formatter.format(new Time(1.0, TimeUnit.CENTURY)),
                "1st c.");
        assertEquals(formatter.format(new Time(2.0, TimeUnit.CENTURY)),
                "2nd c.");
        assertEquals(formatter.format(new Time(value, TimeUnit.CENTURY)),
                "3th c.");
    }

    @Test
    public void testFormatTimeAndStringBuffer() {
        double value = 3.0;
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);

        StringBuffer buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.NANOSECOND),
                buffer, new FieldPosition(0)).toString(), "3 ns");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.MICROSECOND),
                buffer, new FieldPosition(0)).toString(), "3 µs");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.MILLISECOND),
                buffer, new FieldPosition(0)).toString(), "3 ms");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.SECOND),
                buffer, new FieldPosition(0)).toString(), "3 s");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.MINUTE),
                buffer, new FieldPosition(0)).toString(), "3 min");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.HOUR),
                buffer, new FieldPosition(0)).toString(), "3 h");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.DAY),
                buffer, new FieldPosition(0)).toString(), "3 d");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.WEEK),
                buffer, new FieldPosition(0)).toString(), "3 wk");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.MONTH),
                buffer, new FieldPosition(0)).toString(), "3 mon");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.YEAR),
                buffer, new FieldPosition(0)).toString(), "3 yr");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(1.0, TimeUnit.CENTURY),
                buffer, new FieldPosition(0)).toString(), "1st c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(2.0, TimeUnit.CENTURY),
                buffer, new FieldPosition(0)).toString(), "2nd c.");

        buffer = new StringBuffer();
        assertEquals(formatter.format(new Time(value, TimeUnit.CENTURY),
                buffer, new FieldPosition(0)).toString(), "3th c.");
    }

    @Test
    public void testFormatAndConvertNumber() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.NANOSECOND), "5,5 ns");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MICROSECOND), "5,5 µs");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MILLISECOND), "5,5 ms");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.SECOND), "5,5 s");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MINUTE), "5,5 min");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.HOUR), "5,5 h");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.DAY), "5,5 d");
        assertEquals(formatter.formatAndConvert(new BigDecimal(3.50),
                TimeUnit.WEEK), "3,5 wk");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MONTH), "5,5 mon");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.YEAR), "5,5 yr");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.0),
                TimeUnit.CENTURY), "5th c.");
    }

    @Test
    public void testFormatAndConvertDouble() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.NANOSECOND), "5,5 ns");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MICROSECOND), "5,5 µs");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MILLISECOND), "5,5 ms");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.SECOND), "5,5 s");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MINUTE), "5,5 min");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.HOUR), "5,5 h");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.DAY), "5,5 d");
        assertEquals(formatter.formatAndConvert(3.50,
                TimeUnit.WEEK), "3,5 wk");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MONTH), "5,5 mon");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.YEAR), "5,5 yr");
        assertEquals(formatter.formatAndConvert(5.0,
                TimeUnit.CENTURY), "5th c.");
    }

    @Test
    public void testFormatAndConvertTime() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.NANOSECOND)), "5,5 ns");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MICROSECOND)), "5,5 µs");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MILLISECOND)), "5,5 ms");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.SECOND)), "5,5 s");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MINUTE)), "5,5 min");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.HOUR)), "5,5 h");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.DAY)), "5,5 d");
        assertEquals(formatter.formatAndConvert(new Time(3.50,
                TimeUnit.WEEK)), "3,5 wk");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MONTH)), "5,5 mon");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.YEAR)), "5,5 yr");
        assertEquals(formatter.formatAndConvert(new Time(5.0,
                TimeUnit.CENTURY)), "5th c.");
    }

    @Test
    public void testFormatAndConvertNumberAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.NANOSECOND, null), "5,5 ns");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MICROSECOND, null), "5,5 µs");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MILLISECOND, null), "5,5 ms");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.SECOND, null), "5,5 s");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MINUTE, null), "5,5 min");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.HOUR, null), "5,5 h");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.DAY, null), "5,5 d");
        assertEquals(formatter.formatAndConvert(new BigDecimal(3.50),
                TimeUnit.WEEK, null), "3,5 wk");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.MONTH, null), "5,5 mon");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.50),
                TimeUnit.YEAR, null), "5,5 yr");
        assertEquals(formatter.formatAndConvert(new BigDecimal(5.0),
                TimeUnit.CENTURY, null), "5th c.");
    }

    @Test
    public void testFormatAndConvertDoubleAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.NANOSECOND, null), "5,5 ns");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MICROSECOND, null), "5,5 µs");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MILLISECOND, null), "5,5 ms");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.SECOND, null), "5,5 s");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MINUTE, null), "5,5 min");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.HOUR, null), "5,5 h");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.DAY, null), "5,5 d");
        assertEquals(formatter.formatAndConvert(3.50,
                TimeUnit.WEEK, null), "3,5 wk");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.MONTH, null), "5,5 mon");
        assertEquals(formatter.formatAndConvert(5.50,
                TimeUnit.YEAR, null), "5,5 yr");
        assertEquals(formatter.formatAndConvert(5.0,
                TimeUnit.CENTURY, null), "5th c.");
    }

    @Test
    public void testFormatAndConvertTimeAndUnitSystem() {
        Locale l = new Locale("es", "ES");

        TimeFormatter formatter = new TimeFormatter(l);
        formatter.setMaximumFractionDigits(2);

        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.NANOSECOND), null), "5,5 ns");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MICROSECOND), null), "5,5 µs");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MILLISECOND), null), "5,5 ms");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.SECOND), null), "5,5 s");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MINUTE), null), "5,5 min");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.HOUR), null), "5,5 h");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.DAY), null), "5,5 d");
        assertEquals(formatter.formatAndConvert(new Time(3.50,
                TimeUnit.WEEK), null), "3,5 wk");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.MONTH), null), "5,5 mon");
        assertEquals(formatter.formatAndConvert(new Time(5.50,
                TimeUnit.YEAR), null), "5,5 yr");
        assertEquals(formatter.formatAndConvert(new Time(5.0,
                TimeUnit.CENTURY), null), "5th c.");
    }

    @Test
    public void testGetAvailableLocales() {
        Locale[] locales = TimeFormatter.getAvailableLocales();
        assertArrayEquals(locales, NumberFormat.getAvailableLocales());
    }

    @Test
    public void testGetSetMaximumFractionDigits() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getMaximumFractionDigits(),
                NumberFormat.getInstance().getMaximumFractionDigits());

        //set new value
        formatter.setMaximumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMaximumIntegerDigits() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getMaximumIntegerDigits(),
                NumberFormat.getInstance().getMaximumIntegerDigits());

        //set new value
        formatter.setMaximumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMaximumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetMinimumFractionDigits() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getMinimumFractionDigits(),
                NumberFormat.getInstance().getMinimumFractionDigits());

        //set new value
        formatter.setMinimumFractionDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumFractionDigits(), 2);
    }

    @Test
    public void testGetSetMinimumIntegerDigits() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getMinimumIntegerDigits(),
                NumberFormat.getInstance().getMinimumIntegerDigits());

        //set new value
        formatter.setMinimumIntegerDigits(2);

        //check correctness
        assertEquals(formatter.getMinimumIntegerDigits(), 2);
    }

    @Test
    public void testGetSetRoundingMode() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getRoundingMode(),
                NumberFormat.getInstance().getRoundingMode());

        //set new value
        formatter.setRoundingMode(RoundingMode.UNNECESSARY);

        //check correctness
        assertEquals(formatter.getRoundingMode(), RoundingMode.UNNECESSARY);
    }

    @Test
    public void testIsSetGroupingUsed() {
        TimeFormatter formatter = new TimeFormatter();

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
        TimeFormatter formatter = new TimeFormatter();

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
        TimeFormatter formatter = new TimeFormatter();

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
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.METRIC);

        formatter = new TimeFormatter(
                new Locale("en", "US"));
        assertEquals(formatter.getUnitSystem(), UnitSystem.METRIC);
    }

    @Test
    public void testIsValidUnit() {
        TimeFormatter formatter = new TimeFormatter();

        assertTrue(formatter.isValidUnit("ns"));
        assertTrue(formatter.isValidUnit("µs"));
        assertTrue(formatter.isValidUnit("s"));
        assertTrue(formatter.isValidUnit("min"));
        assertTrue(formatter.isValidUnit("h"));
        assertTrue(formatter.isValidUnit("d"));
        assertTrue(formatter.isValidUnit("wk"));
        assertTrue(formatter.isValidUnit("mon"));
        assertTrue(formatter.isValidUnit("yr"));
        assertTrue(formatter.isValidUnit("st c."));
        assertTrue(formatter.isValidUnit("nd c."));
        assertTrue(formatter.isValidUnit("th c."));
    }

    @Test
    public void testIsValidMeasurement() {
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));

        String text = "5 ns";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 µs";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 ms";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 s";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 min";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 h";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 d";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 wk";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 mon";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5 yr";
        assertTrue(formatter.isValidMeasurement(text));

        text = "1st c.";
        assertTrue(formatter.isValidMeasurement(text));

        text = "2nd c.";
        assertTrue(formatter.isValidMeasurement(text));

        text = "5th c.";
        assertTrue(formatter.isValidMeasurement(text));


        text = "m";
        assertFalse(formatter.isValidMeasurement(text));
    }

    @Test
    public void testIsMetricUnit() {
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));

        String text = "5 ns";
        assertTrue(formatter.isMetricUnit(text));

        text = "5 µs";
        assertTrue(formatter.isMetricUnit(text));

        text = "5 ms";
        assertTrue(formatter.isMetricUnit(text));

        text = "5 s";
        assertTrue(formatter.isMetricUnit(text));

        text = "5 min";
        assertFalse(formatter.isMetricUnit(text));

        text = "5 h";
        assertFalse(formatter.isMetricUnit(text));

        text = "5 d";
        assertFalse(formatter.isMetricUnit(text));

        text = "5 wk";
        assertFalse(formatter.isMetricUnit(text));

        text = "5 mon";
        assertFalse(formatter.isMetricUnit(text));

        text = "5 yr";
        assertFalse(formatter.isMetricUnit(text));

        text = "5th c.";
        assertFalse(formatter.isMetricUnit(text));
    }

    @Test
    public void testGetUnitSystemFromSource() {
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));

        String text = "5 ns";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5 µs";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5 ms";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5 s";
        assertEquals(formatter.getUnitSystem(text), UnitSystem.METRIC);

        text = "5 min";
        assertNull(formatter.getUnitSystem(text));

        text = "5 h";
        assertNull(formatter.getUnitSystem(text));

        text = "5 d";
        assertNull(formatter.getUnitSystem(text));

        text = "5 wk";
        assertNull(formatter.getUnitSystem(text));

        text = "5 mon";
        assertNull(formatter.getUnitSystem(text));

        text = "5 yr";
        assertNull(formatter.getUnitSystem(text));

        text = "5th c.";
        assertNull(formatter.getUnitSystem(text));
    }

    @Test
    public void testParse() throws ParseException, UnknownUnitException {
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));

        String text = "5 ns";
        Time t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.NANOSECOND);

        text = "5 µs";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.MICROSECOND);

        text = "5 ms";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.MILLISECOND);

        text = "5 s";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.SECOND);

        text = "5 min";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.MINUTE);

        text = "5 h";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.HOUR);

        text = "5 d";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.DAY);

        text = "5 wk";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.WEEK);

        text = "5 mon";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.MONTH);

        text = "5 yr";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.YEAR);

        text = "1st c.";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 1.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.CENTURY);

        text = "2nd c.";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 2.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.CENTURY);

        text = "5th c.";
        t = formatter.parse(text);
        assertEquals(t.getValue().doubleValue(), 5.0, 0.0);
        assertEquals(t.getUnit(), TimeUnit.CENTURY);

        //Force UnknownUnitException
        text = "5,5 m";
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
        TimeFormatter formatter = new TimeFormatter(
                new Locale("es", "ES"));

        String text = "5 ns";
        assertEquals(formatter.findUnit(text), TimeUnit.NANOSECOND);

        text = "5 µs";
        assertEquals(formatter.findUnit(text), TimeUnit.MICROSECOND);

        text = "5 ms";
        assertEquals(formatter.findUnit(text), TimeUnit.MILLISECOND);

        text = "5 s";
        assertEquals(formatter.findUnit(text), TimeUnit.SECOND);

        text = "5 min";
        assertEquals(formatter.findUnit(text), TimeUnit.MINUTE);

        text = "5 h";
        assertEquals(formatter.findUnit(text), TimeUnit.HOUR);

        text = "5 d";
        assertEquals(formatter.findUnit(text), TimeUnit.DAY);

        text = "5 wk";
        assertEquals(formatter.findUnit(text), TimeUnit.WEEK);

        text = "5 mon";
        assertEquals(formatter.findUnit(text), TimeUnit.MONTH);

        text = "5 yr";
        assertEquals(formatter.findUnit(text), TimeUnit.YEAR);

        text = "1st c.";
        assertEquals(formatter.findUnit(text), TimeUnit.CENTURY);

        text = "2nd c.";
        assertEquals(formatter.findUnit(text), TimeUnit.CENTURY);

        text = "5th c.";
        assertEquals(formatter.findUnit(text), TimeUnit.CENTURY);

        text = "5,5 m";
        assertNull(formatter.findUnit(text));
    }

    @Test
    public void testGetUnitSymbol() {
        TimeFormatter formatter = new TimeFormatter();

        assertEquals(formatter.getUnitSymbol(TimeUnit.NANOSECOND),
                TimeFormatter.NANOSECOND);
        assertEquals(formatter.getUnitSymbol(TimeUnit.MICROSECOND),
                TimeFormatter.MICROSECOND);
        assertEquals(formatter.getUnitSymbol(TimeUnit.MILLISECOND),
                TimeFormatter.MILLISECOND);
        assertEquals(formatter.getUnitSymbol(TimeUnit.SECOND),
                TimeFormatter.SECOND);
        assertEquals(formatter.getUnitSymbol(TimeUnit.MINUTE),
                TimeFormatter.MINUTE);
        assertEquals(formatter.getUnitSymbol(TimeUnit.HOUR),
                TimeFormatter.HOUR);
        assertEquals(formatter.getUnitSymbol(TimeUnit.DAY),
                TimeFormatter.DAY);
        assertEquals(formatter.getUnitSymbol(TimeUnit.WEEK),
                TimeFormatter.WEEK);
        assertEquals(formatter.getUnitSymbol(TimeUnit.MONTH),
                TimeFormatter.MONTH);
        assertEquals(formatter.getUnitSymbol(TimeUnit.YEAR),
                TimeFormatter.YEAR);
        assertEquals(formatter.getUnitSymbol(TimeUnit.CENTURY),
                TimeFormatter.CENTURY);
    }
}
