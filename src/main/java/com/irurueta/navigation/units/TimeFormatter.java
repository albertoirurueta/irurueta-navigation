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

import java.text.FieldPosition;
import java.text.MessageFormat;
import java.text.ParseException;
import java.util.Locale;

/**
 * Formats and parses time value and unit.
 */
public class TimeFormatter extends MeasureFormatter<Time, TimeUnit> implements Cloneable {

    /**
     * Nanosecond symbol.
     */
    public static final String NANOSECOND = "ns";

    /**
     * Microsecond symbol.
     */
    public static final String MICROSECOND = "µs";

    /**
     * Millisecond symbol.
     */
    public static final String MILLISECOND = "ms";

    /**
     * Second symbol.
     */
    public static final String SECOND = "s";

    /**
     * Minute symbol.
     */
    public static final String MINUTE = "min";

    /**
     * Hour symbol.
     */
    public static final String HOUR = "h";

    /**
     * Day symbol.
     */
    public static final String DAY = "d";

    /**
     * Week symbol.
     */
    public static final String WEEK = "wk";

    /**
     * Month symbol.
     */
    public static final String MONTH = "mon";

    /**
     * Year symbol.
     */
    public static final String YEAR = "yr";

    /**
     * n-th century symbol.
     */
    public static final String CENTURY = "th c.";

    /**
     * First century symbol.
     */
    public static final String FIRST_CENTURY = "st c.";

    /**
     * Second century symbol.
     */
    public static final String SECOND_CENTURY = "nd c.";

    /**
     * Default pattern to format centuries.
     * {0} corresponds to the value, {1} corresponds to the unit part.
     */
    public static final String CENTURY_FORMAT_PATTERN =
            "{0}{1}";

    /**
     * Constructor.
     */
    @SuppressWarnings("WeakerAccess")
    public TimeFormatter() {
        super();
    }

    /**
     * Constructor with locale.
     * @param locale locale.
     * @throws IllegalArgumentException if locale is null.
     */
    @SuppressWarnings("WeakerAccess")
    public TimeFormatter(Locale locale) throws IllegalArgumentException {
        super(locale);
    }

    /**
     * Clones this time formatter.
     * @return a copy of this time formatter.
     */
    @Override
    public Object clone() {
        TimeFormatter copy = new TimeFormatter();
        return internalClone(copy);
    }

    /**
     * Determines if two time formatters are equal by comparing all of its internal parameters.
     * @param obj another object to compare.
     * @return true if provided object is assumed to be equal to this instance.
     */
    @Override
    public boolean equals(Object obj) {
        boolean equals = super.equals(obj);
        return (!equals || obj instanceof TimeFormatter) && equals;
    }

    /**
     * Gets unit system for detected unit into provided string representation
     * of a measurement.
     * @param source a measurement string representation to be checked.
     * @return returns metric system only for units belonging to the International
     * System of units.
     */
    @Override
    public UnitSystem getUnitSystem(String source) {
        TimeUnit unit = findUnit(source);
        try {
            return unit != null ? TimeUnit.getUnitSystem(unit) : null;
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    /**
     * Formats provided time value and unit into a string representation.
     * @param value a time value.
     * @param unit a time unit.
     * @return string representation of provided measurement value and unit.
     */
    @Override
    public String format(Number value, TimeUnit unit) {
        if (unit == TimeUnit.CENTURY) {
            double v = value.doubleValue();

            String symbol;
            if(Math.abs(v) <= 1.0) {
                symbol = FIRST_CENTURY;
            } else if (Math.abs(v) <= 2.0) {
                symbol = SECOND_CENTURY;
            } else {
                symbol = CENTURY;
            }
            return MessageFormat.format(CENTURY_FORMAT_PATTERN,
                    mNumberFormat.format(value), symbol);
        } else {
            return super.format(value, unit);
        }
    }

    /**
     * Formats provided time value and unit into a string representation
     * and appends the result into provided string buffer.
     * @param value a time value.
     * @param unit a time unit.
     * @param toAppendTo buffer to append the result to.
     * @param pos field position where result will be appended.
     * @return provided string buffer where result is appended.
     */
    @Override
    public StringBuffer format(Number value, TimeUnit unit,
                               StringBuffer toAppendTo, FieldPosition pos) {
        if (unit == TimeUnit.CENTURY) {
            double v = value.doubleValue();

            String symbol;
            if(Math.abs(v) <= 1.0) {
                symbol = FIRST_CENTURY;
            } else if (Math.abs(v) <= 2.0) {
                symbol = SECOND_CENTURY;
            } else {
                symbol = CENTURY;
            }

            MessageFormat format = new MessageFormat(CENTURY_FORMAT_PATTERN);
            return format.format(new Object[]{mNumberFormat.format(value),
                    symbol}, toAppendTo, pos);
        } else {
            return super.format(value, unit, toAppendTo, pos);
        }
    }

    /**
     * Formats and converts provided time value and unit.
     * Unit system is ignored since time is always expressed in metric system,
     * but some units might not belong to the international system of units.
     * If provided value is too large for provided unit, this method will
     * convert it to a more appropriate unit.
     * @param value a measurment value.
     * @param unit a measurement unit.
     * @param system it is ignored.
     * @return a string representation of measurement value and unit.
     */
    @Override
    public String formatAndConvert(Number value, TimeUnit unit, UnitSystem system) {
        double v = value.doubleValue();

        double nanoseconds = TimeConverter.convert(v, unit, TimeUnit.NANOSECOND);
        if (Math.abs(nanoseconds) < (TimeConverter.SECONDS_PER_MICROSECOND /
                TimeConverter.SECONDS_PER_NANOSECOND)) {
            return format(nanoseconds, TimeUnit.NANOSECOND);
        }

        double microseconds = TimeConverter.convert(v, unit, TimeUnit.MICROSECOND);
        if (Math.abs(microseconds) < (TimeConverter.SECONDS_PER_MILLISECOND /
                TimeConverter.SECONDS_PER_MICROSECOND)) {
            return format(microseconds, TimeUnit.MICROSECOND);
        }

        double milliseconds = TimeConverter.convert(v, unit, TimeUnit.MILLISECOND);
        if (Math.abs(milliseconds) < (1.0 / TimeConverter.SECONDS_PER_MILLISECOND)) {
            return format(milliseconds, TimeUnit.MILLISECOND);
        }

        double seconds = TimeConverter.convert(v, unit, TimeUnit.SECOND);
        if (Math.abs(seconds) < TimeConverter.SECONDS_PER_MINUTE) {
            return format(seconds, TimeUnit.SECOND);
        }

        double minutes = TimeConverter.convert(v, unit, TimeUnit.MINUTE);
        if (Math.abs(minutes) < (TimeConverter.SECONDS_PER_HOUR /
                TimeConverter.SECONDS_PER_MINUTE)) {
            return format(minutes, TimeUnit.MINUTE);
        }

        double hours = TimeConverter.convert(v, unit, TimeUnit.HOUR);
        if (Math.abs(hours) < (TimeConverter.SECONDS_PER_DAY /
                TimeConverter.SECONDS_PER_HOUR)) {
            return format(hours, TimeUnit.HOUR);
        }

        double days = TimeConverter.convert(v, unit, TimeUnit.DAY);
        if (Math.abs(days) < (TimeConverter.SECONDS_PER_WEEK /
                TimeConverter.SECONDS_PER_DAY)) {
            return format(days, TimeUnit.DAY);
        }

        double weeks = TimeConverter.convert(v, unit, TimeUnit.WEEK);
        if (Math.abs(weeks) < (TimeConverter.SECONDS_PER_MONTH /
                TimeConverter.SECONDS_PER_WEEK)) {
            return format(weeks, TimeUnit.WEEK);
        }

        double months = TimeConverter.convert(v, unit, TimeUnit.MONTH);
        if (Math.abs(months) < (TimeConverter.SECONDS_PER_YEAR /
                TimeConverter.SECONDS_PER_MONTH)) {
            return format(months, TimeUnit.MONTH);
        }

        double years = TimeConverter.convert(v, unit, TimeUnit.YEAR);
        if (Math.abs(years) < (TimeConverter.SECONDS_PER_CENTURY /
                TimeConverter.SECONDS_PER_YEAR)) {
            return format(years, TimeUnit.YEAR);
        }

        double centuries = TimeConverter.convert(v, unit, TimeUnit.CENTURY);
        return format(centuries, TimeUnit.CENTURY);
    }

    /**
     * Returns unit system this instance will use based on its assigned locale.
     * @return always returns metric system
     */
    public UnitSystem getUnitSystem() {
        return UnitSystem.METRIC;
    }


    /**
     * Parses provided string and tries to determine a time value and unit.
     * @param source text to be parsed.
     * @return time containing a value and unit.
     * @throws ParseException if provided string cannot be parsed.
     * @throws UnknownUnitException if unit cannot be determined.
     */
    public Time parse(String source) throws ParseException,
            UnknownUnitException {
        return internalParse(source, new Time());
    }

    /**
     * Attempts to determine a time unit within a measurement string
     * representation.
     * @param source a measurement string representation.
     * @return a time unit, or null if none can be determined.
     */
    @Override
    public TimeUnit findUnit(String source) {
        if (source.contains(NANOSECOND + " ") || source.endsWith(NANOSECOND)) {
            return TimeUnit.NANOSECOND;
        }
        if (source.contains(MICROSECOND + " ") || source.endsWith(MICROSECOND)) {
            return TimeUnit.MICROSECOND;
        }
        if (source.contains(MILLISECOND + " ") || source.endsWith(MILLISECOND)) {
            return TimeUnit.MILLISECOND;
        }
        if (source.contains(SECOND + " ") || source.endsWith(SECOND)) {
            return TimeUnit.SECOND;
        }
        if (source.contains(MINUTE + " ") || source.endsWith(MINUTE)) {
            return TimeUnit.MINUTE;
        }
        if (source.contains(WEEK + " ") || source.endsWith(WEEK)) {
            return TimeUnit.WEEK;
        }
        if (source.contains(MONTH + " ") || source.endsWith(MONTH)) {
            return TimeUnit.MONTH;
        }
        if (source.contains(YEAR + " ") || source.endsWith(YEAR)) {
            return TimeUnit.YEAR;
        }
        if (source.contains(CENTURY) || source.contains(FIRST_CENTURY) ||
                source.contains(SECOND_CENTURY)) {
            return TimeUnit.CENTURY;
        }

        if (source.contains(DAY + " ") || source.endsWith(DAY)) {
            return TimeUnit.DAY;
        }
        if (source.contains(HOUR + " ") || source.endsWith(HOUR)) {
            return TimeUnit.HOUR;
        }

        return null;
    }

    /**
     * Returns unit string representation.
     * @param unit a measure unit.
     * @return its string representation.
     */
    @Override
    public String getUnitSymbol(TimeUnit unit) {
        String unitStr;
        switch (unit) {
            case NANOSECOND:
                unitStr = NANOSECOND;
                break;
            case MICROSECOND:
                unitStr = MICROSECOND;
                break;
            case MILLISECOND:
                unitStr = MILLISECOND;
                break;
            case MINUTE:
                unitStr = MINUTE;
                break;
            case HOUR:
                unitStr = HOUR;
                break;
            case DAY:
                unitStr = DAY;
                break;
            case WEEK:
                unitStr = WEEK;
                break;
            case MONTH:
                unitStr = MONTH;
                break;
            case YEAR:
                unitStr = YEAR;
                break;
            case CENTURY:
                unitStr = CENTURY;
                break;

            case SECOND:
            default:
                unitStr = SECOND;
                break;
        }
        return unitStr;
    }
}
