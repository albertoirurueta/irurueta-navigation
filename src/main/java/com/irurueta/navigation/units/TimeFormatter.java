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
import java.text.NumberFormat;
import java.text.ParseException;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Formats and parses time value and unit.
 */
public class TimeFormatter extends MeasureFormatter<Time, TimeUnit> implements Cloneable {

    /**
     * Flag indicating whether nanoseconds must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_NANOSECONDS = 1;

    /**
     * Flag indicating whether microseconds must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_MICROSECONDS = 1 << 1;

    /**
     * Flag indicating whether milliseconds must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_MILLISECONDS = 1 << 2;

    /**
     * Flag indicating whether seconds must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_SECONDS = 1 << 3;

    /**
     * Flag indicating whether minutes must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_MINUTES = 1 << 4;

    /**
     * Flag indicating whether hours must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_HOURS = 1 << 5;

    /**
     * Flag indicating whether days must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_DAYS = 1 << 6;

    /**
     * Flag indicating whether weeks must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_WEEKS = 1 << 7;

    /**
     * Flag indicating whether months must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_MONTHS = 1 << 8;

    /**
     * Flag indicating whether years must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_YEARS = 1 << 9;

    /**
     * Flag indicating whether centuries must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_CENTURIES = 1 << 10;

    /**
     * Flag indicating that all units must be included when formatting a time
     * instance.
     */
    public static final int FORMAT_ALL = FORMAT_NANOSECONDS | FORMAT_MICROSECONDS |
            FORMAT_MILLISECONDS | FORMAT_SECONDS | FORMAT_MINUTES | FORMAT_HOURS |
            FORMAT_DAYS | FORMAT_WEEKS | FORMAT_MONTHS | FORMAT_YEARS |
            FORMAT_CENTURIES;

    /**
     * Flag indicating that all time units (smaller than a day) must be used when
     * formatting a time instance.
     */
    public static final int FORMAT_TIME_ALL = FORMAT_NANOSECONDS | FORMAT_MICROSECONDS |
            FORMAT_MILLISECONDS | FORMAT_SECONDS | FORMAT_MINUTES | FORMAT_HOURS;

    /**
     * Flag indicating that standard time units (hours, minutes and seconds) must be
     * used when formatting a time instance.
     */
    public static final int FORMAT_TIME_STANDARD = FORMAT_SECONDS | FORMAT_MINUTES |
            FORMAT_HOURS;

    /**
     * Flag indicating that all date units (greater than hours) must be used when
     * formatting a time instance.
     */
    public static final int FORMAT_DATE_ALL = FORMAT_DAYS | FORMAT_WEEKS |
            FORMAT_MONTHS | FORMAT_YEARS | FORMAT_CENTURIES;

    /**
     * Flag indicating that standard date units (days, months and years) must be
     * used when formatting a time instance.
     */
    public static final int FORMAT_DATE_STANDARD = FORMAT_DAYS | FORMAT_MONTHS |
            FORMAT_YEARS;

    /**
     * Nanosecond symbol.
     */
    public static final String NANOSECOND_SYMBOL = "ns";

    /**
     * Microsecond symbol.
     */
    public static final String MICROSECOND_SYMBOL = "µs";

    /**
     * Millisecond symbol.
     */
    public static final String MILLISECOND_SYMBOL = "ms";

    /**
     * Second symbol.
     */
    public static final String SECOND_SYMBOL = "s";

    /**
     * Minute symbol.
     */
    public static final String MINUTE_SYMBOL = "min";

    /**
     * Hour symbol.
     */
    public static final String HOUR_SYMBOL = "h";

    /**
     * Day symbol.
     */
    public static final String DAY_SYMBOL = "d";

    /**
     * Week symbol.
     */
    public static final String WEEK_SYMBOL = "wk";

    /**
     * Month symbol.
     */
    public static final String MONTH_SYMBOL = "mon";

    /**
     * Year symbol.
     */
    public static final String YEAR_SYMBOL = "yr";

    /**
     * n-th century symbol.
     */
    public static final String CENTURY_SYMBOL = "th c.";

    /**
     * First century symbol.
     */
    private static final String FIRST_CENTURY_SYMBOL = "st c.";

    /**
     * Second century symbol.
     */
    private static final String SECOND_CENTURY_SYMBOL = "nd c.";

    /**
     * Third century symbol.
     */
    private static final String THIRD_CENTURY_SIMBOL = "rd c.";

    /**
     * Default pattern to format centuries.
     * {0} corresponds to the value, {1} corresponds to the unit part.
     */
    private static final String CENTURY_FORMAT_PATTERN =
            "{0}{1}";

    /**
     * Minimum number of digits for hours and minutes.
     */
    private static final int INTEGER_DIGITS = 2;

    /**
     * Pattern to format hour and minutes (hh:mm.s).
     */
    private static final String HOUR_MINUTE_PATTERN = "^([0-9]+):(\\d{2,})$";


    /**
     * Pattern to format hour, minutes and seconds (hh:mm:ss.ms).
     */
    private static final String HOUR_MINUTE_SECOND_PATTERN = "^([0-9]+):([0-9]{2}):(\\d{2,})$";

    /**
     * Pattern to parse 1st century format.
     */
    private static final String FIRST_CENTURY_PATTERN = "(.*)(\\s+)(\\d+)(\\s?st c.)(\\s+)(.*)";

    /**
     * Pattern to parse 2nd century format.
     */
    private static final String SECOND_CENTURY_PATTERN = "(.*)(\\s+)(\\d+)(\\s?nd c.)(\\s+)(.*)";

    /**
     * Pattern to parse 3rd century format.
     */
    private static final String THIRD_CENTURY_PATTERN = "(.*)(\\s+)(\\d+)(\\s?rd c.)(\\s+)(.*)";

    /**
     * Pattern to parse n-th century format.
     */
    private static final String CENTURY_PATTERN = "(.*)(\\s+)(\\d+)(\\s?th c.)(\\s+)(.*)";

    /**
     * Pattern to parse year format.
     */
    private static final String YEAR_PATTERN = "(.*)(\\s+)(\\d+)(\\s?yr)(\\s+)(.*)";

    /**
     * Pattern to parse month format.
     */
    private static final String MONTH_PATTERN = "(.*)(\\s+)(\\d+)(\\s?mon)(\\s+)(.*)";

    /**
     * Pattern to parse week format.
     */
    private static final String WEEK_PATTERN = "(.*)(\\s+)(\\d+)(\\s?wk)(\\s+)(.*)";

    /**
     * Pattern to parse day format.
     */
    private static final String DAY_PATTERN = "(.*)(\\s+)(\\d+)(\\s?d)(\\s+)(.*)";

    /**
     * Pattern to parse hour format.
     */
    private static final String HOUR_PATTERN = "(.*)(\\s+)(\\d+)(\\s?h)(\\s+)(.*)";

    /**
     * Pattern to parse minute format.
     */
    private static final String MINUTE_PATTERN = "(.*)(\\s+)(\\d+)(\\s?min)(\\s+)(.*)";

    /**
     * Pattern to parse second format.
     */
    private static final String SECOND_PATTERN = "(.*)(\\s+)(\\d+)(\\s?s)(\\s+)(.*)";

    /**
     * Pattern to parse millisecond format.
     */
    private static final String MILLISECOND_PATTERN = "(.*)(\\s+)(\\d+)(\\s?ms)(\\s+)(.*)";

    /**
     * Pattern to parse microsecond format.
     */
    private static final String MICROSECOND_PATTERN = "(.*)(\\s+)(\\d+)(\\s?µs)(\\s+)(.*)";

    /**
     * Pattern to parse nanosecond format.
     */
    private static final String NANOSECOND_PATTERN = "(.*)(\\s+)(\\d+)(\\s?ns)(\\s+)(.*)";

    /**
     * Defines a space.
     */
    private static final String SPACE = " ";

    /**
     * Pattern to parse time in hour and minute format (hh:mm.s)
     */
    private Pattern mHourMinutePattern;

    /**
     * Patern to parse time in hour, minute and second format (hh:mm:ss.ms).
     */
    private Pattern mHourMinuteSecondPattern;

    /**
     * Pattern to parse 1st century format.
     */
    private Pattern mFirstCenturyPattern;

    /**
     * Pattern to parse 2nd century format.
     */
    private Pattern mSecondCenturyPattern;

    /**
     * Pattern to parse 3rd century format.
     */
    private Pattern mThirdCenturyPattern;

    /**
     * Pattern to parse n-th century format.
     */
    private Pattern mCenturyPattern;

    /**
     * Pattern to parse year format.
     */
    private Pattern mYearPattern;

    /**
     * Pattern to parse month format.
     */
    private Pattern mMonthPattern;

    /**
     * Pattern to parse week format.
     */
    private Pattern mWeekPattern;

    /**
     * Pattern to parse day format.
     */
    private Pattern mDayPattern;

    /**
     * Pattern to parse hour format.
     */
    private Pattern mHourPattern;

    /**
     * Pattern to parse minute format.
     */
    private Pattern mMinutePattern;

    /**
     * Pattern to parse second format.
     */
    private Pattern mSecondPattern;

    /**
     * Pattern to parse millisecond format.
     */
    private Pattern mMillisecondPattern;

    /**
     * Pattern to parse microsecond format.
     */
    private Pattern mMicrosecondPattern;

    /**
     * Pattern to parse nanosecond format.
     */
    private Pattern mNanosecondPattern;

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
                symbol = FIRST_CENTURY_SYMBOL;
            } else if (Math.abs(v) <= 2.0) {
                symbol = SECOND_CENTURY_SYMBOL;
            } else if (Math.abs(v) <= 3.0) {
                symbol = THIRD_CENTURY_SIMBOL;
            } else {
                symbol = CENTURY_SYMBOL;
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
                symbol = FIRST_CENTURY_SYMBOL;
            } else if (Math.abs(v) <= 2.0) {
                symbol = SECOND_CENTURY_SYMBOL;
            } else if (Math.abs(v) <= 3.0) {
                symbol = THIRD_CENTURY_SIMBOL;
            } else {
                symbol = CENTURY_SYMBOL;
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
        if (source.contains(NANOSECOND_SYMBOL + " ") || source.endsWith(NANOSECOND_SYMBOL)) {
            return TimeUnit.NANOSECOND;
        }
        if (source.contains(MICROSECOND_SYMBOL + " ") || source.endsWith(MICROSECOND_SYMBOL)) {
            return TimeUnit.MICROSECOND;
        }
        if (source.contains(MILLISECOND_SYMBOL + " ") || source.endsWith(MILLISECOND_SYMBOL)) {
            return TimeUnit.MILLISECOND;
        }
        if (source.contains(SECOND_SYMBOL + " ") || source.endsWith(SECOND_SYMBOL)) {
            return TimeUnit.SECOND;
        }
        if (source.contains(MINUTE_SYMBOL + " ") || source.endsWith(MINUTE_SYMBOL)) {
            return TimeUnit.MINUTE;
        }
        if (source.contains(WEEK_SYMBOL + " ") || source.endsWith(WEEK_SYMBOL)) {
            return TimeUnit.WEEK;
        }
        if (source.contains(MONTH_SYMBOL + " ") || source.endsWith(MONTH_SYMBOL)) {
            return TimeUnit.MONTH;
        }
        if (source.contains(YEAR_SYMBOL + " ") || source.endsWith(YEAR_SYMBOL)) {
            return TimeUnit.YEAR;
        }
        if (source.contains(CENTURY_SYMBOL) || source.contains(FIRST_CENTURY_SYMBOL) ||
                source.contains(SECOND_CENTURY_SYMBOL) ||
                source.contains(THIRD_CENTURY_SIMBOL)) {
            return TimeUnit.CENTURY;
        }

        if (source.contains(DAY_SYMBOL + " ") || source.endsWith(DAY_SYMBOL)) {
            return TimeUnit.DAY;
        }
        if (source.contains(HOUR_SYMBOL + " ") || source.endsWith(HOUR_SYMBOL)) {
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
                unitStr = NANOSECOND_SYMBOL;
                break;
            case MICROSECOND:
                unitStr = MICROSECOND_SYMBOL;
                break;
            case MILLISECOND:
                unitStr = MILLISECOND_SYMBOL;
                break;
            case MINUTE:
                unitStr = MINUTE_SYMBOL;
                break;
            case HOUR:
                unitStr = HOUR_SYMBOL;
                break;
            case DAY:
                unitStr = DAY_SYMBOL;
                break;
            case WEEK:
                unitStr = WEEK_SYMBOL;
                break;
            case MONTH:
                unitStr = MONTH_SYMBOL;
                break;
            case YEAR:
                unitStr = YEAR_SYMBOL;
                break;
            case CENTURY:
                unitStr = CENTURY_SYMBOL;
                break;

            case SECOND:
            default:
                unitStr = SECOND_SYMBOL;
                break;
        }
        return unitStr;
    }

    /**
     * Formats time instance using hour and minute format (hh:mm.ms).
     * @param time time to be formatted.
     * @return a string representation of provided time instance using hour and
     * minute format (hh:mm.ms).
     */
    public String formatHourMinute(Time time) {
        double exactHours = TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.HOUR);
        double hours = Math.floor(exactHours);
        double diffHours = exactHours - hours;

        double minutes = TimeConverter.convert(diffHours, TimeUnit.HOUR,
                TimeUnit.MINUTE);

        NumberFormat hourFormat = NumberFormat.getInstance(getLocale());
        hourFormat.setMinimumIntegerDigits(INTEGER_DIGITS);
        hourFormat.setMinimumFractionDigits(0);
        hourFormat.setMaximumFractionDigits(0);

        NumberFormat minuteFormat = NumberFormat.getInstance(getLocale());
        minuteFormat.setMinimumIntegerDigits(INTEGER_DIGITS);
        minuteFormat.setMaximumIntegerDigits(INTEGER_DIGITS);

        if (mNumberFormat != null) {
            hourFormat.setMaximumIntegerDigits(mNumberFormat.getMaximumIntegerDigits());
            hourFormat.setRoundingMode(mNumberFormat.getRoundingMode());
            hourFormat.setGroupingUsed(mNumberFormat.isGroupingUsed());

            minuteFormat.setMinimumFractionDigits(mNumberFormat.getMinimumFractionDigits());
            minuteFormat.setMaximumFractionDigits(mNumberFormat.getMaximumFractionDigits());
            minuteFormat.setRoundingMode(mNumberFormat.getRoundingMode());
            minuteFormat.setGroupingUsed(mNumberFormat.isGroupingUsed());
        }
        return hourFormat.format(hours) + ":" + minuteFormat.format(minutes);
    }

    /**
     * Parses provided string representation using hour and minute format (hh:mm).
     * Note: decimals are not accepted on either hour or minute values.
     * @param source string to be parsed.
     * @return parsed time.
     * @throws ParseException if parsing fails.
     * @throws UnknownUnitException if format is not recognized.
     */
    public Time parseHourMinute(CharSequence source) throws ParseException,
            UnknownUnitException {
        if (mHourMinutePattern == null) {
            mHourMinutePattern = Pattern.compile(HOUR_MINUTE_PATTERN);
        }

        Matcher matcher = mHourMinutePattern.matcher(source);
        if(!matcher.matches()) {
            throw new UnknownUnitException();
        }

        String hourString = matcher.group(1);
        String minuteString = matcher.group(2);

        Number hour = mNumberFormat.parse(hourString);
        Number minute = mNumberFormat.parse(minuteString);

        Time result = new Time(hour, TimeUnit.HOUR);
        TimeConverter.convert(result, TimeUnit.MINUTE);
        result.add(new Time(minute, TimeUnit.MINUTE));
        return result;
    }

    /**
     * Formats this instance using hour, minute and second format (hh:mm::ss.ms).
     * @param time time to be formatted.
     * @return a string representation of provided time instance using hour, minute
     * and second format (hh:mm:ss.ms).
     */
    public String formatHourMinuteSecond(Time time) {
        double exactHours = TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.HOUR);
        double hours = Math.floor(exactHours);
        double diffHours = exactHours - hours;

        double exactMinutes = TimeConverter.convert(diffHours, TimeUnit.HOUR,
                TimeUnit.MINUTE);
        double minutes = Math.floor(exactMinutes);
        double diffMinutes = exactMinutes - minutes;

        double seconds = TimeConverter.convert(diffMinutes, TimeUnit.MINUTE,
                TimeUnit.SECOND);

        NumberFormat hourFormat = NumberFormat.getInstance(getLocale());
        hourFormat.setMinimumIntegerDigits(INTEGER_DIGITS);
        hourFormat.setMinimumFractionDigits(0);
        hourFormat.setMaximumFractionDigits(0);

        NumberFormat minuteFormat = NumberFormat.getInstance(getLocale());
        minuteFormat.setMinimumIntegerDigits(INTEGER_DIGITS);
        minuteFormat.setMinimumFractionDigits(0);
        minuteFormat.setMaximumFractionDigits(0);

        NumberFormat secondFormat = NumberFormat.getInstance(getLocale());
        secondFormat.setMinimumIntegerDigits(INTEGER_DIGITS);
        secondFormat.setMaximumIntegerDigits(INTEGER_DIGITS);

        if (mNumberFormat != null) {
            hourFormat.setMaximumIntegerDigits(mNumberFormat.getMaximumIntegerDigits());
            hourFormat.setRoundingMode(mNumberFormat.getRoundingMode());
            hourFormat.setGroupingUsed(mNumberFormat.isGroupingUsed());

            minuteFormat.setMaximumIntegerDigits(mNumberFormat.getMaximumIntegerDigits());
            minuteFormat.setRoundingMode(mNumberFormat.getRoundingMode());
            minuteFormat.setGroupingUsed(mNumberFormat.isGroupingUsed());

            secondFormat.setMinimumFractionDigits(mNumberFormat.getMinimumFractionDigits());
            secondFormat.setMaximumFractionDigits(mNumberFormat.getMaximumFractionDigits());
            secondFormat.setRoundingMode(mNumberFormat.getRoundingMode());
            secondFormat.setGroupingUsed(mNumberFormat.isGroupingUsed());
        }
        return hourFormat.format(hours) + ":" + minuteFormat.format(minutes) +
                ":" + secondFormat.format(seconds);
    }

    /**
     * Parses provided string representation using hour, minute and second format (hh:mm:ss).
     * Note: decimals are not accepted on either hour, minute or seconds values
     * @param source string to be parsed.
     * @return parsed time.
     * @throws ParseException if parsing fails.
     * @throws UnknownUnitException if format is not recognized.
     */
    public Time parseHourMinuteSecond(CharSequence source) throws ParseException,
            UnknownUnitException {
        if (mHourMinuteSecondPattern == null) {
            mHourMinuteSecondPattern = Pattern.compile(HOUR_MINUTE_SECOND_PATTERN);
        }

        Matcher matcher = mHourMinuteSecondPattern.matcher(source);
        if (!matcher.matches()) {
            throw new UnknownUnitException();
        }

        String hourString = matcher.group(1);
        String minuteString = matcher.group(2);
        String secondString = matcher.group(3);

        Number hour = mNumberFormat.parse(hourString);
        Number minute = mNumberFormat.parse(minuteString);
        Number second = mNumberFormat.parse(secondString);

        Time result = new Time(hour, TimeUnit.HOUR);
        TimeConverter.convert(result, TimeUnit.MINUTE);
        result.add(new Time(minute, TimeUnit.MINUTE));
        TimeConverter.convert(result, TimeUnit.SECOND);
        result.add(new Time(second, TimeUnit.SECOND));
        return result;
    }

    /**
     * Formats provided time instance using required units as flags.
     * Flags can be provided as bitwise combinations of FORMAT constants.
     * Only non zero units will be included.
     * @param time time to be formattd.
     * @param flags flags indicating units to include.
     * @return formatted time.
     */
    public String formatMultiple(Time time, int flags) {
        return formatMultiple(time, flags, true);
    }

    /**
     * Formats provided time instance using required units as flags.
     * Flags can be provided as bitwise combinations of FORMAT constants.
     * @param time time to be formatted.
     * @param flags flags indicating units to include.
     * @param onlyNonZero true indicates to include only non zero units, false to
     *                    include all selected units even if they are zero.
     * @return formatted time.
     */
    public String formatMultiple(Time time, int flags, boolean onlyNonZero) {

        //centuries
        double exactCenturies = TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.CENTURY);
        double centuries = 0.0, diffCenturies;
        if ((flags & FORMAT_CENTURIES) != 0) {
            if((flags & (FORMAT_YEARS | FORMAT_MONTHS | FORMAT_WEEKS | FORMAT_DAYS |
                    FORMAT_HOURS | FORMAT_MINUTES | FORMAT_SECONDS |
                    FORMAT_MILLISECONDS | FORMAT_MICROSECONDS |
                    FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                centuries = Math.floor(exactCenturies);
            } else {
                centuries = exactCenturies;
            }
            diffCenturies = exactCenturies - centuries;
        } else {
            diffCenturies = exactCenturies;
        }

        //years
        double exactYears = TimeConverter.convert(diffCenturies, TimeUnit.CENTURY,
                TimeUnit.YEAR);
        double years = 0.0, diffYears;
        if ((flags & FORMAT_YEARS) != 0) {
            if ((flags & (FORMAT_MONTHS | FORMAT_WEEKS | FORMAT_DAYS |
                    FORMAT_HOURS | FORMAT_MINUTES | FORMAT_SECONDS |
                    FORMAT_MILLISECONDS | FORMAT_MICROSECONDS |
                    FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                years = Math.floor(exactYears);
            } else {
                years = exactYears;
            }
            diffYears = exactYears - years;
        } else {
            diffYears = exactYears;
        }

        //months
        double exactMonths = TimeConverter.convert(diffYears, TimeUnit.YEAR,
                TimeUnit.MONTH);
        double months = 0.0, diffMonths;
        if ((flags & FORMAT_MONTHS) != 0) {
            if ((flags & (FORMAT_WEEKS | FORMAT_DAYS | FORMAT_HOURS |
                    FORMAT_MINUTES | FORMAT_SECONDS | FORMAT_MILLISECONDS |
                    FORMAT_MICROSECONDS | FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                months = Math.floor(exactMonths);
            } else {
                months = exactMonths;
            }
            diffMonths = exactMonths - months;
        } else {
            diffMonths = exactMonths;
        }

        //weeks
        double exactWeeks = TimeConverter.convert(diffMonths, TimeUnit.MONTH,
                TimeUnit.WEEK);
        double weeks = 0.0, diffWeeks;
        if ((flags & FORMAT_WEEKS) != 0) {
            if ((flags & (FORMAT_DAYS | FORMAT_HOURS | FORMAT_MINUTES |
                    FORMAT_SECONDS | FORMAT_MILLISECONDS | FORMAT_MICROSECONDS |
                    FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                weeks = Math.floor(exactWeeks);
            } else {
                weeks = exactWeeks;
            }
            diffWeeks = exactWeeks - weeks;
        } else {
            diffWeeks = exactWeeks;
        }

        //days
        double exactDays = TimeConverter.convert(diffWeeks, TimeUnit.WEEK,
                TimeUnit.DAY);
        double days = 0.0, diffDays;
        if ((flags & FORMAT_DAYS) != 0) {
            if ((flags & (FORMAT_HOURS | FORMAT_MINUTES | FORMAT_SECONDS |
                    FORMAT_MILLISECONDS | FORMAT_MICROSECONDS |
                    FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                days = Math.floor(exactDays);
            } else {
                days = exactDays;
            }
            diffDays = exactDays - days;
        } else {
            diffDays = exactDays;
        }

        //hours
        double exactHours = TimeConverter.convert(diffDays, TimeUnit.DAY,
                TimeUnit.HOUR);
        double hours = 0.0, diffHours;
        if ((flags & FORMAT_HOURS) != 0) {
            if ((flags & (FORMAT_MINUTES | FORMAT_SECONDS | FORMAT_MILLISECONDS |
                    FORMAT_MICROSECONDS | FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                hours = Math.floor(exactHours);
            } else {
                hours = exactHours;
            }
            diffHours = exactHours - hours;
        } else {
            diffHours = exactHours;
        }

        //minutes
        double exactMinutes = TimeConverter.convert(diffHours, TimeUnit.HOUR,
                TimeUnit.MINUTE);
        double minutes = 0.0, diffMinutes;
        if ((flags & FORMAT_MINUTES) != 0) {
            if ((flags & (FORMAT_SECONDS | FORMAT_MILLISECONDS |
                    FORMAT_MICROSECONDS | FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                minutes = Math.floor(exactMinutes);
            } else {
                minutes = exactMinutes;
            }
            diffMinutes = exactMinutes - minutes;
        } else {
            diffMinutes = exactMinutes;
        }

        //seconds
        double exactSeconds = TimeConverter.convert(diffMinutes, TimeUnit.MINUTE,
                TimeUnit.SECOND);
        double seconds = 0.0, diffSeconds;
        if ((flags & FORMAT_SECONDS) != 0) {
            if ((flags & (FORMAT_MILLISECONDS | FORMAT_MICROSECONDS |
                    FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                seconds = Math.floor(exactSeconds);
            } else {
                seconds = exactSeconds;
            }
            diffSeconds = exactSeconds - seconds;
        } else {
            diffSeconds = exactSeconds;
        }

        //milliseconds
        double exactMilliseconds = TimeConverter.convert(diffSeconds,
                TimeUnit.SECOND, TimeUnit.MILLISECOND);
        double milliseconds = 0.0, diffMilliseconds;
        if ((flags & FORMAT_MILLISECONDS) != 0) {
            if ((flags & (FORMAT_MICROSECONDS | FORMAT_NANOSECONDS)) != 0) {
                //hay unidades más pequeñas
                milliseconds = Math.floor(exactMilliseconds);
            } else {
                milliseconds = exactMilliseconds;
            }
            diffMilliseconds = exactMilliseconds - milliseconds;
        } else {
            diffMilliseconds = exactMilliseconds;
        }

        //microseconds
        double exactMicroseconds = TimeConverter.convert(diffMilliseconds,
                TimeUnit.MILLISECOND, TimeUnit.MICROSECOND);
        double microseconds = 0.0, diffMicroseconds;
        if ((flags & FORMAT_MICROSECONDS) != 0) {
            if ((flags & (FORMAT_NANOSECONDS)) != 0) {
                microseconds = Math.floor(exactMicroseconds);
            } else {
                microseconds = exactMicroseconds;
            }
            diffMicroseconds = exactMicroseconds - microseconds;
        } else {
            diffMicroseconds = exactMicroseconds;
        }

        //nanoseconds
        double nanoseconds = 0.0;
        if ((flags & FORMAT_NANOSECONDS) != 0) {
            nanoseconds = TimeConverter.convert(diffMicroseconds,
                    TimeUnit.MICROSECOND, TimeUnit.NANOSECOND);
        }


        //format result
        StringBuilder builder = new StringBuilder();
        if ((flags & FORMAT_CENTURIES) != 0) {
            if(!onlyNonZero || centuries != 0.0) {
                builder.append(
                        format(centuries, TimeUnit.CENTURY));
            }
        }
        if ((flags & FORMAT_YEARS) != 0) {
            if (!onlyNonZero || years != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(years, TimeUnit.YEAR));
            }
        }
        if ((flags & FORMAT_MONTHS) != 0) {
            if (!onlyNonZero || months != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(months, TimeUnit.MONTH));
            }
        }
        if ((flags & FORMAT_WEEKS) != 0) {
            if (!onlyNonZero || weeks != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(weeks, TimeUnit.WEEK));
            }
        }
        if ((flags & FORMAT_DAYS) != 0) {
            if (!onlyNonZero || days != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(days, TimeUnit.DAY));
            }
        }
        if ((flags & FORMAT_HOURS) != 0) {
            if (!onlyNonZero || hours != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(hours, TimeUnit.HOUR));
            }
        }
        if ((flags & FORMAT_MINUTES) != 0) {
            if (!onlyNonZero || minutes != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(minutes, TimeUnit.MINUTE));
            }
        }
        if ((flags & FORMAT_SECONDS) != 0) {
            if (!onlyNonZero || seconds != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(seconds, TimeUnit.SECOND));
            }
        }
        if ((flags & FORMAT_MILLISECONDS) != 0) {
            if (!onlyNonZero || milliseconds != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(milliseconds, TimeUnit.MILLISECOND));
            }
        }
        if ((flags & FORMAT_MICROSECONDS) != 0) {
            if (!onlyNonZero || microseconds != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(microseconds, TimeUnit.MICROSECOND));
            }
        }
        if ((flags & FORMAT_NANOSECONDS) != 0) {
            if (!onlyNonZero || nanoseconds != 0.0) {
                appendSpaceIfNeeded(builder).append(
                        format(nanoseconds, TimeUnit.NANOSECOND));
            }
        }
        return builder.toString();
    }

    /**
     * Parses a string containing multiple units and returns the summation of all
     * found values as a single Time instance.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return summation of all time values that have been parsed.
     * @throws ParseException if parsing fails.
     * @throws UnknownUnitException if format is not recognized.
     */
    public Time parseMultiple(CharSequence source) throws ParseException,
            UnknownUnitException {
        String wrapped = " " + source + " ";
        Time result = null;

        Time firstCentury = parse1stCentury(wrapped);
        Time secondCentury = parse2ndCentury(wrapped);
        Time thirdCentury = parse3rdCentury(wrapped);
        Time century = parseCentury(wrapped);
        Time year = parseYear(wrapped);
        Time month = parseMonth(wrapped);
        Time week = parseWeek(wrapped);
        Time day = parseDay(wrapped);
        Time hour = parseHour(wrapped);
        Time minute = parseMinute(wrapped);
        Time second = parseSecond(wrapped);
        Time millisecond = parseMillisecond(wrapped);
        Time microsecond = parseMicrosecond(wrapped);
        Time nanosecond = parseNanosecond(wrapped);

        //century
        if (firstCentury != null) {
            result = firstCentury;
        }
        if (secondCentury != null) {
            result = secondCentury;
        }
        if (thirdCentury != null) {
            result = thirdCentury;
        }
        if (century != null) {
            result = century;
        }

        //year
        if (year != null) {
            if (result == null) {
                result = year;
            } else {
                result.add(year);
            }
        }

        //month
        if (month != null) {
            if (result == null) {
                result = month;
            } else {
                result.add(month);
            }
        }

        //week
        if (week != null) {
            if (result == null) {
                result = week;
            } else {
                result.add(week);
            }
        }

        //day
        if (day != null) {
            if (result == null) {
                result = day;
            } else {
                result.add(day);
            }
        }

        //hour
        if (hour != null) {
            if (result == null) {
                result = hour;
            } else {
                result.add(hour);
            }
        }

        //minute
        if (minute != null) {
            if (result == null) {
                result = minute;
            } else {
                result.add(minute);
            }
        }

        //second
        if (second != null) {
            if (result == null) {
                result = second;
            } else {
                result.add(second);
            }
        }

        //millisecond
        if (millisecond != null) {
            if (result == null) {
                result = millisecond;
            } else {
                result.add(millisecond);
            }
        }

        //microsecond
        if (microsecond != null) {
            if (result == null) {
                result = microsecond;
            } else {
                result.add(microsecond);
            }
        }

        //nanosecond
        if (nanosecond != null) {
            if (result == null) {
                result = nanosecond;
            } else {
                result.add(nanosecond);
            }
        }

        if (result == null) {
            //no valid unit was found
            throw new UnknownUnitException();
        }

        return result;
    }

    /**
     * Parses string as 1st century format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in century units.
     * @throws ParseException if parsing fails.
     */
    private Time parse1stCentury(CharSequence source) throws ParseException {
        if (mFirstCenturyPattern == null) {
            mFirstCenturyPattern = Pattern.compile(FIRST_CENTURY_PATTERN);
        }

        Matcher matcher = mFirstCenturyPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.CENTURY);
    }

    /**
     * Parses string as 2nd century format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in century units.
     * @throws ParseException if parsing fails.
     */
    private Time parse2ndCentury(CharSequence source) throws ParseException {
        if (mSecondCenturyPattern == null) {
            mSecondCenturyPattern = Pattern.compile(SECOND_CENTURY_PATTERN);
        }

        Matcher matcher = mSecondCenturyPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.CENTURY);
    }

    /**
     * Parses string as 3rd century format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in century units.
     * @throws ParseException if parsing fails.
     */
    private Time parse3rdCentury(CharSequence source) throws ParseException {
        if (mThirdCenturyPattern == null) {
            mThirdCenturyPattern = Pattern.compile(THIRD_CENTURY_PATTERN);
        }

        Matcher matcher = mThirdCenturyPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.CENTURY);
    }

    /**
     * Parses string as n-th century format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in century units.
     * @throws ParseException if parsing fails.
     */
    private Time parseCentury(CharSequence source) throws ParseException {
        if (mCenturyPattern == null) {
            mCenturyPattern = Pattern.compile(CENTURY_PATTERN);
        }

        Matcher matcher = mCenturyPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.CENTURY);
    }

    /**
     * Parses string as year format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in year units.
     * @throws ParseException if parsing fails.
     */
    private Time parseYear(CharSequence source) throws ParseException {
        if (mYearPattern == null) {
            mYearPattern = Pattern.compile(YEAR_PATTERN);
        }

        Matcher matcher = mYearPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.YEAR);
    }

    /**
     * Parses string as month format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in month units.
     * @throws ParseException if parsing fails.
     */
    private Time parseMonth(CharSequence source) throws ParseException {
        if (mMonthPattern == null) {
            mMonthPattern = Pattern.compile(MONTH_PATTERN);
        }

        Matcher matcher = mMonthPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.MONTH);
    }

    /**
     * Parses string as week format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in week units.
     * @throws ParseException if parsing fails.
     */
    private Time parseWeek(CharSequence source) throws ParseException {
        if (mWeekPattern == null) {
            mWeekPattern = Pattern.compile(WEEK_PATTERN);
        }

        Matcher matcher = mWeekPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.WEEK);
    }

    /**
     * Parses string as day format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in day units.
     * @throws ParseException if parsing fails.
     */
    private Time parseDay(CharSequence source) throws ParseException {
        if (mDayPattern == null) {
            mDayPattern = Pattern.compile(DAY_PATTERN);
        }

        Matcher matcher = mDayPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.DAY);
    }

    /**
     * Parses string as hour format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in hour units.
     * @throws ParseException if parsing fails.
     */
    private Time parseHour(CharSequence source) throws ParseException {
        if (mHourPattern == null) {
            mHourPattern = Pattern.compile(HOUR_PATTERN);
        }

        Matcher matcher = mHourPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.HOUR);
    }

    /**
     * Parses string as minute format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in minute units.
     * @throws ParseException if parsing fails.
     */
    private Time parseMinute(CharSequence source) throws ParseException {
        if (mMinutePattern == null) {
            mMinutePattern = Pattern.compile(MINUTE_PATTERN);
        }

        Matcher matcher = mMinutePattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.MINUTE);
    }

    /**
     * Parses string as second format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in second units.
     * @throws ParseException if parsing fails.
     */
    private Time parseSecond(CharSequence source) throws ParseException {
        if (mSecondPattern == null) {
            mSecondPattern = Pattern.compile(SECOND_PATTERN);
        }

        Matcher matcher = mSecondPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)), TimeUnit.SECOND);
    }

    /**
     * Parses string as millisecond format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in millisecond units.
     * @throws ParseException if parsing fails.
     */
    private Time parseMillisecond(CharSequence source) throws ParseException {
        if (mMillisecondPattern == null) {
            mMillisecondPattern = Pattern.compile(MILLISECOND_PATTERN);
        }

        Matcher matcher = mMillisecondPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)),
                TimeUnit.MILLISECOND);
    }

    /**
     * Parses string as microsecond format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in microsecond units.
     * @throws ParseException if parsing fails.
     */
    private Time parseMicrosecond(CharSequence source) throws ParseException {
        if (mMicrosecondPattern == null) {
            mMicrosecondPattern = Pattern.compile(MICROSECOND_PATTERN);
        }

        Matcher matcher = mMicrosecondPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)),
                TimeUnit.MICROSECOND);
    }

    /**
     * Parses string as nanosecond format.
     * This method does not allow decimal values with fractions or thousand
     * separators.
     * @param source string to be parsed.
     * @return parsed time in nanosecond units.
     * @throws ParseException if parsing fails.
     */
    private Time parseNanosecond(CharSequence source) throws ParseException {
        if (mNanosecondPattern == null) {
            mNanosecondPattern = Pattern.compile(NANOSECOND_PATTERN);
        }

        Matcher matcher = mNanosecondPattern.matcher(source);
        if (!matcher.matches()) {
            return null;
        }

        return new Time(mNumberFormat.parse(matcher.group(3)),
                TimeUnit.NANOSECOND);
    }

    /**
     * Appends a space if builder is not empty.
     * @param builder builder to add space to.
     * @return same instance provided as parameter.
     */
    private StringBuilder appendSpaceIfNeeded(StringBuilder builder) {
        if (builder.length() > 0) {
            builder.append(SPACE);
        }
        return builder;
    }
}
