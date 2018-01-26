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
import java.math.RoundingMode;
import java.text.FieldPosition;
import java.text.MessageFormat;
import java.text.NumberFormat;
import java.text.ParseException;
import java.util.Locale;

/**
 * Base class to format and parse a given measure using its value and unit.
 * @param <M> type of measurement (i.e. Distance or Surface).
 * @param <U> type of unit (i.e. DistanceUnit or SurfaceUnit).
 */
public abstract class MeasureFormatter<M extends Measurement, U extends Enum> implements Cloneable {

    /**
     * Default pattern to format values and units together into a single string.
     * {0} corresponds to the value, {1} corresponds to the unit part.
     */
    public static final String DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN =
            "{0} {1}";

    /**
     * Internal string formatter.
     */
    private NumberFormat mNumberFormat;

    /**
     * Internal string formatter.
     */
    private MessageFormat mFormat;

    /**
     * Internal locale.
     */
    private Locale mLocale;

    /**
     * Pattern to format values and unit together into a single string. {0} corresponds to
     * the vlaue, {1} corresponds to the unit part.
     */
    private String mValueAndUnitFormatPattern;

    /**
     * Constructor.
     */
    MeasureFormatter() {
        mNumberFormat = NumberFormat.getInstance();
        mLocale = Locale.getDefault();
        mValueAndUnitFormatPattern = DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN;
    }

    /**
     * Constructor with locale.
     * @param locale locale.
     * @throws IllegalArgumentException if locale is null.
     */
    MeasureFormatter(Locale locale) throws IllegalArgumentException {
        if (locale == null) {
            throw new IllegalArgumentException();
        }

        mNumberFormat = NumberFormat.getInstance(locale);
        mLocale = locale;
        mValueAndUnitFormatPattern = DEFAULT_VALUE_AND_UNIT_FORMAT_PATTERN;
    }

    /**
     * Clones this measure formatter.
     * @return a copy of this unit formatter.
     */
    @Override
    public abstract Object clone();

    /**
     * Determines if two measure formatters are equal by comparing all of its internal
     * parameters.
     * @param obj another object to compare.
     * @return true if provided object is assumed to be equal to this instance.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (!(obj instanceof MeasureFormatter)) {
            return false;
        }
        if (this == obj) {
            return true;
        }

        MeasureFormatter other = (MeasureFormatter) obj;
        return mNumberFormat.equals(other.mNumberFormat) &&
                mLocale.equals(other.mLocale) &&
                mValueAndUnitFormatPattern.equals(other.mValueAndUnitFormatPattern);
    }

    /**
     * Hash code generated for this instance.
     * Hash codes can be internally used by some collections to coarsely compare objects.
     * @return hash code.
     */
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 19 * hash + (mNumberFormat != null ?
                mNumberFormat.hashCode() : 0);
        hash = 19 * hash + (mFormat != null ? mFormat.hashCode() : 0);
        hash = 19 * hash + (mLocale != null ? mLocale.hashCode() : 0);
        hash = 19 * hash + (mValueAndUnitFormatPattern != null ?
                mValueAndUnitFormatPattern.hashCode() : 0);
        return hash;
    }

    /**
     * Formats provided measurement value and unit into a string representation.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return string representation of provided measurement value and unit.
     */
    public String format(Number value, U unit) {
        return MessageFormat.format(mValueAndUnitFormatPattern,
                mNumberFormat.format(value), getUnitSymbol(unit));
    }

    /**
     * Formats provided measurement value and unit into a string representation
     * and appends the result into provided string buffer.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @param toAppendTo buffer to append the result to.
     * @param pos field position where result will be appended.
     * @return provided string buffer where result is appended.
     */
    public StringBuffer format(Number value, U unit,
                               StringBuffer toAppendTo, FieldPosition pos) {
        if (mFormat == null) {
            mFormat = new MessageFormat(mValueAndUnitFormatPattern);
        }
        return mFormat.format(new Object[]{mNumberFormat.format(value),
                getUnitSymbol(unit)}, toAppendTo, pos);
    }

    /**
     * Formats provided measurement value and unit into a string representation.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return string representation of provided measurement value and unit.
     */
    public String format(double value, U unit) {
        return format(new BigDecimal(value), unit);
    }

    /**
     * Formats provided measurement value and unit into a string representation
     * and appends the result into provided string buffer.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @param toAppendTo buffer to append the result to.
     * @param pos field position where result will be appended.
     * @return provided string buffer where result is appended.
     */
    public StringBuffer format(double value, U unit,
                               StringBuffer toAppendTo, FieldPosition pos) {
        return format(new BigDecimal(value), unit, toAppendTo, pos);
    }

    /**
     * Formats provided measurement into a string representation.
     * @param measurement a measurement.
     * @return string representation of provided measurement.
     */
    @SuppressWarnings("unchecked")
    public String format(M measurement) {
        return format(measurement.getValue(), (U)measurement.getUnit());
    }

    /**
     * Formats provided measurement into a string representation and appends the
     * result into provided string buffer.
     * @param measurement a measurement.
     * @param toAppendTo buffer to append the result to.
     * @param pos field position where result will be appended.
     * @return provided string buffer where result is appended.
     */
    @SuppressWarnings("unchecked")
    public StringBuffer format(M measurement, StringBuffer toAppendTo,
                               FieldPosition pos) {
        return format(measurement.getValue(), (U)measurement.getUnit(),
                toAppendTo, pos);
    }

    /**
     * Formats and converts provided measurement value and unit using unit system
     * assigned to locale of this instance (if no locale has been provided it
     * is assumed that the system default locale is used).
     * If provided value is too large for provided unit, this method will
     * convert it to a more appropriate unit.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return a string representation of measurement value and unit.
     */
    public String formatAndConvert(Number value, U unit) {
        return formatAndConvert(value, unit, getUnitSystem());
    }

    /**
     * Formats and converts provided measurement value and unit using unit system
     * assigned to locale of this instance (if no locale has been provided it
     * is assumed that the system default locale is used).
     * If provided value is too large for provided unit, this method will
     * convert it to a more appropriate unit.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return a string representation of measurement value and unit.
     */
    public String formatAndConvert(double value, U unit) {
        return formatAndConvert(new BigDecimal(value), unit);
    }

    /**
     * Formats and converts provided measurement using unit system assigned to
     * locale of this instance (if no locale has been provided it is assumed
     * that the system default locale is used).
     * If provided measurement value is too large for its unit, this method
     * will convert it to a more appropriate unit.
     * @param measurement measurement to be formatted.
     * @return a string representation of measurement value and unit.
     */
    @SuppressWarnings("unchecked")
    public String formatAndConvert(M measurement) {
        return formatAndConvert(measurement.getValue(),
                (U)measurement.getUnit());
    }

    /**
     * Formats and converts provided measurement vlaue and unit using provided
     * unit system.
     * If provided value is too large for provided unit, this method will
     * convert it to a more appropriate unit using provided unit system (either
     * metric or imperial).
     * @param value a measurment value.
     * @param unit a measurement unit.
     * @param system system unit to convert measurement to.
     * @return a string representation of measurement value and unit.
     */
    public String formatAndConvert(Number value, U unit, UnitSystem system) {
        switch (system) {
            case IMPERIAL:
                return formatAndConvertImperial(value, unit);
            case METRIC:
            default:
                return formatAndConvertMetric(value, unit);
        }
    }

    /**
     * Formats and converts provided measurement value and unit using provided
     * unit system.
     * If provided value is too large for provided unit, this method will
     * convert it to a more appropriate unit using provided unit system (either
     * metric or imperial).
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @param system system unit to convert measurement to.
     * @return a string representation of measurement value and unit.
     */
    public String formatAndConvert(double value, U unit, UnitSystem system) {
        return formatAndConvert(new BigDecimal(value), unit, system);
    }

    /**
     * Formats and converts provided measurement using provided unit system.
     * If provided measurement value is too large for its unit, this method
     * will convert it to a more appropriate unit using provided unit
     * system.
     * @param measurement a measurement to be formatted.
     * @param unitSystem system unit to convert measurement to.
     * @return a string representation of measurement value and unit.
     */
    @SuppressWarnings("unchecked")
    public String formatAndConvert(M measurement, UnitSystem unitSystem) {
        return formatAndConvert(measurement.getValue(),
                (U)measurement.getUnit(), unitSystem);
    }

    /**
     * Formats and converts provided measurement value and unit using
     * metric unit system.
     * If provided measurement value is too large for its unit, this method
     * will convert it to a more appropriate unit.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return a string representation of measurement value and unit using
     * metric unit system.
     */
    public abstract String formatAndConvertMetric(Number value, U unit);

    /**
     * Formats and converts provided measurement value and unit using
     * imperial unit system.
     * If provided measurement value is too large for its unit, this method
     * will convert it to a more appropriate unit.
     * @param value a measurement value.
     * @param unit a measurement unit.
     * @return a string representation of measurement value and unit using
     * imperial unit system.
     */
    public abstract String formatAndConvertImperial(Number value, U unit);

    /**
     * Returns available locales for this formatter.
     * @return available locales.
     */
    public static Locale[] getAvailableLocales() {
        return NumberFormat.getAvailableLocales();
    }

    /**
     * Gets locale assigned to this instance.
     * Locale determines number format and unit system (metric or imperial)
     * if not specified.
     * @return a locale.
     */
    public Locale getLocale() {
        return mLocale;
    }

    /**
     * Returns maximum fraction digits to be shown when formatting a measure.
     * @return maximum fraction digits.
     */
    public int getMaximumFractionDigits() {
        return mNumberFormat.getMaximumFractionDigits();
    }

    /**
     * Sets maximum fraction digits to use when formatting a measure.
     * @param newValue maximum fraction digits to be set.
     */
    public void setMaximumFractionDigits(int newValue) {
        mNumberFormat.setMaximumFractionDigits(newValue);
    }

    /**
     * Returns maximum integer digits to be shown when formatting a measure.
     * @return maximum integer digits.
     */
    public int getMaximumIntegerDigits() {
        return mNumberFormat.getMaximumIntegerDigits();
    }

    /**
     * Sets maximum integer digits to use when formatting a measure.
     * @param newValue maximum integer digits to be set.
     */
    public void setMaximumIntegerDigits(int newValue) {
        mNumberFormat.setMaximumIntegerDigits(newValue);
    }

    /**
     * Returns minimum fraction digits to be shown when formatting a measure.
     * @return minimum fraction digits.
     */
    public int getMinimumFractionDigits() {
        return mNumberFormat.getMinimumFractionDigits();
    }

    /**
     * Sets minimum fraction digits to use when formatting a measure.
     * @param newValue minimum fraction digits to be set.
     */
    public void setMinimumFractionDigits(int newValue) {
        mNumberFormat.setMinimumFractionDigits(newValue);
    }

    /**
     * Returns minimum integer digits to be shown when formatting a measure.
     * @return minimum integer digits.
     */
    public int getMinimumIntegerDigits() {
        return mNumberFormat.getMinimumIntegerDigits();
    }

    /**
     * Sets minimum integer digits to use when formatting a measure.
     * @param newValue minimum integer digits to be set.
     */
    public void setMinimumIntegerDigits(int newValue) {
        mNumberFormat.setMinimumIntegerDigits(newValue);
    }

    /**
     * Returns rounding mode to be used when formatting a measure.
     * @return rounding mode to be used when formatting a measure.
     */
    public RoundingMode getRoundingMode() {
        return mNumberFormat.getRoundingMode();
    }

    /**
     * Specifies rounding mode to use when formatting a measure.
     * @param roundingMode rounding mode to be set.
     */
    public void setRoundingMode(RoundingMode roundingMode) {
        mNumberFormat.setRoundingMode(roundingMode);
    }

    /**
     * Indicates if grouping is used when formatting a measure.
     * @return true if grouping is used, false otherwise.
     */
    public boolean isGroupingUsed() {
        return mNumberFormat.isGroupingUsed();
    }

    /**
     * Sets if grouping is used when formatting a measure.
     * @param newValue true if grouping is enabled, false otherwise.
     */
    public void setGroupingUsed(boolean newValue) {
        mNumberFormat.setGroupingUsed(newValue);
    }

    /**
     * Indicates if only integer values are parsed.
     * @return true if only integer values are parsed, false otherwise.
     */
    public boolean isParseIntegerOnly() {
        return mNumberFormat.isParseIntegerOnly();
    }

    /**
     * Specifies whether only integer values are parsed or not.
     * @param value if true only integer values will be parsed.
     */
    public void setParseIntegerOnly(boolean value) {
        mNumberFormat.setParseIntegerOnly(value);
    }

    /**
     * Obtains pattern to format values and unit together into a single string.
     * {0} corresponds to the value, {1} corresponds to the unit part.
     * @return pattern to format values and unit together.
     */
    public String getValueAndUnitFormatPattern() {
        return mValueAndUnitFormatPattern;
    }

    /**
     * Sets pattern to format values and unit together into a single string.
     * {0} corresponds to the value, {1} corresponds to the unit part.
     * @param valueAndUnitFormatPattern pattern to format values and unit
     * together.
     * @throws IllegalArgumentException if provided pattern is null.
     */
    public void setValueAndUnitFormatPattern(String valueAndUnitFormatPattern)
            throws IllegalArgumentException {
        if (valueAndUnitFormatPattern == null) {
            throw new IllegalArgumentException();
        }
        mValueAndUnitFormatPattern = valueAndUnitFormatPattern;
    }

    /**
     * Returns unit system this instance will use based on its assigned locale.
     * Notice that if no locale was assigned, then the default system locale
     * will be used.
     * @return unit system this instance will use.
     */
    public UnitSystem getUnitSystem() {
        return UnitLocale.getFrom(mLocale);
    }

    /**
     * Indicates whether provided string representation contains a valid
     * measurement unit or not.
     * @param source a string measurement representation to be checked.
     * @return true if provided string has a valid (i.e. recognized) unit, false
     * otherwise.
     */
    public boolean isValidUnit(String source) {
        return findUnit(source) != null;
    }

    /**
     * Indicates whether provided string representation is a valid measurement
     * representation or not.
     * @param source a string measurement representation to be checked.
     * @return true if provided string representation is valid (contains valid
     * value and unit), false otherwise.
     */
    public boolean isValidMeasurement(String source) {
        try {
            mNumberFormat.parse(source);
            return isValidUnit(source);
        } catch (ParseException e) {
            return false;
        }
    }

    /**
     * Indicates whether provided string representation of a measurement
     * contains a metric system unit.
     * @param source a measurement string representation to be checked.
     * @return true if found unit is metric, false otherwise or if unit
     * cannot be determined.
     */
    public boolean isMetricUnit(String source) {
        return getUnitSystem(source) == UnitSystem.METRIC;
    }

    /**
     * Indicates whether provided string representation of a measurement
     * contains an imperial system unit.
     * @param source a measurement string representation to be checked.
     * @return true if found unit is imperial, false otherwise or if unit
     * cannot be determined.
     */
    public boolean isImperialUnit(String source) {
        return getUnitSystem(source) == UnitSystem.IMPERIAL;
    }

    /**
     * Gets unit system for detected unit into provided string representation
     * of a measurement.
     * @param source a measurement string representation to be checked.
     * @return a unit system (either metric or imperial) or null if unit
     * cannot be determined.
     */
    public abstract UnitSystem getUnitSystem(String source);

    /**
     * Parses a string into a measure.
     * @param source text to be parsed.
     * @return a measure containing measure value and unit obtained from parsed
     * text.
     * @throws ParseException if parsing failed.
     * @throws UnknownUnitException if unit cannot be determined.
     */
    public abstract M parse(String source) throws ParseException, UnknownUnitException;

    /**
     * Finds measure unit from within a measurement string representation.
     * @param source a measurement string representation.
     * @return a measure unit or null if none can be determined.
     */
    public abstract U findUnit(String source);

    /**
     * Obtains measure unit symbol.
     * @param unit a measure unit.
     * @return measure unit symbol.
     */
    public abstract String getUnitSymbol(U unit);

    /**
     * Internal method to parse a string into a measure.
     * @param source text to be parsed.
     * @param measure a measure to be initialized with parsed contents.
     * @return provided measure.
     * @throws ParseException if parsing failed.
     * @throws UnknownUnitException if unit cannot be determined.
     */
    @SuppressWarnings("unchecked")
    M internalParse(String source, M measure) throws ParseException,
            UnknownUnitException {
        measure.setValue(mNumberFormat.parse(source));
        try {
            measure.setUnit(findUnit(source));
        } catch (IllegalArgumentException e) {
            throw new UnknownUnitException(e);
        }
        return measure;
    }

    /**
     * Internal method to clone this measure formatter.
     * @param copy an instantiated copy of a measure formatter that needs to be initialized.
     * @return provided copy.
     */
    MeasureFormatter<M, U> internalClone(MeasureFormatter<M, U> copy) {
        copy.mNumberFormat = (NumberFormat)mNumberFormat.clone();
        if (mFormat != null) {
            copy.mFormat = (MessageFormat)mFormat.clone();
        }
        if (mLocale != null) {
            copy.mLocale = (Locale)mLocale.clone();
        }
        return copy;
    }
}
