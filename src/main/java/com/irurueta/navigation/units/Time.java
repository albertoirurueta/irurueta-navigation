package com.irurueta.navigation.units;

import java.math.BigDecimal;

/**
 * Contains a time value and unit.
 */
public class Time extends Measurement<TimeUnit> {

    /**
     * Constructor with value and unit.
     * @param value time value.
     * @param unit unit of distance.
     * @throws IllegalArgumentException if either value or unit is null.
     */
    @SuppressWarnings("WeakerAccess")
    public Time(Number value, TimeUnit unit) throws IllegalArgumentException {
        super(value, unit);
    }

    /**
     * Constructor.
     */
    Time() {
        super();
    }

    /**
     * Determines if two time instances are equal up to a certain tolerance.
     * If needed, this method attempts unit conversion to compare both objects.
     * @param other another time to compare.
     * @param tolerance true if provided time is assumed to be equal to this
     *                  instance up to provided tolerance.
     * @return true if provided time is assumed to be equal to this instance, false
     * otherwise.
     */
    @Override
    public boolean equals(Measurement<TimeUnit> other,
                          double tolerance) {
        if (super.equals(other, tolerance)) {
            return true;
        }

        //attempt conversion to common units
        if (other == null) {
            return false;
        }

        double otherValue = TimeConverter.convert(
                other.getValue().doubleValue(), other.getUnit(),
                getUnit());
        return (Math.abs(getValue().doubleValue() - otherValue)) <= tolerance;
    }

    /**
     * Adds two time values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static double add(double value1, TimeUnit unit1,
                             double value2, TimeUnit unit2,
                             TimeUnit resultUnit) {
        double v1 = TimeConverter.convert(value1, unit1, resultUnit);
        double v2 = TimeConverter.convert(value2, unit2, resultUnit);
        return v1 + v2;
    }

    /**
     * Adds two time values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static Number add(Number value1, TimeUnit unit1,
                           Number value2, TimeUnit unit2,
                           TimeUnit resultUnit) {
        return new BigDecimal(add(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Adds two time instances and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void add(Time arg1, Time arg2, Time result) {
        result.setValue(add(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Adds two time instances.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned time.
     * @return a new instance containing result.
     */
    public static Time addAndReturnNew(Time arg1, Time arg2, TimeUnit unit) {
        Time result = new Time();
        result.setUnit(unit);
        add(arg1, arg2, result);
        return result;
    }

    /**
     * Adds provided time value and unit and returns a new time instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned time.
     * @return a new time containing result.
     */
    public Time addAndReturnNew(double value, TimeUnit unit, TimeUnit resultUnit) {
        Time result = new Time();
        result.setUnit(resultUnit);
        result.setValue(add(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Adds provided time value and unit and returns a new time instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned time.
     * @return a new time containing result.
     */
    public Time addAndReturnNew(Number value, TimeUnit unit, TimeUnit resultUnit) {
        Time result = new Time();
        result.setUnit(resultUnit);
        result.setValue(add(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Adds provided time to current instance and returns a new time
     * instance.
     * @param t time to be added.
     * @param unit unit of returned time.
     * @return a new time containing result.
     */
    public Time addAndReturnNew(Time t, TimeUnit unit) {
        return addAndReturnNew(this, t, unit);
    }

    /**
     * Adds provided time value and unit and updates current time instance.
     * @param value time value to be added.
     * @param unit unit of time value.
     */
    public void add(double value, TimeUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided time value and unit and updates current time instance.
     * @param value time value to be added.
     * @param unit unit of time value.
     */
    public void add(Number value, TimeUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided time and updates current time instance.
     * @param time time to be added.
     */
    public void add(Time time) {
        add(this, time, this);
    }

    /**
     * Adds provided time and stores the result into provided time instance.
     * @param t time to be added.
     * @param result instance where result will be stored.
     */
    public void add(Time t, Time result) {
        add(this, t, result);
    }

    /**
     * Subtracts two time values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static double subtract(double value1, TimeUnit unit1,
                                  double value2, TimeUnit unit2,
                                  TimeUnit resultUnit) {
        double v1 = TimeConverter.convert(value1, unit1, resultUnit);
        double v2 = TimeConverter.convert(value2, unit2, resultUnit);
        return v1 - v2;
    }

    /**
     * Subtracts two time values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static Number subtract(Number value1, TimeUnit unit1,
                                  Number value2, TimeUnit unit2,
                                  TimeUnit resultUnit) {
        return new BigDecimal(subtract(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Subtracts two time instances and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void subtract(Time arg1, Time arg2, Time result) {
        result.setValue(subtract(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Subtracts two time instances.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned time.
     * @return a new instance containing result.
     */
    public static Time subtractAndReturnNew(Time arg1, Time arg2,
                                            TimeUnit unit) {
        Time result = new Time();
        result.setUnit(unit);
        subtract(arg1, arg2, result);
        return result;
    }

    /**
     * Subtracts provided time value and unit and returns a new time instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned time.
     * @return a new time containing result.
     */
    public Time subtractAndReturnNew(double value, TimeUnit unit, TimeUnit resultUnit) {
        Time result = new Time();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Subtracts provided time value and unit and returns a new time instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned time.
     * @return a new time containing result.
     */
    public Time subtractAndReturnNew(Number value, TimeUnit unit, TimeUnit resultUnit) {
        Time result = new Time();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Subtracts provided time to current instance and returns a new
     * instance.
     * @param t time to be subtracted.
     * @param unit unit of returned time.
     * @return a new time containing result.
     */
    public Time subtractAndReturnNew(Time t, TimeUnit unit) {
        return subtractAndReturnNew(this, t, unit);
    }

    /**
     * Subtracts provided time value and unit and updates current time instance.
     * @param value time value to be subtracted.
     * @param unit unit of time value.
     */
    public void subtract(double value, TimeUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtracts provided time value and unit and updates current time instance.
     * @param value time value to be subtracted.
     * @param unit unit of time value.
     */
    public void subtract(Number value, TimeUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtract provided time and updates current time.
     * @param time time to be subtracted.
     */
    public void subtract(Time time) {
        subtract(this, time, this);
    }

    /**
     * Subtracts provided time and stores the result into provided time.
     * @param t time to be subtracted.
     * @param result instance where result will be stored.
     */
    public void subtract(Time t, Time result) {
        subtract(this, t, result);
    }
}
