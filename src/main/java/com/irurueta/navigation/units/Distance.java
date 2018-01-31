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

/**
 * Contains a distance value and unit.
 */
public class Distance extends Measurement<DistanceUnit> {

    /**
     * Constructor with value and unit.
     * @param value distance value.
     * @param unit unit of distance.
     * @throws IllegalArgumentException if either value or unit is null.
     */
    public Distance(Number value, DistanceUnit unit) throws IllegalArgumentException {
        super(value, unit);
    }

    /**
     * Constructor.
     */
    Distance() {
        super();
    }

    /**
     * Determines if two distances are equal up to a certain tolerance.
     * If needed, this method attempts unit conversion to compare both objects.
     * @param other another distance to compare.
     * @param tolerance true if provided distance is assumed to be equal to this
     *                  instance up to provided tolerance.
     * @return true if provided distance is assumed to be equal to this instance,
     * false otherwise.
     */
    @Override
    public boolean equals(Measurement<DistanceUnit> other,
                          double tolerance) {
        if(super.equals(other, tolerance)) {
            return true;
        }

        //attempt conversion to common units
        if (other == null) {
            return false;
        }

        double otherValue = DistanceConverter.convert(
                other.getValue().doubleValue(), other.getUnit(),
                getUnit());
        return (Math.abs(getValue().doubleValue() - otherValue)) <= tolerance;
    }

    /**
     * Adds two distance values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static double add(double value1, DistanceUnit unit1,
                             double value2, DistanceUnit unit2,
                             DistanceUnit resultUnit) {
        double v1 = DistanceConverter.convert(value1, unit1, resultUnit);
        double v2 = DistanceConverter.convert(value2, unit2, resultUnit);
        return v1 + v2;
    }

    /**
     * Adds two distance values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static Number add(Number value1, DistanceUnit unit1,
                             Number value2, DistanceUnit unit2,
                             DistanceUnit resultUnit) {
        return new BigDecimal(add(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Adds two distances and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void add(Distance arg1, Distance arg2, Distance result) {
        result.setValue(add(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Adds two distances.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned distance.
     * @return a new instance containing result.
     */
    public static Distance addAndReturnNew(Distance arg1, Distance arg2,
                                           DistanceUnit unit) {
        Distance result = new Distance();
        result.setUnit(unit);
        add(arg1, arg2, result);
        return result;
    }

    /**
     * Adds provided distance value and unit and returns a new distance instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned distance.
     * @return a new distance containing result.
     */
    public Distance addAndReturnNew(double value, DistanceUnit unit, DistanceUnit resultUnit) {
        Distance result = new Distance();
        result.setUnit(resultUnit);
        result.setValue(add(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Adds provided distance value and unit and returns a new distance instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned distance.
     * @return a new distance containing result.
     */
    public Distance addAndReturnNew(Number value, DistanceUnit unit, DistanceUnit resultUnit) {
        Distance result = new Distance();
        result.setUnit(resultUnit);
        result.setValue(add(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Adds provided distance to current instance and returns a new
     * distance.
     * @param d distance to be added.
     * @param unit unit of returned distance.
     * @return a new distance containing result.
     */
    public Distance addAndReturnNew(Distance d, DistanceUnit unit) {
        return addAndReturnNew(this, d, unit);
    }

    /**
     * Adds provided distance value and unit and updates current distance instance.
     * @param value distance value to be added.
     * @param unit unit of distance value.
     */
    public void add(double value, DistanceUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided distance value and unit and updates current distance instance.
     * @param value distance value to be added.
     * @param unit unit of distance value.
     */
    public void add(Number value, DistanceUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided distance and updates current distance.
     * @param distance distance to be added.
     */
    public void add(Distance distance) {
        add(this, distance, this);
    }

    /**
     * Adds provided distance and stores the result into provided
     * distance.
     * @param d distance to be added.
     * @param result instance where result will be stored.
     */
    public void add(Distance d, Distance result) {
        add(this, d, result);
    }

    /**
     * Subtracts two distance values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static double subtract(double value1, DistanceUnit unit1,
                                  double value2, DistanceUnit unit2,
                                  DistanceUnit resultUnit) {
        double v1 = DistanceConverter.convert(value1, unit1, resultUnit);
        double v2 = DistanceConverter.convert(value2, unit2, resultUnit);
        return v1 - v2;
    }

    /**
     * Subtracts two distance value and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static Number subtract(Number value1, DistanceUnit unit1,
                                  Number value2, DistanceUnit unit2,
                                  DistanceUnit resultUnit) {
        return new BigDecimal(subtract(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Subtracts two distances and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void subtract(Distance arg1, Distance arg2, Distance result) {
        result.setValue(subtract(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Subtracts two distances.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned distance.
     * @return a new instance containing result.
     */
    public static Distance subtractAndReturnNew(Distance arg1, Distance arg2,
                                                DistanceUnit unit) {
        Distance result = new Distance();
        result.setUnit(unit);
        subtract(arg1, arg2, result);
        return result;
    }

    /**
     * Subtracts provided distance value and unit and returns a new distance instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned distance.
     * @return a new distance containing result.
     */
    public Distance subtractAndReturnNew(double value, DistanceUnit unit, DistanceUnit resultUnit) {
        Distance result = new Distance();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Subtracts provided distance value and unit and returns a new distance instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned time.
     * @return a new distance containing result.
     */
    public Distance subtractAndReturnNew(Number value, DistanceUnit unit, DistanceUnit resultUnit) {
        Distance result = new Distance();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Subtracts provided distance to current instance and returns a new
     * distance.
     * @param d distance to be subtracted.
     * @param unit unit of returned distance.
     * @return a new distance containing result.
     */
    public Distance subtractAndReturnNew(Distance d, DistanceUnit unit) {
        return subtractAndReturnNew(this, d, unit);
    }

    /**
     * Subtracts provided distance value and unit and updates current distance instance.
     * @param value distance value to be subtracted.
     * @param unit unit of distance value.
     */
    public void subtract(double value, DistanceUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtracts provided distance value and unit and updates current distance instance.
     * @param value distance value to be subtracted.
     * @param unit unit of distance value.
     */
    public void subtract(Number value, DistanceUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtracts provided distance and updates current distance.
     * @param distance distance to be subtracted.
     */
    public void subtract(Distance distance) {
        subtract(this, distance, this);
    }

    /**
     * Subtracts provided distance and stores the result into provided
     * distance.
     * @param d distance to be subtracted.
     * @param result instance where result will be stored.
     */
    public void subtract(Distance d, Distance result) {
        subtract(this, d, result);
    }
}
