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
 * Contains a surface value and unit.
 */
public class Surface extends Measurement<SurfaceUnit> {

    /**
     * Constructor with value and units.
     * @param value surface value.
     * @param unit unit of surface.
     * @throws IllegalArgumentException if either value or unit is null.
     */
    @SuppressWarnings("WeakerAccess")
    public Surface(Number value, SurfaceUnit unit) throws IllegalArgumentException {
        super(value, unit);
    }

    /**
     * Constructor.
     */
    Surface() {
        super();
    }

    /**
     * Determines if two surfaces are equal up to a certain tolerance.
     * If needed, this method attempts unit conversion to compare both objects.
     * @param other another surface to compare.
     * @param tolerance true if provided surface is assumed to be equal to this
     *                  instance up to provided tolerance.
     * @return true if provided surface is assumed to be equal to this instance,
     * false otherwise.
     */
    @Override
    public boolean equals(Measurement<SurfaceUnit> other,
                          double tolerance) {
        if (super.equals(other, tolerance)) {
            return true;
        }

        //attempt conversion to common units
        if (other == null) {
            return false;
        }

        double otherValue = SurfaceConverter.convert(
                other.getValue().doubleValue(), other.getUnit(),
                getUnit());
        return (Math.abs(getValue().doubleValue() - otherValue)) <= tolerance;
    }

    /**
     * Adds two surface values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static double add(double value1, SurfaceUnit unit1,
                             double value2, SurfaceUnit unit2,
                             SurfaceUnit resultUnit) {
        double v1 = SurfaceConverter.convert(value1, unit1, resultUnit);
        double v2 = SurfaceConverter.convert(value2, unit2, resultUnit);
        return v1 + v2;
    }

    /**
     * Adds to surface values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of addition.
     */
    public static Number add(Number value1, SurfaceUnit unit1,
                             Number value2, SurfaceUnit unit2,
                             SurfaceUnit resultUnit) {
        return new BigDecimal(add(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Adds two surfaces and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void add(Surface arg1, Surface arg2, Surface result) {
        result.setValue(add(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Adds two surfaces.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned surface.
     * @return a new instance containing result.
     */
    public static Surface addAndReturnNew(Surface arg1, Surface arg2,
                                          SurfaceUnit unit) {
        Surface result = new Surface();
        result.setUnit(unit);
        add(arg1, arg2, result);
        return result;
    }

    /**
     * Adds provided surface value and unit and returns a new surface instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned surface.
     * @return a new surface containing result.
     */
    public Surface addAndReturnNew(double value, SurfaceUnit unit, SurfaceUnit resultUnit) {
        Surface result = new Surface();
        result.setUnit(resultUnit);
        result.setValue(add(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Adds provided surface value and unit and returns a new surface instance using
     * provided unit.
     * @param value value to be added.
     * @param unit unit of value to be added.
     * @param resultUnit unit of returned surface.
     * @return a new surface containing result.
     */
    public Surface addAndReturnNew(Number value, SurfaceUnit unit, SurfaceUnit resultUnit) {
        Surface result = new Surface();
        result.setUnit(resultUnit);
        result.setValue(add(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Adds provided surface to current instance and returns a new
     * surface.
     * @param s surface to be added.
     * @param unit unit of returned surface.
     * @return a new surface containing result.
     */
    public Surface addAndReturnNew(Surface s, SurfaceUnit unit) {
        return addAndReturnNew(this, s, unit);
    }

    /**
     * Adds provided surface value and unit and updates current surface instance.
     * @param value surface value to be added.
     * @param unit unit of surface value.
     */
    public void add(double value, SurfaceUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided surface value and unit and updates current surface instance.
     * @param value surface value to be added.
     * @param unit unit of surface value.
     */
    public void add(Number value, SurfaceUnit unit) {
        setValue(add(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Adds provided surface and updates current distance.
     * @param surface surface to be added.
     */
    public void add(Surface surface) {
        add(this, surface, this);
    }

    /**
     * Adds provided surface and stores the result into provided
     * surface.
     * @param s surface to be added.
     * @param result instance where result will be stored.
     */
    public void add(Surface s, Surface result) {
        add(this, s, result);
    }

    /**
     * Subtracts two surface values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static double subtract(double value1, SurfaceUnit unit1,
                                  double value2, SurfaceUnit unit2,
                                  SurfaceUnit resultUnit) {
        double v1 = SurfaceConverter.convert(value1, unit1, resultUnit);
        double v2 = SurfaceConverter.convert(value2, unit2, resultUnit);
        return v1 - v2;
    }

    /**
     * Subtracts two surface values and units and returns the result.
     * @param value1 1st argument value.
     * @param unit1 1st argument unit.
     * @param value2 2nd argument value.
     * @param unit2 2nd argument unit.
     * @param resultUnit unit of result to be returned.
     * @return result of subtraction.
     */
    public static Number subtract(Number value1, SurfaceUnit unit1,
                                  Number value2, SurfaceUnit unit2,
                                  SurfaceUnit resultUnit) {
        return new BigDecimal(subtract(value1.doubleValue(), unit1,
                value2.doubleValue(), unit2, resultUnit));
    }

    /**
     * Subtracts two surfaces and stores the result into provided instance.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param result instance where result will be stored.
     */
    public static void subtract(Surface arg1, Surface arg2, Surface result) {
        result.setValue(subtract(arg1.getValue(), arg1.getUnit(),
                arg2.getValue(), arg2.getUnit(), result.getUnit()));
    }

    /**
     * Subtracts two surfaces.
     * @param arg1 1st argument.
     * @param arg2 2nd argument.
     * @param unit unit of returned surface.
     * @return a new instance containing result.
     */
    public static Surface subtractAndReturnNew(Surface arg1, Surface arg2,
                                               SurfaceUnit unit) {
        Surface result = new Surface();
        result.setUnit(unit);
        subtract(arg1, arg2, result);
        return result;
    }

    /**
     * Subtracts provided surface value and unit and returns a new surface instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned surface.
     * @return a new surface containing result.
     */
    public Surface subtractAndReturnNew(double value, SurfaceUnit unit, SurfaceUnit resultUnit) {
        Surface result = new Surface();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue().doubleValue(), getUnit(), value, unit,
                resultUnit));
        return result;
    }

    /**
     * Subtracts provided surface value and unit and returns a new surface instance using
     * provided unit.
     * @param value value to be subtracted.
     * @param unit unit of value to be subtracted.
     * @param resultUnit unit of returned time.
     * @return a new surface containing result.
     */
    public Surface subtractAndReturnNew(Number value, SurfaceUnit unit, SurfaceUnit resultUnit) {
        Surface result = new Surface();
        result.setUnit(resultUnit);
        result.setValue(subtract(getValue(), getUnit(), value, unit, resultUnit));
        return result;
    }

    /**
     * Subtracts provided surface to current instance and returns a new
     * surface.
     * @param s surface to be subtracted.
     * @param unit unit of returned surface.
     * @return a new surface containing result.
     */
    public Surface subtractAndReturnNew(Surface s, SurfaceUnit unit) {
        return subtractAndReturnNew(this, s, unit);
    }

    /**
     * Subtracts provided surface value and unit and updates current surface instance.
     * @param value surface value to be subtracted.
     * @param unit unit of surface value.
     */
    public void subtract(double value, SurfaceUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtracts provided surface value and unit and updates current surface instance.
     * @param value surface value to be subtracted.
     * @param unit unit of surface value.
     */
    public void subtract(Number value, SurfaceUnit unit) {
        setValue(subtract(getValue(), getUnit(), value, unit, getUnit()));
    }

    /**
     * Subtracts provided surface and updates current surface.
     * @param surface surface to be subtracted.
     */
    public void subtract(Surface surface) {
        subtract(this, surface, this);
    }

    /**
     * Subtracts provided surface and stores the result into provided
     * surface.
     * @param s surface to be subtracted.
     * @param result instance where result will be stored.
     */
    public void subtract(Surface s, Surface result) {
        subtract(this, s, result);
    }
}
