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

/**
 * Base class to define a measurement unit and value.
 * @param <T> a measurement unit.
 */
public abstract class Measurement<T extends Enum> {

    /**
     * Measurement value.
     */
    private Number mValue;

    /**
     * Measurement unit.
     */
    private T mUnit;

    /**
     * Constructor.
     * @param value measurement value.
     * @param unit measurement unit.
     * @throws IllegalArgumentException if either value or unit is null.
     */
    public Measurement(Number value, T unit) throws IllegalArgumentException {
        if (value == null || unit == null) {
            throw new IllegalArgumentException();
        }

        mValue = value;
        mUnit = unit;
    }

    /**
     * Constructor.
     */
    Measurement() { }

    /**
     * Returns measurement value.
     * @return measurement value.
     */
    public Number getValue() {
        return mValue;
    }

    /**
     * Sets measurement value.
     * @param value measurement value.
     * @throws IllegalArgumentException if measurement value is null.
     */
    public void setValue(Number value) throws IllegalArgumentException {
        if (value == null) {
            throw new IllegalArgumentException();
        }

        mValue = value;
    }

    /**
     * Returns measurement unit.
     * @return measurement unit.
     */
    public T getUnit() {
        return mUnit;
    }

    /**
     * Sets measurement unit.
     * @param unit measurement unit.
     * @throws IllegalArgumentException if measurement unit is null.
     */
    public void setUnit(T unit) throws IllegalArgumentException {
        if (unit == null) {
            throw new IllegalArgumentException();
        }

        mUnit = unit;
    }
}
