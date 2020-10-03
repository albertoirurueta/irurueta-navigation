/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

/**
 * Contains a triad of acceleration measurements.
 */
public class AccelerationTriad extends Triad<AccelerationUnit, Acceleration> implements Cloneable {

    /**
     * Default acceleration unit.
     */
    public static final AccelerationUnit DEFAULT_UNIT = AccelerationUnit.METERS_PER_SQUARED_SECOND;

    /**
     * Constructor.
     */
    public AccelerationTriad() {
        this(DEFAULT_UNIT);
    }

    /**
     * Constructor.
     *
     * @param unit acceleration unit for stored values.
     */
    public AccelerationTriad(final AccelerationUnit unit) {
        super(unit);
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in default unit.
     * @param valueY y-coordinate of measurement value expressed in default unit.
     * @param valueZ z-coordinate of measurement value expressed in default unit.
     */
    public AccelerationTriad(
            final double valueX, final double valueY, final double valueZ) {
        this(DEFAULT_UNIT, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param unit   acceleration unit for stored values.
     * @param valueX x-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueY y-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueZ z-coordinate of measurement value expressed in
     *               provided unit.
     */
    public AccelerationTriad(
            final AccelerationUnit unit,
            final double valueX, final double valueY, final double valueZ) {
        super(unit, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param measurementX x-coordinate of measurement.
     * @param measurementY y-coordinate of measurement.
     * @param measurementZ z-coordinate of measurement.
     */
    public AccelerationTriad(
            final Acceleration measurementX,
            final Acceleration measurementY,
            final Acceleration measurementZ) {
        super(DEFAULT_UNIT);
        setMeasurementCoordinates(measurementX, measurementY, measurementZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    public AccelerationTriad(final AccelerationTriad other) {
        super(other);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    @Override
    public Acceleration getMeasurementX() {
        return new Acceleration(getValueX(), getUnit());
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementX(final Acceleration result) {
        result.setValue(getValueX());
        result.setUnit(getUnit());
    }

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    @Override
    public void setMeasurementX(final Acceleration measurementX) {
        setValueX(AccelerationConverter.convert(measurementX.getValue(),
                measurementX.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    @Override
    public Acceleration getMeasurementY() {
        return new Acceleration(getValueY(), getUnit());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementY(final Acceleration result) {
        result.setValue(getValueY());
        result.setUnit(getUnit());
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    @Override
    public void setMeasurementY(final Acceleration measurementY) {
        setValueY(AccelerationConverter.convert(measurementY.getValue(),
                measurementY.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    @Override
    public Acceleration getMeasurementZ() {
        return new Acceleration(getValueZ(), getUnit());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementZ(final Acceleration result) {
        result.setValue(getValueZ());
        result.setUnit(getUnit());
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementZ(final Acceleration measurementZ) {
        setValueZ(AccelerationConverter.convert(measurementZ.getValue(),
                measurementZ.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Sets measurement coordinates.
     *
     * @param measurementX x coordinate of measurement value.
     * @param measurementY y coordinate of measurement value.
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementCoordinates(
            final Acceleration measurementX,
            final Acceleration measurementY,
            final Acceleration measurementZ) {
        setMeasurementX(measurementX);
        setMeasurementY(measurementY);
        setMeasurementZ(measurementZ);
    }

    /**
     * Gets norm as an acceleration.
     *
     * @return acceleration containing triad norm.
     */
    @Override
    public Acceleration getMeasurementNorm() {
        return new Acceleration(getNorm(), getUnit());
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final AccelerationTriad result = (AccelerationTriad) super.clone();
        copyTo(result);
        return result;
    }
}
