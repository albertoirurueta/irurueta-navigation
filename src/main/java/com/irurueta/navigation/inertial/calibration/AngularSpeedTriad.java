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

import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

/**
 * Contains a triad of angular speed measurements.
 */
public class AngularSpeedTriad extends Triad<AngularSpeedUnit, AngularSpeed> implements Cloneable {

    /**
     * Default angular speed unit.
     */
    public static final AngularSpeedUnit DEFAULT_UNIT = AngularSpeedUnit.RADIANS_PER_SECOND;

    /**
     * Constructor.
     */
    public AngularSpeedTriad() {
        this(DEFAULT_UNIT);
    }

    /**
     * Constructor.
     *
     * @param unit angular speed unit for stored values.
     */
    public AngularSpeedTriad(final AngularSpeedUnit unit) {
        super(unit);
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in default unit.
     * @param valueY y-coordinate of measurement value expressed in default unit.
     * @param valueZ z-coordinate of measurement value expressed in default unit.
     */
    public AngularSpeedTriad(
            final double valueX, final double valueY, final double valueZ) {
        this(DEFAULT_UNIT, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param unit   angular speed unit for stored values.
     * @param valueX x-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueY y-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueZ z-coordinate of measurement value expressed in
     *               provided unit.
     */
    public AngularSpeedTriad(
            final AngularSpeedUnit unit,
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
    public AngularSpeedTriad(
            final AngularSpeed measurementX,
            final AngularSpeed measurementY,
            final AngularSpeed measurementZ) {
        super(DEFAULT_UNIT);
        setMeasurementCoordinates(measurementX, measurementY, measurementZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    public AngularSpeedTriad(final AngularSpeedTriad other) {
        super(other);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    @Override
    public AngularSpeed getMeasurementX() {
        return new AngularSpeed(getValueX(), getUnit());
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementX(final AngularSpeed result) {
        result.setValue(getValueX());
        result.setUnit(getUnit());
    }

    /**
     * Sets x coordinate of measurement value.
     *
     * @param measurementX x coordinate of measurement value.
     */
    @Override
    public void setMeasurementX(final AngularSpeed measurementX) {
        setValueX(AngularSpeedConverter.convert(measurementX.getValue(),
                measurementX.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    @Override
    public AngularSpeed getMeasurementY() {
        return new AngularSpeed(getValueY(), getUnit());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementY(final AngularSpeed result) {
        result.setValue(getValueY());
        result.setUnit(getUnit());
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    @Override
    public void setMeasurementY(final AngularSpeed measurementY) {
        setValueY(AngularSpeedConverter.convert(measurementY.getValue(),
                measurementY.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    @Override
    public AngularSpeed getMeasurementZ() {
        return new AngularSpeed(getValueZ(), getUnit());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementZ(final AngularSpeed result) {
        result.setValue(getValueZ());
        result.setUnit(getUnit());
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementZ(final AngularSpeed measurementZ) {
        setValueZ(AngularSpeedConverter.convert(measurementZ.getValue(),
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
            final AngularSpeed measurementX,
            final AngularSpeed measurementY,
            final AngularSpeed measurementZ) {
        setMeasurementX(measurementX);
        setMeasurementY(measurementY);
        setMeasurementZ(measurementZ);
    }

    /**
     * Gets norm as an angular speed.
     *
     * @return angular speed containing triad norm.
     */
    @Override
    public AngularSpeed getMeasurementNorm() {
        return new AngularSpeed(getNorm(), getUnit());
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final AngularSpeedTriad result = (AngularSpeedTriad) super.clone();
        copyTo(result);
        return result;
    }
}
