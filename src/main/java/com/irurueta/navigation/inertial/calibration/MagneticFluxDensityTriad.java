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

import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

/**
 * Contains a triad of magnetic flux density measurements.
 */
public class MagneticFluxDensityTriad extends Triad<MagneticFluxDensityUnit, MagneticFluxDensity> implements Cloneable {

    /**
     * Default magnetic flux density unit.
     */
    public static final MagneticFluxDensityUnit DEFAULT_UNIT = MagneticFluxDensityUnit.TESLA;

    /**
     * Constructor.
     */
    public MagneticFluxDensityTriad() {
        this(DEFAULT_UNIT);
    }

    /**
     * Constructor.
     *
     * @param unit magnetic flux density unit for stored values.
     */
    public MagneticFluxDensityTriad(final MagneticFluxDensityUnit unit) {
        super(unit);
    }

    /**
     * Constructor.
     *
     * @param valueX x-coordinate of measurement value expressed in default unit.
     * @param valueY y-coordinate of measurement value expressed in default unit.
     * @param valueZ z-coordinate of measurement value expressed in default unit.
     */
    public MagneticFluxDensityTriad(
            final double valueX, final double valueY, final double valueZ) {
        this(DEFAULT_UNIT, valueX, valueY, valueZ);
    }

    /**
     * Constructor.
     *
     * @param unit   magnetic flux density unit for stored values.
     * @param valueX x-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueY y-coordinate of measurement value expressed in
     *               provided unit.
     * @param valueZ z-coordinate of measurement value expressed in
     *               provided unit.
     */
    public MagneticFluxDensityTriad(
            final MagneticFluxDensityUnit unit,
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
    public MagneticFluxDensityTriad(
            final MagneticFluxDensity measurementX,
            final MagneticFluxDensity measurementY,
            final MagneticFluxDensity measurementZ) {
        super(DEFAULT_UNIT);
        setMeasurementCoordinates(measurementX, measurementY, measurementZ);
    }

    /**
     * Copy constructor.
     *
     * @param other instance to copy from.
     */
    public MagneticFluxDensityTriad(final MagneticFluxDensityTriad other) {
        super(other);
    }

    /**
     * Gets x coordinate of measurement value.
     *
     * @return x coordinate of measurement value.
     */
    @Override
    public MagneticFluxDensity getMeasurementX() {
        return new MagneticFluxDensity(getValueX(), getUnit());
    }

    /**
     * Gets x coordinate of measurement value.
     * @param result instance where x coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementX(final MagneticFluxDensity result) {
        result.setValue(getValueX());
        result.setUnit(getUnit());
    }

    /**
     * Sets x coordinate of measurement value.
     * @param measurementX x coordinate of measurement value.
     */
    @Override
    public void setMeasurementX(final MagneticFluxDensity measurementX) {
        setValueX(MagneticFluxDensityConverter.convert(measurementX.getValue(),
                measurementX.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @return y coordinate of measurement value.
     */
    @Override
    public MagneticFluxDensity getMeasurementY() {
        return new MagneticFluxDensity(getValueY(), getUnit());
    }

    /**
     * Gets y coordinate of measurement value.
     *
     * @param result instance where y coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementY(final MagneticFluxDensity result) {
        result.setValue(getValueY());
        result.setUnit(getUnit());
    }

    /**
     * Sets y coordinate of measurement value.
     *
     * @param measurementY y coordinate of measurement value.
     */
    @Override
    public void setMeasurementY(final MagneticFluxDensity measurementY) {
        setValueY(MagneticFluxDensityConverter.convert(measurementY.getValue(),
                measurementY.getUnit(), getUnit()).doubleValue());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @return z coordinate of measurement value.
     */
    @Override
    public MagneticFluxDensity getMeasurementZ() {
        return new MagneticFluxDensity(getValueZ(), getUnit());
    }

    /**
     * Gets z coordinate of measurement value.
     *
     * @param result instance where z coordinate of measurement value
     *               will be stored.
     */
    @Override
    public void getMeasurementZ(final MagneticFluxDensity result) {
        result.setValue(getValueZ());
        result.setUnit(getUnit());
    }

    /**
     * Sets z coordinate of measurement value.
     *
     * @param measurementZ z coordinate of measurement value.
     */
    @Override
    public void setMeasurementZ(final MagneticFluxDensity measurementZ) {
        setValueZ(MagneticFluxDensityConverter.convert(measurementZ.getValue(),
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
            final MagneticFluxDensity measurementX,
            final MagneticFluxDensity measurementY,
            final MagneticFluxDensity measurementZ) {
        setMeasurementX(measurementX);
        setMeasurementY(measurementY);
        setMeasurementZ(measurementZ);
    }

    /**
     * Gets norm as a magnetic flux density.
     *
     * @return  magnetic flux density containing triad norm.
     */
    @Override
    public MagneticFluxDensity getMeasurementNorm() {
        return new MagneticFluxDensity(getNorm(), getUnit());
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final MagneticFluxDensityTriad result =
                (MagneticFluxDensityTriad) super.clone();
        copyTo(result);
        return result;
    }
}
