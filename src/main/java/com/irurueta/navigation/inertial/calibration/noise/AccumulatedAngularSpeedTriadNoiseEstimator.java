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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.IMUTimeIntervalEstimator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

/**
 * Estimates accumulated angular speed noise variances and PSD's (Power Spectral Densities)
 * along with their average values.
 * This estimator must be used when the body where the gyroscope is attached to
 * keeps a constant angular speed while capturing data (i.e. when body is static has
 * a constant overall angular speed due to Earth rotation).
 * To compute PSD's, this estimator assumes that measurement samples are obtained
 * at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, gyroscope sampling rate average can be estimated using
 * {@link IMUTimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not known, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations
 * (variance or standard deviation) can be safely used.
 * This implementation of noise estimator will use the following units:
 * - radians per second (rad/s) for angular speed, average or standard deviation values.
 * - (rad^2/s^2) fr angular speed variances.
 * - (rad^2/s) for gyroscope PSD (Power Spectral Density).
 * - (rad * s^-0.5) for gyroscope root PSD (Power Spectral Density).
 */
public class AccumulatedAngularSpeedTriadNoiseEstimator extends
        AccumulatedTriadNoiseEstimator<AngularSpeedUnit, AngularSpeed, AngularSpeedTriad,
                AccumulatedAngularSpeedTriadNoiseEstimator,
                AccumulatedAngularSpeedTriadNoiseEstimatorListener> {

    /**
     * Constructor.
     */
    public AccumulatedAngularSpeedTriadNoiseEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public AccumulatedAngularSpeedTriadNoiseEstimator(
            final AccumulatedAngularSpeedTriadNoiseEstimatorListener listener) {
        super(listener);
    }

    /**
     * Creates a triad with provided values and unit.
     *
     * @param valueX x coordinate value.
     * @param valueY y coordinate value.
     * @param valueZ z coordinate value.
     * @param unit unit.
     * @return created triad.
     */
    @Override
    protected AngularSpeedTriad createTriad(
            final double valueX, final double valueY, final double valueZ,
            final AngularSpeedUnit unit) {
        return new AngularSpeedTriad(unit, valueX, valueY, valueZ);
    }

    /**
     * Creates a triad with provided values.
     *
     * @param valueX x coordinate value.
     * @param valueY y coordinate value.
     * @param valueZ z coordinate value.
     * @return created triad.
     */
    @Override
    protected AngularSpeedTriad createTriad(
            final AngularSpeed valueX, final AngularSpeed valueY, final AngularSpeed valueZ) {
        return new AngularSpeedTriad(valueX, valueY, valueZ);
    }

    /**
     * Gets default unit for a measurement.
     *
     * @return default unit for a measurement.
     */
    @Override
    protected AngularSpeedUnit getDefaultUnit() {
        return AngularSpeedUnit.RADIANS_PER_SECOND;
    }

    /**
     * Creates a measurement with provided value and unit.
     *
     * @param value value to be set.
     * @param unit unit to be set.
     * @return created measurement.
     */
    @Override
    protected AngularSpeed createMeasurement(double value, AngularSpeedUnit unit) {
        return new AngularSpeed(value, unit);
    }

    /**
     * Converts provided value and unit into default unit.
     *
     * @param value measurement value to be converted.
     * @param unit unit of measurement value to be converted.
     * @return converted value.
     */
    @Override
    protected double convertToDefaultUnit(double value, AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit, getDefaultUnit());
    }
}
