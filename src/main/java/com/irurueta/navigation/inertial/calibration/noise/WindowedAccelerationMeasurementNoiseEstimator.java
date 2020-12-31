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

import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

/**
 * Estimates acceleration noise variances and PSD's (Power Spectral Density)
 * along with acceleration average value for a windowed amount of samples.
 * This estimator must be used when the body where the accelerometer is attached
 * remains static on the same position with zero velocity while capturing data.
 * To compute PSD's, this estimator assumes that accelerometer samples are
 * obtained at a constant provided rate equal to {@link #getTimeInterval()} seconds.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * Notice that if there are less than {@link #getWindowSize()} processed
 * samples in the window, this estimator will assume that the remaining ones
 * until the window is completed have zero values.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * This implementation of noise estimator will use the following units:
 * - meters per squared second (m/s^2) for acceleration, average or standard deviation values.
 * - (m^2/s^4) for acceleration variances.
 * - (m^2 * s^-3) for acceleration PSD (Power Spectral Density).
 * - (m * s^-1.5) for acceleration root PSD (Power Spectral Density).
 */
public class WindowedAccelerationMeasurementNoiseEstimator extends
        WindowedMeasurementNoiseEstimator<AccelerationUnit, Acceleration,
                WindowedAccelerationMeasurementNoiseEstimator,
                WindowedAccelerationMeasurementNoiseEstimatorListener>
        implements AccelerometerNoiseRootPsdSource {

    /**
     * Constructor.
     */
    public WindowedAccelerationMeasurementNoiseEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public WindowedAccelerationMeasurementNoiseEstimator(
            final WindowedAccelerationMeasurementNoiseEstimatorListener listener) {
        super(listener);
    }

    /**
     * Gets default unit for a measurement.
     *
     * @return default unit for a measurement.
     */
    @Override
    protected AccelerationUnit getDefaultUnit() {
        return AccelerationUnit.METERS_PER_SQUARED_SECOND;
    }

    /**
     * Creates a measurement with provided value and unit.
     *
     * @param value value to be set.
     * @param unit unit to be set.
     * @return created measurement.
     */
    @Override
    protected Acceleration createMeasurement(double value, AccelerationUnit unit) {
        return new Acceleration(value, unit);
    }

    /**
     * Converts provided measurement into default unit.
     *
     * @param value measurement to be converted.
     * @return converted value.
     */
    @Override
    protected double convertToDefaultUnit(Acceleration value) {
        return AccelerationConverter.convert(value.getValue().doubleValue(),
                value.getUnit(), getDefaultUnit());
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return getRootPsd();
    }
}
