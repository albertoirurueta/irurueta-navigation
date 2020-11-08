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

import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;

/**
 * Estimates magnetometer noise variances and PSD's (Power Spectral Densities)
 * along with the magnetometer average values for a windowed amount of samples.
 * This estimator must be used when the body where the magnetometer is attached
 * remains static on the same position and orientation with zero velocity while
 * capturing data.
 * To compute PSD's, this estimator assumes that magnetometer samples are
 * obtained at a constant provided rate equal to {@link #getTimeInterval()} second.
 * If not available, accelerometer sampling rate average can be estimated using
 * {@link TimeIntervalEstimator}.
 * This estimator does NOT require the knowledge of current location and body
 * orientation.
 * Because body location and orientation is not know, estimated average values
 * cannot be used to determine biases. Only norm of noise estimations can be
 * safely used.
 * Notice that if there are less than {@link #getWindowSize()} processed
 * samples in the window, this estimator will assume that the remaining ones
 * until the window is completed have zero values.
 * This implementation of noise estimator will use the following units:
 * - Teslas (T) for magnetic flux density, average or standard deviation values.
 * - squared Teslas (T^2) for magnetic flux density variances.
 * - (T^2 * s) for magnetometer PSD (Power Spectral Density).
 * - (T * s^0.5) for magnetometer root PSD (Power Spectral Density).
 */
public class WindowedMagneticFluxDensityTriadNoiseEstimator extends
        WindowedTriadNoiseEstimator<MagneticFluxDensityUnit, MagneticFluxDensity,
                        MagneticFluxDensityTriad, WindowedMagneticFluxDensityTriadNoiseEstimator,
                WindowedMagneticFluxDensityTriadNoiseEstimatorListener> {

    /**
     * Constructor.
     */
    public WindowedMagneticFluxDensityTriadNoiseEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public WindowedMagneticFluxDensityTriadNoiseEstimator(
            final WindowedMagneticFluxDensityTriadNoiseEstimatorListener listener) {
        super(listener);
    }

    /**
     * Creates a copy of a triad.
     *
     * @param input triad to be copied.
     * @return copy of a triad.
     */
    @Override
    protected MagneticFluxDensityTriad copyTriad(final MagneticFluxDensityTriad input) {
        return new MagneticFluxDensityTriad(input);
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
    protected MagneticFluxDensityTriad createTriad(
            final double valueX, final double valueY, final double valueZ,
            final MagneticFluxDensityUnit unit) {
        return new MagneticFluxDensityTriad(unit, valueX, valueY, valueZ);
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
    protected MagneticFluxDensityTriad createTriad(
            final MagneticFluxDensity valueX,
            final MagneticFluxDensity valueY,
            final MagneticFluxDensity valueZ) {
        return new MagneticFluxDensityTriad(valueX, valueY, valueZ);
    }

    /**
     * Gets default unit for a measurement.
     *
     * @return default unit for a measurement.
     */
    @Override
    protected MagneticFluxDensityUnit getDefaultUnit() {
        return MagneticFluxDensityUnit.TESLA;
    }

    /**
     * Creates a measurement with provided value and unit.
     *
     * @param value value to be set.
     * @param unit unit to be set.
     * @return created measurement.
     */
    @Override
    protected MagneticFluxDensity createMeasurement(
            final double value, final MagneticFluxDensityUnit unit) {
        return new MagneticFluxDensity(value, unit);
    }
}
