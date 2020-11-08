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

package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator;

/**
 * Generates measurements for the calibration of magnetometers by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with magnetometer calibrators based
 * on the knowledge of position on Earth and time instant.
 * Such calibrators are the following ones:
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronPositionAndInstantMagnetometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownPositionAndInstantMagnetometerCalibrator}
 * and all its implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.magnetometer.RobustKnownHardIronPositionAndInstantMagnetometerCalibrator}
 * and all its implementations.
 */
public class MagnetometerMeasurementGenerator extends
        MeasurementsGenerator<StandardDeviationBodyMagneticFluxDensity,
                MagnetometerMeasurementGenerator,
                MagnetometerMeasurementGeneratorListener, BodyKinematicsAndMagneticFluxDensity> {

    /**
     * Accumulated noise estimator for magnetic flux density measurements.
     */
    private final AccumulatedMagneticFluxDensityTriadNoiseEstimator mAccumulatedEstimator =
            new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

    /**
     * Accumulated average x-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double mAvgBx;

    /**
     * Accumulated average y-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double mAvgBy;

    /**
     * Accumulated average z-coordinate of magnetic flux density while body remains in a static interval and
     * expressed in Teslas (T).
     */
    private double mAvgBz;

    /**
     * Contains standard deviatoin of magnetic flux density during initialization (i.e. base noise level)
     * expressed in Teslas (T).
     */
    private double mStdBNorm;

    /**
     * Constructor.
     */
    public MagnetometerMeasurementGenerator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param staticIntervalDetector a static interval detector for accelerometer samples.
     * @throws LockedException if provided detector is busy.
     */
    protected MagnetometerMeasurementGenerator(
            final AccelerationTriadStaticIntervalDetector staticIntervalDetector)
            throws LockedException {
        super(staticIntervalDetector);
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    @Override
    public void reset() throws LockedException {
        super.reset();

        mAccumulatedEstimator.reset();
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final BodyKinematicsAndMagneticFluxDensity sample) throws LockedException {
        final BodyMagneticFluxDensity b = sample.getMagneticFluxDensity();
        if (mStaticIntervalDetector.getStatus() == TriadStaticIntervalDetector.Status.STATIC_INTERVAL
                || mStaticIntervalDetector.getStatus() == TriadStaticIntervalDetector.Status.INITIALIZING) {

            mAccumulatedEstimator.addTriad(b.getBx(), b.getBy(), b.getBz());
            mAvgBx = mAccumulatedEstimator.getAvgX();
            mAvgBy = mAccumulatedEstimator.getAvgY();
            mAvgBz = mAccumulatedEstimator.getAvgZ();

        } else {
            mAccumulatedEstimator.reset();
        }
    }

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #mTriad}.
     *
     * @param sample input sample.
     */
    @Override
    protected void getAccelerationTriadFromInputSample(
            final BodyKinematicsAndMagneticFluxDensity sample) {
        sample.getKinematics().getSpecificForceTriad(mTriad);
    }

    /**
     * Handles a static-to-dynamic interval change.
     *
     * @param accumulatedAvgX   average x-coordinate of measurements during last
     *                          static period expressed in meters per squared
     *                          second (m/s^2).
     * @param accumulatedAvgY   average y-coordinate of specific force during last
     *                          static period expressed in meters per squared
     *                          second (m/s^2).
     * @param accumulatedAvgZ   average z-coordinate of specific force during last
     *                          static period expressed in meters per squared
     *                          second (m/s^2).
     * @param accumulatedStdX   standard deviation of x-coordinate of measurements
     *                          during last static period expressed in meters per
     *                          squared second (m/s^2).
     * @param accumulatedStdY   standard deviation of y-coordinate of measurements
     *                          during last static period expressed in meters per
     *                          squared second (m/s^2).
     * @param accumulatedStdZ   standard deviation of z-coordinate of measurements
     *                          during last static period expressed in meters per
     *                          squared second (m/s^2).
     */
    @Override
    protected void handleStaticToDynamicChange(
            final double accumulatedAvgX,
            final double accumulatedAvgY,
            final double accumulatedAvgZ,
            final double accumulatedStdX,
            final double accumulatedStdY,
            final double accumulatedStdZ) {

        if (!isStaticIntervalSkipped()) {

            final StandardDeviationBodyMagneticFluxDensity measurement =
                    new StandardDeviationBodyMagneticFluxDensity(
                            new BodyMagneticFluxDensity(mAvgBx, mAvgBy, mAvgBz),
                            mStdBNorm);

            if (mListener != null) {
                mListener.onGeneratedMeasurement(this, measurement);
            }
        }
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        // no action needed.
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        mStdBNorm = mAccumulatedEstimator.getStandardDeviationNorm();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        try {
            mAccumulatedEstimator.reset();
        } catch (final LockedException ignore) {
        }
    }
}
