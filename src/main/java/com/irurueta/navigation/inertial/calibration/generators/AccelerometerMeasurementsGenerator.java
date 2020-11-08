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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;

/**
 * Generates measurements for the calibration of accelerometers by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with accelerometer calibrators based
 * on the knowledge of gravity norm (or Earth position) when the device orientation
 * is unknown.
 * Such calibrators are the following ones:
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownPositionAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndGravityNormAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAndPositionAccelerometerCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownGravityNormAccelerometerCalibrator}
 * and all its implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownPositionAccelerometerCalibrator}
 * and all its implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownBiasAndGravityNormAccelerometerCalibrator}
 * and all its implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.accelerometer.RobustKnownBiasAndPositionAccelerometerCalibrator}
 * and all its implementations.
 */
public class AccelerometerMeasurementsGenerator extends
        MeasurementsGenerator<StandardDeviationBodyKinematics,
                AccelerometerMeasurementsGenerator,
                AccelerometerMeasurementsGeneratorListener, BodyKinematics> {

    /**
     * Constructor.
     */
    public AccelerometerMeasurementsGenerator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public AccelerometerMeasurementsGenerator(
            final AccelerometerMeasurementsGeneratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param staticIntervalDetector a static interval detector for accelerometer samples.
     * @throws LockedException if provided detector is busy.
     */
    protected AccelerometerMeasurementsGenerator(
            final AccelerationTriadStaticIntervalDetector staticIntervalDetector)
            throws LockedException {
        super(staticIntervalDetector);
    }

    /**
     * Constructor.
     *
     * @param staticIntervalDetector a static interval detector for accelerometer samples.
     * @param listener               listener to handle events raised by this generator.
     * @throws LockedException if provided detector is busy.
     */
    protected AccelerometerMeasurementsGenerator(
            final AccelerationTriadStaticIntervalDetector staticIntervalDetector,
            final AccelerometerMeasurementsGeneratorListener listener)
            throws LockedException {
        super(staticIntervalDetector, listener);
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     */
    @Override
    protected void postProcess(BodyKinematics sample) {
        // no action required for accelerometer calibration
    }

    /**
     * Gets corresponding acceleration triad from provided input sample.
     * This method must store the result into {@link #mTriad}.
     *
     * @param sample input sample.
     */
    @Override
    protected void getAccelerationTriadFromInputSample(
            final BodyKinematics sample) {
        sample.getSpecificForceTriad(mTriad);
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
        // if last static interval must not be skipped, keep accumulated average
        // specific force during last static interval and generate new measurement
        // NOTE: generated body kinematics instances will have zero angular rate
        // since it is not needed for accelerometer calibration
        if (!isStaticIntervalSkipped()) {

            final BodyKinematics kinematics = new BodyKinematics();
            kinematics.setSpecificForceCoordinates(
                    accumulatedAvgX, accumulatedAvgY, accumulatedAvgZ);

            final StandardDeviationBodyKinematics measurement = new StandardDeviationBodyKinematics();
            measurement.setKinematics(kinematics);

            final double avgStd = (accumulatedStdX + accumulatedStdY + accumulatedStdZ) / 3.0;
            measurement.setSpecificForceStandardDeviation(avgStd);

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
        // no action needed.
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        // no action needed.
    }
}
