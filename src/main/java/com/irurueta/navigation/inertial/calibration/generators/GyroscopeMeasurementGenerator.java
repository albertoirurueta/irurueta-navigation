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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates measurements for the calibration of gyroscopes by alternating
 * static and dynamic intervals where device is kept static or moved.
 * Generated measurements must be used with easy gyroscope calibrators.
 * Such calibrators are the following ones:
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.KnownBiasEasyGyroscopeCalibrator}
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustEasyGyroscopeCalibrator} and all its
 * implementations.
 * - {@link com.irurueta.navigation.inertial.calibration.gyroscope.RobustKnownBiasEasyGyroscopeCalibrator} and all its
 * implementations.
 */
public class GyroscopeMeasurementGenerator extends
        MeasurementsGenerator<
                BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>,
                GyroscopeMeasurementGenerator,
                GyroscopeMeasurementGeneratorListener, TimedBodyKinematics> {

    /**
     * An angular speed triad.
     * This is reused for memory efficiency.
     */
    protected final AngularSpeedTriad mAngularSpeedTriad =
            new AngularSpeedTriad();

    /**
     * Items to be added to a generated sequence when next static period occurs.
     */
    private List<StandardDeviationTimedBodyKinematics> mCurrentSequenceItems;

    /**
     * Accumulated noise estimator for angular speed measurements.
     */
    private final AccumulatedAngularSpeedTriadNoiseEstimator mAccumulatedEstimator =
            new AccumulatedAngularSpeedTriadNoiseEstimator();

    /**
     * Estimated acceleration standard deviation during initialization expressed
     * in meters per squared second (m/s^2).
     */
    private double mAccelerationStandardDeviation;

    /**
     * Estimated angular speed standard deviation during initialization expressed
     * in radians per second (rad/s).
     */
    private double mAngularSpeedStandardDeviation;

    /**
     * Previous average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgX;

    /**
     * Previous average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgY;

    /**
     * Previous average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mPreviousAvgZ;

    /**
     * Current average x-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgX;

    /**
     * Current average y-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgY;

    /**
     * Current average z-coordinate of measurements expressed in meters
     * per squared second (m/s^2).
     */
    private Double mCurrentAvgZ;

    /**
     * Contains previous status while processing samples.
     */
    private TriadStaticIntervalDetector.Status mPreviousStatus;

    /**
     * Constructor.
     */
    public GyroscopeMeasurementGenerator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this generator.
     */
    public GyroscopeMeasurementGenerator(
            final GyroscopeMeasurementGeneratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param staticIntervalDetector a static interval detector for accelerometer samples.
     * @throws LockedException if provided detector is busy.
     */
    protected GyroscopeMeasurementGenerator(
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
    protected GyroscopeMeasurementGenerator(
            final AccelerationTriadStaticIntervalDetector staticIntervalDetector,
            final GyroscopeMeasurementGeneratorListener listener)
            throws LockedException {
        super(staticIntervalDetector, listener);
    }

    /**
     * Resets this generator.
     *
     * @throws LockedException if generator is busy.
     */
    @Override
    public void reset() throws LockedException {
        super.reset();

        mCurrentSequenceItems = null;

        mAccelerationStandardDeviation = 0.0;
        mAngularSpeedStandardDeviation = 0.0;

        mPreviousAvgX = null;
        mPreviousAvgY = null;
        mPreviousAvgZ = null;

        mCurrentAvgX = null;
        mCurrentAvgY = null;
        mCurrentAvgZ = null;

        mAccumulatedEstimator.reset();

        mPreviousStatus = null;
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @return estimated average angular rate during initialization phase.
     */
    public AngularSpeedTriad getInitialAvgAngularSpeedTriad() {
        return mAccumulatedEstimator.getAvgTriad();
    }

    /**
     * Gets estimated average angular rate during initialization phase.
     *
     * @param result instance where result will be stored.
     */
    public void getInitialAvgAngularSpeedTriad(final AngularSpeedTriad result) {
        mAccumulatedEstimator.getAvgTriad(result);
    }

    /**
     * Post process provided input sample.
     *
     * @param sample an input sample.
     * @throws LockedException if generator is busy.
     */
    @Override
    protected void postProcess(final TimedBodyKinematics sample)
            throws LockedException {
        final TriadStaticIntervalDetector.Status status =
                mStaticIntervalDetector.getStatus();

        if (status == TriadStaticIntervalDetector.Status.INITIALIZING) {
            sample.getKinematics().getAngularRateTriad(mAngularSpeedTriad);
            mAccumulatedEstimator.addTriad(mAngularSpeedTriad);
        }

        // while we are in a dynamic interval, we must record all timed kinematics
        // along with accelerometer and gyroscope standard deviations
        if (status == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL) {
            if (isDynamicIntervalSkipped()) {
                // dynamic interval has been skipped because there were too many
                // items in the sequence.
                mCurrentSequenceItems = null;
            } else {
                if (mPreviousStatus == TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
                    mPreviousAvgX = mStaticIntervalDetector.getAccumulatedAvgX();
                    mPreviousAvgY = mStaticIntervalDetector.getAccumulatedAvgY();
                    mPreviousAvgZ = mStaticIntervalDetector.getAccumulatedAvgZ();
                }

                addSequenceItem(sample);
            }
        } else if (status == TriadStaticIntervalDetector.Status.STATIC_INTERVAL) {
            if (mPreviousStatus == TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL
                && mCurrentSequenceItems != null && !mCurrentSequenceItems.isEmpty()) {

                mCurrentAvgX = mStaticIntervalDetector.getInstantaneousAvgX();
                mCurrentAvgY = mStaticIntervalDetector.getInstantaneousAvgY();
                mCurrentAvgZ = mStaticIntervalDetector.getInstantaneousAvgZ();

                // we have all required data to generate a sequence
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        mPreviousAvgX, mPreviousAvgY, mPreviousAvgZ);
                sequence.setItems(mCurrentSequenceItems);
                sequence.setAfterMeanSpecificForceCoordinates(
                        mCurrentAvgX, mCurrentAvgY, mCurrentAvgZ);

                mCurrentSequenceItems = null;

                if (mListener != null) {
                    mListener.onGeneratedMeasurement(this, sequence);
                }
            }
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
            final TimedBodyKinematics sample) {
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
        mPreviousStatus = TriadStaticIntervalDetector.Status.STATIC_INTERVAL;
    }

    /**
     * Handles a dynamic-to-static interval change.
     */
    @Override
    protected void handleDynamicToStaticChange() {
        mPreviousStatus = TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL;
    }

    /**
     * Handles an initialization completion.
     */
    @Override
    protected void handleInitializationCompleted() {
        mAccelerationStandardDeviation = mStaticIntervalDetector.getBaseNoiseLevel();
        mAngularSpeedStandardDeviation = mAccumulatedEstimator.getStandardDeviationNorm();

        mPreviousStatus = mStaticIntervalDetector.getStatus();
    }

    /**
     * Handles an error during initialization.
     */
    @Override
    protected void handleInitializationFailed() {
        mPreviousStatus = null;

        try {
            mAccumulatedEstimator.reset();
        } catch (final LockedException ignore) {
        }
    }

    /**
     * Adds an item to current sequence items.
     *
     * @param sample sample to generate a sequence item from.
     */
    private void addSequenceItem(final TimedBodyKinematics sample) {
        if (mCurrentSequenceItems == null) {
            mCurrentSequenceItems = new ArrayList<>();
        }

        final BodyKinematics kinematics = new BodyKinematics(
                sample.getKinematics());
        final double timestampSeconds = sample.getTimestampSeconds();
        final StandardDeviationTimedBodyKinematics stdTimedKinematics =
                new StandardDeviationTimedBodyKinematics(kinematics,
                        timestampSeconds, mAccelerationStandardDeviation,
                        mAngularSpeedStandardDeviation);
        mCurrentSequenceItems.add(stdTimedKinematics);
    }
}
