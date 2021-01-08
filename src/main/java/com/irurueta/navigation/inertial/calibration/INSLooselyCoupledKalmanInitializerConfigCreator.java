/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;

/**
 * Utility class to create {@link INSLooselyCoupledKalmanInitializerConfig} by combining
 * different sources of estimated data.
 * Sources of data can be any accelerometer calibrator implementing
 * {@link AccelerometerBiasUncertaintySource}, or any gyroscope calibrator implementing
 * {@link GyroscopeBiasUncertaintySource}.
 */
public class INSLooselyCoupledKalmanInitializerConfigCreator {

    /**
     * A source of estimated accelerometer bias uncertainty.
     */
    private AccelerometerBiasUncertaintySource mAccelerometerBiasUncertaintySource;

    /**
     * A source of estimated gyroscope bias uncertainty.
     */
    private GyroscopeBiasUncertaintySource mGyroscopeBiasUncertaintySource;

    /**
     * A source of attitude uncertainty.
     */
    private AttitudeUncertaintySource mAttitudeUncertaintySource;

    /**
     * A source of velocity uncertainty.
     */
    private VelocityUncertaintySource mVelocityUncertaintySource;

    /**
     * A source of position uncertainty.
     */
    private PositionUncertaintySource mPositionUncertaintySource;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiasUncertaintySource a source of estimated accelerometer bias uncertainty.
     * @param gyroscopeBiasUncertaintySource     a source of estimated gyroscope bias uncertainty.
     * @param attitudeUncertaintySource          a source of attitude uncertainty.
     * @param velocityUncertaintySource          a source of velocity uncertainty.
     * @param positionUncertaintySource          a source of position uncertainty.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource,
            final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource,
            final AttitudeUncertaintySource attitudeUncertaintySource,
            final VelocityUncertaintySource velocityUncertaintySource,
            final PositionUncertaintySource positionUncertaintySource) {
        mAccelerometerBiasUncertaintySource = accelerometerBiasUncertaintySource;
        mGyroscopeBiasUncertaintySource = gyroscopeBiasUncertaintySource;
        mAttitudeUncertaintySource = attitudeUncertaintySource;
        mVelocityUncertaintySource = velocityUncertaintySource;
        mPositionUncertaintySource = positionUncertaintySource;
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiasUncertaintySource a source of estimated accelerometer bias uncertainty.
     * @param gyroscopeBiasUncertaintySource     a source of estimated gyroscope bias uncertainty.
     * @param randomWalkEstimator                a random walk estimator.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource,
            final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource,
            final RandomWalkEstimator randomWalkEstimator) {
        this(accelerometerBiasUncertaintySource, gyroscopeBiasUncertaintySource,
                randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Gets source of estimated accelerometer bias uncertainty.
     *
     * @return source of estimated accelerometer bias uncertainty.
     */
    public AccelerometerBiasUncertaintySource getAccelerometerBiasUncertaintySource() {
        return mAccelerometerBiasUncertaintySource;
    }

    /**
     * Sets source of estimated accelerometer bias uncertainty.
     *
     * @param accelerometerBiasUncertaintySource source of estimated accelerometer bias uncertainty.
     */
    public void setAccelerometerBiasUncertaintySource(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource) {
        mAccelerometerBiasUncertaintySource = accelerometerBiasUncertaintySource;
    }

    /**
     * Gets source of estimated gyroscope bias uncertainty.
     *
     * @return source of estimated gyroscope bias uncertainty.
     */
    public GyroscopeBiasUncertaintySource getGyroscopeBiasUncertaintySource() {
        return mGyroscopeBiasUncertaintySource;
    }

    /**
     * Sets source of estimated gyroscope bias uncertainty.
     *
     * @param gyroscopeBiasUncertaintySource source of estimated gyroscope bias uncertainty.
     */
    public void setGyroscopeBiasUncertaintySource(
            final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource) {
        mGyroscopeBiasUncertaintySource = gyroscopeBiasUncertaintySource;
    }

    /**
     * Gets source of attitude uncertainty.
     *
     * @return source of attitude uncertainty.
     */
    public AttitudeUncertaintySource getAttitudeUncertaintySource() {
        return mAttitudeUncertaintySource;
    }

    /**
     * Sets source of attitude uncertainty.
     *
     * @param attitudeUncertaintySource source of attitude uncertainty.
     */
    public void setAttitudeUncertaintySource(
            final AttitudeUncertaintySource attitudeUncertaintySource) {
        mAttitudeUncertaintySource = attitudeUncertaintySource;
    }

    /**
     * Gets source of velocity uncertainty.
     *
     * @return source of velocity uncertainty.
     */
    public VelocityUncertaintySource getVelocityUncertaintySource() {
        return mVelocityUncertaintySource;
    }

    /**
     * Sets source of velocity uncertainty.
     *
     * @param velocityUncertaintySource source of velocity uncertainty.
     */
    public void setVelocityUncertaintySource(
            final VelocityUncertaintySource velocityUncertaintySource) {
        mVelocityUncertaintySource = velocityUncertaintySource;
    }

    /**
     * Gets source of position uncertainty.
     *
     * @return source of position uncertainty.
     */
    public PositionUncertaintySource getPositionUncertaintySource() {
        return mPositionUncertaintySource;
    }

    /**
     * Sets source of position uncertainty.
     *
     * @param positionUncertaintySource source of position uncertainty.
     */
    public void setPositionUncertaintySource(
            final PositionUncertaintySource positionUncertaintySource) {
        mPositionUncertaintySource = positionUncertaintySource;
    }

    /**
     * Indicates whether all sources have been provided in order to be able to
     * create a {@link INSLooselyCoupledKalmanInitializerConfig} instance.
     *
     * @return true if creator is ready, false otherwise.
     */
    public boolean isReady() {
        return mAccelerometerBiasUncertaintySource != null
                && mAccelerometerBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm() != null
                && mGyroscopeBiasUncertaintySource != null
                && mGyroscopeBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm() != null
                && mAttitudeUncertaintySource != null
                && mVelocityUncertaintySource != null
                && mPositionUncertaintySource != null;
    }

    /**
     * Creates a {@link INSLooselyCoupledKalmanInitializerConfig} instance containing estimated
     * parameters during calibration.
     *
     * @return instance containing initial configuration data.
     * @throws NotReadyException if creator is not ready.
     */
    public INSLooselyCoupledKalmanInitializerConfig create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final double attitudeUncertainty =
                mAttitudeUncertaintySource.getAttitudeStandardDeviation();
        final double velocityUncertainty =
                mVelocityUncertaintySource.getVelocityStandardDeviation();
        final double positionUncertainty =
                mPositionUncertaintySource.getPositionStandardDeviation();
        final double accelerationBiasUncertainty =
                mAccelerometerBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm();
        final double gyroBiasUncertainty =
                mGyroscopeBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm();

        return new INSLooselyCoupledKalmanInitializerConfig(
                attitudeUncertainty, velocityUncertainty, positionUncertainty,
                accelerationBiasUncertainty, gyroBiasUncertainty);
    }
}
