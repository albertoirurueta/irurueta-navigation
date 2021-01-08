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
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator;

/**
 * Utility class to create {@link INSLooselyCoupledKalmanConfig} by combining
 * different sources of estimated data.
 * Sources of data can be any measurement generator, static interval detector or
 * noise estimator implementing {@link AccelerometerNoiseRootPsdSource}
 * or {@link GyroscopeNoiseRootPsdSource}.
 */
public class INSLooselyCoupledKalmanConfigCreator {

    /**
     * A source of estimated accelerometer noise root PSD.
     */
    private AccelerometerNoiseRootPsdSource mAccelerometerNoiseRootPsdSource;

    /**
     * A source of estimated gyroscope noise root PSD.
     */
    private GyroscopeNoiseRootPsdSource mGyroscopeNoiseRootPsdSource;

    /**
     * A source of estimated accelerometer bias random walk PSD.
     */
    private AccelerometerBiasRandomWalkSource mAccelerometerBiasRandomWalkSource;

    /**
     * A source of estimated gyroscope bias random walk PSD.
     */
    private GyroscopeBiasRandomWalkSource mGyroscopeBiasRandomWalkSource;

    /**
     * A source of position noise standard deviation.
     */
    private PositionUncertaintySource mPositionUncertaintySource;

    /**
     * A source of velocity noise standard deviation.
     */
    private VelocityUncertaintySource mVelocityUncertaintySource;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanConfigCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerNoiseRootPsdSource   a source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource       a source of estimated gyroscope noise root PSD.
     * @param accelerometerBiasRandomWalkSource a source of estimated accelerometer bias random walk PSD.
     * @param gyroscopeBiasRandomWalkSource     a source of estimated gyroscope bias random walk PSD.
     * @param positionUncertaintySource         a source of position noise standard deviation.
     * @param velocityUncertaintySource         a source of velocity noise standard deviation.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource,
            final AccelerometerBiasRandomWalkSource accelerometerBiasRandomWalkSource,
            final GyroscopeBiasRandomWalkSource gyroscopeBiasRandomWalkSource,
            final PositionUncertaintySource positionUncertaintySource,
            final VelocityUncertaintySource velocityUncertaintySource) {
        mAccelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
        mGyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
        mAccelerometerBiasRandomWalkSource = accelerometerBiasRandomWalkSource;
        mGyroscopeBiasRandomWalkSource = gyroscopeBiasRandomWalkSource;
        mPositionUncertaintySource = positionUncertaintySource;
        mVelocityUncertaintySource = velocityUncertaintySource;
    }

    /**
     * Constructor.
     *
     * @param generator           an accelerometer + gyroscope measurement
     *                            generator.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final RandomWalkEstimator randomWalkEstimator) {
        this(generator, generator, randomWalkEstimator,
                randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Constructor.
     *
     * @param generator           an accelerometer + gyroscope + magnetometer
     *                            measurement generator.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final RandomWalkEstimator randomWalkEstimator) {
        this(generator, generator, randomWalkEstimator,
                randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Gets source of estimated accelerometer noise root PSD.
     *
     * @return source of estimated accelerometer noise root PSD.
     */
    public AccelerometerNoiseRootPsdSource getAccelerometerNoiseRootPsdSource() {
        return mAccelerometerNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated accelerometer noise root PSD.
     *
     * @param accelerometerNoiseRootPsdSource source of estimated accelerometer
     *                                        noise root PSD.
     */
    public void setAccelerometerNoiseRootPsdSource(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource) {
        mAccelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
    }

    /**
     * Gets source of estimated gyroscope noise root PSD.
     *
     * @return source of estimated gyroscope noise root PSD.
     */
    public GyroscopeNoiseRootPsdSource getGyroscopeNoiseRootPsdSource() {
        return mGyroscopeNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated gyroscope noise root PSD.
     *
     * @param gyroscopeNoiseRootPsdSource source of estimated gyroscope noise
     *                                    root PSD.
     */
    public void sstGyroscopeNoiseRootPsdSource(
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        mGyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Gets source of estimated accelerometer bias random walk PSD.
     *
     * @return source of estimated accelerometer bias random walk PSD.
     */
    public AccelerometerBiasRandomWalkSource getAccelerometerBiasRandomWalkSource() {
        return mAccelerometerBiasRandomWalkSource;
    }

    /**
     * Sets source of estimated accelerometer bias random walk PSD.
     *
     * @param accelerometerBiasRandomWalkSource source of estimated accelerometer
     *                                          bias random walk PSD.
     */
    public void setAccelerometerBiasRandomWalkSource(
            final AccelerometerBiasRandomWalkSource accelerometerBiasRandomWalkSource) {
        mAccelerometerBiasRandomWalkSource = accelerometerBiasRandomWalkSource;
    }

    /**
     * Gets source of estimated gyroscope bias random walk PSD.
     *
     * @return source of estimated gyroscope bias random walk PSD.
     */
    public GyroscopeBiasRandomWalkSource getGyroscopeBiasRandomWalkSource() {
        return mGyroscopeBiasRandomWalkSource;
    }

    /**
     * Sets source of estimated gyroscope bias random walk PSD.
     *
     * @param gyroscopeBiasRandomWalkSource source of estimated gyroscope bias
     *                                      random walk PSD.
     */
    public void setGyroscopeBiasRandomWalkSource(
            final GyroscopeBiasRandomWalkSource gyroscopeBiasRandomWalkSource) {
        mGyroscopeBiasRandomWalkSource = gyroscopeBiasRandomWalkSource;
    }

    /**
     * Gets source of position noise standard deviation.
     *
     * @return source of position noise standard deviation.
     */
    public PositionUncertaintySource getPositionUncertaintySource() {
        return mPositionUncertaintySource;
    }

    /**
     * Sets source of position noise standard deviation.
     *
     * @param positionUncertaintySource source of position noise standard
     *                                  deviation.
     */
    public void setPositionUncertaintySource(
            final PositionUncertaintySource positionUncertaintySource) {
        mPositionUncertaintySource = positionUncertaintySource;
    }

    /**
     * Gets source of velocity noise standard deviation.
     *
     * @return source of velocity noise standard deviation.
     */
    public VelocityUncertaintySource getVelocityUncertaintySource() {
        return mVelocityUncertaintySource;
    }

    /**
     * Sets source of velocity noise standard deviation.
     *
     * @param velocityUncertaintySource source of velocity noise standard
     *                                  deviation.
     */
    public void setVelocityUncertaintySource(
            final VelocityUncertaintySource velocityUncertaintySource) {
        mVelocityUncertaintySource = velocityUncertaintySource;
    }

    /**
     * Indicates whether all sources have been provided in order to be able to
     * create a {@link INSLooselyCoupledKalmanConfig} instance.
     *
     * @return true if creator is ready, false otherwise.
     */
    public boolean isReady() {
        return mAccelerometerNoiseRootPsdSource != null
                && mGyroscopeNoiseRootPsdSource != null
                && mAccelerometerBiasRandomWalkSource != null
                && mGyroscopeBiasRandomWalkSource != null
                && mPositionUncertaintySource != null
                && mVelocityUncertaintySource != null;
    }

    /**
     * Creates a {@link INSLooselyCoupledKalmanConfig} instance containing estimated
     * parameters during calibration.
     *
     * @return instance containing configuration data.
     * @throws NotReadyException if creator is not ready.
     */
    public INSLooselyCoupledKalmanConfig create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final double gyroNoisePsd = Math.pow(
                mGyroscopeNoiseRootPsdSource.getGyroscopeBaseNoiseLevelRootPsd(), 2.0);
        final double accelNoisePsd = Math.pow(
                mAccelerometerNoiseRootPsdSource.getAccelerometerBaseNoiseLevelRootPsd(), 2.0);
        final double accelRandomWalkBiasPsd = mAccelerometerBiasRandomWalkSource.getAccelerometerBiasPSD();
        final double gyroRandomWalkBiasPsd = mGyroscopeBiasRandomWalkSource.getGyroBiasPSD();
        final double positionNoiseSd = mPositionUncertaintySource.getPositionStandardDeviation();
        final double velocityNoiseSd = mVelocityUncertaintySource.getVelocityStandardDeviation();
        return new INSLooselyCoupledKalmanConfig(gyroNoisePsd, accelNoisePsd,
                accelRandomWalkBiasPsd, gyroRandomWalkBiasPsd,
                positionNoiseSd, velocityNoiseSd);
    }
}
