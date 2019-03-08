/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RangingReadingLocated;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class to robustly estimate position of a radio source (e.g. WiFi
 * access point or bluetooth beacon), by discarding outliers.
 *
 * @param <S> a {@link RadioSource} type.
 * @param <P> a {@link Point} type.
 */
public abstract class RobustRangingRadioSourceEstimator<S extends RadioSource, P extends Point> extends
        RobustRadioSourceEstimator<P, RangingReadingLocated<S, P>, RobustRangingRadioSourceEstimatorListener<S, P>> {

    /**
     * Indicates that by default position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    public static final boolean DEFAULT_USE_READING_POSITION_COVARIANCES = true;

    /**
     * Initial position to start the estimation of radio source position.
     */
    protected P mInitialPosition;

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard deviation
     * assuming that both measures are statistically independent.
     */
    protected boolean mUseReadingPositionCovariances = DEFAULT_USE_READING_POSITION_COVARIANCES;

    /**
     * Constructor.
     */
    public RobustRangingRadioSourceEstimator() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same
     *                 radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator(
            List<? extends RangingReadingLocated<S, P>> readings) {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator(
            RobustRangingRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator(
            List<? extends RangingReadingLocated<S, P>> readings,
            RobustRangingRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public RobustRangingRadioSourceEstimator(P initialPosition) {
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator(
            List<? extends RangingReadingLocated<S, P>> readings,
            P initialPosition) {
        super(readings);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RobustRangingRadioSourceEstimator(P initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, P> listener) {
        super(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RobustRangingRadioSourceEstimator(
            List<? extends RangingReadingLocated<S, P>> readings,
            P initialPosition,
            RobustRangingRadioSourceEstimatorListener<S, P> listener) {
        super(readings, listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Gets initial position to start the non-linear estimation of radio source position.
     * If not defined, a linear solution is found instead.
     * @return initial position.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the non-linear estimation of radio source position.
     * If not defined, a linear solution is found instead.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position or null.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Indicates whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     * @return true to take into account reading position covariances, false otherwise.
     */
    public boolean getUseReadingPositionCovariance() {
        return mUseReadingPositionCovariances;
    }

    /**
     * Specifies whether position covariances of readings must be taken into account to increase
     * the amount of standard deviation of each ranging measure by the amount of position standard
     * deviation assuming that both measures are statistically independent.
     * @param useReadingPositionCovariances true to take into account reading position covariances, false
     *                                      otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseReadingPositionCovariances(boolean useReadingPositionCovariances)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseReadingPositionCovariances = useReadingPositionCovariances;
    }

    /**
     * Indicates whether this instance is ready to start the estimation.
     * @return true if this instance is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        //readings must be valid
        return areValidReadings(mReadings);
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate an initial
     * position.
     * @return true if homogeneous linear solver is used, false if an inhomogeneous linear
     * one is used instead.
     */
    public abstract boolean isHomogeneousLinearSolverUsed();

    /**
     * Specifies whether an homogeneous linear solver is used to estimate an initial
     * position.
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    public abstract void setHomogeneousLinearSolverUsed(
            boolean useHomogeneousLinearSolver) throws LockedException;

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Solves preliminar solution for a subset of samples.
     * @param samplesIndices indices of subset samples.
     * @param solutions instance where solution will be stored.
     */
    protected abstract void solvePreliminarSolutions(int[] samplesIndices,
            List<Solution<P>> solutions);

    /**
     * Estimates residual for a solution obtained for a subset of samples.
     * @param currentEstimation solution obtained for a subset of samples.
     * @param i i-th fingerprint to obtain residual for.
     * @return difference between measured and expected RSSI value.
     */
    protected double residual(Solution<P> currentEstimation, int i) {
        RangingReadingLocated<S, P> reading = mReadings.get(i);
        double distance = reading.getDistance();

        //get distance from estimated radio source position and reading position
        P readingPosition = reading.getPosition();
        P radioSourcePosition = currentEstimation.getEstimatedPosition();

        //noinspection unchecked
        return Math.abs(radioSourcePosition.distanceTo(readingPosition) - distance);
    }

    /**
     * Contains a solution obtained during robust estimation for a subset of
     * samples.
     * @param <P> a {@link Point} type.
     */
    static class Solution<P extends Point> {
        /**
         * Estimated position for a subset of samples.
         */
        private P mEstimatedPosition;

        /**
         * Constructor.
         * @param estimatedPosition estimated position for a subset of samples.
         */
        public Solution(P estimatedPosition) {
            mEstimatedPosition = estimatedPosition;
        }

        /**
         * Gets estimated position for a subset of samples.
         * @return estimated position for a subset of samples.
         */
        public P getEstimatedPosition() {
            return mEstimatedPosition;
        }
    }
}
