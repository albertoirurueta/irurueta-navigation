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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.Fingerprint;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RadioSourceLocated;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.trilateration.NonLinearLeastSquaresTrilaterationSolver;
import com.irurueta.navigation.trilateration.TrilaterationException;
import com.irurueta.navigation.trilateration.TrilaterationSolver;
import com.irurueta.navigation.trilateration.TrilaterationSolverListener;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates position non-linearly using located radio sources and their readings at unknown
 * locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting readings at an unknown location of different radio source whose locations are
 * known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class NonLinearPositionEstimator<P extends Point> extends PositionEstimator<P> {

    /**
     * Minimum required number of sources to solve trilateration.
     */
    public static final int MIN_SOURCES = NonLinearLeastSquaresTrilaterationSolver.MIN_POINTS;

    /**
     * Distance standard deviation assumed for provided distances as a fallback when
     * none can be determined.
     */
    public static final double FALLBACK_DISTANCE_STANDARD_DEVIATION =
            NonLinearLeastSquaresTrilaterationSolver.DEFAULT_DISTANCE_STANDARD_DEVIATION;

    /**
     * A non-linear trilateration solver to solve position.
     */
    protected NonLinearLeastSquaresTrilaterationSolver<P> mTrilaterationSolver;

    /**
     * Listener for the trilateration solver.
     */
    protected TrilaterationSolverListener<P> mTrilaterationSolverListener;

    /**
     * Initial position to start position estimation.
     * If not defined, centroid of provided located sources will be used.
     */
    private P mInitialPosition;

    /**
     * Indicates whether located radio source position covariance must be taken
     * into account (if available) to determine distance standard deviation.
     */
    private boolean mUseRadioSourcePositionCovariance;

    /**
     * Distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     */
    private double mFallbackDistanceStandardDeviation =
            FALLBACK_DISTANCE_STANDARD_DEVIATION;

    /**
     * Constructor.
     */
    public NonLinearPositionEstimator() {
        super();
        init();
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public NonLinearPositionEstimator(PositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start position estimation.
     */
    public NonLinearPositionEstimator(P initialPosition) {
        this();
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start position estimation.
     * @param listener listener in charge of handling events.
     */
    public NonLinearPositionEstimator(P initialPosition,
            PositionEstimatorListener<P> listener) {
        this(listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Gets initial position to start position estimation.
     * If not defined, centroid of located sources position will be used to start
     * the estimation.
     * @return initial position to start position estimation.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start position estimation.
     * If not defined, centroid of located sources position will be used to start
     * the estimation.
     * @param initialPosition initial position to start position estimation.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mInitialPosition = initialPosition;
    }

    /**
     * Indicates whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     * @return true to take radio source position covariance into account, false
     * otherwise.
     */
    public boolean isRadioSourcePositionCovarianceUsed() {
        return mUseRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether located radio source position covariance must be taken into
     * account (if available) to determine distance standard deviation.
     * @param useRadioSourcePositionCovariance true to take radio source position
     *                                         covariance into account, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovarianceUsed(
            boolean useRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseRadioSourcePositionCovariance = useRadioSourcePositionCovariance;
    }

    /**
     * Gets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     * @return distance standard deviation to use as fallback.
     */
    public double getFallbackDistanceStandardDeviation() {
        return mFallbackDistanceStandardDeviation;
    }

    /**
     * Sets distance standard deviation fallback value to use when none can be
     * determined from provided radio sources and fingerprint readings.
     * @param fallbackDistanceStandardDeviation distance standard deviation to use
     *                                          as fallback.
     * @throws LockedException if estimator is locked.
     */
    public void setFallbackDistanceStandardDeviation(
            double fallbackDistanceStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mFallbackDistanceStandardDeviation = fallbackDistanceStandardDeviation;
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     * @return minimum required number of located radio sources to perform trilateration.
     */
    @Override
    public int getMinRequiredSources() {
        return MIN_SOURCES;
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mTrilaterationSolver.isReady();
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation is already in progress.
     * @return true if estimator is locked, false otherwise.
     */
    @Override
    public boolean isLocked() {
        return mTrilaterationSolver.isLocked();
    }

    /**
     * Gets standard deviations of distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance standard deviations are used internally to solve trilateration.
     * @return standard deviations used internally.
     */
    public double[] getDistanceStandardDeviations() {
        return mTrilaterationSolver.getDistanceStandardDeviations();
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException,
            PositionEstimationException {
        try {
            mTrilaterationSolver.setInitialPosition(mInitialPosition);

            mTrilaterationSolver.solve();
            mEstimatedPositionCoordinates =
                    mTrilaterationSolver.getEstimatedPositionCoordinates();
        } catch (TrilaterationException e) {
            throw new PositionEstimationException(e);
        }
    }

    /**
     * Gets known positions of radio sources used internally to solve trilateration.
     * @return known positions used internally.
     */
    @Override
    public P[] getPositions() {
        return mTrilaterationSolver.getPositions();
    }

    /**
     * Gets euclidean distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance values are used internally to solve trilateration.
     * @return euclidean distances used internally.
     */
    @Override
    public double[] getDistances() {
        return mTrilaterationSolver.getDistances();
    }

    /**
     * Internally sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources)
            throws IllegalArgumentException {
        super.internalSetSources(sources);
        buildPositionsDistancesAndDistanceStandardDeviations();
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        super.internalSetFingerprint(fingerprint);
        buildPositionsDistancesAndDistanceStandardDeviations();
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal trilateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set.
     */
    protected abstract void setPositionsDistancesAndDistanceStandardDeviations(List<P> positions,
            List<Double> distances, List<Double> distanceStandardDeviations);

    /**
     * Initializes trilateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new TrilaterationSolverListener<P>() {
            @Override
            public void onSolveStart(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(NonLinearPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(NonLinearPositionEstimator.this);
                }
            }
        };
    }

    /**
     * Builds positions, distances and standard deviation of distances for the internal
     * trilateration solver.
     */
    @SuppressWarnings("unchecked")
    private void buildPositionsDistancesAndDistanceStandardDeviations() {
        if (mTrilaterationSolver == null) {
            return;
        }

        int min = getMinRequiredSources();
        if (mSources == null || mFingerprint == null ||
                mSources.size() < min ||
                mFingerprint.getReadings() == null ||
                mFingerprint.getReadings().size() < min) {
            return;
        }

        List<P> positions = new ArrayList<>();
        List<Double> distances = new ArrayList<>();
        List<Double> distanceStandardDeviations = new ArrayList<>();
        PositionEstimatorHelper.buildPositionsDistancesAndDistanceStandardDeviations(
                mSources, mFingerprint, mUseRadioSourcePositionCovariance,
                mFallbackDistanceStandardDeviation, positions, distances,
                distanceStandardDeviations);

        setPositionsDistancesAndDistanceStandardDeviations(positions, distances,
                distanceStandardDeviations);
    }
}
