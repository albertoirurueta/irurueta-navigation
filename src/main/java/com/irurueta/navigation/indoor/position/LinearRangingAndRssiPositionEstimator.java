/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.navigation.indoor.*;
import com.irurueta.navigation.trilateration.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Linearly estimates position using located radio sources and their RSSI+ranging readings
 * at unknown locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting RSSI+ranging readings at an unknown location of different radio sources whose
 * locations are known.
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class LinearRangingAndRssiPositionEstimator<P extends Point> extends
        RangingAndRssiPositionEstimator<P> {

    /**
     * Indicates that by default an homogeneous linear solver is used to estimate
     * position.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER = true;

    /**
     * An homogeneous linear trilateration solver to solve position.
     */
    protected HomogeneousLinearLeastSquaresTrilaterationSolver<P> mHomogeneousTrilaterationSolver;

    /**
     * An inhomogeneous linear trilateration solver to solve position.
     */
    protected InhomogeneousLinearLeastSquaresTrilaterationSolver<P> mInhomogeneousTrilaterationSolver;

    /**
     * Listener for the trilateration solver.
     */
    protected TrilaterationSolverListener<P> mTrilaterationSolverListener;

    /**
     * Indicates whether an homogeneous linear solver is used to estimate position.
     */
    protected boolean mUseHomogeneousLinearSolver = DEFAULT_USE_HOMOGENEOUS_LINEAR_SOLVER;

    /**
     * Constructor.
     */
    public LinearRangingAndRssiPositionEstimator() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public LinearRangingAndRssiPositionEstimator(
            RangingAndRssiPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Indicates whether an homogeneous linear solver is used to estimate position.
     *
     * @return  true if homogeneous linear solver is used, false if an inhomogeneous
     *          linear one is used instead.
     */
    public boolean isHomogeneousLinearSolverUsed() {
        return mUseHomogeneousLinearSolver;
    }

    /**
     * Specifies whether an homogeneous linear solver is used to estimate position.
     *
     * @param useHomogeneousLinearSolver true if homogeneous linear solver is used, false
     *                                   if an inhomogeneous linear one is used instead.
     * @throws LockedException if estimator is locked.
     */
    public void setHomogeneousLinearSolverUsed(boolean useHomogeneousLinearSolver)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mUseHomogeneousLinearSolver = useHomogeneousLinearSolver;
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     *
     * @return minimum required number of located radio sources to perform trilateration.
     */
    @Override
    public int getMinRequiredSources() {
        return mUseHomogeneousLinearSolver?
                mHomogeneousTrilaterationSolver.getMinRequiredPositionsAndDistances() :
                mInhomogeneousTrilaterationSolver.getMinRequiredPositionsAndDistances();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return (!mUseHomogeneousLinearSolver && mInhomogeneousTrilaterationSolver.isReady()) ||
                (mUseHomogeneousLinearSolver && mHomogeneousTrilaterationSolver.isReady());
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation is already in progress.
     * @return true if estimator is locked, false otherwise.
     */
    @Override
    public boolean isLocked() {
        return mInhomogeneousTrilaterationSolver.isLocked() ||
                mHomogeneousTrilaterationSolver.isLocked();
    }

    /**
     * Estimates position based on provided located radio sources and RSSI readings of
     * such radio sources at an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    @SuppressWarnings("Duplicates")
    @Override
    public void estimate() throws LockedException, NotReadyException,
            PositionEstimationException {
        try {
            if (mUseHomogeneousLinearSolver) {
                mHomogeneousTrilaterationSolver.solve();
                mEstimatedPositionCoordinates =
                        mHomogeneousTrilaterationSolver.getEstimatedPositionCoordinates();
            } else {
                mInhomogeneousTrilaterationSolver.solve();
                mEstimatedPositionCoordinates =
                        mInhomogeneousTrilaterationSolver.getEstimatedPositionCoordinates();
            }
        } catch (TrilaterationException e) {
            throw new PositionEstimationException(e);
        }
    }

    /**
     * Gets known positions of radio sources used internally to solve trilateration.
     *
     * @return known positions used internally.
     */
    @Override
    public P[] getPositions() {
        return mUseHomogeneousLinearSolver ?
                mHomogeneousTrilaterationSolver.getPositions() :
                mInhomogeneousTrilaterationSolver.getPositions();
    }

    /**
     * Gets euclidean distances from known located radio sources to
     * the location of provided readings in a fingerprint.
     * Distance values are used internally to solve trilateration.
     *
     * @return euclidean distances used internally.
     */
    @Override
    public double[] getDistances() {
        return mUseHomogeneousLinearSolver ?
                mHomogeneousTrilaterationSolver.getDistances() :
                mInhomogeneousTrilaterationSolver.getDistances();
    }

    /**
     * Internally sets located radio sources used for trilateration.
     *
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided value is null or the number of
     *                                  provided sources is less than the required
     *                                  minimum.
     */
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources) {
        super.internalSetSources(sources);
        buildPositionsAndDistances();
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided
     * located radio sources.
     *
     * @param fingerprint fingerprint containing readings at an unknown location for
     *                    provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends RangingAndRssiReading<? extends RadioSource>> fingerprint) {
        super.internalSetFingerprint(fingerprint);
        buildPositionsAndDistances();
    }

    /**
     * Sets positions and distances on internal trilateration solver.
     *
     * @param positions positions to be set.
     * @param distances distances to be set.
     */
    protected abstract void setPositionsAndDistances(List<P> positions,
                                                     List<Double> distances);

    /**
     * Initializes trilateration solver listener.
     */
    @SuppressWarnings("Duplicates")
    private void init() {
        mTrilaterationSolverListener = new TrilaterationSolverListener<P>() {
            @Override
            public void onSolveStart(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            LinearRangingAndRssiPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            LinearRangingAndRssiPositionEstimator.this);
                }
            }
        };
    }

    /**
     * Builds positions and distances for the internal trilateration solver.
     */
    @SuppressWarnings("Duplicates")
    private void buildPositionsAndDistances() {
        if ((mUseHomogeneousLinearSolver && mHomogeneousTrilaterationSolver == null) ||
                (!mUseHomogeneousLinearSolver && mInhomogeneousTrilaterationSolver == null)) {
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
        PositionEstimatorHelper.buildPositionsAndDistances(
                mSources, mFingerprint, positions, distances);

        setPositionsAndDistances(positions, distances);
    }
}
