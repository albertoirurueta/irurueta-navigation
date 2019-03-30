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
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.Reading;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolver;
import com.irurueta.navigation.trilateration.RobustTrilaterationSolverListener;

/**
 * Base class for robust position estimators using located radio sources and their
 * readings at unknown locations.
 * These kind of estimators can be used to robustly determine the position of a given
 * device by getting readings at an unknown location of different radio sources whose
 * locations are known.
 * Implementations of this class should be able to detect and discard outliers in order to
 * find the best solution.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class RobustMixedPositionEstimator<P extends Point> extends
        RobustPositionEstimator<P, Reading<? extends RadioSource>,
        RobustMixedPositionEstimatorListener<P>> {

    /**
     * Constructor.
     */
    public RobustMixedPositionEstimator() {
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustMixedPositionEstimator(
            RobustMixedPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Initializes robust trilateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new RobustTrilaterationSolverListener<P>() {
            @Override
            public void onSolveStart(RobustTrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(RobustMixedPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(RobustTrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(RobustMixedPositionEstimator.this);
                }
            }

            @Override
            public void onSolveNextIteration(RobustTrilaterationSolver<P> solver,
                                             int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RobustMixedPositionEstimator.this, iteration);
                }
            }

            @Override
            public void onSolveProgressChange(RobustTrilaterationSolver<P> solver,
                                              float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RobustMixedPositionEstimator.this, progress);
                }
            }
        };
    }
}
