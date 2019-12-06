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
import com.irurueta.navigation.indoor.RangingReading;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolverListener;

/**
 * Base class for robust ranging position estimators using located radio sources and their
 * ranging readings at unknown locations.
 * These kind of estimators can be used to robustly determine the position of a given device
 * by getting ranging readings at an unknown location of different radio sources whose
 * locations are known.
 * Implementations of this class should be able to detect and discard outliers in order to
 * find the best solution.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class RobustRangingPositionEstimator<P extends Point<?>> extends
        RobustPositionEstimator<P, RangingReading<? extends RadioSource>,
                RobustRangingPositionEstimatorListener<P>> {

    /**
     * Constructor.
     */
    public RobustRangingPositionEstimator() {
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustRangingPositionEstimator(
            RobustRangingPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Initializes robust lateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new RobustLaterationSolverListener<P>() {
            @Override
            public void onSolveStart(RobustLaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            RobustRangingPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(RobustLaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            RobustRangingPositionEstimator.this);
                }
            }

            @Override
            public void onSolveNextIteration(RobustLaterationSolver<P> solver,
                                             int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RobustRangingPositionEstimator.this, iteration);
                }
            }

            @Override
            public void onSolveProgressChange(RobustLaterationSolver<P> solver,
                                              float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RobustRangingPositionEstimator.this, progress);
                }
            }
        };
    }
}
