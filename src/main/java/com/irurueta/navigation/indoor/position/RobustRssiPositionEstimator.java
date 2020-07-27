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
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.lateration.RobustLaterationSolver;
import com.irurueta.navigation.lateration.RobustLaterationSolverListener;


/**
 * Base class for robust RSSI position estimators using located radio sources and their
 * RSSI readings at unknown locations.
 * These kind of estimators can be used to robustly determine the position of a given
 * device by getting RSSI readings at an unknown location of different radio sources
 * whose locations are known.
 * Implementations of this class should be able to detect and discard outliers in order
 * to find the best solution.
 *
 * @param <P> a {@link Point} type.
 */
public abstract class RobustRssiPositionEstimator<P extends Point<?>> extends RobustPositionEstimator<P,
        RssiReading<? extends RadioSource>, RobustRssiPositionEstimatorListener<P>> {

    /**
     * Constructor.
     */
    public RobustRssiPositionEstimator() {
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public RobustRssiPositionEstimator(final RobustRssiPositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Initializes robust lateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new RobustLaterationSolverListener<P>() {
            @Override
            public void onSolveStart(final RobustLaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            RobustRssiPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(final RobustLaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            RobustRssiPositionEstimator.this);
                }
            }

            @Override
            public void onSolveNextIteration(final RobustLaterationSolver<P> solver,
                                             final int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RobustRssiPositionEstimator.this, iteration);
                }
            }

            @Override
            public void onSolveProgressChange(final RobustLaterationSolver<P> solver,
                                              final float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RobustRssiPositionEstimator.this, progress);
                }
            }
        };
    }
}
