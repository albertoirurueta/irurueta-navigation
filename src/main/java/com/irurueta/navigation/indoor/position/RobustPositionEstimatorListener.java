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

/**
 * Listener to be notified of events produced by a robust position estimator when
 * estimation starts, ends or when progress changes.
 *
 * @param <E> a {@link RobustPositionEstimator} type.
 */
public interface RobustPositionEstimatorListener<E extends RobustPositionEstimator> {

    /**
     * Called when estimation starts.
     *
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(E estimator);

    /**
     * Called when estimation ends.
     *
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(E estimator);

    /**
     * Called when estimator iterates to refine a possible solution.
     *
     * @param estimator estimator raising the event.
     * @param iteration current iteration.
     */
    void onEstimateNextIteration(E estimator, int iteration);

    /**
     * Called when estimation progress changes significantly.
     *
     * @param estimator estimator raising the event.
     * @param progress  progress of estimation expressed as a value between 0.0 and 1.0.
     */
    void onEstimateProgressChange(E estimator, float progress);
}
