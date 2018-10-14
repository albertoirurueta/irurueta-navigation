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
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.Point;

/**
 * Listener to be notified of events produced by a sequential robust radio source
 * estimator using mixed readings when estimation starts, ends or when progress
 * changes.
 */
public interface SequentialRobustMixedRadioSourceEstimatorListener<S extends RadioSource,
        P extends Point> {

    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(SequentialRobustMixedRadioSourceEstimator<S, P> estimator);

    /**
     * Called when estimation ends.
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(SequentialRobustMixedRadioSourceEstimator<S, P> estimator);

    /**
     * Called when estimation progress significantly changes.
     * @param estimator estimator raising the event.
     * @param progress progress of estimation expressed as a value between 0.0 and 1.0.
     */
    void onEstimateProgressChange(SequentialRobustMixedRadioSourceEstimator<S, P> estimator,
                                  float progress);
}
