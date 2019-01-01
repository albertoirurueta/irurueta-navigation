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
package com.irurueta.navigation.indoor.fingerprint;


/**
 * Listener to be notified of events such as when estimation of position
 * using fingerprints starts or ends.
 * @param <E> a {@link BaseRssiPositionEstimator} type.
 */
public interface BaseRssiPositionEstimatorListener<E extends BaseRssiPositionEstimator> {

    /**
     * Called when estimation starts.
     * @param estimator estimator raising the event.
     */
    void onEstimateStart(E estimator);

    /**
     * Called when estimation ends.
     * @param estimator estimator raising the event.
     */
    void onEstimateEnd(E estimator);
}
