/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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

/**
 * Listener for IMUTimeIntervalEstimator to handle generated events.
 */
public interface IMUTimeIntervalEstimatorListener {

    /**
     * Called when estimation starts.
     *
     * @param estimator estimator that raised the event.
     */
    void onStart(final IMUTimeIntervalEstimator estimator);

    /**
     * Called when a timestamp is added.
     *
     * @param estimator estimator that raised the event.
     */
    void onTimestampAdded(final IMUTimeIntervalEstimator estimator);

    /**
     * Called when estimation finishes after processing all required samples.
     *
     * @param estimator estimator that raised the event.
     */
    void onFinish(final IMUTimeIntervalEstimator estimator);

    /**
     * Called when estimation is reset.
     *
     * @param estimator estimator that raised the event.
     */
    void onReset(final IMUTimeIntervalEstimator estimator);
}
