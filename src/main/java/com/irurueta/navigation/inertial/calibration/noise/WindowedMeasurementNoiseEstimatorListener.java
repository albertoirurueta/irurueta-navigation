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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.units.Measurement;

/**
 * Base listener for {@link WindowedMeasurementNoiseEstimator} to handle generated events.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <E> an estimator type.
 */
public interface WindowedMeasurementNoiseEstimatorListener<U extends Enum<?>,
        M extends Measurement<U>, E extends WindowedMeasurementNoiseEstimator<U, M, E, ?>> {

    /**
     * Called when estimation starts.
     *
     * @param estimator estimator that raised the event.
     */
    void onStart(final E estimator);

    /**
     * Called when a triad sample is added containing new measures.
     *
     * @param estimator estimator that raised the event.
     */
    void onMeasurementAdded(final E estimator);

    /**
     * Called when window of samples is filled.
     *
     * @param estimator estimator that raised the event.
     */
    void onWindowFilled(final E estimator);

    /**
     * Called when estimation is reset.
     *
     * @param estimator estimator that raised the event.
     */
    void onReset(final E estimator);
}
