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

import com.irurueta.navigation.inertial.calibration.Triad;
import com.irurueta.units.Measurement;

/**
 * Base listener for {@link WindowedTriadNoiseEstimator} to handle generated events.
 *
 * @param <U> a measurement unit type.
 * @param <M> a measurement type.
 * @param <T> a triad type.
 * @param <E> an estimator type.
 */
public interface WindowedTriadNoiseEstimatorListener<U extends Enum<?>,
        M extends Measurement<U>, T extends Triad<U, M>,
        E extends WindowedTriadNoiseEstimator<U, M, T, E, ?>> {

    /**
     * Called when estimation starts.
     *
     * @param estimator estimator that raised the event.
     */
    void onStart(final E estimator);

    /**
     * Called when a triad sample is added containing new measurement values.
     *
     * @param estimator estimator that raised the event.
     */
    void onTriadAdded(final E estimator);

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
