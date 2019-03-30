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

/**
 * Estimates position using located radio sources and any kind of readings at an unknown
 * location (i.e. a non located fingerprint).
 * Implementations of this estimator can be used to determine the position of a given
 * device by getting a mixture of different kind of readings at an unknown location of
 * different radio sources whose locations are known.
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class MixedPositionEstimator<P extends Point> extends PositionEstimator<P,
        Reading<? extends RadioSource>,
        MixedPositionEstimatorListener<P>> {

    /**
     * Constructor.
     */
    public MixedPositionEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener in charge of handling events.
     */
    public MixedPositionEstimator(MixedPositionEstimatorListener<P> listener) {
        super(listener);
    }
}
