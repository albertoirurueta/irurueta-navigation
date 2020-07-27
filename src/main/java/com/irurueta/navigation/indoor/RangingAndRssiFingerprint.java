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

import java.util.List;

/**
 * Contains ranging and RSSI readings from several radio sources for an unknown
 * location to be determined.
 *
 * @param <R> a {@link RssiReading} type.
 * @param <S> a {@link RadioSource} type.
 */
public class RangingAndRssiFingerprint<S extends RadioSource,
        R extends RangingAndRssiReading<S>> extends Fingerprint<S, R> {

    /**
     * Constructor.
     */
    public RangingAndRssiFingerprint() {
        super();
    }

    /**
     * Constructor.
     *
     * @param readings non-located ranging and RSSI readings.
     * @throws IllegalArgumentException if provided readings is null.
     */
    public RangingAndRssiFingerprint(final List<R> readings) {
        super(readings);
    }
}
