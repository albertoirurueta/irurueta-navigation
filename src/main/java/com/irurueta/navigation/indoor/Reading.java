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

import java.io.Serializable;

/**
 * Contains a reading related to a given WiFi access point.
 * @param <S> a {@link RadioSource} type.
 */
public abstract class Reading<S extends RadioSource> implements Serializable {

    /**
     * Radio source associated to this reading.
     */
    private S mSource;

    /**
     * Constructor.
     * @param source radio source associated to this reading.
     * @throws IllegalArgumentException if radios source data is null.
     */
    public Reading(S source) {
        if (source == null) {
            throw new IllegalArgumentException();
        }
        mSource = source;
    }

    /**
     * Empty constructor.
     */
    protected Reading() { }

    /**
     * Gets radio source associated to this reading.
     * @return radio source associated to this reading.
     */
    public S getSource() {
        return mSource;
    }

    /**
     * Determine whether another reading is made for the same radio source as
     * this reading or not.
     * @param otherReading other reading to be compared.
     * @return true if both readings are associated to the same access point,
     * false otherwise.
     */
    public boolean hasSameSource(Reading<S> otherReading) {
        return otherReading != null && otherReading.mSource.equals(mSource);
    }

    /**
     * Contains radio source reading type.
     * @return reading type.
     */
    public abstract ReadingType getType();
}
