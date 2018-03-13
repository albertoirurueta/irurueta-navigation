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
package com.irurueta.navigation.fingerprinting;

import java.io.Serializable;

/**
 * Contains a reading related to a given WiFi access point.
 * @param <AP> a {@link WifiAccessPoint} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class WifiReading<AP extends WifiAccessPoint> implements Serializable {

    /**
     * Access point associated to this reading.
     */
    private AP mAccessPoint;

    /**
     * Constructor.
     * @param accessPoint access point associated to this reading.
     * @throws IllegalArgumentException if access point data is null.
     */
    public WifiReading(AP accessPoint)
            throws IllegalArgumentException {
        if (accessPoint == null) {
            throw new IllegalArgumentException();
        }
        mAccessPoint = accessPoint;
    }

    /**
     * Empty constructor.
     */
    protected WifiReading() { }

    /**
     * Gets access point associated to this reading.
     * @return access point associated to this reading.
     */
    public WifiAccessPoint getAccessPoint() {
        return mAccessPoint;
    }

    /**
     * Determine whether another reading is made for the same access point as
     * this reading or not.
     * @param otherReading other reading to be compared.
     * @return true if both readings are associated to the same access point,
     * false otherwise.
     */
    public boolean hasSameAccessPoint(WifiReading<AP> otherReading) {
        return otherReading != null && otherReading.mAccessPoint.equals(mAccessPoint);
    }
}
