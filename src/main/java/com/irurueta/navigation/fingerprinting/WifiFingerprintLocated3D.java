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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;

import java.util.List;

/**
 * Contains 3D located WiFi fingerprint.
 * @param <AP> a {@link WifiAccessPoint} type.
 * @param <R> a {@link WifiRssiReading} type.
 */
@SuppressWarnings("WeakerAccess")
public class WifiFingerprintLocated3D<AP extends WifiAccessPoint, R extends WifiRssiReading<AP>>
        extends WifiFingerprintLocated<AP, R, Point3D> {

    /**
     * Constructor.
     * @param readings non-located WiFi readings defining the fingerprint.
     * @param position position where readings were made.
     * @throws IllegalArgumentException if either readings or position are null.
     */
    public WifiFingerprintLocated3D(List<R> readings, Point3D position)
            throws IllegalArgumentException {
        super(readings, position);
    }

    /**
     * Constructor.
     * @param readings non-located WiFi readings defining the fingerprint.
     * @param position position where readings were made.
     * @param positionCovariance 2x2 covariance of inhomogeneous coordinates of
     *                           current position (if available).
     * @throws IllegalArgumentException if either readings or position are null.
     */
    public WifiFingerprintLocated3D(List<R> readings, Point3D position,
                                    Matrix positionCovariance) throws IllegalArgumentException {
        super(readings, position, positionCovariance);
    }

    /**
     * Empty constructor.
     */
    protected WifiFingerprintLocated3D() {
        super();
    }
}
