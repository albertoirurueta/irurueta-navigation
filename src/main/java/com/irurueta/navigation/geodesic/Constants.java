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
package com.irurueta.navigation.geodesic;

/**
 * Constants used for GNSS/INS navigation.
 */
@SuppressWarnings("WeakerAccess")
public class Constants {

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = 6378137;

    /**
     * The flattening of WGS84 ellipsoid (1 / 298.257223563).
     */
    public static final double EARTH_FLATTENING_WGS84 = 1/298.257223563;

    /**
     * Constructor.
     * Prevents instantiation.
     */
    Constants() { }
}
