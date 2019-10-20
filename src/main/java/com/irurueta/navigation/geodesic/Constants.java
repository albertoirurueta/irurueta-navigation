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
    public static final double EARTH_FLATTENING_WGS84 = 1 / 298.257223563;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = 0.0818191908425;

    /**
     * WGS84 Earth gravitational constant expressed in m^3 * s^-2
     */
    public static final double EARTH_GRAVITATIONAL_CONSTANT = 3.986004418E14;

    /**
     * WGS84 Earth's second gravitational constant.
     */
    public static final double EARTH_SECOND_GRAVITATIONAL_CONSTANT = 1.082627E-3;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = 7.292115E-5;

    /**
     * Constructor.
     * Prevents instantiation.
     */
    Constants() {
    }
}
