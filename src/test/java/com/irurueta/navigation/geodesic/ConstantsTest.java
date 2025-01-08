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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class ConstantsTest {

    @Test
    void testConstants() {
        assertEquals(6378137, Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(6356752.31425, Constants.EARTH_POLAR_RADIUS_WGS84, 0.0);
        assertEquals(1 / 298.257223563, Constants.EARTH_FLATTENING_WGS84, 0.0);
        assertEquals(0.0818191908425, Constants.EARTH_ECCENTRICITY, 0.0);
        assertEquals(3.986004418E14, Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(1.082627E-3, Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(7.292115E-5, Constants.EARTH_ROTATION_RATE, 0.0);
        assertEquals(299792458.0, Constants.SPEED_OF_LIGHT, 0.0);
    }
}
