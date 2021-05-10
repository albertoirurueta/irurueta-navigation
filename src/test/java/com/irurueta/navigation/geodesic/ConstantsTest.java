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

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ConstantsTest {

    @Test
    public void testConstants() {
        assertEquals(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 6378137, 0.0);
        assertEquals(Constants.EARTH_POLAR_RADIUS_WGS84, 6356752.31425, 0.0);
        assertEquals(Constants.EARTH_FLATTENING_WGS84, 1 / 298.257223563, 0.0);
        assertEquals(Constants.EARTH_ECCENTRICITY, 0.0818191908425, 0.0);
        assertEquals(Constants.EARTH_GRAVITATIONAL_CONSTANT, 3.986004418E14, 0.0);
        assertEquals(Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 1.082627E-3, 0.0);
        assertEquals(Constants.EARTH_ROTATION_RATE, 7.292115E-5, 0.0);
        assertEquals(Constants.SPEED_OF_LIGHT, 299792458.0, 0.0);
    }
}
