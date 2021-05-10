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

public class GeodesicMaskTest {

    @Test
    public void testConstants() {
        assertEquals(GeodesicMask.NONE, 0);
        assertEquals(GeodesicMask.LATITUDE, 1 << 7);
        assertEquals(GeodesicMask.LONGITUDE, 1 << 8 | 1 << 3);
        assertEquals(GeodesicMask.AZIMUTH, 1 << 9);
        assertEquals(GeodesicMask.DISTANCE, 1 << 10 | 1);
        assertEquals(GeodesicMask.STANDARD,
                GeodesicMask.LATITUDE | GeodesicMask.LONGITUDE |
                        GeodesicMask.AZIMUTH | GeodesicMask.DISTANCE);
        assertEquals(GeodesicMask.DISTANCE_IN, 1 << 11 | 1 | 1 << 1);
        assertEquals(GeodesicMask.REDUCED_LENGTH, 1 << 12 | 1 | 1 << 2);
        assertEquals(GeodesicMask.GEODESIC_SCALE, 1 << 13 | 1 | 1 << 2);
        assertEquals(GeodesicMask.AREA, 1 << 14 | 1 << 4);
        assertEquals(GeodesicMask.ALL, 0x7F80 | 0x1F);
        assertEquals(GeodesicMask.LONG_UNROLL, 1 << 15);
    }
}
