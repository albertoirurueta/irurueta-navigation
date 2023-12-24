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
import static org.junit.Assert.assertNotNull;

public class GnomonicTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    @Test
    public void testConstructor() {
        final Gnomonic g = new Gnomonic(Geodesic.WGS84);

        //check
        assertNotNull(Geodesic.WGS84);
        assertEquals(Geodesic.WGS84.getMajorRadius(), g.getMajorRadius(), 0.0);
        assertEquals(Geodesic.WGS84.getFlattening(), g.getFlattening(), 0.0);
    }

    @Test
    public void testForwardReverse() {
        final Gnomonic g = new Gnomonic(Geodesic.WGS84);

        //use the following coordinates
        final double lat1 = 41.382643;
        final double lon1 = 2.176700;

        final double lat2 = 41.382524;
        final double lon2 = 2.176861;

        final GnomonicData data1 = g.forward(lat1, lon1, lat2, lon2);
        final GnomonicData data2 = g.reverse(data1.getLat0(), data1.getLon0(), data1.getX(), data1.getY());

        assertEquals(lat1, data1.getLat0(), 0.0);
        assertEquals(lon1, data1.getLon0(), 0.0);
        assertEquals(lat2, data1.getLat(), 0.0);
        assertEquals(lon2, data1.getLon(), 0.0);

        assertEquals(lat1, data2.getLat0(), 0.0);
        assertEquals(lon1, data2.getLon0(), 0.0);
        assertEquals(lat2, data2.getLat(), ABSOLUTE_ERROR);
        assertEquals(lon2, data2.getLon(), ABSOLUTE_ERROR);
    }
}
