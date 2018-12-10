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

import org.junit.*;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class GnomonicTest {

    private static final double ABSOLUTE_ERROR = 1e-9;

    public GnomonicTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        Gnomonic g = new Gnomonic(Geodesic.WGS84);

        //check
        assertNotNull(Geodesic.WGS84);
        assertEquals(g.getMajorRadius(), Geodesic.WGS84.getMajorRadius(), 0.0);
        assertEquals(g.getFlattening(), Geodesic.WGS84.getFlattening(), 0.0);
    }

    @Test
    public void testForwardReverse() {
        Gnomonic g = new Gnomonic(Geodesic.WGS84);

        //use the following coordinates
        double lat1 = 41.382643;
        double lon1 = 2.176700;

        double lat2 = 41.382524;
        double lon2 = 2.176861;

        GnomonicData data1 = g.forward(lat1, lon1, lat2, lon2);
        GnomonicData data2 = g.reverse(data1.getLat0(), data1.getLon0(), data1.getX(), data1.getY());

        assertEquals(data1.getLat0(), lat1, 0.0);
        assertEquals(data1.getLon0(), lon1, 0.0);
        assertEquals(data1.getLat(), lat2, 0.0);
        assertEquals(data1.getLon(), lon2, 0.0);

        assertEquals(data2.getLat0(), lat1, 0.0);
        assertEquals(data2.getLon0(), lon1, 0.0);
        assertEquals(data2.getLat(), lat2, ABSOLUTE_ERROR);
        assertEquals(data2.getLon(), lon2, ABSOLUTE_ERROR);
    }
}
