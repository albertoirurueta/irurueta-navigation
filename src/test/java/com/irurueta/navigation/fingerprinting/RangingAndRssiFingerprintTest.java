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

import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RangingAndRssiFingerprintTest {

    public RangingAndRssiFingerprintTest() { }

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
        //empty constructor
        RangingAndRssiFingerprint<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint =
                new RangingAndRssiFingerprint<>();

        //check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());


        //constructor with readings
        List<RangingAndRssiReading<RadioSource>> readings = new ArrayList<>();
        fingerprint = new RangingAndRssiFingerprint<>(readings);

        //check
        assertSame(fingerprint.getReadings(), readings);

        //force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingAndRssiFingerprint<>(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(fingerprint);
    }

    @Test
    public void testGetSetReadings() {
        RangingAndRssiFingerprint<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint =
                new RangingAndRssiFingerprint<>();

        //check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());

        //set new value
        List<RangingAndRssiReading<RadioSource>> readings = new ArrayList<>();
        fingerprint.setReadings(readings);

        //check
        assertSame(fingerprint.getReadings(), readings);

        //force IllegalArgumentException
        try {
            fingerprint.setReadings(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
