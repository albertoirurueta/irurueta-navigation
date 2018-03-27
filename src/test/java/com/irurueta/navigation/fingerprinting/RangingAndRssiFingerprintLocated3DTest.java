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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RangingAndRssiFingerprintLocated3DTest {

    public RangingAndRssiFingerprintLocated3DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws AlgebraException {
        //empty constructor
        RangingAndRssiFingerprintLocated3D<RadioSource, RangingAndRssiReading<RadioSource>> fingerprint =
                new RangingAndRssiFingerprintLocated3D<>();

        //check
        assertNotNull(fingerprint.getReadings());
        assertTrue(fingerprint.getReadings().isEmpty());
        assertNull(fingerprint.getPosition());
        assertNull(fingerprint.getPositionCovariance());

        //constructor with readings and position
        List<RangingAndRssiReading<RadioSource>> readings = new ArrayList<>();
        Point3D position = new InhomogeneousPoint3D();
        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position);

        //check
        assertSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertNull(fingerprint.getPositionCovariance());

        //force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(fingerprint);

        //constructor with readings, position and covariance
        Matrix cov = new Matrix(3,3);
        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position, cov);

        //check
        assertSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertSame(fingerprint.getPositionCovariance(), cov);

        fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position, null);

        //check
        assertSame(fingerprint.getReadings(), readings);
        assertSame(fingerprint.getPosition(), position);
        assertNull(fingerprint.getPositionCovariance());

        //force IllegalArgumentException
        fingerprint = null;
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(null, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            fingerprint = new RangingAndRssiFingerprintLocated3D<>(readings, position,
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(fingerprint);
    }
}
