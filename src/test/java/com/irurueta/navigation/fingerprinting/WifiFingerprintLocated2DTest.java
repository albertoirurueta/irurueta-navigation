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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class WifiFingerprintLocated2DTest {

    public WifiFingerprintLocated2DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws WrongSizeException {
        //test empty constructor
        WifiFingerprintLocated2D<WifiAccessPoint, RssiReading<WifiAccessPoint>> f =
                new WifiFingerprintLocated2D<>();

        //check default values
        assertTrue(f.getReadings().isEmpty());
        assertNull(f.getPosition());
        assertNull(f.getPositionCovariance());


        //test with readings and position
        List<RssiReading<WifiAccessPoint>> readings = new ArrayList<>();
        InhomogeneousPoint2D position = new InhomogeneousPoint2D();
        f = new WifiFingerprintLocated2D<>(readings, position);

        //check
        assertSame(f.getReadings(), readings);
        assertSame(f.getPosition(), position);
        assertNull(f.getPositionCovariance());

        //force IllegalArgumentException
        f = null;
        try {
            f = new WifiFingerprintLocated2D<>(null, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            f = new WifiFingerprintLocated2D<>(readings, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(f);


        //test with readings, position and covariance
        Matrix cov = new Matrix(2, 2);
        f = new WifiFingerprintLocated2D<>(readings, position, cov);

        //check
        assertSame(f.getReadings(), readings);
        assertSame(f.getPosition(), position);
        assertSame(f.getPositionCovariance(), cov);

        f = null;
        try {
            f = new WifiFingerprintLocated2D<>(null, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            f = new WifiFingerprintLocated2D<>(readings, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            f = new WifiFingerprintLocated2D<>(readings, position,
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(f);
    }
}
