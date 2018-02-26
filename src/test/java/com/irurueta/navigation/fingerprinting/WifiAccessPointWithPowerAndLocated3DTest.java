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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class WifiAccessPointWithPowerAndLocated3DTest {

    private static final String BSSID = "bssid";
    private static final String SSID = "ssid";
    private static final double FREQUENCY = 2.4e9;
    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;
    private static final double TRANSMITTED_POWER = -50.0;
    private static final double TRANSMITTED_POWER_STD = 0.5;

    public WifiAccessPointWithPowerAndLocated3DTest() { }

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
        //test empty constructor
        WifiAccessPointWithPowerAndLocated3D ap =
                new WifiAccessPointWithPowerAndLocated3D();

        //check default values
        assertNull(ap.getBssid());
        assertEquals(ap.getFrequency(), 0.0, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), 0.0, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertNull(ap.getPosition());
        assertNull(ap.getPositionCovariance());


        //test with bssid, frequency, transmitted power and position
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, position);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    TRANSMITTED_POWER, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid, transmitted power and position
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, position);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    SSID, TRANSMITTED_POWER, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    SSID, TRANSMITTED_POWER, position);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, transmitted power, transmitted power
        // standard deviation
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid, transmitted power and transmitted power
        //standard deviation
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, transmitted power, position and position
        // covariance
        Matrix cov = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, position, cov);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, position, null);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    TRANSMITTED_POWER, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid, transmitted power, position and
        //position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, position, cov);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, position, null);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(), 0.0, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    SSID, TRANSMITTED_POWER, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    SSID, TRANSMITTED_POWER, position, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, null, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, position, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, transmitted power, transmitted power standard
        //deviation, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                null);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertNull(ap.getSsid());
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position, cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null,
                    cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                    new Matrix(1, 1));
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);


        //test with bssid, frequency, ssid, transmitted power,
        // transmitted power standard deviation, position and position covariance
        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position, cov);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertSame(ap.getPositionCovariance(), cov);

        ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY, SSID,
                TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                null);

        //check default values
        assertEquals(ap.getBssid(), BSSID);
        assertEquals(ap.getFrequency(), FREQUENCY, 0.0);
        assertEquals(ap.getSsid(), SSID);
        assertEquals(ap.getTransmittedPower(), TRANSMITTED_POWER, 0.0);
        assertEquals(ap.getTransmittedPowerStandardDeviation(),
                TRANSMITTED_POWER_STD, 0.0);
        assertSame(ap.getPosition(), position);
        assertNull(ap.getPositionCovariance());

        //force IllegalArgumentException
        ap = null;
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(null, FREQUENCY,
                    SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                    cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, -FREQUENCY,
                    SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                    cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, -TRANSMITTED_POWER_STD, position,
                    cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, null,
                    cov);
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            ap = new WifiAccessPointWithPowerAndLocated3D(BSSID, FREQUENCY,
                    SSID, TRANSMITTED_POWER, TRANSMITTED_POWER_STD, position,
                    new Matrix(1, 1));
            fail("IllgalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ap);
    }

    @Test
    public void testEquals() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        WifiAccessPointWithPowerAndLocated3D ap1 =
                new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY,
                        TRANSMITTED_POWER, position);
        WifiAccessPointWithPowerAndLocated3D ap2 =
                new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY,
                        TRANSMITTED_POWER, position);
        WifiAccessPointWithPowerAndLocated3D ap3 =
                new WifiAccessPointWithPowerAndLocated3D("bssid2", FREQUENCY,
                        TRANSMITTED_POWER, position);

        //check
        //noinspection all
        assertTrue(ap1.equals(ap1));
        assertTrue(ap1.equals(ap2));
        assertFalse(ap1.equals(ap3));
    }

    @Test
    public void testHashCode() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        InhomogeneousPoint3D position = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS),
                randomizer.nextDouble(MIN_POS, MAX_POS));
        WifiAccessPointWithPowerAndLocated3D ap1 =
                new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY,
                        TRANSMITTED_POWER, position);
        WifiAccessPointWithPowerAndLocated3D ap2 =
                new WifiAccessPointWithPowerAndLocated3D("bssid1", FREQUENCY,
                        TRANSMITTED_POWER, position);
        WifiAccessPointWithPowerAndLocated3D ap3 =
                new WifiAccessPointWithPowerAndLocated3D("bssid2", FREQUENCY,
                        TRANSMITTED_POWER, position);

        //check
        assertEquals(ap1.hashCode(), ap2.hashCode());
        assertNotEquals(ap1.hashCode(), ap3.hashCode());
    }

}
