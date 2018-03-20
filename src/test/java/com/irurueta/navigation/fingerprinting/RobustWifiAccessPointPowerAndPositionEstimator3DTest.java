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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustWifiAccessPointPowerAndPositionEstimator3DTest implements
        RobustWifiAccessPointPowerAndPositionEstimatorListener<Point3D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final double MAX_RSSI = -50.0;

    private static final double MIN_PATH_LOSS_EXPONENT = 1.6;

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testCreate() {
        //create with method

        //RANSAC
        RobustWifiAccessPointPowerAndPositionEstimator3D estimator =
                RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                        RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings and method
        List<RssiReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            InhomogeneousPoint3D position = new InhomogeneousPoint3D();
            readings.add(new RssiReadingLocated3D<>(accessPoint, 0.0, position));
        }

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);


        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial position and method

        //RANSAC
        InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //check with readings, initial position, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);


        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //crate with readings, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with readings, initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores and method
        double[] qualityScores = new double[4];

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //create with quality scores, initial position and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial position, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof  RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMeds
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial position, initial transmitted power and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial position, initial transmitted power,
        //initial path loss exponent and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, initial position, initial transmitted power,
        // initial path loss exponent, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with quality scores, readings, initial position, initial transmitted power, listener and method

        //RANSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //LMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustWifiAccessPointPowerAndPositionEstimator3D);

        //MSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROSAC
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustWifiAccessPointPowerAndPositionEstimator3D);

        //PROMedS
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(qualityScores,
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustWifiAccessPointPowerAndPositionEstimator3D);


        //create with default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create();

        //check
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(this);

        //check
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, this);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, this);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, this);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                MAX_RSSI);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                MAX_RSSI, this);

        //check
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, MAX_RSSI, this);

        //check
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, this);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, MAX_RSSI, this);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position, initial transmitted power,
        // initial path loss exponent and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(readings,
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT, this);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with readings, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                readings, initialPosition, MAX_RSSI, MIN_PATH_LOSS_EXPONENT,
                this);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, MAX_RSSI);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, MAX_RSSI);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, MAX_RSSI, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, MAX_RSSI, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition, MAX_RSSI);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition, MAX_RSSI);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition, MAX_RSSI, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition, MAX_RSSI, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position,
        // initial transmitted power, initial path loss exponent and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position, initial transmitted power and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);


        //create with quality scores, readings, initial position, initial transmitted power, listener and default method
        estimator = RobustWifiAccessPointPowerAndPositionEstimator3D.create(
                qualityScores, readings, initialPosition, MAX_RSSI,
                MIN_PATH_LOSS_EXPONENT, this);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertEquals(estimator.getInitialTransmittedPowerdBm(), MAX_RSSI, 0.0);
        assertEquals(estimator.getInitialTransmittedPower(),
                Utils.dBmToPower(MAX_RSSI), 0.0);
        assertEquals(estimator.getInitialPathLossExponent(),
                MIN_PATH_LOSS_EXPONENT, 0.0);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getMethod(),
                RobustWifiAccessPointPowerAndPositionEstimator.DEFAULT_ROBUST_METHOD);
    }

    @Override
    public void onEstimateStart(RobustWifiAccessPointPowerAndPositionEstimator<Point3D> estimator) { }

    @Override
    public void onEstimateEnd(RobustWifiAccessPointPowerAndPositionEstimator<Point3D> estimator) { }

    @Override
    public void onEstimateNextIteration(RobustWifiAccessPointPowerAndPositionEstimator<Point3D> estimator,
                                        int iteration) { }

    @Override
    public void onEstimateProgressChange(RobustWifiAccessPointPowerAndPositionEstimator<Point3D> estimator,
                                         float progress) { }
}

