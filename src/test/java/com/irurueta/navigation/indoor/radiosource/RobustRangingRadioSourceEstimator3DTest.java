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
package com.irurueta.navigation.indoor.radiosource;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.RangingReadingLocated3D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustRangingRadioSourceEstimator3DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point3D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    @BeforeClass
    public static void setUpClass() {
    }

    @AfterClass
    public static void tearDownClass() {
    }

    @Before
    public void setUp() {
    }

    @After
    public void tearDown() {
    }

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustRangingRadioSourceEstimator3D<WifiAccessPoint> estimator =
                RobustRangingRadioSourceEstimator3D.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // crete with readings and method
        final List<RangingReadingLocated3D<WifiAccessPoint>> readings = new ArrayList<>();
        final WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D();
            readings.add(new RangingReadingLocated3D<>(accessPoint, 0.0, position));
        }

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with initial position and method
        final InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D();

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(initialPosition,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, this, RobustEstimatorMethod.LMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(readings,
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores and method
        final double[] qualityScores = new double[4];

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // crete with quality scores, readings and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores, readings,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, readings, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, readings, initial position and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);


        // create with quality scores, readings, initial position, listener and method

        // RANSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator3D);

        // LMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator3D);

        // MSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator3D);

        // PROSAC
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator3D);

        // PROMedS
        estimator = RobustRangingRadioSourceEstimator3D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator3D);
    }

    @Override
    public void onEstimateStart(
            final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateEnd(
            final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator) {
    }

    @Override
    public void onEstimateNextIteration(
            final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
            final int iteration) {
    }

    @Override
    public void onEstimateProgressChange(
            final RobustRangingRadioSourceEstimator<WifiAccessPoint, Point3D> estimator,
            final float progress) {
    }
}
