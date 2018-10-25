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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.RangingReadingLocated2D;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustRangingRadioSourceEstimator2DTest implements
        RobustRangingRadioSourceEstimatorListener<WifiAccessPoint, Point2D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

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
        RobustRangingRadioSourceEstimator2D<WifiAccessPoint> estimator =
                RobustRangingRadioSourceEstimator2D.create(
                        RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //crete with readings and method
        List<RangingReadingLocated2D<WifiAccessPoint>> readings = new ArrayList<>();
        WifiAccessPoint accessPoint = new WifiAccessPoint("bssid", FREQUENCY);
        for (int i = 0; i < 4; i++) {
            InhomogeneousPoint2D position = new InhomogeneousPoint2D();
            readings.add(new RangingReadingLocated2D<>(accessPoint, 0.0, position));
        }

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(this,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with readings, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings, this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with initial position and method
        InhomogeneousPoint2D initialPosition = new InhomogeneousPoint2D();

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with readings, initial position and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with initial position, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(initialPosition,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with readings, initial position, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(readings,
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores and method
        double[] qualityScores = new double[4];

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                        RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //crete with quality scores, readings and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores, readings,
                RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, readings, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, initial position and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, readings, initial position and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, initial position, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);


        //create with quality scores, readings, initial position, listener and method

        //RANSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.RANSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof RANSACRobustRangingRadioSourceEstimator2D);

        //LMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.LMedS);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof LMedSRobustRangingRadioSourceEstimator2D);

        //MSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.MSAC);

        //check
        assertNull(estimator.getQualityScores());
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof MSACRobustRangingRadioSourceEstimator2D);

        //PROSAC
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROSAC);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROSACRobustRangingRadioSourceEstimator2D);

        //PROMedS
        estimator = RobustRangingRadioSourceEstimator2D.create(qualityScores,
                readings, initialPosition, this, RobustEstimatorMethod.PROMedS);

        //check
        assertSame(estimator.getQualityScores(), qualityScores);
        assertSame(estimator.getReadings(), readings);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator instanceof PROMedSRobustRangingRadioSourceEstimator2D);
    }

    @Override
    public void onEstimateStart(
            RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) { }

    @Override
    public void onEstimateEnd(
            RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator) { }

    @Override
    public void onEstimateNextIteration(
            RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            int iteration) { }

    @Override
    public void onEstimateProgressChange(
            RobustRangingRadioSourceEstimator<WifiAccessPoint, Point2D> estimator,
            float progress) { }
}
