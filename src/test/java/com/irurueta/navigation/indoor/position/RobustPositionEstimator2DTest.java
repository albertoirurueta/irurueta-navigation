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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointLocated2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustPositionEstimator2DTest implements
        RobustPositionEstimatorListener<Point2D> {

    private static final double FREQUENCY = 2.4e9; //(Hz)

    @Test
    public void testCreate() {
        //create with method

        //RANSAC
        RobustPositionEstimator2D estimator = RobustPositionEstimator2D.create(
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);

        //LMedS
        estimator = RobustPositionEstimator2D.create(RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);

        //MSAC
        estimator = RobustPositionEstimator2D.create(RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);


        //create with sources and method
        List<WifiAccessPointLocated2D> sources = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            sources.add(new WifiAccessPointLocated2D("id1", FREQUENCY,
                    new InhomogeneousPoint2D()));
        }

        //RANSAC
        estimator = RobustPositionEstimator2D.create(sources,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);

        //LMedS
        estimator = RobustPositionEstimator2D.create(sources,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);

        //MSAC
        estimator = RobustPositionEstimator2D.create(sources,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(sources,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(sources,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);


        //create with fingerprint and method
        RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint =
                new RssiFingerprint<>();

        //RANSAC
        estimator = RobustPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);

        //LMedS
        estimator = RobustPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);

        //MSAC
        estimator = RobustPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(fingerprint,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);


        //create with sources, fingerprint and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        //LMedS
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        //MSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);


        //create with listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);

        //LMedS
        estimator = RobustPositionEstimator2D.create(this,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);

        //MSAC
        estimator = RobustPositionEstimator2D.create(this,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);


        //create with sources, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        //LMedS
        estimator = RobustPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        //MSAC
        estimator = RobustPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(sources, this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);


        //create with fingerprint, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //LMedS
        estimator = RobustPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //MSAC
        estimator = RobustPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(fingerprint, this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);


        //create with sources, fingerprint, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //LMedS
        estimator = RobustPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //MSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //PROSAC
        estimator = RobustPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(sources, fingerprint, this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);


        //create with quality scores and method
        double[] qualityScores = new double[3];

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, fingerprint and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, fingerprint and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores,this,
                RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores,this,
                RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores,this,
                RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores,this,
                RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, fingerprint, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this, RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this, RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this, RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this, RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this, RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, fingerprint, listener and method

        //RANSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this, RobustEstimatorMethod.RANSAC);

        //check
        assertTrue(estimator instanceof RANSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //LMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this, RobustEstimatorMethod.LMedS);

        //check
        assertTrue(estimator instanceof LMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //MSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this, RobustEstimatorMethod.MSAC);

        //check
        assertTrue(estimator instanceof MSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getQualityScores());

        //PROSAC
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this, RobustEstimatorMethod.PROSAC);

        //check
        assertTrue(estimator instanceof PROSACRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);

        //PROMedS
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this, RobustEstimatorMethod.PROMedS);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with default method

        estimator = RobustPositionEstimator2D.create();

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);


        //create with sources and default method
        estimator = RobustPositionEstimator2D.create(sources);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);


        //create with fingerprint and default method
        estimator = RobustPositionEstimator2D.create(fingerprint);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);


        //create with sources, fingerprint and default method
        estimator = RobustPositionEstimator2D.create(sources, fingerprint);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);


        //create with listener and default method
        estimator = RobustPositionEstimator2D.create(this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);


        //create with sources, listener and default method
        estimator = RobustPositionEstimator2D.create(sources, this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);


        //create with fingerprint, listener and default method
        estimator = RobustPositionEstimator2D.create(fingerprint, this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);


        //create with sources, fingerprint, listener and default method
        estimator = RobustPositionEstimator2D.create(sources, fingerprint,
                this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);


        //create with quality scores and default method
        estimator = RobustPositionEstimator2D.create(qualityScores);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, sources);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, fingerprint and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, fingerprint and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, listener and default method
        estimator = RobustPositionEstimator2D.create(qualityScores,this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, listener and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, fingerprint, listener and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, fingerprint,
                this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);


        //create with quality scores, sources, fingerprint, listener and default method
        estimator = RobustPositionEstimator2D.create(qualityScores, sources,
                fingerprint, this);

        //check
        assertTrue(estimator instanceof PROMedSRobustPositionEstimator2D);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getListener(), this);
        assertSame(estimator.getQualityScores(), qualityScores);
    }

    @Override
    public void onEstimateStart(RobustPositionEstimator<Point2D> estimator) { }

    @Override
    public void onEstimateEnd(RobustPositionEstimator<Point2D> estimator) { }

    @Override
    public void onEstimateNextIteration(RobustPositionEstimator<Point2D> estimator,
                                        int iteration) { }

    @Override
    public void onEstimateProgressChange(RobustPositionEstimator<Point2D> estimator,
                                         float progress) { }
}
