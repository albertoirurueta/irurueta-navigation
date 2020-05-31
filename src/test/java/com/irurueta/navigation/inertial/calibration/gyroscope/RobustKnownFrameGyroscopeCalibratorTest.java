/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownFrameGyroscopeCalibratorTest
        implements RobustKnownFrameGyroscopeCalibratorListener {

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustKnownFrameGyroscopeCalibrator calibrator =
                RobustKnownFrameGyroscopeCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);


        // test create with listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and method

        // RANSAC
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores and method
        final double[] qualityScores = new double[
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);


        // test create with quality scores, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameGyroscopeCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameGyroscopeCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);


        // test create with listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(true,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(measurements,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());


        // test create with quality scores, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measurements, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, common axis used and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, common axis used, listener and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and default
        // method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener
        // and default method
        calibrator = RobustKnownFrameGyroscopeCalibrator.create(qualityScores,
                measurements, true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownFrameGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownFrameGyroscopeCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownFrameGyroscopeCalibrator calibrator, final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownFrameGyroscopeCalibrator calibrator, final float progress) {
    }
}
