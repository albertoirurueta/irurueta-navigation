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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class RobustKnownFrameAccelerometerCalibratorTest implements
        RobustKnownFrameAccelerometerCalibratorListener {

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustKnownFrameAccelerometerCalibrator calibrator =
                RobustKnownFrameAccelerometerCalibrator.create(
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);


        // test create with listener and method
        calibrator =
                RobustKnownFrameAccelerometerCalibrator.create(this,
                        RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and method

        // RANSAC
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores and method
        final double[] qualityScores = new double[
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);


        // test create with quality scores, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof  MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measurements, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, common axis used, listener and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener and
        // method

        // RANSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(calibrator instanceof RANSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // LMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertTrue(calibrator instanceof LMedSRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // MSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertTrue(calibrator instanceof MSACRobustKnownFrameAccelerometerCalibrator);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROSAC
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(calibrator instanceof PROSACRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);

        // PROMedS
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(calibrator instanceof PROMedSRobustKnownFrameAccelerometerCalibrator);
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create();

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);


        // test create with listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getListener(), this);


        // test create with measurements and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(true,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with measurements, common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with measurements, common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(measurements,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());


        // test create with quality scores, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);


        // test create with quality scores, measurements, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, common axis used and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, common axis used, listener and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                true, this);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);


        // test create with quality scores, measurements, common axis used and default
        // method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
                measurements, true);

        // check
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertNull(calibrator.getQualityScores());
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());


        // test create with quality scores, measurements, common axis used, listener
        // and default method
        calibrator = RobustKnownFrameAccelerometerCalibrator.create(qualityScores,
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
            final RobustKnownFrameAccelerometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownFrameAccelerometerCalibrator calibrator) {
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownFrameAccelerometerCalibrator calibrator,
            final int iteration) {
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownFrameAccelerometerCalibrator calibrator,
            final float progress) {
    }
}
