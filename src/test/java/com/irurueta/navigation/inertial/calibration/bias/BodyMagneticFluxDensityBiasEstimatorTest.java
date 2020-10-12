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
package com.irurueta.navigation.inertial.calibration.bias;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.Distance;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Random;

import static org.junit.Assert.*;

public class BodyMagneticFluxDensityBiasEstimatorTest
        implements BodyMagneticFluxDensityBiasEstimatorListener {

    private static final int N_SAMPLES = 100000;

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-9;

    private int mStart;

    private int mBodyMagneticFluxDensityAdded;

    private int mReset;

    @Test
    public void testConstructor1() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor2() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor3() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor4() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor5() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance());

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor6() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor7() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor8() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor9() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor10() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor11() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor12() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor13() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor14() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        final double year = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor15() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor16() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor17() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor18() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor19() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor20() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor21() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, year);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor22() throws IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor23() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor24() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor25() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor26() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor27() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor28() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, year, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor29() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor30() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor31() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor32() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor33() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor34() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor35() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, date);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor36() throws IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor37() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor38() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor39() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor40() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor41() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor42() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, date, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertNull(estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor43() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor44() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor45() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor46() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor47() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor48() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year,
                        magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor49() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, year, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor50() throws IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(year, magneticModel,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor51() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, year,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor52() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), year, magneticModel,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor53() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), year, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor54() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), year,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor55() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, year,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor56() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                new Date(createTimestamp(randomizer)));
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, year,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor57() throws IOException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), 0.0);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor58() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor59() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor60() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor61() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor62() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date,
                        magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor63() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, date, magneticModel);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor64() throws IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(date, magneticModel,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor65() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(cbn, date,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertEquals(nedFrame, estimator.getNedFrame());
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertEquals(nedFrame, nedFrame1);
        assertEquals(nedFrame.getPosition(), estimator.getNedPosition());
        final NEDPosition nedPosition = new NEDPosition();
        estimator.getNedPosition(nedPosition);
        assertEquals(nedFrame.getPosition(), nedPosition);
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertEquals(nedFrame.getCoordinateTransformation(), nedC1);
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedFrame.getCoordinateTransformation(), nedC2);
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor66() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitude(), nedPosition.getLongitude(),
                        nedPosition.getHeight(), date, magneticModel,
                        this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor67() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeight(), date, magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor68() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition.getLatitudeAngle(),
                        nedPosition.getLongitudeAngle(),
                        nedPosition.getHeightDistance(), date,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(
                nedC2, LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor69() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn, date,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(ecefFrame.getECEFPosition(), estimator.getEcefPosition());
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition);
        assertEquals(ecefFrame, estimator.getEcefFrame());
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertEquals(ecefFrame, ecefFrame1);
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC2);
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testConstructor70() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final Date date = new Date(createTimestamp(randomizer));
        final double year = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        ecefFrame.getECEFPosition(), cbn, date,
                        magneticModel, this);

        // check default values
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(ecefFrame.getECEFPosition().equals(
                estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition);
        assertTrue(ecefFrame.getECEFPosition().equals(
                ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame1 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame1);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final NEDFrame nedFrame1 = new NEDFrame();
        estimator.getNedFrame(nedFrame1);
        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.getPosition().equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        estimator.getNedPosition(nedPosition1);
        assertTrue(nedFrame.getPosition().equals(nedPosition1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC1 = estimator.getEcefC();
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC1, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefFrame.getCoordinateTransformation().equals(
                ecefC2, LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC1 = estimator.getNedC();
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC1,
                LARGE_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedFrame.getCoordinateTransformation().equals(nedC2,
                LARGE_ABSOLUTE_ERROR));
        assertEquals(year, estimator.getYear(), LARGE_ABSOLUTE_ERROR);
        assertSame(magneticModel, estimator.getMagneticModel());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertEquals(0.0, estimator.getBiasX(), 0.0);
        final MagneticFluxDensity biasX1 = estimator.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, biasX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasX1.getUnit());
        final MagneticFluxDensity biasX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasX2);
        assertEquals(biasX1, biasX2);
        final MagneticFluxDensity biasY1 = estimator.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, biasY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasY1.getUnit());
        final MagneticFluxDensity biasY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasY2);
        assertEquals(biasY1, biasY2);
        final MagneticFluxDensity biasZ1 = estimator.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, biasZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasZ1.getUnit());
        final MagneticFluxDensity biasZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasZ2);
        assertEquals(biasZ1, biasZ2);
        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 = new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMagneticFluxDensity();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMagneticFluxDensity();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMagneticFluxDensity();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(),
                0.0);
        final MagneticFluxDensity avgStd1 = estimator
                .getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgPsd(), 0.0);
        assertEquals(0.0, estimator.getAvgRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        final BodyMagneticFluxDensity expectedB1 = estimator
                .getExpectedBodyMagneticFluxDensity();
        assertNotNull(expectedB1);
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertEquals(expectedB1, expectedB2);
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException, IOException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(estimator.getTimeInterval(),
                BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(estimator.getTimeInterval(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetTimeIntervalAsTime() throws LockedException, IOException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();

        assertEquals(time1.getValue().doubleValue(),
                BodyMagneticFluxDensityBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);

        // set new value
        final Time time2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setTimeInterval(time2);

        // check
        final Time time3 = estimator.getTimeIntervalAsTime();
        final Time time4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertEquals(time3, time4);
    }

    @Test
    public void testGetSetEcefPosition1() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final ECEFPosition ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final ECEFPosition ecefPosition3 = ecefFrame.getECEFPosition();
        estimator.setEcefPosition(ecefPosition3);

        // check
        final ECEFPosition ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final ECEFPosition ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    public void testGetSetEcefPosition2() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final ECEFPosition ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final ECEFPosition ecefPosition3 = ecefFrame.getECEFPosition();
        final double x = ecefPosition3.getX();
        final double y = ecefPosition3.getY();
        final double z = ecefPosition3.getZ();
        estimator.setEcefPosition(x, y, z);

        // check
        final ECEFPosition ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final ECEFPosition ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    public void testGetSetEcefPosition3() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final ECEFPosition ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final ECEFPosition ecefPosition3 = ecefFrame.getECEFPosition();
        final Distance x = ecefPosition3.getDistanceX();
        final Distance y = ecefPosition3.getDistanceY();
        final Distance z = ecefPosition3.getDistanceZ();
        estimator.setEcefPosition(x, y, z);

        // check
        final ECEFPosition ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final ECEFPosition ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    public void testGetSetEcefPosition4() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final ECEFPosition ecefPosition1 = estimator.getEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition, ecefPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition = createPosition(randomizer);

        nedFrame.setPosition(nedPosition);

        NEDtoECEFFrameConverter.convertNEDtoECEF(nedFrame, ecefFrame);

        final ECEFPosition ecefPosition3 = ecefFrame.getECEFPosition();
        final Point3D point = ecefPosition3.getPosition();
        estimator.setEcefPosition(point);

        // check
        final ECEFPosition ecefPosition4 = estimator.getEcefPosition();
        assertEquals(ecefPosition3, ecefPosition4);
        final ECEFPosition ecefPosition5 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition5);
        assertEquals(ecefPosition3, ecefPosition5);
    }

    @Test
    public void testGetEcefFrame() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(ecefPosition, cbn);

        // check
        final ECEFFrame ecefFrame1 = estimator.getEcefFrame();
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame.equals(ecefFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefFrame.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetNedFrame() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, cbn);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(nedPosition, cbn);

        // check
        final NEDFrame nedFrame1 = estimator.getNedFrame();
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);

        assertTrue(nedFrame.equals(nedFrame1, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedFrame.equals(nedFrame2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetNedPosition1() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final NEDPosition nedPosition = nedFrame.getPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final NEDPosition nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition3 = createPosition(randomizer);

        estimator.setNedPosition(nedPosition3);

        // check
        final NEDPosition nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetNedPosition2() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final NEDPosition nedPosition = nedFrame.getPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final NEDPosition nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition3 = createPosition(randomizer);
        final double latitude = nedPosition3.getLatitude();
        final double longitude = nedPosition3.getLongitude();
        final double height = nedPosition3.getHeight();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetNedPosition3() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final NEDPosition nedPosition = nedFrame.getPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final NEDPosition nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition3 = createPosition(randomizer);
        final Angle latitude = nedPosition3.getLatitudeAngle();
        final Angle longitude = nedPosition3.getLongitudeAngle();
        final double height = nedPosition3.getHeight();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetNedPosition4() throws IOException, LockedException {
        final NEDFrame nedFrame = new NEDFrame();
        final NEDPosition nedPosition = nedFrame.getPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default position
        final NEDPosition nedPosition1 = estimator.getNedPosition();
        assertEquals(nedPosition, nedPosition1);
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertEquals(nedPosition, nedPosition2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition nedPosition3 = createPosition(randomizer);
        final Angle latitude = nedPosition3.getLatitudeAngle();
        final Angle longitude = nedPosition3.getLongitudeAngle();
        final Distance height = nedPosition3.getHeightDistance();

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition4 = estimator.getNedPosition();
        assertTrue(nedPosition3.equals(nedPosition4, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition5 = new NEDPosition();
        estimator.getNedPosition(nedPosition5);
        assertTrue(nedPosition3.equals(nedPosition5, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetEcefC() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final CoordinateTransformation ecefC2 = estimator.getEcefC();
        assertEquals(ecefC1, ecefC2);
        final CoordinateTransformation ecefC3 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC3);
        assertEquals(ecefC1, ecefC3);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame2 = new NEDFrame(cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final CoordinateTransformation ecefC4 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setEcefC(ecefC4);

        // check
        final CoordinateTransformation ecefC5 = estimator.getEcefC();
        assertEquals(ecefC4, ecefC5);
        final CoordinateTransformation ecefC6 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC6);
        assertEquals(ecefC4, ecefC5);
    }

    @Test
    public void testGetSetNedC() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final CoordinateTransformation nedC2 = estimator.getNedC();
        assertEquals(nedC1, nedC2);
        final CoordinateTransformation nedC3 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC3);
        assertEquals(nedC1, nedC3);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        estimator.setNedC(cbn);

        // check
        final CoordinateTransformation nedC4 = estimator.getNedC();
        assertEquals(cbn, nedC4);
        final CoordinateTransformation nedC5 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC5);
        assertEquals(cbn, nedC5);
    }

    @Test
    public void testSetNedPositionAndNedOrientation1() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);

        estimator.setNedPositionAndNedOrientation(nedPosition2, cbn);

        // check
        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final CoordinateTransformation nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetNedPositionAndNedOrientation2() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final double latitude = nedPosition2.getLatitude();
        final double longitude = nedPosition2.getLongitude();
        final double height = nedPosition2.getHeight();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                cbn);

        // check
        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final CoordinateTransformation nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetNedPositionAndNedOrientation3() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final Angle latitude = nedPosition2.getLatitudeAngle();
        final Angle longitude = nedPosition2.getLongitudeAngle();
        final double height = nedPosition2.getHeight();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                cbn);

        // check
        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final CoordinateTransformation nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetNedPositionAndNedOrientation4() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(nedPosition1, estimator.getNedPosition());
        assertEquals(nedC1, estimator.getNedC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final Angle latitude = nedPosition2.getLatitudeAngle();
        final Angle longitude = nedPosition2.getLongitudeAngle();
        final Distance height = nedPosition2.getHeightDistance();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                cbn);

        // check
        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final CoordinateTransformation nedC3 = estimator.getNedC();
        assertTrue(nedPosition3.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(nedC3.equals(cbn, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation1() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setEcefPositionAndEcefOrientation(ecefPosition2, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation2() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final double x = ecefPosition2.getX();
        final double y = ecefPosition2.getY();
        final double z = ecefPosition2.getZ();

        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation3() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final Distance x = ecefPosition2.getDistanceX();
        final Distance y = ecefPosition2.getDistanceY();
        final Distance z = ecefPosition2.getDistanceZ();

        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation4() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final Point3D point = ecefPosition2.getPosition();

        estimator.setEcefPositionAndEcefOrientation(point, ecefC2);

        // check
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetNedPositionAndEcefOrientation1() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(nedPosition2, ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetNedPositionAndEcefOrientation2() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final double latitude = nedPosition2.getLatitude();
        final double longitude = nedPosition2.getLongitude();
        final double height = nedPosition2.getHeight();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetNedPositionAndEcefOrientation3() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final Angle latitude = nedPosition2.getLatitudeAngle();
        final Angle longitude = nedPosition2.getLongitudeAngle();
        final double height = nedPosition2.getHeight();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetNedPositionAndEcefOrientation4() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertTrue(nedPosition1.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final CoordinateTransformation ecefC2 = ecefFrame2.getCoordinateTransformation();
        final Angle latitude = nedPosition2.getLatitudeAngle();
        final Angle longitude = nedPosition2.getLongitudeAngle();
        final Distance height = nedPosition2.getHeightDistance();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(nedPosition2.equals(estimator.getNedPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertEquals(ecefC2, estimator.getEcefC());
    }

    @Test
    public void testSetEcefPositionAndNedOrientation1() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();

        estimator.setEcefPositionAndNedOrientation(ecefPosition2, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetEcefPositionAndNedOrientation2() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final double x = ecefPosition2.getX();
        final double y = ecefPosition2.getY();
        final double z = ecefPosition2.getZ();

        estimator.setEcefPositionAndNedOrientation(x, y, z, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetEcefPositionAndNedOrientation3() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final Distance x = ecefPosition2.getDistanceX();
        final Distance y = ecefPosition2.getDistanceY();
        final Distance z = ecefPosition2.getDistanceZ();

        estimator.setEcefPositionAndNedOrientation(x, y, z, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testSetEcefPositionAndNedOrientation4() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final NEDFrame nedFrame1 = new NEDFrame();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertTrue(nedC1.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition2 = createPosition(randomizer);
        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, cbn);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);
        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final Point3D point = ecefPosition2.getPosition();

        estimator.setEcefPositionAndNedOrientation(point, cbn);

        // check
        assertTrue(ecefPosition2.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(cbn.equals(estimator.getNedC(), LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetYear() throws IOException, LockedException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final double year1 = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Date date = new Date(createTimestamp(randomizer));
        final double year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        estimator.setYear(year2);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    public void testSetTime1() throws IOException, LockedException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final double year1 = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Date date = new Date(createTimestamp(randomizer));
        final double year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);

        estimator.setTime(date);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    public void testSetTime2() throws IOException, LockedException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        final double year1 = BodyMagneticFluxDensityBiasEstimator
                .convertTime(new Date());
        assertEquals(year1, estimator.getYear(), LARGE_ABSOLUTE_ERROR);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Date date = new Date(createTimestamp(randomizer));
        final double year2 = BodyMagneticFluxDensityBiasEstimator.convertTime(
                date);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(date);

        estimator.setTime(calendar);

        // check
        assertEquals(year2, estimator.getYear(), 0.0);
    }

    @Test
    public void testGetSetMagneticModel() throws IOException, LockedException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertNull(estimator.getMagneticModel());

        // set new value
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        estimator.setMagneticModel(magneticModel);

        // check
        assertSame(magneticModel, estimator.getMagneticModel());
    }

    @Test
    public void testGetSetListener() throws IOException, LockedException {
        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddBodyMagneticFluxDensityAndReset()
            throws InvalidSourceAndDestinationFrameTypeException, IOException,
            LockedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDPosition nedPosition = createPosition(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        // Expected body magnetic flux density for a static body at provided
        // location, orientation and timestamp
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                nedPosition, timestamp);
        final BodyMagneticFluxDensity truthMagnetic =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final BodyMagneticFluxDensityBiasEstimator estimator =
                new BodyMagneticFluxDensityBiasEstimator(
                        nedPosition, cbn, timestamp, this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyMagneticFluxDensityAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.isRunning());

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        BodyMagneticFluxDensity bodyMagneticFluxDensity = null;
        final BodyMagneticFluxDensity lastBodyMagneticFluxDensity =
                new BodyMagneticFluxDensity();
        for (int i = 0; i < N_SAMPLES; i++) {
            if (estimator.getLastBodyMagneticFluxDensity(
                    lastBodyMagneticFluxDensity)) {
                assertEquals(estimator.getLastBodyMagneticFluxDensity(),
                        lastBodyMagneticFluxDensity);
                assertEquals(bodyMagneticFluxDensity, lastBodyMagneticFluxDensity);
            }

            bodyMagneticFluxDensity = generateMeasure(truthMagnetic, hardIron,
                    mm, noiseRandomizer);

            estimator.addBodyMagneticFluxDensity(bodyMagneticFluxDensity);

            assertTrue(estimator.getLastBodyMagneticFluxDensity(
                    lastBodyMagneticFluxDensity));
            assertEquals(lastBodyMagneticFluxDensity, bodyMagneticFluxDensity);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyMagneticFluxDensityAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double biasBx = estimator.getBiasX();
        final double biasBy = estimator.getBiasY();
        final double biasBz = estimator.getBiasZ();

        assertEquals(hardIron[0], biasBx, ABSOLUTE_ERROR);
        assertEquals(hardIron[1], biasBy, ABSOLUTE_ERROR);
        assertEquals(hardIron[2], biasBz, ABSOLUTE_ERROR);

        final MagneticFluxDensity biasBx1 = new MagneticFluxDensity(biasBx,
                MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity biasBx2 = estimator.getBiasXAsMagneticFluxDensity();
        final MagneticFluxDensity biasBx3 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasXAsMagneticFluxDensity(biasBx3);

        assertEquals(biasBx1, biasBx2);
        assertEquals(biasBx1, biasBx3);

        final MagneticFluxDensity biasBy1 = new MagneticFluxDensity(biasBy,
                MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity biasBy2 = estimator.getBiasYAsMagneticFluxDensity();
        final MagneticFluxDensity biasBy3 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasYAsMagneticFluxDensity(biasBy3);

        assertEquals(biasBy1, biasBy2);
        assertEquals(biasBy1, biasBy3);

        final MagneticFluxDensity biasBz1 = new MagneticFluxDensity(biasBz,
                MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity biasBz2 = estimator.getBiasZAsMagneticFluxDensity();
        final MagneticFluxDensity biasBz3 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        estimator.getBiasZAsMagneticFluxDensity(biasBz3);

        assertEquals(biasBz1, biasBz2);
        assertEquals(biasBz1, biasBz3);

        final MagneticFluxDensityTriad biasTriad1 = estimator.getBiasTriad();
        assertEquals(biasBx, biasTriad1.getValueX(), 0.0);
        assertEquals(biasBy, biasTriad1.getValueY(), 0.0);
        assertEquals(biasBz, biasTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, biasTriad1.getUnit());
        final MagneticFluxDensityTriad biasTriad2 =
                new MagneticFluxDensityTriad();
        estimator.getBiasTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);

        final double varianceBx = estimator.getVarianceX();
        final double varianceBy = estimator.getVarianceY();
        final double varianceBz = estimator.getVarianceZ();

        final double standardDeviationBx = estimator.getStandardDeviationX();
        final double standardDeviationBy = estimator.getStandardDeviationY();
        final double standardDeviationBz = estimator.getStandardDeviationZ();

        final double avgStdB = (standardDeviationBx + standardDeviationBy
                + standardDeviationBz) / 3.0;

        assertEquals(avgStdB, estimator.getAverageStandardDeviation(), 0.0);
        assertEquals(Math.sqrt(varianceBx), standardDeviationBx, 0.0);
        assertEquals(Math.sqrt(varianceBy), standardDeviationBy, 0.0);
        assertEquals(Math.sqrt(varianceBz), standardDeviationBz, 0.0);

        final MagneticFluxDensity standardDeviationBx1 =
                new MagneticFluxDensity(standardDeviationBx,
                        MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity standardDeviationBx2 =
                estimator.getStandardDeviationXAsMagneticFluxDensity();
        final MagneticFluxDensity standardDeviationBx3 =
                new MagneticFluxDensity(1.0,
                        MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMagneticFluxDensity(
                standardDeviationBx3);

        assertEquals(standardDeviationBx1, standardDeviationBx2);
        assertEquals(standardDeviationBx1, standardDeviationBx3);

        final MagneticFluxDensity standardDeviationBy1 =
                new MagneticFluxDensity(standardDeviationBy,
                        MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity standardDeviationBy2 =
                estimator.getStandardDeviationYAsMagneticFluxDensity();
        final MagneticFluxDensity standardDeviationBy3 =
                new MagneticFluxDensity(1.0,
                        MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMagneticFluxDensity(
                standardDeviationBy3);

        assertEquals(standardDeviationBy1, standardDeviationBy2);
        assertEquals(standardDeviationBy1, standardDeviationBy3);

        final MagneticFluxDensity standardDeviationBz1 =
                new MagneticFluxDensity(standardDeviationBz,
                        MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity standardDeviationBz2 =
                estimator.getStandardDeviationZAsMagneticFluxDensity();
        final MagneticFluxDensity standardDeviationBz3 =
                new MagneticFluxDensity(1.0,
                        MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMagneticFluxDensity(
                standardDeviationBz3);

        assertEquals(standardDeviationBz1, standardDeviationBz2);
        assertEquals(standardDeviationBz1, standardDeviationBz3);

        final MagneticFluxDensityTriad stdTriad1 =
                estimator.getStandardDeviationTriad();
        assertEquals(standardDeviationBx, stdTriad1.getValueX(), 0.0);
        assertEquals(standardDeviationBy, stdTriad1.getValueY(), 0.0);
        assertEquals(standardDeviationBz, stdTriad1.getValueZ(), 0.0);
        final MagneticFluxDensityTriad stdTriad2 =
                new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);

        assertEquals(avgStdB, estimator.getAverageStandardDeviation(), 0.0);
        final MagneticFluxDensity avgStdB1 =
                estimator.getAverageStandardDeviationAsMagneticFluxDensity();
        assertEquals(avgStdB1.getValue().doubleValue(), avgStdB, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStdB1.getUnit());
        final MagneticFluxDensity avgStdB2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMagneticFluxDensity(avgStdB2);
        assertEquals(avgStdB1, avgStdB2);

        final double psdBx = estimator.getPsdX();
        final double psdBy = estimator.getPsdY();
        final double psdBz = estimator.getPsdZ();

        final double timeInterval = estimator.getTimeInterval();
        assertEquals(varianceBx * timeInterval, psdBx, 0.0);
        assertEquals(varianceBy * timeInterval, psdBy, 0.0);
        assertEquals(varianceBz * timeInterval, psdBz, 0.0);

        final double rootPsdBx = estimator.getRootPsdX();
        final double rootPsdBy = estimator.getRootPsdY();
        final double rootPsdBz = estimator.getRootPsdZ();

        assertEquals(Math.sqrt(psdBx), rootPsdBx, 0.0);
        assertEquals(Math.sqrt(psdBy), rootPsdBy, 0.0);
        assertEquals(Math.sqrt(psdBz), rootPsdBz, 0.0);

        final double expectedRootPsd = Math.sqrt(
                MAGNETOMETER_NOISE_STD * MAGNETOMETER_NOISE_STD * timeInterval);
        assertEquals(expectedRootPsd, rootPsdBx, SMALL_ABSOLUTE_ERROR);
        assertEquals(expectedRootPsd, rootPsdBy, SMALL_ABSOLUTE_ERROR);
        assertEquals(expectedRootPsd, rootPsdBz, SMALL_ABSOLUTE_ERROR);

        final double avgPsdB = estimator.getAvgPsd();
        final double expectedPsdB = (psdBx + psdBy + psdBz) / 3.0;
        assertEquals(expectedPsdB, avgPsdB, 0.0);

        final double avgRootPsdB = estimator.getAvgRootPsd();
        assertEquals(Math.sqrt(avgPsdB), avgRootPsdB, 0.0);

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());

        final BodyMagneticFluxDensity expectedB1 =
                estimator.getExpectedBodyMagneticFluxDensity();
        final BodyMagneticFluxDensity expectedB2 = new BodyMagneticFluxDensity();
        estimator.getExpectedBodyMagneticFluxDensity(expectedB2);
        assertTrue(truthMagnetic.equals(expectedB1, SMALL_ABSOLUTE_ERROR));
        assertTrue(truthMagnetic.equals(expectedB2, SMALL_ABSOLUTE_ERROR));

        // reset
        assertTrue(estimator.reset());

        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyMagneticFluxDensity());
        assertFalse(estimator.getLastBodyMagneticFluxDensity(null));
        assertFalse(estimator.isRunning());
        assertEquals(estimator.getBiasX(), 0.0, 0.0);
        assertEquals(estimator.getBiasY(), 0.0, 0.0);
        assertEquals(estimator.getBiasZ(), 0.0, 0.0);
        assertEquals(estimator.getVarianceX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceZ(), 0.0, 0.0);
        assertEquals(mReset, 1);

        assertFalse(estimator.reset());
        assertEquals(mReset, 1);
    }

    @Override
    public void onStart(final BodyMagneticFluxDensityBiasEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onBodyMagneticFluxDensityAdded(
            final BodyMagneticFluxDensityBiasEstimator estimator) {
        if (mBodyMagneticFluxDensityAdded == 0) {
            checkLocked(estimator);
        }
        mBodyMagneticFluxDensityAdded++;
    }

    @Override
    public void onReset(final BodyMagneticFluxDensityBiasEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mBodyMagneticFluxDensityAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final BodyMagneticFluxDensityBiasEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Time time = new Time(0.0, TimeUnit.SECOND);
        try {
            estimator.setTimeInterval(time);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEcefPosition((ECEFPosition) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEcefPosition(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEcefPosition(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEcefPosition((Point3D) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNedPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNedPosition(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNedPosition(null, null, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setNedPosition(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEcefC(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedC(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndNedOrientation(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndNedOrientation(
                    0.0, 0.0, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndNedOrientation(
                    null, null, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndNedOrientation(
                    null, null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndEcefOrientation(
                    (ECEFPosition) null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndEcefOrientation(
                    0.0, 0.0, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndEcefOrientation(
                    null, null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndEcefOrientation(
                    (Point3D) null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndEcefOrientation(null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndEcefOrientation(
                    0.0, 0.0, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndEcefOrientation(
                    null, null, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setNedPositionAndEcefOrientation(
                    null, null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndNedOrientation(
                    (ECEFPosition) null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndNedOrientation(
                    0.0, 0.0, 0.0, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndNedOrientation(
                    null, null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setEcefPositionAndNedOrientation(
                    (Point3D) null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setYear(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTime((Date) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTime((GregorianCalendar) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setMagneticModel(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final IOException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private static BodyMagneticFluxDensity generateMeasure(
            final BodyMagneticFluxDensity truthMagnetic,
            final double[] hardIron, final Matrix softIron,
            final GaussianRandomizer noiseRandomizer) {

        final BodyMagneticFluxDensity measuredMagnetic =
                generateMeasuredMagneticFluxDensity(truthMagnetic,
                        hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz()
                    + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic;
    }

    private static CoordinateTransformation generateBodyC(
            final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron,
            final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron,
                softIron);
    }

    private static double[] generateHardIron(
            final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(
            final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
