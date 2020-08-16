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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Random;

import static org.junit.Assert.*;

public class StandardDeviationFrameBodyMagneticFluxDensityTest {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAX_MAGNETIC_FLUX_DENSITY = 70e-6;

    private static final double THRESHOLD = 1e-6;

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

    @Test
    public void testConstructor() throws IOException {
        // test empty constructor
        StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density
        final BodyMagneticFluxDensity magneticFluxDensity =
                createMagneticFluxDensity();
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with ECEF frame
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with NED frame
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density and ECEF frame
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density and NED frame
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), 0.0,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        final Date timestamp = createTimestamp();
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final double year = FrameBodyMagneticFluxDensity
                .convertTime(timestamp);

        // test constructor with year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density and year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, year);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with ECEF frame and year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with NED frame and year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density, ECEF frame and year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, year);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density,  NED frame and year
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, year);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density and date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, timestamp);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with ECEF frame and date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with NED frame and date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density, ECEF frame and date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, timestamp);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density, NED frame and date
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, timestamp);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density and calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, calendar);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with ECEF frame and calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with NED frame and calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density, ECEF frame and
        // calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, calendar);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        // test constructor with magnetic flux density, NED frame and
        // calendar
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, calendar);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);


        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        // test constructor with year and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(year,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, year and standard
        // deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, year,
                        magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with ECEF frame, year and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, year, magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            ecefFrame, year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with NED frame, year and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, year, magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            nedFrame, year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, ECEF frame, year
        // and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, year,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, ecefFrame, year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, NED frame, year
        // and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, year,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, nedFrame, year,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with date and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(timestamp,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            timestamp, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, date and standard
        // deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, timestamp,
                        magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, timestamp,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with ECEF frame, date and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, timestamp,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            ecefFrame, timestamp,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with NED frame, date and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, timestamp, magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            nedFrame, timestamp,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, ECEF frame, date
        // and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, timestamp,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, ecefFrame, timestamp,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, NED frame, date
        // and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, timestamp,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, nedFrame, timestamp,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with calendar and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(calendar,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            calendar, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, calendar and
        // standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, calendar,
                        magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, calendar,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with ECEF frame, calendar and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        ecefFrame, calendar,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            ecefFrame, calendar,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with NED frame, calendar and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        nedFrame, calendar,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            nedFrame, calendar,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, ECEF frame,
        // calendar and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, ecefFrame, calendar,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertSame(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, ecefFrame, calendar,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test constructor with magnetic flux density, NED frame,
        // calendar and standard deviation
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, calendar,
                        magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        frameBodyMagneticFluxDensity = null;
        try {
            frameBodyMagneticFluxDensity =
                    new StandardDeviationFrameBodyMagneticFluxDensity(
                            magneticFluxDensity, nedFrame, calendar,
                            -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(frameBodyMagneticFluxDensity);


        // test copy constructor
        frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        magneticFluxDensity, nedFrame, calendar,
                        magneticFluxDensityStandardDeviation);

        final StandardDeviationFrameBodyMagneticFluxDensity
                frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        frameBodyMagneticFluxDensity);

        // check default values
        assertEquals(frameBodyMagneticFluxDensity2.getMagneticFluxDensity(),
                magneticFluxDensity);
        assertEquals(frameBodyMagneticFluxDensity2.getFrame(), ecefFrame);
        assertEquals(frameBodyMagneticFluxDensity2.getNedFrame(), nedFrame);
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity2.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(frameBodyMagneticFluxDensity2.getYear(), year,
                0.0);
        assertEquals(frameBodyMagneticFluxDensity2
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);
    }

    @Test
    public void testGetSetMagneticFluxDensityStandardDeviation() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        frameBodyMagneticFluxDensity
                .setMagneticFluxDensityStandardDeviation(
                        magneticFluxDensityStandardDeviation);

        // check
        assertEquals(frameBodyMagneticFluxDensity
                        .getMagneticFluxDensityStandardDeviation(),
                magneticFluxDensityStandardDeviation, 0.0);
    }

    @Test
    public void testGetSetMagneticFluxDensity() throws IOException {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());

        // set new value
        final BodyMagneticFluxDensity magneticFluxDensity =
                createMagneticFluxDensity();
        frameBodyMagneticFluxDensity.setMagneticFluxDensity(
                magneticFluxDensity);

        // check
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(),
                magneticFluxDensity);
    }

    @Test
    public void testGetSetFrame() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getFrame());

        // set new value
        ECEFFrame frame = new ECEFFrame();
        frameBodyMagneticFluxDensity.setFrame(frame);

        // check
        assertSame(frameBodyMagneticFluxDensity.getFrame(), frame);
    }

    @Test
    public void testGetSetNedFrame() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyMagneticFluxDensity.setNedFrame(nedFrame);

        // check
        assertEquals(frameBodyMagneticFluxDensity.getNedFrame(), nedFrame);
        assertEquals(frameBodyMagneticFluxDensity.getFrame(), ecefFrame);
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    public void testGetSetYear() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(frameBodyMagneticFluxDensity.getYear(),
                0.0, 0.0);

        // set new value
        final Date timestamp = createTimestamp();
        final double year = FrameBodyMagneticFluxDensity
                .convertTime(timestamp);

        frameBodyMagneticFluxDensity.setYear(year);

        // check
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
    }

    @Test
    public void testSetTime1() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(frameBodyMagneticFluxDensity.getYear(),
                0.0, 0.0);

        // set new value
        final Date timestamp = createTimestamp();
        final double year = FrameBodyMagneticFluxDensity
                .convertTime(timestamp);

        frameBodyMagneticFluxDensity.setTime(timestamp);

        // check
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
    }

    @Test
    public void testSetTime2() {
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(frameBodyMagneticFluxDensity.getYear(),
                0.0, 0.0);

        // set new value
        final Date timestamp = createTimestamp();
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final double year = FrameBodyMagneticFluxDensity
                .convertTime(timestamp);

        frameBodyMagneticFluxDensity.setTime(calendar);

        // check
        assertEquals(frameBodyMagneticFluxDensity.getYear(), year,
                0.0);
    }

    @Test
    public void testCopyFromWhenBodyMagneticFluxAndFrameAreAvailableAtSourceAndDestinationIsEmpty()
            throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity, ecefFrame, timestamp,
                        magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertEquals(bodyMagneticFluxDensity, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyFromWhenOnlyBodyMagneticFluxAreavailableAtSourceAndDestinationIsEmpty()
            throws IOException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity, timestamp,
                        magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertEquals(bodyMagneticFluxDensity, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyFromWhenOnlyFrameIsAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {

        final NEDPosition position = createPosition();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertNull(frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity2.getFrame());
    }

    @Test
    public void testCopyFromWhenEmptySourceAndDestinationHasData()
            throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity();
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity, ecefFrame, timestamp,
                        magneticFluxDensityStandardDeviation);

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertNull(frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(0.0,
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyFromWhenBothSourceAndDestinationHaveData()
            throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final double magneticFluxDensityStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position1 = createPosition();
        final NEDPosition position2 = createPosition();
        final Date timestamp1 = createTimestamp();
        final Date timestamp2 = createTimestamp();
        final CoordinateTransformation cnb1 = createAttitude();
        final CoordinateTransformation cnb2 = createAttitude();
        final CoordinateTransformation cbn1 = cnb1.inverseAndReturnNew();
        final CoordinateTransformation cbn2 = cnb1.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity1 =
                createMagneticFluxDensity(position1, timestamp1, cnb1);
        final BodyMagneticFluxDensity bodyMagneticFluxDensity2 =
                createMagneticFluxDensity(position2, timestamp2, cnb2);

        final NEDFrame nedFrame1 = new NEDFrame(position1, cbn1);
        final NEDFrame nedFrame2 = new NEDFrame(position2, cbn2);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity1, ecefFrame1, timestamp1,
                        magneticFluxDensityStandardDeviation1);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity2, ecefFrame2, timestamp2,
                        magneticFluxDensityStandardDeviation2);

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        assertEquals(bodyMagneticFluxDensity1,
                frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame1, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(frameBodyMagneticFluxDensity1.getYear(),
                frameBodyMagneticFluxDensity2.getYear(), 0.0);
        assertEquals(frameBodyMagneticFluxDensity1.getMagneticFluxDensityStandardDeviation(),
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testCopyTo() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation1 =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final double magneticFluxDensityStandardDeviation2 =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position1 = createPosition();
        final NEDPosition position2 = createPosition();
        final Date timestamp1 = createTimestamp();
        final Date timestamp2 = createTimestamp();
        final CoordinateTransformation cnb1 = createAttitude();
        final CoordinateTransformation cnb2 = createAttitude();
        final CoordinateTransformation cbn1 = cnb1.inverseAndReturnNew();
        final CoordinateTransformation cbn2 = cnb1.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity1 =
                createMagneticFluxDensity(position1, timestamp1, cnb1);
        final BodyMagneticFluxDensity bodyMagneticFluxDensity2 =
                createMagneticFluxDensity(position2, timestamp2, cnb2);

        final NEDFrame nedFrame1 = new NEDFrame(position1, cbn1);
        final NEDFrame nedFrame2 = new NEDFrame(position2, cbn2);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity1, ecefFrame1, timestamp1,
                        magneticFluxDensityStandardDeviation1);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(
                        bodyMagneticFluxDensity2, ecefFrame2, timestamp2,
                        magneticFluxDensityStandardDeviation2);

        frameBodyMagneticFluxDensity1.copyTo(frameBodyMagneticFluxDensity2);

        // check
        assertEquals(bodyMagneticFluxDensity1,
                frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame1, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(frameBodyMagneticFluxDensity1.getYear(),
                frameBodyMagneticFluxDensity2.getYear(), 0.0);
        assertEquals(frameBodyMagneticFluxDensity1.getMagneticFluxDensityStandardDeviation(),
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(),
                0.0);
    }

    @Test
    public void testHashCode() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity3 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        assertEquals(frameBodyMagneticFluxDensity1.hashCode(),
                frameBodyMagneticFluxDensity2.hashCode());
        assertNotEquals(frameBodyMagneticFluxDensity1.hashCode(),
                frameBodyMagneticFluxDensity3.hashCode());
    }

    @Test
    public void testEquals() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity3 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frameBodyMagneticFluxDensity1.equals((Object) frameBodyMagneticFluxDensity1));
        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity1));
        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity2));
        assertFalse(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(frameBodyMagneticFluxDensity1.equals((Object) null));
        assertFalse(frameBodyMagneticFluxDensity1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(frameBodyMagneticFluxDensity1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity2 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity3 =
                new StandardDeviationFrameBodyMagneticFluxDensity();

        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity1, THRESHOLD));
        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity2, THRESHOLD));
        assertFalse(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity3, THRESHOLD));
        assertFalse(frameBodyMagneticFluxDensity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            CloneNotSupportedException {

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magneticFluxDensityStandardDeviation =
                randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();

        final BodyMagneticFluxDensity bodyMagneticFluxDensity =
                createMagneticFluxDensity(position, timestamp, cnb);

        final NEDFrame nedFrame = new NEDFrame(position, cbn);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final StandardDeviationFrameBodyMagneticFluxDensity frameBodyMagneticFluxDensity1 =
                new StandardDeviationFrameBodyMagneticFluxDensity(bodyMagneticFluxDensity,
                        ecefFrame, timestamp,
                        magneticFluxDensityStandardDeviation);

        final Object frameBodyMagneticFluxDensity2 =
                frameBodyMagneticFluxDensity1.clone();

        // check
        assertEquals(frameBodyMagneticFluxDensity1,
                frameBodyMagneticFluxDensity2);
    }

    private static BodyMagneticFluxDensity createMagneticFluxDensity()
            throws IOException {
        final NEDPosition position = createPosition();
        final Date timestamp = createTimestamp();
        final CoordinateTransformation cnb = createAttitude();
        return createMagneticFluxDensity(position, timestamp, cnb);
    }

    private static BodyMagneticFluxDensity createMagneticFluxDensity(
            final NEDPosition position, final Date timestamp,
            final CoordinateTransformation cnb)
            throws IOException {
        final WMMEarthMagneticFluxDensityEstimator wMMEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final NEDMagneticFluxDensity earthB = wMMEstimator.estimate(
                position, timestamp);
        return BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
    }

    private static CoordinateTransformation createAttitude() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());

        final double roll = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw,
                FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static NEDPosition createPosition() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static Date createTimestamp() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        return new Date(randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS));
    }
}
