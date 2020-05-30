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
package com.irurueta.navigation.geodesic.wmm;

import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Random;

import static org.junit.Assert.*;

public class WMMEarthMagneticFluxDensityEstimatorTest {

    private static final String FILE_PATH =
            "./src/main/resources/com/irurueta/navigation/geodesic/wmm/wmm.cof";

    private static final double TO_NANO = 1e9;

    private static final double ANGLE_ERROR =
            WMMEarthMagneticFluxDensityEstimator
                    .ANGLE_ACCURACY_DEGREES;
    private static final double INTENSITY_ERROR =
            WMMEarthMagneticFluxDensityEstimator
                    .INTENSITY_ACCURACY;
    private static final double TIME_ERROR = 0.0005;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

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
    public void testConstants() {
        assertEquals(WMMEarthMagneticFluxDensityEstimator
                .ANGLE_ACCURACY_DEGREES, 5e-3, 0.0);
        assertEquals(WMMEarthMagneticFluxDensityEstimator
                        .ANGLE_ACCURACY_RADIANS,
                Math.toRadians(WMMEarthMagneticFluxDensityEstimator
                        .ANGLE_ACCURACY_DEGREES), 0.0);
        assertEquals(WMMEarthMagneticFluxDensityEstimator
                .INTENSITY_ACCURACY, 5e-2, 0.0);
    }

    @Test
    public void testConstructor() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator1 =
                new WMMEarthMagneticFluxDensityEstimator();
        assertNotNull(estimator1);

        final WorldMagneticModel model = WMMLoader.loadFromFile(FILE_PATH);
        final WMMEarthMagneticFluxDensityEstimator estimator2 =
                new WMMEarthMagneticFluxDensityEstimator(model);
        assertNotNull(estimator2);

        // Force NullPointerException
        try {
            new WMMEarthMagneticFluxDensityEstimator(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testDeclinationModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-112.41, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(89.0),
                        Math.toRadians(-121.0), 28e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-112.41, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(89.0),
                        Math.toRadians(-121.0), 28e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-112.39, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(89.0),
                        Math.toRadians(-121.0), 27e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-37.40, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(80.0),
                        Math.toRadians(-96), 48e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(51.30, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(82.0),
                        Math.toRadians(87.0), 54e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(0.71, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(43.0),
                        Math.toRadians(93.0), 65e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-5.78, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-33.0),
                        Math.toRadians(109.0), 51e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-15.79, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-59.0),
                        Math.toRadians(-8.0), 39e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(28.10, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-50.0),
                        Math.toRadians(-103.0), 3e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(15.82, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-29.0),
                        Math.toRadians(-110.0), 94e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(0.12, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(14.0),
                        Math.toRadians(143.0), 66e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(1.05, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(0.0),
                        Math.toRadians(21.0), 18e3, 2020.0)),
                ANGLE_ERROR);


        assertEquals(20.16, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-36.0),
                        Math.toRadians(-137.0), 6e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(0.43, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(26.0),
                        Math.toRadians(81.0), 63e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(13.39, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(38.0),
                        Math.toRadians(-144.0), 69e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(57.40, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-70.0),
                        Math.toRadians(-133.0), 50e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(15.39, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-52.0),
                        Math.toRadians(-75.0), 8e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-32.56, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-66.0),
                        Math.toRadians(17.0), 8e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(9.15, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-37.0),
                        Math.toRadians(140.0), 22e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(10.83, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-12.0),
                        Math.toRadians(-129.0), 40e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(11.46, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(33.0),
                        Math.toRadians(-118.0), 44e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(28.65, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-81.0),
                        Math.toRadians(-67.0), 50e3, 2020.5)),
                ANGLE_ERROR);


        assertEquals(-22.29, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-57.0),
                        Math.toRadians(3.0), 74e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(14.02, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-24.0),
                        Math.toRadians(-122.0), 46e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(1.08, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(23.0),
                        Math.toRadians(63.0), 69e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(9.74, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-3.0),
                        Math.toRadians(-147.0), 33e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-6.05, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-72.0),
                        Math.toRadians(-22.0), 47e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-1.71, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-14.0),
                        Math.toRadians(99.0), 62e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-36.71, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(86.0),
                        Math.toRadians(-46.0), 83e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-80.81, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-64.0),
                        Math.toRadians(87.0), 82e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-14.32, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-19.0),
                        Math.toRadians(43.0), 34e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-59.03, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-81.0),
                        Math.toRadians(40.0), 56e3, 2021.0)),
                ANGLE_ERROR);


        assertEquals(-3.41, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(0.0),
                        Math.toRadians(80.0), 14e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(30.36, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-82.0),
                        Math.toRadians(-68.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-11.54, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-46.0),
                        Math.toRadians(-42.0), 44e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(1.23, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(17.0),
                        Math.toRadians(52.0), 43e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-1.71, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(10.0),
                        Math.toRadians(78.0), 64e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(12.36, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(33.0),
                        Math.toRadians(-145.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-136.34, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-79.0),
                        Math.toRadians(115.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(18.10, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-33.0),
                        Math.toRadians(-114), 14e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(2.13, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(29.0),
                        Math.toRadians(66.0), 19e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(10.11, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-11.0),
                        Math.toRadians(167.0), 86e3, 2021.5)),
                ANGLE_ERROR);


        assertEquals(-16.99, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-66.0),
                        Math.toRadians(-5.0), 37e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(15.47, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(72.0),
                        Math.toRadians(-115.0), 67e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(6.56, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(22.0),
                        Math.toRadians(174.0), 44e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(1.43, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(54.0),
                        Math.toRadians(178.0), 54e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-47.43, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-43.0),
                        Math.toRadians(50.0), 57e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(24.32, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-43.0),
                        Math.toRadians(-111.0), 44e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(57.08, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-63.0),
                        Math.toRadians(178.0), 12e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(8.76, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(27.0),
                        Math.toRadians(-169.0), 38e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-17.63, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(59.0),
                        Math.toRadians(-77.0), 61e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-14.09, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-47.0),
                        Math.toRadians(-32.0), 67e3, 2022.0)),
                ANGLE_ERROR);


        assertEquals(18.95, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(62.0),
                        Math.toRadians(53.0), 8e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-15.94, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-68.0),
                        Math.toRadians(-7.0), 77e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(7.79, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-5.0),
                        Math.toRadians(159.0), 98e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(15.68, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-29.0),
                        Math.toRadians(-107.0), 34e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(1.78, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(27.0),
                        Math.toRadians(65.0), 60e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-101.49, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-72.0),
                        Math.toRadians(95.0), 73e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(18.38, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-46.0),
                        Math.toRadians(-85.0), 96e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-16.65, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-13.0),
                        Math.toRadians(-59.0), 0e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(1.92, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(66.0),
                        Math.toRadians(-178), 16e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-64.66, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-87.0),
                        Math.toRadians(38.0), 72e3, 2022.5)),
                ANGLE_ERROR);


        assertEquals(5.20, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(20.0),
                        Math.toRadians(167.0), 49e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-7.26, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(5.0),
                        Math.toRadians(-13.0), 71e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-0.56, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(14.0),
                        Math.toRadians(65.0), 95e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(41.76, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-85.0),
                        Math.toRadians(-79.0), 86e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-3.87, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-36.0),
                        Math.toRadians(-64.0), 30e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-14.54, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(79.0),
                        Math.toRadians(125.0), 75e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-15.22, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(6.0),
                        Math.toRadians(-32.0), 21e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(30.36, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-76.0),
                        Math.toRadians(-75.0), 1e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-11.94, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-46),
                        Math.toRadians(-41.0), 45e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-24.12, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-22.0),
                        Math.toRadians(-21.0), 11e3, 2023.0)),
                ANGLE_ERROR);


        assertEquals(16.20, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(54.0),
                        Math.toRadians(-120.0), 28e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(40.48, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-58.0),
                        Math.toRadians(156.0), 68e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(29.86, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-65.0),
                        Math.toRadians(-88.0), 39e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-13.98, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-23.0),
                        Math.toRadians(81.0), 27e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(1.08, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(34.0),
                        Math.toRadians(0.0), 11e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-66.98, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-62.0),
                        Math.toRadians(65.0), 72e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(61.19, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(86.0),
                        Math.toRadians(70.0), 55e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(0.36, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(32.0),
                        Math.toRadians(163.0), 59e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-9.39, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(48.0),
                        Math.toRadians(148.0), 65e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(4.49, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(30.0),
                        Math.toRadians(28.0), 95e3, 2023.5)),
                ANGLE_ERROR);


        assertEquals(8.86, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-60.0),
                        Math.toRadians(-59.0), 95e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-54.29, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-70.0),
                        Math.toRadians(42.0), 95e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-82.22, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(87.0),
                        Math.toRadians(-154.0), 50e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(3.94, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(32.0),
                        Math.toRadians(19.0), 58e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-2.62, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(34.0),
                        Math.toRadians(-13.0), 57e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-63.51, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-76.0),
                        Math.toRadians(49.0), 38e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(31.57, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-50.0),
                        Math.toRadians(-179.0), 49e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(38.07, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-55.0),
                        Math.toRadians(-171.0), 90e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-5.00, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(42.0),
                        Math.toRadians(-19.0), 41e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-6.60, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(46.0),
                        Math.toRadians(-22.0), 19e3, 2024.0)),
                ANGLE_ERROR);


        assertEquals(9.21, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(13.0),
                        Math.toRadians(-132.0), 31e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(7.16, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-2.0),
                        Math.toRadians(158.0), 93e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-55.63, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-76.0),
                        Math.toRadians(40.0), 51e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(10.52, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(22.0),
                        Math.toRadians(-132.0), 64e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-62.60, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-65.0),
                        Math.toRadians(55.0), 26e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-13.34, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-21.0),
                        Math.toRadians(32.0), 66e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(9.39, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(9.0),
                        Math.toRadians(-172.0), 18e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(29.81, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(88.0),
                        Math.toRadians(26.0), 63e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(0.61, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(17.0),
                        Math.toRadians(5.0), 33e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(4.63, Math.toDegrees(
                estimator.getDeclination(Math.toRadians(-18.0),
                        Math.toRadians(138.0), 77e3, 2024.5)),
                ANGLE_ERROR);
    }

    @Test
    public void testDipModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(88.46, Math.toDegrees(
                estimator.getDip(Math.toRadians(89.0),
                        Math.toRadians(-121.0), 28e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(88.03, Math.toDegrees(
                estimator.getDip(Math.toRadians(80.0),
                        Math.toRadians(-96.0), 48e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(87.48, Math.toDegrees(
                estimator.getDip(Math.toRadians(82.0),
                        Math.toRadians(87.0), 54e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(63.87, Math.toDegrees(
                estimator.getDip(Math.toRadians(43.0),
                        Math.toRadians(93.0), 65e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-67.64, Math.toDegrees(
                estimator.getDip(Math.toRadians(-33.0),
                        Math.toRadians(109.0), 51e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-58.82, Math.toDegrees(
                estimator.getDip(Math.toRadians(-59.0),
                        Math.toRadians(-8.0), 39e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-55.01, Math.toDegrees(
                estimator.getDip(Math.toRadians(-50.0),
                        Math.toRadians(-103.0), 3e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-38.38, Math.toDegrees(
                estimator.getDip(Math.toRadians(-29.0),
                        Math.toRadians(-110.0), 94e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(13.08, Math.toDegrees(
                estimator.getDip(Math.toRadians(14.0),
                        Math.toRadians(143.0), 66e3, 2020.0)),
                ANGLE_ERROR);
        assertEquals(-26.46, Math.toDegrees(
                estimator.getDip(Math.toRadians(0.0),
                        Math.toRadians(21.0), 18e3, 2020.0)),
                ANGLE_ERROR);


        assertEquals(-52.21, Math.toDegrees(
                estimator.getDip(Math.toRadians(-36.0),
                        Math.toRadians(-137), 6e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(40.84, Math.toDegrees(
                estimator.getDip(Math.toRadians(26.0),
                        Math.toRadians(81.0), 63e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(56.99, Math.toDegrees(
                estimator.getDip(Math.toRadians(38.0),
                        Math.toRadians(-144.0), 69e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-72.18, Math.toDegrees(
                estimator.getDip(Math.toRadians(-70.0),
                        Math.toRadians(-133.0), 50e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-49.50, Math.toDegrees(
                estimator.getDip(Math.toRadians(-52.0),
                        Math.toRadians(-75.0), 8e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-59.78, Math.toDegrees(
                estimator.getDip(Math.toRadians(-66.0),
                        Math.toRadians(17.0), 8e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-68.61, Math.toDegrees(
                estimator.getDip(Math.toRadians(-37.0),
                        Math.toRadians(140.0), 22e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-15.68, Math.toDegrees(
                estimator.getDip(Math.toRadians(-12.0),
                        Math.toRadians(-129.0), 40e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(57.97, Math.toDegrees(
                estimator.getDip(Math.toRadians(33.0),
                        Math.toRadians(-118.0), 44e3, 2020.5)),
                ANGLE_ERROR);
        assertEquals(-67.74, Math.toDegrees(
                estimator.getDip(Math.toRadians(-81.0),
                        Math.toRadians(-67.0), 50e3, 2020.5)),
                ANGLE_ERROR);


        assertEquals(-59.07, Math.toDegrees(
                estimator.getDip(Math.toRadians(-57.0),
                        Math.toRadians(3.0), 74e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-34.29, Math.toDegrees(
                estimator.getDip(Math.toRadians(-24.0),
                        Math.toRadians(-122.0), 46e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(35.82, Math.toDegrees(
                estimator.getDip(Math.toRadians(23.0),
                        Math.toRadians(63.0), 69e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-2.35, Math.toDegrees(
                estimator.getDip(Math.toRadians(-3.0),
                        Math.toRadians(-147.0), 33e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-61.27, Math.toDegrees(
                estimator.getDip(Math.toRadians(-72.0),
                        Math.toRadians(-22.0), 47e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-45.11, Math.toDegrees(
                estimator.getDip(Math.toRadians(-14.0),
                        Math.toRadians(99.0), 62e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(86.83, Math.toDegrees(
                estimator.getDip(Math.toRadians(86.0),
                        Math.toRadians(-46.0), 83e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-75.25, Math.toDegrees(
                estimator.getDip(Math.toRadians(-64.0),
                        Math.toRadians(87.0), 82e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-52.47, Math.toDegrees(
                estimator.getDip(Math.toRadians(-19.0),
                        Math.toRadians(43.0), 34e3, 2021.0)),
                ANGLE_ERROR);
        assertEquals(-68.54, Math.toDegrees(
                estimator.getDip(Math.toRadians(-81.0),
                        Math.toRadians(40.0), 56e3, 2021.0)),
                ANGLE_ERROR);


        assertEquals(-17.32, Math.toDegrees(
                estimator.getDip(Math.toRadians(0.0),
                        Math.toRadians(80.0), 14e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-68.18, Math.toDegrees(
                estimator.getDip(Math.toRadians(-82.0),
                        Math.toRadians(-68.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-53.82, Math.toDegrees(
                estimator.getDip(Math.toRadians(-46.0),
                        Math.toRadians(-42.0), 44e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(23.87, Math.toDegrees(
                estimator.getDip(Math.toRadians(17.0),
                        Math.toRadians(52.0), 43e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(7.37, Math.toDegrees(
                estimator.getDip(Math.toRadians(10.0),
                        Math.toRadians(78.0), 64e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(52.51, Math.toDegrees(
                estimator.getDip(Math.toRadians(33.0),
                        Math.toRadians(-145.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-77.43, Math.toDegrees(
                estimator.getDip(Math.toRadians(-79.0),
                        Math.toRadians(115.0), 12e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-44.23, Math.toDegrees(
                estimator.getDip(Math.toRadians(-33.0),
                        Math.toRadians(-114.0), 14e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(45.97, Math.toDegrees(
                estimator.getDip(Math.toRadians(29.0),
                        Math.toRadians(66.0), 19e3, 2021.5)),
                ANGLE_ERROR);
        assertEquals(-31.45, Math.toDegrees(
                estimator.getDip(Math.toRadians(-11.0),
                        Math.toRadians(167.0), 86e3, 2021.5)),
                ANGLE_ERROR);


        assertEquals(-59.27, Math.toDegrees(
                estimator.getDip(Math.toRadians(-66.0),
                        Math.toRadians(-5.0), 37e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(85.19, Math.toDegrees(
                estimator.getDip(Math.toRadians(72.0),
                        Math.toRadians(-115.0), 67e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(31.91, Math.toDegrees(
                estimator.getDip(Math.toRadians(22.0),
                        Math.toRadians(174.0), 44e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(65.41, Math.toDegrees(
                estimator.getDip(Math.toRadians(54.0),
                        Math.toRadians(178.0), 54e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-62.96, Math.toDegrees(
                estimator.getDip(Math.toRadians(-43.0),
                        Math.toRadians(50.0), 57e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-52.71, Math.toDegrees(
                estimator.getDip(Math.toRadians(-43.0),
                        Math.toRadians(-111.0), 44e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-79.33, Math.toDegrees(
                estimator.getDip(Math.toRadians(-63.0),
                        Math.toRadians(178.0), 12e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(42.60, Math.toDegrees(
                estimator.getDip(Math.toRadians(27.0),
                        Math.toRadians(-169.0), 38e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(79.04, Math.toDegrees(
                estimator.getDip(Math.toRadians(59.0),
                        Math.toRadians(-77.0), 61e3, 2022.0)),
                ANGLE_ERROR);
        assertEquals(-57.63, Math.toDegrees(
                estimator.getDip(Math.toRadians(-47.0),
                        Math.toRadians(-32.0), 67e3, 2022.0)),
                ANGLE_ERROR);


        assertEquals(76.54, Math.toDegrees(
                estimator.getDip(Math.toRadians(62.0),
                        Math.toRadians(53.0), 8e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-60.00, Math.toDegrees(
                estimator.getDip(Math.toRadians(-68.0),
                        Math.toRadians(-7.0), 77e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-23.04, Math.toDegrees(
                estimator.getDip(Math.toRadians(-5.0),
                        Math.toRadians(159.0), 98e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-37.65, Math.toDegrees(
                estimator.getDip(Math.toRadians(-29.0),
                        Math.toRadians(-107.0), 34e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(42.84, Math.toDegrees(
                estimator.getDip(Math.toRadians(27.0),
                        Math.toRadians(65.0), 60e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-76.44, Math.toDegrees(
                estimator.getDip(Math.toRadians(-72.0),
                        Math.toRadians(95.0), 73e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-47.38, Math.toDegrees(
                estimator.getDip(Math.toRadians(-46.0),
                        Math.toRadians(-85.0), 96e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-13.41, Math.toDegrees(
                estimator.getDip(Math.toRadians(-13.0),
                        Math.toRadians(-59.0), 0e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(75.67, Math.toDegrees(
                estimator.getDip(Math.toRadians(66.0),
                        Math.toRadians(-178.0), 16e3, 2022.5)),
                ANGLE_ERROR);
        assertEquals(-71.05, Math.toDegrees(
                estimator.getDip(Math.toRadians(-87.0),
                        Math.toRadians(38.0), 72e3, 2022.5)),
                ANGLE_ERROR);


        assertEquals(26.85, Math.toDegrees(
                estimator.getDip(Math.toRadians(20.0),
                        Math.toRadians(167.0), 49e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-17.38, Math.toDegrees(
                estimator.getDip(Math.toRadians(5.0),
                        Math.toRadians(-13.0), 71e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(17.52, Math.toDegrees(
                estimator.getDip(Math.toRadians(14.0),
                        Math.toRadians(65.0), 95e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-70.36, Math.toDegrees(
                estimator.getDip(Math.toRadians(-85.0),
                        Math.toRadians(-79.0), 86e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-39.40, Math.toDegrees(
                estimator.getDip(Math.toRadians(-36.0),
                        Math.toRadians(-64.0), 30e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(87.30, Math.toDegrees(
                estimator.getDip(Math.toRadians(79.0),
                        Math.toRadians(125.0), 75e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-7.26, Math.toDegrees(
                estimator.getDip(Math.toRadians(6.0),
                        Math.toRadians(-32.0), 21e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-65.32, Math.toDegrees(
                estimator.getDip(Math.toRadians(-76.0),
                        Math.toRadians(-75.0), 1e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-54.45, Math.toDegrees(
                estimator.getDip(Math.toRadians(-46.0),
                        Math.toRadians(-41.0), 45e3, 2023.0)),
                ANGLE_ERROR);
        assertEquals(-56.82, Math.toDegrees(
                estimator.getDip(Math.toRadians(-22.0),
                        Math.toRadians(-21.0), 11e3, 2023.0)),
                ANGLE_ERROR);


        assertEquals(74.02, Math.toDegrees(
                estimator.getDip(Math.toRadians(54.0),
                        Math.toRadians(-120.0), 28e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-81.60, Math.toDegrees(
                estimator.getDip(Math.toRadians(-58.0),
                        Math.toRadians(156.0), 68e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-60.29, Math.toDegrees(
                estimator.getDip(Math.toRadians(-65.0),
                        Math.toRadians(-88.0), 39e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-58.52, Math.toDegrees(
                estimator.getDip(Math.toRadians(-23.0),
                        Math.toRadians(81.0), 27e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(46.69, Math.toDegrees(
                estimator.getDip(Math.toRadians(34.0),
                        Math.toRadians(0.0), 11e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(-68.38, Math.toDegrees(
                estimator.getDip(Math.toRadians(-62.0),
                        Math.toRadians(65.0), 72e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(87.51, Math.toDegrees(
                estimator.getDip(Math.toRadians(86.0),
                        Math.toRadians(70.0), 55e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(43.05, Math.toDegrees(
                estimator.getDip(Math.toRadians(32.0),
                        Math.toRadians(163.0), 59e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(61.70, Math.toDegrees(
                estimator.getDip(Math.toRadians(48.0),
                        Math.toRadians(148.0), 65e3, 2023.5)),
                ANGLE_ERROR);
        assertEquals(44.12, Math.toDegrees(
                estimator.getDip(Math.toRadians(30.0),
                        Math.toRadians(28.0), 95e3, 2023.5)),
                ANGLE_ERROR);


        assertEquals(-55.03, Math.toDegrees(
                estimator.getDip(Math.toRadians(-60.0),
                        Math.toRadians(-59.0), 95e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-64.59, Math.toDegrees(
                estimator.getDip(Math.toRadians(-70.0),
                        Math.toRadians(42.0), 95e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(89.39, Math.toDegrees(
                estimator.getDip(Math.toRadians(87.0),
                        Math.toRadians(-154.0), 50e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(45.89, Math.toDegrees(
                estimator.getDip(Math.toRadians(32.0),
                        Math.toRadians(19.0), 58e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(45.83, Math.toDegrees(
                estimator.getDip(Math.toRadians(34.0),
                        Math.toRadians(-13.0), 57e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-67.40, Math.toDegrees(
                estimator.getDip(Math.toRadians(-76.0),
                        Math.toRadians(49.0), 38e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-71.40, Math.toDegrees(
                estimator.getDip(Math.toRadians(-50.0),
                        Math.toRadians(-179.0), 49e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(-72.91, Math.toDegrees(
                estimator.getDip(Math.toRadians(-55.0),
                        Math.toRadians(-171.0), 90e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(56.57, Math.toDegrees(
                estimator.getDip(Math.toRadians(42.0),
                        Math.toRadians(-19.0), 41e3, 2024.0)),
                ANGLE_ERROR);
        assertEquals(61.04, Math.toDegrees(
                estimator.getDip(Math.toRadians(46.0),
                        Math.toRadians(-22.0), 19e3, 2024.0)),
                ANGLE_ERROR);


        assertEquals(31.51, Math.toDegrees(
                estimator.getDip(Math.toRadians(13.0),
                        Math.toRadians(-132.0), 31e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-17.78, Math.toDegrees(
                estimator.getDip(Math.toRadians(-2.0),
                        Math.toRadians(158.0), 93e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-66.27, Math.toDegrees(
                estimator.getDip(Math.toRadians(-76.0),
                        Math.toRadians(40.0), 51e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(43.88, Math.toDegrees(
                estimator.getDip(Math.toRadians(22.0),
                        Math.toRadians(-132.0), 64e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-65.67, Math.toDegrees(
                estimator.getDip(Math.toRadians(-65.0),
                        Math.toRadians(55.0), 26e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-56.95, Math.toDegrees(
                estimator.getDip(Math.toRadians(-21.0),
                        Math.toRadians(32.0), 66e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(15.78, Math.toDegrees(
                estimator.getDip(Math.toRadians(9.0),
                        Math.toRadians(-172.0), 18e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(87.38, Math.toDegrees(
                estimator.getDip(Math.toRadians(88.0),
                        Math.toRadians(26.0), 63e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(13.58, Math.toDegrees(
                estimator.getDip(Math.toRadians(17.0),
                        Math.toRadians(5.0), 33e3, 2024.5)),
                ANGLE_ERROR);
        assertEquals(-47.71, Math.toDegrees(
                estimator.getDip(Math.toRadians(-18.0),
                        Math.toRadians(138.0), 77e3, 2024.5)),
                ANGLE_ERROR);
    }

    @Test
    public void testHorizontalIntensityModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(1510.0,
                estimator.getHorizontalIntensity(
                        Math.toRadians(89.0), Math.toRadians(-121.0),
                        28e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1910.8, estimator.getHorizontalIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0),
                48e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2487.8, estimator.getHorizontalIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0),
                54e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24377.2, estimator.getHorizontalIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0),
                65e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(21666.6, estimator.getHorizontalIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0),
                51e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14933.4, estimator.getHorizontalIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0),
                39e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22315.5, estimator.getHorizontalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0),
                3e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24392.0, estimator.getHorizontalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0),
                94e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34916.9, estimator.getHorizontalIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0),
                66e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29316.1, estimator.getHorizontalIntensity(
                Math.toRadians(0.0), Math.toRadians(21.0),
                18e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(25511.4, estimator.getHorizontalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0),
                6e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34738.7, estimator.getHorizontalIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0),
                63e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23279.9, estimator.getHorizontalIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0),
                69e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16597.2, estimator.getHorizontalIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20299.7, estimator.getHorizontalIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18089.7, estimator.getHorizontalIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(21705.2, estimator.getHorizontalIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0),
                22e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29295.6, estimator.getHorizontalIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0),
                40e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23890.9, estimator.getHorizontalIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0),
                44e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18332.1, estimator.getHorizontalIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(14296.6, estimator.getHorizontalIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0),
                74e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(26836.5, estimator.getHorizontalIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0),
                46e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34456.5, estimator.getHorizontalIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0),
                69e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31138.6, estimator.getHorizontalIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0),
                33e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18455.7, estimator.getHorizontalIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0),
                47e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33227.7, estimator.getHorizontalIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(3004.8, estimator.getHorizontalIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0),
                83e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14087.8, estimator.getHorizontalIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0),
                82e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19947.3, estimator.getHorizontalIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0),
                34e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17872.0, estimator.getHorizontalIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0),
                56e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(39316.2, estimator.getHorizontalIntensity(
                Math.toRadians(0.0), Math.toRadians(80.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18536.8, estimator.getHorizontalIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14450.9, estimator.getHorizontalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0),
                44e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35904.4, estimator.getHorizontalIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0),
                43e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(39311.5, estimator.getHorizontalIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0),
                64e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24878.3, estimator.getHorizontalIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(12997.3, estimator.getHorizontalIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24820.9, estimator.getHorizontalIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32640.6, estimator.getHorizontalIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0),
                19e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33191.7, estimator.getHorizontalIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0),
                86e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(17152.6, estimator.getHorizontalIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0),
                37e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4703.2, estimator.getHorizontalIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28859.7, estimator.getHorizontalIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20631.2, estimator.getHorizontalIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0),
                54e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16769.3, estimator.getHorizontalIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0),
                57e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22656.4, estimator.getHorizontalIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(11577.2, estimator.getHorizontalIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0),
                12e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(26202.3, estimator.getHorizontalIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0),
                38e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10595.8, estimator.getHorizontalIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0),
                61e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13056.5, estimator.getHorizontalIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(13043.3, estimator.getHorizontalIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0),
                8e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17268.3, estimator.getHorizontalIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0),
                77e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33927.0, estimator.getHorizontalIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24657.0, estimator.getHorizontalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0),
                34e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32954.8, estimator.getHorizontalIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0),
                60e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13362.8, estimator.getHorizontalIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0),
                73e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20165.1, estimator.getHorizontalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0),
                96e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22751.7, estimator.getHorizontalIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0),
                0e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13812.2, estimator.getHorizontalIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0),
                16e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16666.4, estimator.getHorizontalIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0),
                72e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(30223.6, estimator.getHorizontalIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0),
                49e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28445.5, estimator.getHorizontalIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0),
                71e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36805.8, estimator.getHorizontalIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0),
                95e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16888.7, estimator.getHorizontalIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0),
                86e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17735.3, estimator.getHorizontalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0),
                30e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2692.1, estimator.getHorizontalIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0),
                75e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28634.3, estimator.getHorizontalIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0),
                21e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19700.9, estimator.getHorizontalIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0),
                1e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14187.4, estimator.getHorizontalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0),
                45e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13972.9, estimator.getHorizontalIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21),
                11e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(15158.8, estimator.getHorizontalIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0),
                28e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9223.1, estimator.getHorizontalIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0),
                68e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20783.0, estimator.getHorizontalIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0),
                39e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(25568.2, estimator.getHorizontalIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0),
                27e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29038.8, estimator.getHorizontalIntensity(
                Math.toRadians(34.0), Math.toRadians(0.0),
                11e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17485.0, estimator.getHorizontalIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0),
                72e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2424.9, estimator.getHorizontalIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0),
                55e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28170.6, estimator.getHorizontalIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0),
                59e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23673.8, estimator.getHorizontalIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0),
                65e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29754.7, estimator.getHorizontalIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0),
                95e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(18317.9, estimator.getHorizontalIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18188.3, estimator.getHorizontalIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(597.9, estimator.getHorizontalIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0),
                50e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29401.0, estimator.getHorizontalIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0),
                58e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28188.3, estimator.getHorizontalIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0),
                57e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18425.8, estimator.getHorizontalIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0),
                38e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18112.2, estimator.getHorizontalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0),
                49e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16409.7, estimator.getHorizontalIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0),
                90e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24410.2, estimator.getHorizontalIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0),
                41e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22534.0, estimator.getHorizontalIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0),
                19e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(28413.4, estimator.getHorizontalIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0),
                31e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34124.3, estimator.getHorizontalIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0),
                93e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18529.2, estimator.getHorizontalIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0),
                51e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(26250.1, estimator.getHorizontalIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0),
                64e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18702.1, estimator.getHorizontalIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0),
                26e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15940.8, estimator.getHorizontalIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31031.6, estimator.getHorizontalIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0),
                18e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2523.6, estimator.getHorizontalIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0),
                63e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34062.9, estimator.getHorizontalIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0),
                33e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31825.9, estimator.getHorizontalIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0),
                77e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
    }

    @Test
    public void testNorthIntensityModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-575.7, estimator.getNorthIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0),
                28e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1518.0, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0),
                48e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1555.6, estimator.getNorthIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0),
                54e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24375.3, estimator.getNorthIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0),
                65e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(21556.3, estimator.getNorthIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0),
                51e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14369.9, estimator.getNorthIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8),
                39e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19684.4, estimator.getNorthIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0),
                3e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23467.8, estimator.getNorthIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0),
                94e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34916.8, estimator.getNorthIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0),
                66e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29311.2, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(21.0),
                18e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(23948.6, estimator.getNorthIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0),
                6e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34737.7, estimator.getNorthIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0),
                63e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22647.3, estimator.getNorthIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0),
                69e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8943.1, estimator.getNorthIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19571.7, estimator.getNorthIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15247.0, estimator.getNorthIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(21429.0, estimator.getNorthIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0),
                22e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28773.9, estimator.getNorthIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0),
                40e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23414.3, estimator.getNorthIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0),
                44e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16087.9, estimator.getNorthIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(13228.0, estimator.getNorthIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0),
                74e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(26037.0, estimator.getNorthIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0),
                46e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34450.4, estimator.getNorthIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0),
                69e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30690.1, estimator.getNorthIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0),
                33e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18352.8, estimator.getNorthIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0),
                47e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33213.0, estimator.getNorthIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2408.8, estimator.getNorthIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0),
                83e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2249.5, estimator.getNorthIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0),
                82e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19327.6, estimator.getNorthIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0),
                34e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9198.0, estimator.getNorthIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0),
                56e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(39246.6, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(80.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15995.1, estimator.getNorthIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(14159.0, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0),
                44e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35896.1, estimator.getNorthIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0),
                43e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(39294.0, estimator.getNorthIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0),
                64e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24301.2, estimator.getNorthIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-9403.6, estimator.getNorthIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23592.3, estimator.getNorthIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32618.0, estimator.getNorthIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0),
                19e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32676.0, estimator.getNorthIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0),
                86e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(16404.3, estimator.getNorthIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0),
                37e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4532.7, estimator.getNorthIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28671.0, estimator.getNorthIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20624.8, estimator.getNorthIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0),
                54e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(11344.6, estimator.getNorthIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0),
                57e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(20646.1, estimator.getNorthIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6292.0, estimator.getNorthIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0),
                12e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(25896.5, estimator.getNorthIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0),
                38e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10098.3, estimator.getNorthIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0),
                61e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(12663.7, estimator.getNorthIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(12336.1, estimator.getNorthIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0),
                8e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16604.7, estimator.getNorthIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0),
                77e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33613.6, estimator.getNorthIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23739.9, estimator.getNorthIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0),
                34e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32938.8, estimator.getNorthIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0),
                60e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2661.0, estimator.getNorthIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0),
                73e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(19136.4, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0),
                96e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(21797.3, estimator.getNorthIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0),
                0e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13804.5, estimator.getNorthIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0),
                16e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(7132.3, estimator.getNorthIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0),
                72e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(30099.4, estimator.getNorthIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0),
                49e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28217.7, estimator.getNorthIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0),
                71e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36804.1, estimator.getNorthIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0),
                95e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(12598.0, estimator.getNorthIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0),
                86e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17694.8, estimator.getNorthIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0),
                30e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2605.8, estimator.getNorthIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0),
                75e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(27630.5, estimator.getNorthIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0),
                21e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(16998.9, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0),
                1e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13880.3, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0),
                45e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(12752.8, estimator.getNorthIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0),
                11e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(14556.6, estimator.getNorthIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0),
                28e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(7015.1, estimator.getNorthIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0),
                68e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(18024.0, estimator.getNorthIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0),
                39e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24811.0, estimator.getNorthIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0),
                27e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29033.7, estimator.getNorthIntensity(
                Math.toRadians(34), Math.toRadians(0.0),
                11e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6836.8, estimator.getNorthIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0),
                72e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1168.7, estimator.getNorthIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0),
                55e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28170.0, estimator.getNorthIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0),
                59e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23356.8, estimator.getNorthIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0),
                65e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29663.3, estimator.getNorthIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0),
                95e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(18099.3, estimator.getNorthIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10615.3, estimator.getNorthIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(80.9, estimator.getNorthIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0),
                50e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29331.6, estimator.getNorthIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0),
                58e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28158.7, estimator.getNorthIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0),
                57e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8218.0, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0),
                38e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15431.4, estimator.getNorthIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0),
                49e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(12918.7, estimator.getNorthIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0),
                90e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24317.3, estimator.getNorthIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0),
                41e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22384.7, estimator.getNorthIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0),
                19e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(28046.9, estimator.getNorthIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0),
                31e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33858.1, estimator.getNorthIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0),
                93e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10459.6, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0),
                51e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(25808.9, estimator.getNorthIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0),
                64e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8607.6, estimator.getNorthIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0),
                26e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15510.7, estimator.getNorthIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15510.7, estimator.getNorthIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30615.4, estimator.getNorthIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0),
                18e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2189.6, estimator.getNorthIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0),
                63e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34060.9, estimator.getNorthIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0),
                33e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31722.0, estimator.getNorthIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0),
                77e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
    }

    @Test
    public void testEastIntensityModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-1396.0, estimator.getEastIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0),
                28e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1160.5, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0),
                48e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1941.4, estimator.getEastIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0),
                54e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(303.2, estimator.getEastIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0),
                65e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2183.2, estimator.getEastIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0),
                51e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-4063.3, estimator.getEastIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0),
                39e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10512.2, estimator.getEastIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0),
                3e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6650.9, estimator.getEastIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0),
                94e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(70.2, estimator.getEastIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0),
                66e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(536.0, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(21.0),
                18e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(8791.9, estimator.getEastIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0),
                6e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(259.2, estimator.getEastIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0),
                63e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5390.0, estimator.getEastIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0),
                69e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(13981.7, estimator.getEastIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5387.5, estimator.getEastIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-9734.8, estimator.getEastIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(3451.3, estimator.getEastIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0),
                22e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5503.9, estimator.getEastIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0),
                40e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4748.3, estimator.getEastIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0),
                44e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8788.9, estimator.getEastIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-5423.3, estimator.getEastIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0),
                74e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6501.8, estimator.getEastIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0),
                46e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(646.9, estimator.getEastIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0),
                69e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5265.7, estimator.getEastIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0),
                33e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1946.6, estimator.getEastIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0),
                47e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-990.3, estimator.getEastIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1796.2, estimator.getEastIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0),
                83e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-13907.0, estimator.getEastIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0),
                82e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-4933.5, estimator.getEastIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0),
                34e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-15323.4, estimator.getEastIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0),
                56e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-2338.9, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(80.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9368.5, estimator.getEastIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2889.8, estimator.getEastIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0),
                44e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(773.7, estimator.getEastIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0),
                43e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1172.2, estimator.getEastIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0),
                64e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5327.4, estimator.getEastIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-8972.3, estimator.getEastIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(7712.3, estimator.getEastIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1215.2, estimator.getEastIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0),
                19e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5828.1, estimator.getEastIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0),
                86e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-5011.2, estimator.getEastIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0),
                37e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1254.7, estimator.getEastIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(3294.9, estimator.getEastIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(514.8, estimator.getEastIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0),
                54e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-12349.5, estimator.getEastIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0),
                57e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9330.1, estimator.getEastIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9718.1, estimator.getEastIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0),
                12e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(3991.3, estimator.getEastIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0),
                38e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3208.9, estimator.getEastIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0),
                61e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3178.2, estimator.getEastIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(4236.6, estimator.getEastIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0),
                8e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-4741.2, estimator.getEastIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0),
                77e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4601.1, estimator.getEastIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6662.3, estimator.getEastIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0),
                34e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1025.8, estimator.getEastIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0),
                60e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-13095.2, estimator.getEastIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0),
                73e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(6358.2, estimator.getEastIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0),
                96e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-6520.6, estimator.getEastIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0),
                0e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(463.2, estimator.getEastIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0),
                16e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-15063.2, estimator.getEastIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0),
                72e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(2737.4, estimator.getEastIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0),
                49e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3592.6, estimator.getEastIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0),
                71e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-356.8, estimator.getEastIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0),
                95e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(11248.0, estimator.getEastIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0),
                86e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1198.1, estimator.getEastIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0),
                30e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-676.0, estimator.getEastIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0),
                75e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-7515.0, estimator.getEastIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0),
                21e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9958.1, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0),
                1e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2936.1, estimator.getEastIntensity(
                Math.toRadians(-46), Math.toRadians(-41.0),
                45e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-5710.4, estimator.getEastIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0),
                11e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(4230.3, estimator.getEastIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0),
                28e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5987.8, estimator.getEastIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0),
                68e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10347.2, estimator.getEastIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0),
                39e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-6176.1, estimator.getEastIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0),
                27e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(545.3, estimator.getEastIntensity(
                Math.toRadians(34.0), Math.toRadians(0.0),
                11e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-16092.9, estimator.getEastIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0),
                72e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2124.7, estimator.getEastIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0),
                55e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(176.0, estimator.getEastIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0),
                59e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3861.2, estimator.getEastIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0),
                65e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2331.2, estimator.getEastIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0),
                95e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(2821.2, estimator.getEastIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-14769.3, estimator.getEastIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-592.4, estimator.getEastIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0),
                50e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2019.5, estimator.getEastIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0),
                58e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1290.8, estimator.getEastIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0),
                57e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-16491.7, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0),
                38e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(9482.7, estimator.getEastIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0),
                49e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(10118.6, estimator.getEastIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0),
                90e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2127.0, estimator.getEastIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0),
                41e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-2590.4, estimator.getEastIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0),
                19e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(4548.8, estimator.getEastIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0),
                31e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4253.5, estimator.getEastIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0),
                93e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-15294.7, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0),
                51e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(4792.5, estimator.getEastIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0),
                64e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-16603.5, estimator.getEastIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0),
                26e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3677.9, estimator.getEastIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3677.9, estimator.getEastIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5065.5, estimator.getEastIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0),
                18e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(1254.6, estimator.getEastIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0),
                63e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(362.9, estimator.getEastIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0),
                33e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(2569.6, estimator.getEastIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0),
                77e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
    }

    @Test
    public void testVerticalIntensityModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(56082.3, estimator.getVerticalIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0),
                28e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55671.9, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0),
                48e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(56520.5, estimator.getVerticalIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0),
                54e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(49691.4, estimator.getVerticalIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0),
                65e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-52676.0, estimator.getVerticalIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0),
                51e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-24679.0, estimator.getVerticalIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0),
                39e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-31883.6, estimator.getVerticalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0),
                3e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-19320.7, estimator.getVerticalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0),
                94e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8114.9, estimator.getVerticalIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0),
                66e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-14589.0, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(21.0),
                18e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-32897.6, estimator.getVerticalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0),
                6e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30023.4, estimator.getVerticalIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0),
                63e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35831.9, estimator.getVerticalIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0),
                69e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-51628.5, estimator.getVerticalIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-23769.0, estimator.getVerticalIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-31062.0, estimator.getVerticalIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-55415.6, estimator.getVerticalIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0),
                22e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-8221.8, estimator.getVerticalIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0),
                40e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38184.5, estimator.getVerticalIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0),
                44e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-44780.8, estimator.getVerticalIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-23859.2, estimator.getVerticalIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0),
                74e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-18297.4, estimator.getVerticalIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0),
                46e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24869.2, estimator.getVerticalIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0),
                69e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-1277.4, estimator.getVerticalIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0),
                33e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-33665.0, estimator.getVerticalIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0),
                47e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-33354.0, estimator.getVerticalIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(54184.7, estimator.getVerticalIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0),
                83e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-53526.9, estimator.getVerticalIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0),
                82e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-25969.5, estimator.getVerticalIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0),
                34e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-45453.8, estimator.getVerticalIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0),
                56e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-12258.0, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(80.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-46308.7, estimator.getVerticalIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-19762.2, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0),
                44e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(15885.6, estimator.getVerticalIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0),
                43e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(5088.1, estimator.getVerticalIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0),
                64e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32429.3, estimator.getVerticalIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-58271.0, estimator.getVerticalIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-24163.1, estimator.getVerticalIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33763.7, estimator.getVerticalIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0),
                19e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-20299.1, estimator.getVerticalIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0),
                86e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-28849.5, estimator.getVerticalIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0),
                37e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55923.4, estimator.getVerticalIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(17967.2, estimator.getVerticalIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(45076.8, estimator.getVerticalIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0),
                54e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-32850.3, estimator.getVerticalIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0),
                57e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-29747.5, estimator.getVerticalIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-61429.1, estimator.getVerticalIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0),
                12e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24097.4, estimator.getVerticalIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0),
                38e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(54735.3, estimator.getVerticalIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0),
                61e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-20600.9, estimator.getVerticalIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(54498.1, estimator.getVerticalIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0),
                8e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-29908.6, estimator.getVerticalIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0),
                77e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-14426.9, estimator.getVerticalIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-14426.9, estimator.getVerticalIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-19023.5, estimator.getVerticalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0),
                34e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30561.3, estimator.getVerticalIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0),
                60e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-55423.4, estimator.getVerticalIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0),
                73e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-21915.4, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0),
                96e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-5425.9, estimator.getVerticalIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0),
                0e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(54055.4, estimator.getVerticalIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0),
                16e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-48550.5, estimator.getVerticalIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0),
                72e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(15301.2, estimator.getVerticalIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0),
                49e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-8905.8, estimator.getVerticalIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0),
                71e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(11616.0, estimator.getVerticalIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0),
                95e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-47331.2, estimator.getVerticalIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0),
                86e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-14566.5, estimator.getVerticalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0),
                30e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(57085.9, estimator.getVerticalIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0),
                75e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-3646.9, estimator.getVerticalIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0),
                21e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-42873.8, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0),
                1e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-19853.0, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0),
                45e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-21372.4, estimator.getVerticalIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0),
                11e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(52945.6, estimator.getVerticalIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0),
                28e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-62459.5, estimator.getVerticalIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0),
                68e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-36415.6, estimator.getVerticalIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0),
                39e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-41759.2, estimator.getVerticalIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0),
                27e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30807.0, estimator.getVerticalIntensity(
                Math.toRadians(34.0), Math.toRadians(0.0),
                11e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-44111.8, estimator.getVerticalIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0),
                72e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55750.6, estimator.getVerticalIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0),
                55e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(26318.3, estimator.getVerticalIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0),
                59e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(43968.0, estimator.getVerticalIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0),
                65e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28857.6, estimator.getVerticalIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0),
                95e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(-26193.2, estimator.getVerticalIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-38293.4, estimator.getVerticalIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55904.6, estimator.getVerticalIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0),
                50e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30329.8, estimator.getVerticalIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0),
                58e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29015.5, estimator.getVerticalIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0),
                57e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-44260.7, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0),
                38e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-53818.3, estimator.getVerticalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0),
                49e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-53373.5, estimator.getVerticalIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0),
                90e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36981.3, estimator.getVerticalIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0),
                41e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(40713.3, estimator.getVerticalIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0),
                19e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(17417.2, estimator.getVerticalIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0),
                31e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-10940.3, estimator.getVerticalIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0),
                93e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-42141.5, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0),
                51e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(25239.1, estimator.getVerticalIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0),
                64e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-41366.0, estimator.getVerticalIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0),
                26e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-24502.7, estimator.getVerticalIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8768.5, estimator.getVerticalIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0),
                18e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55156.1, estimator.getVerticalIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0),
                63e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(8230.6, estimator.getVerticalIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0),
                33e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(-34986.2, estimator.getVerticalIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0),
                77e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
    }

    @Test
    public void testIntensityModel() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(56102.7, estimator.getIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0),
                28e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55704.7, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0),
                48e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(56575.2, estimator.getIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0),
                54e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55348.7, estimator.getIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0),
                65e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(56957.9, estimator.getIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0),
                51e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28845.4, estimator.getIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0),
                39e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38917.2, estimator.getIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0),
                3e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31116.9, estimator.getIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0),
                94e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35847.5, estimator.getIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0),
                66e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32745.6, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(21.0),
                18e3, 2020.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(41630.3, estimator.getIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0),
                6e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(45914.9, estimator.getIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0),
                63e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(42730.3, estimator.getIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0),
                69e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(54230.7, estimator.getIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31257.7, estimator.getIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35945.6, estimator.getIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0),
                8e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(59514.7, estimator.getIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0),
                22e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(30427.4, estimator.getIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0),
                40e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(45042.6, estimator.getIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0),
                44e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(48387.8, estimator.getIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0),
                50e3, 2020.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(27814.7, estimator.getIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0),
                74e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32480.6, estimator.getIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0),
                46e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(42493.9, estimator.getIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0),
                69e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31164.8, estimator.getIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0),
                33e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38392.0, estimator.getIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0),
                47e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47080.5, estimator.getIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47080.5, estimator.getIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0),
                62e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(54268.0, estimator.getIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0),
                83e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55349.8, estimator.getIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0),
                82e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32746.2, estimator.getIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0),
                34e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(48841.2, estimator.getIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0),
                56e3, 2021.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(41182.8, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(80.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(49880.9, estimator.getIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24482.1, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0),
                44e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(39261.7, estimator.getIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0),
                43e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(39639.4, estimator.getIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0),
                64e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(40872.8, estimator.getIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(59702.9, estimator.getIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0),
                12e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34640.0, estimator.getIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0),
                14e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(46961.7, estimator.getIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0),
                19e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38906.8, estimator.getIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0),
                86e3, 2021.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(33563.5, estimator.getIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0),
                37e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(56120.8, estimator.getIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(33995.6, estimator.getIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(49573.8, estimator.getIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0),
                54e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36883.0, estimator.getIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0),
                57e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(37392.8, estimator.getIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0),
                44e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(62510.5, estimator.getIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0),
                12e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35598.4, estimator.getIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0),
                38e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55751.4, estimator.getIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0),
                61e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24389.9, estimator.getIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0),
                67e3, 2022.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(56037.2, estimator.getIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0),
                8e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(34535.8, estimator.getIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0),
                77e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36867.1, estimator.getIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0),
                98e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(31142.6, estimator.getIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0),
                34e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(44944.6, estimator.getIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0),
                60e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(57011.5, estimator.getIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0),
                73e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29781.1, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0),
                96e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(23389.8, estimator.getIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0),
                0e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55792.1, estimator.getIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0),
                16e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(51331.5, estimator.getIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0),
                72e3, 2022.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(33876.1, estimator.getIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0),
                49e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29807.1, estimator.getIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0),
                71e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38595.3, estimator.getIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0),
                95e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(50254.0, estimator.getIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0),
                86e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(22950.5, estimator.getIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0),
                30e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(57149.3, estimator.getIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0),
                75e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(28865.6, estimator.getIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0),
                21e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47183.6, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0),
                1e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(24401.3, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0),
                45e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(25534.7, estimator.getIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0),
                11e3, 2023.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(55072.9, estimator.getIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0),
                28e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(63136.8, estimator.getIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0),
                68e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(41928.8, estimator.getIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0),
                39e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(48965.0, estimator.getIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0),
                27e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(42335.9, estimator.getIntensity(
                Math.toRadians(34.0), Math.toRadians(0.0),
                11e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47450.8, estimator.getIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0),
                72e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55803.4, estimator.getIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0),
                55e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(38551.7, estimator.getIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0),
                59e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(49936.3, estimator.getIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0),
                65e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(41450.1, estimator.getIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0),
                95e3, 2023.5) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(31962.9, estimator.getIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(42393.4, estimator.getIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0),
                95e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55907.8, estimator.getIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0),
                50e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(42241.2, estimator.getIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0),
                58e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(40453.4, estimator.getIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0),
                57e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47942.9, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0),
                38e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(56784.4, estimator.getIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0),
                49e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55839.2, estimator.getIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0),
                90e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(44311.1, estimator.getIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0),
                41e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(46533.4, estimator.getIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0),
                19e3, 2024.0) * TO_NANO,
                INTENSITY_ERROR);


        assertEquals(33326.9, estimator.getIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0),
                31e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35835.1, estimator.getIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0),
                93e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(46035.2, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0),
                51e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(36415.3, estimator.getIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0),
                64e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(45397.3, estimator.getIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0),
                26e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(29231.7, estimator.getIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0),
                66e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(32246.7, estimator.getIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0),
                18e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(55213.8, estimator.getIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0),
                63e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(35043.1, estimator.getIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0),
                33e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
        assertEquals(47296.1, estimator.getIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0),
                77e3, 2024.5) * TO_NANO,
                INTENSITY_ERROR);
    }

    @Test
    public void testConvertTime() {
        final GregorianCalendar cal = new GregorianCalendar(
                2010, Calendar.JANUARY, 0);
        assertEquals(2010.0,
                WMMEarthMagneticFluxDensityEstimator.convertTime(cal),
                0.0);

        // the full day of July 1, 0 hours into 2 July
        final GregorianCalendar cal2 = new GregorianCalendar(
                2012, Calendar.JULY, 1);
        assertTrue(cal2.isLeapYear(2012));
        assertEquals(2012.5,
                WMMEarthMagneticFluxDensityEstimator.convertTime(cal2),
                0.0);

        final GregorianCalendar cal3 = new GregorianCalendar(
                2013, Calendar.APRIL, 13);
        assertFalse(cal3.isLeapYear(2013));
        assertEquals(2013.282,
                WMMEarthMagneticFluxDensityEstimator.convertTime(cal3),
                TIME_ERROR);
    }

    @Test
    public void testGetDeclinationWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double declination1 = estimator.getDeclination(
                latitude, longitude);
        final double declination2 = estimator.getDeclination(
                latitudeAngle, longitudeAngle);

        assertEquals(declination1, declination2, 0.0);

        final Angle result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, result1);
        final Angle result2 = estimator.getDeclinationAsAngle(
                latitude, longitude);
        final Angle result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, result3);
        final Angle result4 = estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle);

        assertEquals(result1.getValue().doubleValue(), declination1,
                0.0);
        assertEquals(result1.getUnit(), AngleUnit.RADIANS);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
    }

    @Test
    public void testGetDeclination() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double declination1 = estimator.getDeclination(
                latitude, longitude, height, year);
        final double declination2 = estimator.getDeclination(
                latitude, longitude, height, calendar);
        final double declination3 = estimator.getDeclination(
                latitude, longitude, height, date);
        final double declination4 = estimator.getDeclination(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double declination5 = estimator.getDeclination(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double declination6 = estimator.getDeclination(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double declination7 = estimator.getDeclination(
                position, year);
        final double declination8 = estimator.getDeclination(
                position, calendar);
        final double declination9 = estimator.getDeclination(
                position, date);

        assertEquals(declination1, declination2, 0.0);
        assertEquals(declination1, declination3, 0.0);
        assertEquals(declination1, declination4, 0.0);
        assertEquals(declination1, declination5, 0.0);
        assertEquals(declination1, declination6, 0.0);
        assertEquals(declination1, declination7, 0.0);
        assertEquals(declination1, declination8, 0.0);
        assertEquals(declination1, declination9, 0.0);

        final Angle result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height,
                year, result1);
        final Angle result2 = estimator.getDeclinationAsAngle(
                latitude, longitude, height, year);
        final Angle result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height,
                calendar, result3);
        final Angle result4 = estimator.getDeclinationAsAngle(
                latitude, longitude, height, calendar);
        final Angle result5 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height,
                date, result5);
        final Angle result6 = estimator.getDeclinationAsAngle(
                latitude, longitude, height, date);
        final Angle result7 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle,
                heightDistance, year, result7);
        final Angle result8 = estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, year);
        final Angle result9 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar, result9);
        final Angle result10 = estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, calendar);
        final Angle result11 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, heightDistance,
                date, result11);
        final Angle result12 = estimator.getDeclinationAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, date);
        final Angle result13 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, year, result13);
        final Angle result14 = estimator.getDeclinationAsAngle(
                position, year);
        final Angle result15 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, calendar, result15);
        final Angle result16 = estimator.getDeclinationAsAngle(
                position, calendar);
        final Angle result17 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, date, result17);
        final Angle result18 = estimator.getDeclinationAsAngle(
                position, date);

        assertEquals(result1.getValue().doubleValue(), declination1,
                0.0);
        assertEquals(result1.getUnit(), AngleUnit.RADIANS);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test
    public void testGetDipWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double dip1 = estimator.getDip(
                latitude, longitude);
        final double dip2 = estimator.getDip(
                latitudeAngle, longitudeAngle);

        assertEquals(dip1, dip2, 0.0);

        final Angle result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, result1);
        final Angle result2 = estimator.getDipAsAngle(
                latitude, longitude);
        final Angle result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle, result3);
        final Angle result4 = estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle);

        assertEquals(result1.getValue().doubleValue(), dip1,
                0.0);
        assertEquals(result1.getUnit(), AngleUnit.RADIANS);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
    }

    @Test
    public void testGetDip() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double dip1 = estimator.getDip(
                latitude, longitude, height, year);
        final double dip2 = estimator.getDip(
                latitude, longitude, height, calendar);
        final double dip3 = estimator.getDip(
                latitude, longitude, height, date);
        final double dip4 = estimator.getDip(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double dip5 = estimator.getDip(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double dip6 = estimator.getDip(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double dip7 = estimator.getDip(
                position, year);
        final double dip8 = estimator.getDip(
                position, calendar);
        final double dip9 = estimator.getDip(
                position, date);

        assertEquals(dip1, dip2, 0.0);
        assertEquals(dip1, dip3, 0.0);
        assertEquals(dip1, dip4, 0.0);
        assertEquals(dip1, dip5, 0.0);
        assertEquals(dip1, dip6, 0.0);
        assertEquals(dip1, dip7, 0.0);
        assertEquals(dip1, dip8, 0.0);
        assertEquals(dip1, dip9, 0.0);

        final Angle result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height,
                year, result1);
        final Angle result2 = estimator.getDipAsAngle(
                latitude, longitude, height, year);
        final Angle result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height,
                calendar, result3);
        final Angle result4 = estimator.getDipAsAngle(
                latitude, longitude, height, calendar);
        final Angle result5 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height,
                date, result5);
        final Angle result6 = estimator.getDipAsAngle(
                latitude, longitude, height, date);
        final Angle result7 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle,
                heightDistance, year, result7);
        final Angle result8 = estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, year);
        final Angle result9 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar, result9);
        final Angle result10 = estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, calendar);
        final Angle result11 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle, heightDistance,
                date, result11);
        final Angle result12 = estimator.getDipAsAngle(
                latitudeAngle, longitudeAngle, heightDistance, date);
        final Angle result13 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, year, result13);
        final Angle result14 = estimator.getDipAsAngle(
                position, year);
        final Angle result15 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, calendar, result15);
        final Angle result16 = estimator.getDipAsAngle(
                position, calendar);
        final Angle result17 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, date, result17);
        final Angle result18 = estimator.getDipAsAngle(
                position, date);

        assertEquals(result1.getValue().doubleValue(), dip1,
                0.0);
        assertEquals(result1.getUnit(), AngleUnit.RADIANS);
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
    }

    @Test
    public void testGetIntensityWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getIntensity(
                latitude, longitude);
        final double intensity2 = estimator.getIntensity(
                latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    public void testGetIntensity() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getIntensity(
                latitude, longitude, height, year);
        final double intensity2 = estimator.getIntensity(
                latitude, longitude, height, calendar);
        final double intensity3 = estimator.getIntensity(
                latitude, longitude, height, date);
        final double intensity4 = estimator.getIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double intensity5 = estimator.getIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double intensity6 = estimator.getIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double intensity7 = estimator.getIntensity(
                position, year);
        final double intensity8 = estimator.getIntensity(
                position, calendar);
        final double intensity9 = estimator.getIntensity(
                position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    public void testGetHorizontalIntensityWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getHorizontalIntensity(
                latitude, longitude);
        final double intensity2 = estimator.getHorizontalIntensity(
                latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    public void testGetHorizontalIntensity() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getHorizontalIntensity(
                latitude, longitude, height, year);
        final double intensity2 = estimator.getHorizontalIntensity(
                latitude, longitude, height, calendar);
        final double intensity3 = estimator.getHorizontalIntensity(
                latitude, longitude, height, date);
        final double intensity4 = estimator.getHorizontalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double intensity5 = estimator.getHorizontalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double intensity6 = estimator.getHorizontalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double intensity7 = estimator.getHorizontalIntensity(
                position, year);
        final double intensity8 = estimator.getHorizontalIntensity(
                position, calendar);
        final double intensity9 = estimator.getHorizontalIntensity(
                position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    public void testGetVerticalIntensityWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getVerticalIntensity(
                latitude, longitude);
        final double intensity2 = estimator.getVerticalIntensity(
                latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    public void testGetVerticalIntensity() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getVerticalIntensity(
                latitude, longitude, height, year);
        final double intensity2 = estimator.getVerticalIntensity(
                latitude, longitude, height, calendar);
        final double intensity3 = estimator.getVerticalIntensity(
                latitude, longitude, height, date);
        final double intensity4 = estimator.getVerticalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double intensity5 = estimator.getVerticalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double intensity6 = estimator.getVerticalIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double intensity7 = estimator.getVerticalIntensity(
                position, year);
        final double intensity8 = estimator.getVerticalIntensity(
                position, calendar);
        final double intensity9 = estimator.getVerticalIntensity(
                position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    public void testGetNorthIntensityWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getNorthIntensity(
                latitude, longitude);
        final double intensity2 = estimator.getNorthIntensity(
                latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    public void testGetNorthIntensity() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getNorthIntensity(
                latitude, longitude, height, year);
        final double intensity2 = estimator.getNorthIntensity(
                latitude, longitude, height, calendar);
        final double intensity3 = estimator.getNorthIntensity(
                latitude, longitude, height, date);
        final double intensity4 = estimator.getNorthIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double intensity5 = estimator.getNorthIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double intensity6 = estimator.getNorthIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double intensity7 = estimator.getNorthIntensity(
                position, year);
        final double intensity8 = estimator.getNorthIntensity(
                position, calendar);
        final double intensity9 = estimator.getNorthIntensity(
                position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    public void testGetEastIntensityWithDefaultTimeAndHeight()
            throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getEastIntensity(
                latitude, longitude);
        final double intensity2 = estimator.getEastIntensity(
                latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    public void testGetEastIntensity() throws IOException {
        final NEDPosition position = createPosition();
        final double latitude = position.getLatitude();
        final double longitude = position.getLongitude();
        final double height = position.getHeight();
        final Angle latitudeAngle = new Angle(latitude,
                AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude,
                AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height,
                DistanceUnit.METER);

        final long timestamp = createTimestamp();
        final Date date = createDate(timestamp);
        final GregorianCalendar calendar = createCalendar(timestamp);
        final double year = createYear(calendar);

        final WMMEarthMagneticFluxDensityEstimator estimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double intensity1 = estimator.getEastIntensity(
                latitude, longitude, height, year);
        final double intensity2 = estimator.getEastIntensity(
                latitude, longitude, height, calendar);
        final double intensity3 = estimator.getEastIntensity(
                latitude, longitude, height, date);
        final double intensity4 = estimator.getEastIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                year);
        final double intensity5 = estimator.getEastIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final double intensity6 = estimator.getEastIntensity(
                latitudeAngle, longitudeAngle, heightDistance,
                date);
        final double intensity7 = estimator.getEastIntensity(
                position, year);
        final double intensity8 = estimator.getEastIntensity(
                position, calendar);
        final double intensity9 = estimator.getEastIntensity(
                position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    public static double createYear(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimator
                .convertTime(calendar);
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

    private static long createTimestamp() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }

    private static Date createDate(final long timestamp) {
        return new Date(timestamp);
    }

    private static GregorianCalendar createCalendar(final long timestamp) {
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        return calendar;
    }
}
