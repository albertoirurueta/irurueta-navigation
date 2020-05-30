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

import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class EarthMagneticFluxDensityEstimatorTest {

    private static final double MIN_MAGNITUDE = 30e-6;
    private static final double MAX_MAGNITUDE = 60e-6;

    private static final double MIN_DIP_DEGREES = -90.0;
    private static final double MAX_DIP_DEGREES = 90.0;

    private static final double MIN_DECLINATION = -180.0;
    private static final double MAX_DECLINATION = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testEstimate() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magnitude = randomizer.nextDouble(MIN_MAGNITUDE,
                MAX_MAGNITUDE);
        final double dip = Math.toRadians(
                randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final double declination = Math.toRadians(
                randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final NEDMagneticFluxDensity b1 = new NEDMagneticFluxDensity();
        EarthMagneticFluxDensityEstimator.estimate(magnitude, declination,
                dip, b1);
        final NEDMagneticFluxDensity b2 = EarthMagneticFluxDensityEstimator
                .estimate(magnitude, declination, dip);

        assertEquals(b1.getBn(),
                Math.cos(declination) * Math.cos(dip) * magnitude,
                0.0);
        assertEquals(b1.getBe(),
                Math.sin(declination) * Math.cos(dip) * magnitude,
                0.0);
        assertEquals(b1.getBd(), Math.sin(dip) * magnitude,
                0.0);
        assertEquals(b1, b2);

        final Angle dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final Angle declinationAngle = new Angle(declination,
                AngleUnit.RADIANS);

        final NEDMagneticFluxDensity b3 = new NEDMagneticFluxDensity();
        EarthMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle,
                dipAngle, b3);
        final NEDMagneticFluxDensity b4 = EarthMagneticFluxDensityEstimator
                .estimate(magnitude, declinationAngle, dipAngle);

        assertEquals(b1, b3);
        assertEquals(b1, b4);
    }

    @Test
    public void testDeclination() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magnitude = randomizer.nextDouble(MIN_MAGNITUDE,
                MAX_MAGNITUDE);
        final double dip = Math.toRadians(
                randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final double declination1 = Math.toRadians(
                randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final NEDMagneticFluxDensity b = EarthMagneticFluxDensityEstimator
                .estimate(magnitude, declination1, dip);

        // get declination
        final double declination2 = EarthMagneticFluxDensityEstimator
                .getDeclination(b);
        final Angle declination3 = EarthMagneticFluxDensityEstimator
                .getDeclinationAsAngle(b);
        final Angle declination4 = new Angle(0.0, AngleUnit.DEGREES);
        EarthMagneticFluxDensityEstimator.getDeclinationAsAngle(b,
                declination4);

        assertEquals(declination1, declination2, ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, declination3.getUnit());
        assertEquals(declination2, declination3.getValue().doubleValue(),
                0.0);
        assertEquals(declination3, declination4);
    }

    @Test
    public void testDip() {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double magnitude = randomizer.nextDouble(MIN_MAGNITUDE,
                MAX_MAGNITUDE);
        final double dip1 = Math.toRadians(
                randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final double declination = Math.toRadians(
                randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final NEDMagneticFluxDensity b = EarthMagneticFluxDensityEstimator
                .estimate(magnitude, declination, dip1);

        // get dip
        final double dip2 = EarthMagneticFluxDensityEstimator.getDip(b);
        final Angle dip3 = EarthMagneticFluxDensityEstimator
                .getDipAsAngle(b);
        final Angle dip4 = new Angle(0.0, AngleUnit.DEGREES);
        EarthMagneticFluxDensityEstimator.getDipAsAngle(b, dip4);

        assertEquals(dip1, dip2, ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, dip3.getUnit());
        assertEquals(dip2, dip3.getValue().doubleValue(), 0.0);
        assertEquals(dip3, dip4);
    }
}
