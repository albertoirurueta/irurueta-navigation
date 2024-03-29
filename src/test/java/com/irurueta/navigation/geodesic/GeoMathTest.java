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
package com.irurueta.navigation.geodesic;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.numerical.polynomials.Polynomial;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GeoMathTest {

    private static final double MIN_VALUE = 1.0;
    private static final double MAX_VALUE = 25.0;

    private static final double MIN_DEGREES = 0.0;
    private static final double MAX_DEGREES = 360.0;

    private static final double ABSOLUTE_ERROR = 1e-9;

    @Test
    public void testSq() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.sq(x), x * x, 0.0);
    }

    @Test
    public void testHypot() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.hypot(x, y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test with negative values
        assertEquals(GeoMath.hypot(-x, -y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(0.0, GeoMath.hypot(0.0, 0.0), 0.0);
    }

    @Test
    public void testLog1p() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.log1p(x), Math.log1p(x), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(0.0, GeoMath.log1p(0.0), 0.0);
    }

    @Test
    public void testAtanh() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        double y = Math.abs(x);
        y = Math.log1p(2 * y / (1 - y)) / 2;
        assertEquals(y, GeoMath.atanh(x), 0.0);

        //test for negative value
        assertEquals(-y, GeoMath.atanh(-x), 0.0);
    }

    @Test
    public void testCopysign() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(x, GeoMath.copysign(x, y), 0.0);
        assertEquals(-x, GeoMath.copysign(x, -y), 0.0);
    }

    @Test
    public void testCbrt() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x3 = x * x * x;

        assertEquals(x, GeoMath.cbrt(x3), ABSOLUTE_ERROR);
    }

    @Test
    public void testNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angleDegrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        final double angleRadians = Math.toRadians(angleDegrees);

        final double sinX = Math.sin(angleRadians);
        final double cosX = Math.cos(angleRadians);
        final double r = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Pair p = GeoMath.norm(r * sinX, r * cosX);

        assertEquals(sinX, p.getFirst(), ABSOLUTE_ERROR);
        assertEquals(cosX, p.getSecond(), ABSOLUTE_ERROR);

        //force IllegalArgumentException
        try {
            GeoMath.norm(0.0, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSum() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double u = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double v = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Pair p = GeoMath.sum(u, v);

        assertEquals(u + v, p.getFirst(), ABSOLUTE_ERROR);
    }

    @Test
    public void testPolyval() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] p = new double[3];
        randomizer.fill(p, MIN_VALUE, MAX_VALUE);

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Polynomial polynomial = new Polynomial(p);
        assertEquals(polynomial.evaluate(x),
                GeoMath.polyval(p.length - 1, ArrayUtils.reverseAndReturnNew(p), 0, x),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testAngRound() {
        assertEquals(0.0, GeoMath.angRound(0.0), 0.0);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(value, GeoMath.angRound(value), 0.0);
        assertEquals(-value, GeoMath.angRound(-value), 0.0);

        value = 1 / 32.0;
        assertEquals(value, GeoMath.angRound(value), ABSOLUTE_ERROR);
    }

    @Test
    public void testAngNormalize() {
        assertEquals(0.0, GeoMath.angNormalize(360.0), ABSOLUTE_ERROR);
        assertEquals(0.0, GeoMath.angNormalize(720.0), ABSOLUTE_ERROR);
        assertEquals(180.0, GeoMath.angNormalize(180.0), ABSOLUTE_ERROR);
        assertEquals(180.0, GeoMath.angNormalize(-180.0), ABSOLUTE_ERROR);
        assertEquals(-90.0, GeoMath.angNormalize(270.0), ABSOLUTE_ERROR);
    }

    @Test
    public void testLatFix() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(Double.NaN, GeoMath.latFix(91.0), 0.0);

        final double value = randomizer.nextDouble(0.0, 90.0);
        assertEquals(value, GeoMath.latFix(value), 0.0);
    }

    @Test
    public void testAngDiff() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Pair p = GeoMath.angDiff(x, y);

        assertEquals(y - x, p.getFirst(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSincosd() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double x = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);

        final Pair p = GeoMath.sincosd(x);

        assertEquals(Math.sin(Math.toRadians(x)), p.getFirst(), ABSOLUTE_ERROR);
        assertEquals(Math.cos(Math.toRadians(x)), p.getSecond(), ABSOLUTE_ERROR);
    }

    @Test
    public void testAtan2d() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(GeoMath.atan2d(y, x), GeoMath.angNormalize(Math.toDegrees(Math.atan2(y, x))),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testIsFinite() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertTrue(GeoMath.isFinite(randomizer.nextDouble(MIN_VALUE, MAX_VALUE)));
        assertFalse(GeoMath.isFinite(Double.NaN));
        assertFalse(GeoMath.isFinite(Double.NEGATIVE_INFINITY));
        assertFalse(GeoMath.isFinite(Double.POSITIVE_INFINITY));
    }
}
