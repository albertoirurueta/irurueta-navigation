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
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class GeoMathTest {

    private static final double MIN_VALUE = 1.0;
    private static final double MAX_VALUE = 25.0;

    private static final double MIN_DEGREES = 0.0;
    private static final double MAX_DEGREES = 360.0;

    private static final double ABSOLUTE_ERROR = 1e-9;

    public GeoMathTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testSq() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.sq(x), x * x, 0.0);
    }

    @Test
    public void testHypot() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.hypot(x, y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test with negative values
        assertEquals(GeoMath.hypot(-x, -y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(GeoMath.hypot(0.0, 0.0), 0.0, 0.0);
    }

    @Test
    public void testLog1p() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.log1p(x), Math.log1p(x), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(GeoMath.log1p(0.0), 0.0, 0.0);
    }

    @Test
    public void testAtanh() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        double y = Math.abs(x);
        y = Math.log1p(2 * y / (1 - y)) / 2;
        assertEquals(GeoMath.atanh(x), y, 0.0);

        //test for negative value
        assertEquals(GeoMath.atanh(-x), -y, 0.0);
    }

    @Test
    public void testCopysign() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(GeoMath.copysign(x, y), x, 0.0);
        assertEquals(GeoMath.copysign(x, -y), -x, 0.0);
    }

    @Test
    public void testCbrt() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double x3 = x * x * x;

        assertEquals(GeoMath.cbrt(x3), x, ABSOLUTE_ERROR);
    }

    @Test
    public void testNorm() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double angleDegrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        double angleRadians = Math.toRadians(angleDegrees);

        double sinX = Math.sin(angleRadians);
        double cosX = Math.cos(angleRadians);
        double r = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        Pair p = GeoMath.norm(r * sinX, r * cosX);

        assertEquals(p.getFirst(), sinX, ABSOLUTE_ERROR);
        assertEquals(p.getSecond(), cosX, ABSOLUTE_ERROR);

        //force IllegalArgumentException
        try {
            GeoMath.norm(0.0, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testSum() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double u = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double v = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        Pair p = GeoMath.sum(u, v);

        assertEquals(p.getFirst(), u + v, ABSOLUTE_ERROR);
    }

    @Test
    public void testPolyval() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] p = new double[3];
        randomizer.fill(p, MIN_VALUE, MAX_VALUE);

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        Polynomial polynomial = new Polynomial(p);
        assertEquals(polynomial.evaluate(x),
                GeoMath.polyval(p.length - 1, ArrayUtils.reverseAndReturnNew(p), 0, x), ABSOLUTE_ERROR);
    }

    @Test
    public void testAngRound() {
        assertEquals(GeoMath.angRound(0.0), 0.0, 0.0);

        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.angRound(value), value, 0.0);
        assertEquals(GeoMath.angRound(-value), -value, 0.0);

        value = 1 / 32.0;
        assertEquals(GeoMath.angRound(value), value, ABSOLUTE_ERROR);
    }

    @Test
    public void testAngNormalize() {
        assertEquals(GeoMath.angNormalize(360.0), 0.0, ABSOLUTE_ERROR);
        assertEquals(GeoMath.angNormalize(720.0), 0.0, ABSOLUTE_ERROR);
        assertEquals(GeoMath.angNormalize(180.0), 180.0, ABSOLUTE_ERROR);
        assertEquals(GeoMath.angNormalize(-180.0), 180.0, ABSOLUTE_ERROR);
        assertEquals(GeoMath.angNormalize(270.0), -90.0, ABSOLUTE_ERROR);
    }

    @Test
    public void testLatFix() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(GeoMath.latFix(91.0), Double.NaN, 0.0);

        double value = randomizer.nextDouble(0.0, 90.0);
        assertEquals(GeoMath.latFix(value), value, 0.0);
    }

    @Test
    public void testAngDiff() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        Pair p = GeoMath.angDiff(x, y);

        assertEquals(p.getFirst(), y - x, ABSOLUTE_ERROR);
    }

    @Test
    public void testSincosd() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double x = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);

        Pair p = GeoMath.sincosd(x);

        assertEquals(p.getFirst(), Math.sin(Math.toRadians(x)), ABSOLUTE_ERROR);
        assertEquals(p.getSecond(), Math.cos(Math.toRadians(x)), ABSOLUTE_ERROR);
    }

    @Test
    public void testAtan2d() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(GeoMath.angNormalize(Math.toDegrees(Math.atan2(y, x))), GeoMath.atan2d(y, x), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsFinite() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertTrue(GeoMath.isFinite(randomizer.nextDouble(MIN_VALUE, MAX_VALUE)));
        assertFalse(GeoMath.isFinite(Double.NaN));
        assertFalse(GeoMath.isFinite(Double.NEGATIVE_INFINITY));
        assertFalse(GeoMath.isFinite(Double.POSITIVE_INFINITY));
    }
}
