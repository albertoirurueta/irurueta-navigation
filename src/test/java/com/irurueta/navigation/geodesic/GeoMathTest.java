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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class GeoMathTest {

    private static final double MIN_VALUE = 1.0;
    private static final double MAX_VALUE = 25.0;

    private static final double MIN_DEGREES = 0.0;
    private static final double MAX_DEGREES = 360.0;

    private static final double ABSOLUTE_ERROR = 1e-9;

    @Test
    void testSq() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.sq(x), x * x, 0.0);
    }

    @Test
    void testHypot() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.hypot(x, y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test with negative values
        assertEquals(GeoMath.hypot(-x, -y), Math.hypot(x, y), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(0.0, GeoMath.hypot(0.0, 0.0), 0.0);
    }

    @Test
    void testLog1p() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(GeoMath.log1p(x), Math.log1p(x), ABSOLUTE_ERROR);

        //test for zero
        assertEquals(0.0, GeoMath.log1p(0.0), 0.0);
    }

    @Test
    void testAtanh() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        var y = Math.abs(x);
        y = Math.log1p(2 * y / (1 - y)) / 2;
        assertEquals(y, GeoMath.atanh(x), 0.0);

        //test for negative value
        assertEquals(-y, GeoMath.atanh(-x), 0.0);
    }

    @Test
    void testCopysign() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(x, GeoMath.copysign(x, y), 0.0);
        assertEquals(-x, GeoMath.copysign(x, -y), 0.0);
    }

    @Test
    void testCbrt() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x3 = x * x * x;

        assertEquals(x, GeoMath.cbrt(x3), ABSOLUTE_ERROR);
    }

    @Test
    void testNorm() {
        final var randomizer = new UniformRandomizer();
        final var angleDegrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        final var angleRadians = Math.toRadians(angleDegrees);

        final var sinX = Math.sin(angleRadians);
        final var cosX = Math.cos(angleRadians);
        final var r = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var p = GeoMath.norm(r * sinX, r * cosX);

        assertEquals(sinX, p.getFirst(), ABSOLUTE_ERROR);
        assertEquals(cosX, p.getSecond(), ABSOLUTE_ERROR);

        //force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> GeoMath.norm(0.0, 0.0));
    }

    @Test
    void testSum() {
        final var randomizer = new UniformRandomizer();
        final var u = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var v = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var p = GeoMath.sum(u, v);

        assertEquals(u + v, p.getFirst(), ABSOLUTE_ERROR);
    }

    @Test
    void testPolyval() {
        final var randomizer = new UniformRandomizer();
        final var p = new double[3];
        randomizer.fill(p, MIN_VALUE, MAX_VALUE);

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var polynomial = new Polynomial(p);
        assertEquals(polynomial.evaluate(x),
                GeoMath.polyval(p.length - 1, ArrayUtils.reverseAndReturnNew(p), 0, x),
                ABSOLUTE_ERROR);
    }

    @Test
    void testAngRound() {
        assertEquals(0.0, GeoMath.angRound(0.0), 0.0);

        final var randomizer = new UniformRandomizer();
        var value = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        assertEquals(value, GeoMath.angRound(value), 0.0);
        assertEquals(-value, GeoMath.angRound(-value), 0.0);

        value = 1.0 / 32.0;
        assertEquals(value, GeoMath.angRound(value), ABSOLUTE_ERROR);
    }

    @Test
    void testAngNormalize() {
        assertEquals(0.0, GeoMath.angNormalize(360.0), ABSOLUTE_ERROR);
        assertEquals(0.0, GeoMath.angNormalize(720.0), ABSOLUTE_ERROR);
        assertEquals(180.0, GeoMath.angNormalize(180.0), ABSOLUTE_ERROR);
        assertEquals(180.0, GeoMath.angNormalize(-180.0), ABSOLUTE_ERROR);
        assertEquals(-90.0, GeoMath.angNormalize(270.0), ABSOLUTE_ERROR);
    }

    @Test
    void testLatFix() {
        final var randomizer = new UniformRandomizer();

        assertEquals(Double.NaN, GeoMath.latFix(91.0), 0.0);

        final var value = randomizer.nextDouble(0.0, 90.0);
        assertEquals(value, GeoMath.latFix(value), 0.0);
    }

    @Test
    void testAngDiff() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var p = GeoMath.angDiff(x, y);

        assertEquals(y - x, p.getFirst(), ABSOLUTE_ERROR);
    }

    @Test
    void testSincosd() {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);

        final var p = GeoMath.sincosd(x);

        assertEquals(Math.sin(Math.toRadians(x)), p.getFirst(), ABSOLUTE_ERROR);
        assertEquals(Math.cos(Math.toRadians(x)), p.getSecond(), ABSOLUTE_ERROR);
    }

    @Test
    void testAtan2d() {
        final var randomizer = new UniformRandomizer();

        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        assertEquals(GeoMath.atan2d(y, x), GeoMath.angNormalize(Math.toDegrees(Math.atan2(y, x))), ABSOLUTE_ERROR);
    }

    @Test
    void testIsFinite() {
        final var randomizer = new UniformRandomizer();

        assertTrue(GeoMath.isFinite(randomizer.nextDouble(MIN_VALUE, MAX_VALUE)));
        assertFalse(GeoMath.isFinite(Double.NaN));
        assertFalse(GeoMath.isFinite(Double.NEGATIVE_INFINITY));
        assertFalse(GeoMath.isFinite(Double.POSITIVE_INFINITY));
    }
}
