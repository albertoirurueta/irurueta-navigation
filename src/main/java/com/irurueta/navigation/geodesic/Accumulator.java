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

/**
 * An accumulator for sums.
 * This allows many double precision numbers to be added together with twice the normal
 * precision. Thus, the effective precision of the sum is 106 bits or about 32 decimal places.
 * The implementation follows J. R. Shewchuk,
 * <a href="https://doi.org/10.1007/PL00009321">Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
 * Predicates</a>, Discrete &amp; Computational Geometry 18(3) 305&ndash;363 (1997).
 * In the documentation of the member functions, <i>sum</i> stands for the value currently held in the
 * accumulator.
 */
public class Accumulator {

    /**
     * s + t accumulators for the sum.
     */
    private double s;

    /**
     * s + t accumulators for the sum.
     */
    private double t;

    /**
     * Constructor from a double.
     *
     * @param y set <i>sum</i> = <i>y</i>.
     */
    public Accumulator(final double y) {
        s = y;
        t = 0;
    }

    /**
     * Constructor from another Accumulator.
     *
     * @param a set <i>sum</i> = <i>a</i>.
     */
    public Accumulator(final Accumulator a) {
        s = a.s;
        t = a.t;
    }

    /**
     * Sets the value to a double.
     *
     * @param y set <i>sum</i> = <i>y</i>.
     */
    public void set(final double y) {
        s = y;
        t = 0;
    }

    /**
     * Returns the value held in the accumulator.
     *
     * @return <i>sum</i>.
     */
    public double getSum() {
        return s;
    }

    /**
     * Returns the result of adding a number to <i>sum</i> (but don't change <i>sum</i>).
     *
     * @param y the number to be added to the sum.
     * @return <i>sum</i> + <i>y</i>.
     */
    public double sum(final double y) {
        final var a = new Accumulator(this);
        a.add(y);
        return a.s;
    }

    /**
     * Add a number to the accumulator.
     *
     * @param y set <i>sum</i> += <i>y</i>.
     */
    public void add(double y) {
        // Here's Shewchuk's solution...

        // hold exact sum as [s, t, u]
        double u;

        // accumulate starting at least significant end
        var r = GeoMath.sum(y, t);
        y = r.getFirst();
        u = r.getSecond();

        r = GeoMath.sum(y, s);
        s = r.getFirst();
        t = r.getSecond();

        // Start is mS, mT decreasing and non-adjacent. Sum is now (s + t + u) exactly with s, t, u
        // non-adjacent and in decreasing order (except for possible zeros). The following code tries
        // to normalize the result.
        // Ideally, we want mS = round(s + t + u) and mU = round(s + t + u - mS). The following does an
        // approximate job (and maintains the decreasing non-adjacent property). Here are two "failures"
        // using 3-bit floats:

        // Case 1: mS is not equal to round(s + t + u) -- off by 1 ulp
        // [12, -1] - 8 -> [4, 0, -1] -> [4, -1] = 3 should be [3, 0] = 3

        // Case 2: mS + mT is not as close to s + t + u as it should be
        // [64, 5] + 4 -> [64, 8, 1] -> [64, 8] = 72 (off by 1)
        //               should be [80, -7] = 73 (exact)

        // "Fixing" these problems is probably not worth the expense. The representation inevitably
        // leads to small errors in the accumulated values.
        // The additional errors illustrated here amount to 1 ulp of the less significant word during
        // each addition to the Accumulator and an additional possible error of 1 ulp in the reported sum.

        // Incidentally, the "ideal" representation described above is not canonical, because
        // mS = round(mS + mT) may not be true. For example with 3-bit floats:

        // [128, 16] + 1 -> [160, -16] -- 160 = round(145).
        // But [160, 0] - 16 -> [128, 16] -- 128 = round(144).

        if (s == 0) {
            // this implies t == 0, so result is u.
            s = u;
        } else {
            // otherwise just accumulate u to t.
            t += u;
        }
    }

    /**
     * Negate an accumulator.
     * Set <i>sum</i> = &minus;<i>sum</i>.
     */
    public void negate() {
        s = -s;
        t = -t;
    }
}
