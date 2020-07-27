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
 * Defines mathematical functions and constants.
 * Based on net.sf.geographiclib library.
 */
@SuppressWarnings("WeakerAccess")
public class GeoMath {

    /**
     * Number of binary digits in the fraction of double precision number.
     * This is equivalent to C++'s {@code numeric_limits<double>::digits}.
     */
    public static final int DIGITS = 53;

    /**
     * Equivalent to C++'s {@code numeric_limits<double>::epsilon()}. This is equal to
     * 0.5^(DIGITS - 1).
     */
    public static final double EPSILON = Math.ulp(1.0);

    /**
     * Equivalent to C++'s {@code numeric_limits<double>::min()}. This is equal to 0.5^1022.
     */
    public static final double MIN = Double.MIN_NORMAL;

    /**
     * Constructor.
     * Prevents instantiation.
     */
    private GeoMath() {
    }

    /**
     * Square a number.
     *
     * @param x the argument.
     * @return <i>x</i><sup>2</sup>.
     */
    public static double sq(final double x) {
        return x * x;
    }

    /**
     * The hypotenuse function avoiding underflow and overflow. This is equivalent
     * to {@link Math#hypot(double, double)}.
     *
     * @param x the first argument.
     * @param y the second argument.
     * @return sqrt(<i>x</i><sup>2</sup> + <i>y</i><sup>2</sup>).
     */
    public static double hypot(double x, double y) {
        x = Math.abs(x);
        y = Math.abs(y);

        final double a = Math.max(x, y);
        final double b = Math.min(x, y) / (a != 0 ? a : 1);
        return a * Math.sqrt(1 + b * b);
        //For an alternative method see
        //C. Moler and D. Morrisin (1983) https://doi.org/10.1147/rd.276.0577
        //and A. A. Dubrulle (1983) https://doi.org/10.1147/rd.276.0582
    }

    /**
     * log(1 + <i>x</i>) accurate near <i>x</i> = 0. This is equivalent to {@link Math#log1p(double)}.
     * <p>
     * This is taken from D.Goldberg,
     * <a href="https://doi.org/10.1145/103162.103163">What every computer scientist should
     * know about floating-point arithmetic</a> (1991),
     * Theorem 4. See also, N. J. Higham, Accuracy and Stability of Numerical Algorithms, 2nd Edition
     * (SIAM, 2002), Answer to Problem 1.5, p 528.
     * </p>
     *
     * @param x the argument.
     * @return log(1 + <i>x</i>).
     */
    public static double log1p(final double x) {
        final double y = 1 + x;
        final double z = y - 1;
        //Here's the explanation for this magic: y = 1 + z, exactly, and z approx x, thus log(y)/z
        //(which is nearly constant near z = 0) returns a good approximation to the true log(1 + x)/x.
        //The multiplication of x * (log(y)/z) introduces little additional error.
        return z == 0 ? x : x * Math.log(y) / z;
    }

    /**
     * The inverse hyperbolic tangent function. This is defined in terms of {@link GeoMath#log1p(double)} in order
     * to maintain accuracy near <i>x</i> = 0.
     * In addition, the odd parity of the function is enforced.
     *
     * @param x the argument.
     * @return atanh(<i>x</i>).
     */
    public static double atanh(final double x) {
        double y = Math.abs(x); //Enforce odd parity
        y = Math.log1p(2 * y / (1 - y)) / 2;
        return x < 0 ? -y : y;
    }

    /**
     * Copy the sign. This is equivalent to {@link Math#copySign(double, double)}
     *
     * @param x gives the magnitude of the result.
     * @param y gives the sign of the result.
     * @return value with the magnitude of <i>x</i> and with the sign of <i>y</i>.
     */
    public static double copysign(final double x, final double y) {
        return Math.abs(x) * (y < 0 || y == 0 ? -1 : 1);
    }

    /**
     * The cube root function. This is equivalent to {@link Math#cbrt(double)}.
     *
     * @param x the argument.
     * @return the real cube root of <i>x</i>.
     */
    public static double cbrt(final double x) {
        final double y = Math.pow(Math.abs(x), 1 / 3.0); //Return the real cube root
        return x < 0 ? -y : y;
    }

    /**
     * Normalizes sinus and cosinus.
     *
     * @param sinx sinus of x.
     * @param cosx cosinus of x.
     * @return normalized values.
     * @throws IllegalArgumentException if provided sinus and cosinus values have zero
     *                                  norm.
     */
    public static Pair norm(final double sinx, final double cosx) {
        final double r = hypot(sinx, cosx);
        if (r == 0.0) {
            throw new IllegalArgumentException();
        }

        return new Pair(sinx / r, cosx / r);
    }

    /**
     * The error-free sum of two numbers.
     * See D.E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.
     *
     * @param u the first number in the sum.
     * @param v the second number in the sum.
     * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = round(<i>u</i> +
     * <i>v</i>) and <i>t</i> = <i>u</i> + <i>v</i> - <i>s</i>.
     */
    public static Pair sum(final double u, final double v) {
        final double s = u + v;
        double up = s - v;
        double vpp = s - up;
        up -= u;
        vpp -= v;
        final double t = -(up + vpp);
        //u + v = s + t = round(u + v) + t
        return new Pair(s, t);
    }

    /**
     * Evaluate a polynomial.
     * <p>
     * Evaluate <i>y</i> = &sum;<sub><i>n</i>=0..<i>n</i></sub>
     * <i>p</i><sub><i>s</i>+<i>n</i></sub>
     * <i>x</i><sup><i>n</i>&minus;<i>n</i></sup>.
     * Return 0 if <i>n</i> &lt; 0.
     * Return <i>p</i><sub><i>s</i></sub>, if <i>n</i> = 0 (even if <i>x</i> is
     * infinite or a nan). The evaluation uses Horner's method.
     * This is equivalent to {@link com.irurueta.numerical.polynomials.Polynomial#evaluate(double)}.
     *
     * @param n the order of the polynomial.
     * @param p the coefficient array (of size <i>n</i> + <i>s</i> + 1 or more).
     * @param s starting index of the array.
     * @param x the variable.
     * @return the value of the polynomial.
     */
    public static double polyval(int n, final double[] p, int s, final double x) {
        double y = n < 0 ? 0 : p[s++];
        while (--n >= 0) y = y * x + p[s++];
        return y;
    }

    /**
     * Makes the smallest gap in x = 1 / 16 - nextafter(1/16, 0) = 1/2^57 for reals = 0.7 pm on the earth if x is an
     * angle in degrees. (This is about 1000 times more resolution than we get with angles around 90 degrees.). We use
     * this to avoid having to deal with near singular cases when x is non-zero but tiny (e.g. 1.0e-200). This converts
     * -0 to +0; however tiny negative numbers get converted to -0.
     *
     * @param x value to be converted
     * @return rounded value.
     */
    public static double angRound(final double x) {
        final double z = 1 / 16.0;
        if (x == 0) {
            return 0;
        }

        double y = Math.abs(x);
        //The compiler mustn't "simplify" z - (z - y) to y
        y = y < z ? z - (z - y) : y;
        return x < 0 ? -y : y;
    }

    /**
     * Normalizes an angle (restricted input range).
     * The range of <i>x</i> is unrestricted.
     *
     * @param x the angle in degrees.
     * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
     */
    public static double angNormalize(double x) {
        x = x % 360.0;
        if (x <= -180) {
            return x + 360;
        } else {
            return x <= 180 ? x : x - 360;
        }
    }

    /**
     * Normalizes latitude.
     *
     * @param x the angle in degrees.
     * @return x if it is in the range [&minus;90&deg;, 90&deg;], otherwise return NaN.
     */
    public static double latFix(final double x) {
        return Math.abs(x) > 90 ? Double.NaN : x;
    }

    /**
     * The exact difference of two angles reduced to (&minus;180&deg;, 180&deg;].
     * This computes <i>z</i> = <i>y</i> &minus; <i>x</i> exactly, reduced to (&minus;180&deg;, 180&deg;]; and then sets
     * <i>z</i> = <i>d</i> + <i>e</i> where <i>d</i> is the nearest representable number to <i>z</i> and <i>e</i> is the
     * truncation error. If <i>d</i> = &minus;180, then <i>e</i> &gt; 0; If <i>d</i> = 180, then <i>e</i> &le; 0.
     *
     * @param x the first angle in degrees.
     * @param y the second angle in degrees.
     * @return Pair(<i>d</i>, <i>e</i>) with <i>d</i> being the rounded difference and <i>e</i> being the error.
     */
    public static Pair angDiff(final double x, final double y) {
        final double d;
        final double t;

        //noinspection all
        final Pair r = sum(angNormalize(-x), angNormalize(y));
        d = angNormalize(r.getFirst());
        t = r.getSecond();

        return sum(d == 180 && t > 0 ? -180 : d, t);
    }

    /**
     * Evaluate the sine and cosine function with the argument in degrees.
     * The results obey exactly the elementary properties of the trigonometric functions, e.g.
     * sin 9&deg; = cos 81&deg; = &minus; sin 123456789&deg;.
     *
     * @param x in degrees.
     * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = sin(<i>x</i> and <i>c</i> = cos(<i>x</i>).
     */
    public static Pair sincosd(final double x) {
        //In order to minimize round-off errors, this function exactly reduces the argument to the range [-45, 45]
        // before converting it to radians.
        double r;
        final int q;
        r = x % 360.0;
        q = (int) Math.floor(r / 90 + 0.5);
        r -= 90 * q;
        //now abs(r) <= 45
        r = Math.toRadians(r);
        //Possibly could call the gnu extension sincos
        final double s = Math.sin(r);
        final double c = Math.cos(r);
        double sinx;
        double cosx;
        switch (q & 3) {
            case 0:
                sinx = s;
                cosx = c;
                break;
            case 1:
                sinx = c;
                cosx = -s;
                break;
            case 2:
                sinx = -s;
                cosx = -c;
                break;
            default:
                //case 3
                sinx = -c;
                cosx = s;
        }
        if (x != 0) {
            sinx += 0.0;
            cosx += 0.0;
        }
        return new Pair(sinx, cosx);
    }

    /**
     * Evaluate the atan2 function with the result in degrees.
     * The result is in the range (&minus;180&deg; 180&deg;]. N.B.,
     * atan2d(&plusmn;0, &minus;1) = +180&deg;; atan2d(&minus;&epsilon;,&minus;1) = &minus;180&deg;, for &epsilon;
     * positive and tiny; atan2d(&plusmn;0, 1) = +plusmn;0&deg;.
     *
     * @param y the sine of the angle.
     * @param x the cosine of the angle.
     * @return atan2(<i>y</i>, <i>x</i>) in degrees.
     */
    public static double atan2d(double y, double x) {
        //In order to minimize round-off errors, this function rearranges the arguments so that result of atan2 is in
        // the range [-pi/4, pi/4] before converting it to degrees and mapping the result to the correct quadrant.
        int q = 0;
        if (Math.abs(y) > Math.abs(x)) {
            final double t;
            t = x;
            //noinspection all
            x = y;
            y = t;
            q = 2;
        }
        if (x < 0) {
            x = -x;
            ++q;
        }
        //here x >= 0 and x >= abs(y), so angle is in [-pi/4, pi/4]
        double ang = Math.toDegrees(Math.atan2(y, x));
        switch (q) {
            //Note that atan2d(-0.0, 1.0) will return -0. However, we expect that atan2d will not be called with y = -0.
            //If need be, include case 0: ang = 0 + ang; break
            //and handle mpfr as in angRound.
            case 1:
                ang = (y >= 0 ? 180 : -180) - ang;
                break;
            case 2:
                ang = 90 - ang;
                break;
            case 3:
                ang = -90 + ang;
                break;
            default:
                break;
        }
        return ang;
    }

    /**
     * Test for finiteness.
     *
     * @param x the argument.
     * @return true if number is finite, false if NaN or infinite.
     */
    public static boolean isFinite(final double x) {
        return Math.abs(x) <= Double.MAX_VALUE;
    }
}
