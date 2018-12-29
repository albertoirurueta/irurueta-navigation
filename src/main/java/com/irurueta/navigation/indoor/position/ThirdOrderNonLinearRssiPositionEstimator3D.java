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
package com.irurueta.navigation.indoor.position;

import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.*;

import java.util.List;

/**
 * 2D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This implementation uses a third-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint using
 * a non-linear solving algorithm.
 * An initial position can be provided as a starting point to solve the position,
 * otherwise the average point of selected nearest fingerprints is used as a starting
 * point.
 */
@SuppressWarnings("WeakerAccess")
public class ThirdOrderNonLinearRssiPositionEstimator3D extends
        NonLinearRssiPositionEstimator3D {

    /**
     * Constructor.
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D(
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 3D position estimation 3 located
     * total readings are required among all fingerprints).
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources) {
        super(locatedFingerprints, fingerprint, sources);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 3D position estimation 3 located
     * total readings are required among all fingerprints).
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources,
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param initialPosition initial position to start the solving algorithm or null.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 3D position estimation 3 located
     *      * total readings are required among all fingerprints).
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources, Point3D initialPosition) {
        super(locatedFingerprints, fingerprint, sources, initialPosition);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param sources located radio sources.
     * @param initialPosition initial position to start the solving algorithm or null.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided non located fingerprint is null,
     * located fingerprints value is null or there are not enough fingerprints or
     * readings within provided fingerprints (for 3D position estimation 3 located
     *      *      * total readings are required among all fingerprints).
     */
    public ThirdOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources, Point3D initialPosition,
            SourcedRssiPositionEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, sources, initialPosition, listener);
    }

    /**
     * Gets type of position estimator.
     * @return type of position estimator.
     */
    @Override
    public NonLinearRssiPositionEstimatorType getType() {
        return NonLinearRssiPositionEstimatorType.THIRD_ORDER;
    }

    /**
     * Evaluates a non-linear multi dimension function at provided point using
     * provided parameters and returns its evaluation and derivatives of the
     * function respect the function parameters.
     * @param i number of sample being evaluated.
     * @param point point where function will be evaluated.
     * @param params initial parameters estimation to be tried. These will
     * change as the Levenberg-Marquard algorithm iterates to the best solution.
     * These are used as input parameters along with point to evaluate function.
     * @param derivatives partial derivatives of the function respect to each
     * provided parameter.
     * @return function evaluation at provided point.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected double evaluate(int i, double[] point, double[] params, double[] derivatives) {
        //Demonstration in 3D:
        //--------------------
        //Taylor series expansion can be expressed as:
        //f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + 1/3!*f'''(a)*(x - a)^3 ...

        //where f'(x) is the derivative of f respect x, which can also be expressed as:
        //f'(x) = diff(f(x))/diff(x)

        //and f'(a) is the derivative of f respect x evaluated at a, which can be expressed
        //as f'(a) = diff(f(a))/diff(x)

        //consequently f''(a) is the second derivative respect x evaluated at a, which can
        //be expressed as:
        //f''(x) = diff(f(x))/diff(x^2)

        //and:
        //f''(a) = diff(f(a))/diff(x^2)

        //and finally f'''(a) is the third derivative respect x evaluated at a, which can
        //be expressed as:
        //f'''(x) = diff(f(x))/diff(x^3)

        //and:
        //f'''(a) = diff(f(a))/diff(x^3)

        //Received power expressed in dBm is:
        //k = (c/(4*pi*f))
        //Pr = Pte*k^n / d^n

        //where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
        //Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
        // (typically 2.0), and d is the distance from a point to the location of the radio source.

        //Hence:
        //Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
        //          10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

        //The former 2 terms are constant, and only the last term depends on distance

        //Hence, assuming the constant K = 10*n*log(k) + Pte(dBm), where Pte(dBm) = 10*log(Pte),
        //assuming that transmitted power by the radio source Pte is known (so that K is also known),
        //and assuming that the location of the radio source is known and it is located at pa = (xa, ya)
        //so that d^2 = (x - xa)^2 + (y - ya)^2 then the received power at an unknown point pi = (xi, yi) is:

        //Pr(pi) = Pr(xi,yi) = K - 5*n*log(d^2) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2)

        //Suppose that received power at point p1=(x1,y1) is known on a located fingerprint
        //containing readings Pr(p1).

        //Then, for an unknown point pi=(xi,yi) close to fingerprint 1 located at p1 where we
        //have measured received power Pr(pi), we can get the following third-order Taylor
        //approximation:

        //Pr(pi = (xi,yi)) = Pr(p1) +
        //  diff(Pr(p1))/diff(x)*(xi - x1) +
        //  diff(Pr(p1))/diff(y)*(yi - y1) +
        //  diff(Pr(p1))/diff(z)*(zi - z1) +
        //  1/2*(diff(Pr(p1))/diff(x^2)*(xi - x1)^2 +
        //      diff(Pr(p1))/diff(y^2)*(yi - y1)^2 +
        //      diff(Pr(p1))/diff(z^2)*(zi - z1)^2 +
        //      2*diff(Pr(p1))/diff(x*y)*(xi - x1)*(yi - y1)) +
        //      2*diff(Pr(p1))/diff(y*z)*(yi - y1)*(zi - z1) +
        //      2*diff(Pr(p1))/diff(x*z)*(xi - x1)*(zi - z1)
        //  1/6*(diff(Pr(p1))/diff(x^3)*(xi - x1)^3 +
        //      diff(Pr(p1))/diff(y^3)*(yi - y1)^3 +
        //      diff(Pr(p1))/diff(z^3)*(zi - z1)^3 +
        //      3*diff(Pr(p1))/diff(x^2*y)*(xi - x1)^2*(yi - y1) +
        //      3*diff(Pr(p1))/diff(x^2*z)*(xi - x1)^2*(zi - z1) +
        //      3*diff(Pr(p1))/diff(x*y^2)*(xi - x1)*(yi - y1)^2 +
        //      3*diff(Pr(p1))/diff(x*z^2)*(xi - x1)*(zi - z1)^2 +
        //      3*diff(Pr(p1))/diff(y^2*z)*(yi - y1)^2*(zi - z1) +
        //      3*diff(Pr(p1))/diff(y*z^2)*(yi - y1)*(zi - z1)^2 +
        //      6*diff(Pr(p1))/diff(x*y*z)*(xi - x1)*(yi - y1)*(zi - z1)

        //where the first order derivatives of Pr(p = (x,y)) are:
        //diff(Pr(x,y,z))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa)
        //diff(Pr(x,y,z))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

        //diff(Pr(x,y,z))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya)
        //diff(Pr(x,y,z))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

        //diff(Pr(x,y,z))/diff(z) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za)
        //diff(Pr(x,y,z))/diff(z) = -10*n*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

        //If we evaluate first order derivatives at p1 = (x1,y1), we get:
        //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
        //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
        //diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

        //where square distance from fingerprint 1 to radio source a can be expressed as:
        //d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

        //where both the fingerprint and radio source positions are known, and hence d1a is known.

        //Then first order derivatives can be expressed as:
        //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
        //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
        //diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*d1a^2)

        //To obtain second order derivatives we take into account that:
        //(f(x)/g(x))' = (f'(x)*g(x) - f(x)*g'(x))/g(x)^2

        //hence, second order derivatives of Pr(p = (x,y,z)) are:
        //diff(Pr(x,y,z))/diff(x^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(x - xa))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(x^2) = -10*n*((y - ya)^2 + (z - za)^2 - (x - xa)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //diff(Pr(x,y,z))/diff(y^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (y - ya)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(y^2) = -10*n*((x - xa)^2 - (y - ya)^2 + (z - za)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //diff(Pr(x,y,z))/diff(z^2) = -10*n/ln(10)*(1*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (z - za)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(z^2) = -10*n*((x - xa)^2 + (y - ya)^2 - (z - za)^2)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //diff(Pr(x,y,z))/diff(x*y) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(x*y) = 20*n*(x - xa)*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //diff(Pr(x,y,z))/diff(x*z) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (x - xa)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(x*z) = 20*n*(x - xa)*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //diff(Pr(x,y,z))/diff(y*z) = -10*n/ln(10)*(0*((x - xa)^2 + (y - ya)^2 + (z - za)^2) - (y - ya)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2
        //diff(Pr(x,y,z))/diff(y*z) = 20*n*(y - ya)*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2)

        //If we evaluate second order derivatives at p1 = (x1,y1,z1), we get:
        //diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
        //diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
        //diff(Pr(p1))/diff(z^2) = -10*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
        //diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
        //diff(Pr(p1))/diff(x*z) = 20*n*(x1 - xa)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)
        //diff(Pr(p1))/diff(y*z) = 20*n*(y1 - ya)*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2)

        //and expressing the second order derivatives in terms of distance between
        //fingerprint 1 and radio source a d1a, we get:
        //diff(Pr(p1))/diff(x^2) = -10*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(y^2) = -10*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(z^2) = -10*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(x*y) = 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(x*z) = 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)
        //diff(Pr(p1))/diff(y*z) = 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)

        //Finally, third order derivatives of Pr(p = (x,y,z)) are:
        //diff(Pr(x,y,z))/diff(x^3) = -10*n/ln(10)*(-2*(x - xa)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((y - ya)^2 + (z - za)^2 - (x - xa)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(y^2) = -10*n/ln(10)*(-2*(y - ya)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 - (y - ya)^2 + (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(z^3) = -10*n/ln(10)*(-2*(z - za)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 + (y - ya)^2 - (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(x^2*y) = -10*n/ln(10)*(2*(y - ya)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((y - ya)^2 + (z - za)^2 - (x - xa)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(x^2*z) = -10*n/ln(10)*(2*(z - za)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((y - ya)^2 + (z - za)^2 - (x - xa)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(x*y^2) = -10*n/ln(10)*(2*(x - xa)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 - (y - ya)^2 + (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(x*z^2) = -10*n/ln(10)*(2*(x - xa)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 + (y - ya)^2 - (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(y^2*z) = -10*n/ln(10)*(2*(z - za)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 - (y - ya)^2 + (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(y*z^2) = -10*n/ln(10)*(2*(y - ya)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)^2 - ((x - xa)^2 + (y - ya)^2 - (z - za)^2)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4
        //diff(Pr(x,y,z))/diff(x*y*z) = 20*n/ln(10)*(-(x - xa)*(y - ya)*2*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za))/((x - xa)^2 + (y - ya)^2 + (z - za)^2)^4

        //evaluating at p1 = (x1, y1, z1), we get:
        //diff(Pr(p1))/diff(x^3) = -10*n/ln(10)*(-2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(y^2) = -10*n/ln(10)*(-2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(z^3) = -10*n/ln(10)*(-2*(z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(x^2*y) = -10*n/ln(10)*(2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(x^2*z) = -10*n/ln(10)*(2*(z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(x*y^2) = -10*n/ln(10)*(2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(x*z^2) = -10*n/ln(10)*(2*(x1 - xa)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(x1 - xa))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(y^2*z) = -10*n/ln(10)*(2*(z1 - za)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(y*z^2) = -10*n/ln(10)*(2*(y1 - ya)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^2 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(y1 - ya))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4
        //diff(Pr(p1))/diff(x*y*z) = 20*n/ln(10)*(-(x1 - xa)*(y1 - ya)*2*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)*2*(z1 - za))/((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)^4

        //and substituting the distance between fingerprint and radio source d1a, we get:
        //diff(Pr(p1))/diff(x^3) = -10*n/ln(10)*(-2*(x1 - xa)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8
        //diff(Pr(p1))/diff(y^3) = -10*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8
        //diff(Pr(p1))/diff(z^3) = -10*n/ln(10)*(-2*(z1 - za)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8
        //diff(Pr(p1))/diff(x^2*y) = -10*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8
        //diff(Pr(p1))/diff(x^2*z) = -10*n/ln(10)*(2*(z1 - za)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(z1 - za))/d1a^8
        //diff(Pr(p1))/diff(x*y^2) = -10*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8
        //diff(Pr(p1))/diff(x*z^2) = -10*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8
        //diff(Pr(p1))/diff(y^2*z) = -10*n/ln(10)*(2*(z1 - za)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8
        //diff(Pr(p1))/diff(y*z^2) = -10*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8
        //diff(Pr(p1))/diff(x*y*z) = -80*n/ln(10)*((x1 - xa)*(y1 - ya)*(z1 - za)*d1a^2)/d1a^8


        //Hence, the third order Taylor expansion can be expressed as:
        //Pr(pi = (xi,yi)) = Pr(p1) +
        //  diff(Pr(p1))/diff(x)*(xi - x1) +
        //  diff(Pr(p1))/diff(y)*(yi - y1) +
        //  diff(Pr(p1))/diff(z)*(zi - z1) +
        //  1/2*(diff(Pr(p1))/diff(x^2)*(xi - x1)^2 +
        //      diff(Pr(p1))/diff(y^2)*(yi - y1)^2 +
        //      diff(Pr(p1))/diff(z^2)*(zi - z1)^2 +
        //      2*diff(Pr(p1))/diff(x*y)*(xi - x1)*(yi - y1)) +
        //      2*diff(Pr(p1))/diff(y*z)*(yi - y1)*(zi - z1) +
        //      2*diff(Pr(p1))/diff(x*z)*(xi - x1)*(zi - z1)
        //  1/6*(diff(Pr(p1))/diff(x^3)*(xi - x1)^3 +
        //      diff(Pr(p1))/diff(y^3)*(yi - y1)^3 +
        //      diff(Pr(p1))/diff(z^3)*(zi - z1)^3 +
        //      3*diff(Pr(p1))/diff(x^2*y)*(xi - x1)^2*(yi - y1) +
        //      3*diff(Pr(p1))/diff(x^2*z)*(xi - x1)^2*(zi - z1) +
        //      3*diff(Pr(p1))/diff(x*y^2)*(xi - x1)*(yi - y1)^2 +
        //      3*diff(Pr(p1))/diff(x*z^2)*(xi - x1)*(zi - z1)^2 +
        //      3*diff(Pr(p1))/diff(y^2*z)*(yi - y1)^2*(zi - z1) +
        //      3*diff(Pr(p1))/diff(y*z^2)*(yi - y1)*(zi - z1)^2 +
        //      6*diff(Pr(p1))/diff(x*y*z)*(xi - x1)*(yi - y1)*(zi - z1)


        //Pr(pi = (xi,yi)) = Pr(p1) +
        //  -10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) +
        //  -10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1) +
        //  -10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1) +
        //  -5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2 +
        //  -5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)^2 +
        //  -5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)^2 +
        //  20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1) +
        //  20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1) +
        //  20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1) +
        //  -10/6*n/ln(10)*(-2*(x1 - xa)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)^3 +
        //  -10/6*n/ln(10)*(-2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)^3 +
        //  -10/6*n/ln(10)*(-2*(z1 - za)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8*(zi - z1)^3 +
        //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(y1 - ya))/d1a^8*(xi - x1)^2*(yi - y1) +
        //  -5*n/ln(10)*(2*(z1 - za)*d1a^4 - ((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)*4*d1a^2*(z1 - za))/d1a^8*(xi - x1)^2*(zi - z1) +
        //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(yi - y1)^2 +
        //  -5*n/ln(10)*(2*(x1 - xa)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(x1 - xa))/d1a^8*(xi - x1)*(zi - z1)^2 +
        //  -5*n/ln(10)*(2*(z1 - za)*d1a^4 - ((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)*4*d1a^2*(z1 - za))/d1a^8*(yi - y1)^2*(zi - z1) +
        //  -5*n/ln(10)*(2*(y1 - ya)*d1a^4 - ((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)*4*d1a^2*(y1 - ya))/d1a^8*(yi - y1)*(zi - z1)^2 +
        //  -80*n/ln(10)*((x1 - xa)*(y1 - ya)*(z1 - za)*d1a^2)/d1a^8*(xi - x1)*(yi - y1)*(zi - z1)

        //The equation above can be solved using a non-linear fitter such as Levenberg-Marquardt

        //This method implements received power at point pi = (xi, yi) and its derivatives

        double xi = params[0];
        double yi = params[1];
        double zi = params[2];

        //received power
        double pr = point[0];

        //fingerprint coordinates
        double x1 = point[1];
        double y1 = point[2];
        double z1 = point[3];

        //radio source coordinates
        double xa = point[4];
        double ya = point[5];
        double za = point[6];

        //path loss exponent
        double n = point[7];

        double ln10 = Math.log(10.0);

        double diffXi1 = xi - x1;
        double diffYi1 = yi - y1;
        double diffZi1 = zi - z1;

        double diffX1a = x1 - xa;
        double diffY1a = y1 - ya;
        double diffZ1a = z1 - za;

        double diffXi12 = diffXi1 * diffXi1;
        double diffYi12 = diffYi1 * diffYi1;
        double diffZi12 = diffZi1 * diffZi1;

        double diffXi13 = diffXi12 * diffXi1;
        double diffYi13 = diffYi12 * diffYi1;
        double diffZi13 = diffZi12 * diffZi1;

        double diffX1a2 = diffX1a * diffX1a;
        double diffY1a2 = diffY1a * diffY1a;
        double diffZ1a2 = diffZ1a * diffZ1a;

        double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
        double d1a4 = d1a2 * d1a2;
        double d1a8 = d1a4 * d1a4;

        double value1 = - 10.0 * n * diffX1a / (ln10 * d1a2);
        double value2 = - 10.0 * n * diffY1a / (ln10 * d1a2);
        double value3 = - 10.0 * n * diffZ1a / (ln10 * d1a2);
        double value4 = - 5.0 * n * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4);
        double value5 = - 5.0 * n * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4);
        double value6 = - 5.0 * n * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4);
        double value7 = 20.0 * n * diffX1a * diffY1a / (ln10 * d1a4);
        double value8 = 20.0 * n * diffY1a * diffZ1a / (ln10 * d1a4);
        double value9 = 20.0 * n * diffX1a * diffZ1a / (ln10 * d1a4);
        double value10 = - 10.0 / 6.0 * n / ln10 * (-2.0 * diffX1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
        double value11 = - 10.0 / 6.0 * n / ln10 * (-2.0 * diffY1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
        double value12 = - 10.0 / 6.0 * n / ln10 * (-2.0 * diffZ1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
        double value13 = - 5.0 * n / ln10 * (2.0 * diffY1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
        double value14 = - 5.0 * n / ln10 * (2.0 * diffZ1a * d1a4 - (-diffX1a2 + diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
        double value15 = - 5.0 * n / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
        double value16 = - 5.0 * n / ln10 * (2.0 * diffX1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffX1a) / d1a8;
        double value17 = - 5.0 * n / ln10 * (2.0 * diffZ1a * d1a4 - (diffX1a2 - diffY1a2 + diffZ1a2) * 4.0 * d1a2 * diffZ1a) / d1a8;
        double value18 = - 5.0 * n / ln10 * (2.0 * diffY1a * d1a4 - (diffX1a2 + diffY1a2 - diffZ1a2) * 4.0 * d1a2 * diffY1a) / d1a8;
        double value19 = - 80.0 * n / ln10 * (diffX1a * diffY1a * diffZ1a * d1a2) / d1a8;


        //hence:
        //Pr(pi) = Pr(p1) +
        //  value1*(xi - x1) +
        //  value2*(yi - y1) +
        //  value3*(zi - z1) +
        //  value4*(xi - x1)^2 +
        //  value5*(yi - y1)^2 +
        //  value6*(zi - z1)^2 +
        //  value7*(xi - x1)*(yi - y1) +
        //  value8*(yi - y1)*(zi - z1) +
        //  value9*(xi - x1)*(zi - z1) +
        //  value10*(xi - x1)^3 +
        //  value11*(yi - y1)^3 +
        //  value12*(zi - z1)^3 +
        //  value13*(xi - x1)^2*(yi - y1) +
        //  value14*(xi - x1)^2*(zi - z1) +
        //  value15*(xi - x1)*(yi - y1)^2 +
        //  value16*(xi - x1)*(zi - z1)^2 +
        //  value17*(yi - y1)^2*(zi - z1) +
        //  value18*(yi - y1)*(zi - z1)^2 +
        //  value19*(xi - x1)*(yi - y1)*(zi - z1)

        double result = pr
                + value1 * diffXi1
                + value2 * diffYi1
                + value3 * diffZi1
                + value4 * diffXi12
                + value5 * diffYi12
                + value6 * diffZi12
                + value7 * diffXi1 * diffYi1
                + value8 * diffYi1 * diffZi1
                + value9 * diffXi1 * diffZi1
                + value10 * diffXi13
                + value11 * diffYi13
                + value12 * diffZi13
                + value13 * diffXi12 * diffYi1
                + value14 * diffXi12 * diffZi1
                + value15 * diffXi1 * diffYi12
                + value16 * diffXi1 * diffZi12
                + value17 * diffYi12 * diffZi1
                + value18 * diffYi1 * diffZi12
                + value19 * diffXi1 * diffYi1 * diffZi1;

        //derivative respect xi

        //diff(Pr(pi))/diff(xi) = value1 +
        //  2*value4*(xi - x1) +
        //  value7*(yi - y1) +
        //  value9*(zi - z1) +
        //  3*value10*(xi - x1)^2 +
        //  2*value13*(xi - x1)*(yi - y1) +
        //  2*value14*(xi - x1)*(zi - z1) +
        //  value15*(yi - y1)^2 +
        //  value16*(zi - z1)^2 +
        //  value19*(yi - y1)*(zi - z1)

        derivatives[0] = value1 + 2.0 * value4 * diffXi1 + value7 * diffYi1 +
                value9 * diffZi1 + 3.0 * value10 * diffXi12 +
                2.0 * value13 * diffXi1 * diffYi1 +
                2.0 * value14 * diffXi1 * diffZi1 +
                value15 * diffYi12 + value16 * diffZi12 +
                value19 * diffYi1 * diffZi1;

        //derivative respect yi

        //diff(Pr(pi))/diff(yi) = value2 +
        //  2*value5*(yi - y1) +
        //  value7*(xi - x1) +
        //  value8*(zi - z1) +
        //  3*value11*(yi - y1)^2 +
        //  value13*(xi - x1)^2 +
        //  2*value15*(xi - x1)*(yi - y1) +
        //  2*value17*(yi - y1)*(zi - z1) +
        //  value18*(zi - z1)^2 +
        //  value19*(xi - x1)*(zi - z1)

        derivatives[1] = value2 + 2.0 * value5 * diffYi1 + value7 * diffXi1 +
                value8 * diffZi1 + 3.0 * value11 * diffYi12 +
                value13 * diffXi12 + 2.0 * value15 * diffXi1 * diffYi1 +
                2.0 * value17 * diffYi1 * diffZi1 +
                value18 * diffZi12 + value19 * diffXi1 * diffZi1;

        //derivative respect zi

        //diff(Pr(pi))/diff(zi) = value3 +
        //  2*value6*(zi - z1) +
        //  value8*(yi - y1) +
        //  value9*(xi - x1) +
        //  3*value12*(zi - z1)^2 +
        //  value14*(xi - x1)^2 +
        //  2*value16*(xi - x1)*(zi - z1) +
        //  value17*(yi - y1)^2 +
        //  2*value18*(yi - y1)*(zi - z1) +
        //  value19*(xi - x1)*(yi - y1)

        derivatives[2] = value3 + 2.0 * value6 * diffZi1 + value8 * diffYi1 +
                value9 * diffXi1 + 3.0 * value12 * diffZi12 +
                value14 * diffXi12 + 2.0 * value16 * diffXi1 * diffZi1 +
                value17 * diffYi12 + 2.0 * value18 * diffYi1 * diffZi1 +
                value19 * diffXi1 * diffYi1;

        return result;
    }
}

