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
package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.MultivariateNormalDist;

import java.util.List;

/**
 * 3D position estimator based on located fingerprints containing only RSSI readings and
 * having as well prior knowledge of the location of radio sources associated to those
 * readings.
 * This implementation uses a second-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint using
 * a non-linear solving algorithm.
 * An initial position can be provided as a starting point to solve the position,
 * otherwise the average point of selected nearest fingerprints is used as a starting
 * point.
 */
@SuppressWarnings("WeakerAccess")
public class SecondOrderNonLinearRssiPositionEstimator3D extends
        NonLinearRssiPositionEstimator3D {

    /**
     * Constructor.
     */
    public SecondOrderNonLinearRssiPositionEstimator3D() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public SecondOrderNonLinearRssiPositionEstimator3D(
            RssiPositionEstimatorListener<Point3D> listener) {
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
    public SecondOrderNonLinearRssiPositionEstimator3D(
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
    public SecondOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources,
            RssiPositionEstimatorListener<Point3D> listener) {
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
     * total readings are required among all fingerprints).
     */
    public SecondOrderNonLinearRssiPositionEstimator3D(
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
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public SecondOrderNonLinearRssiPositionEstimator3D(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, Point3D>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<Point3D>> sources, Point3D initialPosition,
            RssiPositionEstimatorListener<Point3D> listener) {
        super(locatedFingerprints, fingerprint, sources, initialPosition, listener);
    }

    /**
     * Gets type of position estimator.
     * @return type of position estimator.
     */
    @Override
    public NonLinearRssiPositionEstimatorType getType() {
        return NonLinearRssiPositionEstimatorType.SECOND_ORDER;
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
        //This method implements received power at point pi = (xi, yi, zi) and its derivatives

        //Pr(pi) = Pr(p1)
        // - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1)
        // - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)
        // - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)
        // - 5*n*((y1 - ya)^2 + (z1 - za)^2 - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)^2
        // - 5*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)^2
        // - 5*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)^2
        // + 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)*(yi - y1)
        // + 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)*(zi - z1)
        // + 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)*(zi - z1)

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

        double diffX1a2 = diffX1a * diffX1a;
        double diffY1a2 = diffY1a * diffY1a;
        double diffZ1a2 = diffZ1a * diffZ1a;

        double d1a2 = diffX1a2 + diffY1a2 + diffZ1a2;
        double d1a4 = d1a2 * d1a2;

        double value1 = - 10.0 * n * diffX1a / (ln10 * d1a2);
        double value2 = - 10.0 * n * diffY1a / (ln10 * d1a2);
        double value3 = - 10.0 * n * diffZ1a / (ln10 * d1a2);
        double value4 = - 5.0 * n * (-diffX1a2 + diffY1a2 + diffZ1a2) / (ln10 * d1a4);
        double value5 = - 5.0 * n * (diffX1a2 - diffY1a2 + diffZ1a2) / (ln10 * d1a4);
        double value6 = - 5.0 * n * (diffX1a2 + diffY1a2 - diffZ1a2) / (ln10 * d1a4);
        double value7 = 20.0 * n * diffX1a * diffY1a / (ln10 * d1a4);
        double value8 = 20.0 * n * diffY1a * diffZ1a / (ln10 * d1a4);
        double value9 = 20.0 * n * diffX1a * diffZ1a / (ln10 * d1a4);

        double result = pr
                + value1 * diffXi1
                + value2 * diffYi1
                + value3 * diffZi1
                + value4 * diffXi12
                + value5 * diffYi12
                + value6 * diffZi12
                + value7 * diffXi1 * diffYi1
                + value8 * diffYi1 * diffZi1
                + value9 * diffXi1 * diffZi1;

        //derivative respect xi
        //diff(Pr(pi))/diff(xi) = - 10*n*(x1 - xa)/(ln(10)*d1a^2)
        //- 10*n*((y1 - ya)^2 + (z1 - za)^2) - (x1 - xa)^2)/(ln(10)*d1a^4)*(xi - x1)
        //+ 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(yi - y1)
        //+ 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(zi - z1)
        derivatives[0] = value1 + 2.0 * value4 * diffXi1 + value7 * diffYi1 +
                value8 * diffZi1;

        //derivative respect yi
        //diff(Pr(pi))/diff(yi) = - 10*n*(y1 - ya)/(ln(10)*d1a^2)
        //- 10*n*((x1 - xa)^2 - (y1 - ya)^2 + (z1 - za)^2)/(ln(10)*d1a^4)*(yi - y1)
        //+ 20*n*(x1 - xa)*(y1 - ya)/(ln(10)*d1a^4)*(xi - x1)
        //+ 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(zi - z1)
        derivatives[1] = value2 + 2.0 * value5 * diffYi1 + value7 * diffXi1 +
                value8 * diffZi1;

        //derivative respect zi
        //diff(Pr(pi))/diff(zi) = - 10*n*(z1 - za)/(ln(10)*d1a^2)
        //- 10*n*((x1 - xa)^2 + (y1 - ya)^2 - (z1 - za)^2)/(ln(10)*d1a^4)*(zi - z1)
        //+ 20*n*(y1 - ya)*(z1 - za)/(ln(10)*d1a^4)*(yi - y1)
        //+ 20*n*(x1 - xa)*(z1 - za)/(ln(10)*d1a^4)*(xi - x1)
        derivatives[2] = value3 + 2.0 * value6 * diffZi1 + value8 * diffYi1 +
                value9 * diffXi1;

        return result;
    }

    /**
     * Propagates provided variances into RSSI variance of non-located fingerprint
     * reading.
     * @param fingerprintRssi closest located fingerprint reading RSSI expressed in dBm's.
     * @param pathlossExponent path-loss exponent.
     * @param fingerprintPosition position of closest fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition position to be estimated. Usually this is equal to the
     *                          initial position used by a non linear algorithm.
     * @param fingerprintRssiVariance variance of fingerprint RSSI or null if unknown.
     * @param pathlossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null if
     *                                      unknown.
     * @return variance of RSSI measured at non located fingerprint reading.
     */
    @Override
    @SuppressWarnings("Duplicates")
    protected Double propagateVariances(double fingerprintRssi,
            double pathlossExponent, Point3D fingerprintPosition,
            Point3D radioSourcePosition, Point3D estimatedPosition,
            Double fingerprintRssiVariance, Double pathlossExponentVariance,
            Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance) {
        try {
            MultivariateNormalDist dist =
                    Utils.propagateVariancesToRssiVarianceSecondOrderNonLinear3D(
                            fingerprintRssi, pathlossExponent, fingerprintPosition,
                            radioSourcePosition, estimatedPosition, fingerprintRssiVariance,
                            pathlossExponentVariance, fingerprintPositionCovariance,
                            radioSourcePositionCovariance, null);
            if (dist == null) {
                return null;
            }

            Matrix covariance = dist.getCovariance();
            if (covariance == null) {
                return null;
            }

            return covariance.getElementAt(0, 0);

        } catch (IndoorException e) {
            return null;
        }
    }
}
