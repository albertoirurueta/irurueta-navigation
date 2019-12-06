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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;

import java.util.Collection;
import java.util.List;

/**
 * Base class for position estimators based on located fingerprints containing only
 * RSSI readings and having as well prior knowledge of the location of radio sources
 * associated to those readings.
 * This implementation uses a first-order Taylor approximation over provided located
 * fingerprints to determine an approximate position for a non-located fingerprint and
 * solves the problem in a linear way.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class LinearFingerprintPositionEstimator<P extends Point<P>> extends
        FingerprintPositionEstimator<P> {

    /**
     * Constructor.
     */
    public LinearFingerprintPositionEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public LinearFingerprintPositionEstimator(
            FingerprintPositionEstimatorListener<P> listener) {
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
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public LinearFingerprintPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
                                              RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
                                              List<? extends RadioSourceLocated<P>> sources) {
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
     * readings within provided fingerprints (for 2D position estimation at least 2
     * located total readings are required among all fingerprints, for example 2
     * readings are required in a single fingerprint, or at least 2 fingerprints at
     * different locations containing a single reading are required. For 3D position
     * estimation 3 located total readings are required among all fingerprints).
     */
    public LinearFingerprintPositionEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
                                              RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
                                              List<? extends RadioSourceLocated<P>> sources,
                                              FingerprintPositionEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, sources, listener);
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws FingerprintEstimationException if estimation fails for some other reason.
     */
    @Override
    @SuppressWarnings("Duplicates")
    public void estimate() throws LockedException, NotReadyException,
            FingerprintEstimationException {

        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            RadioSourceNoMeanKNearestFinder<P, RadioSource> noMeanfinder = null;
            RadioSourceKNearestFinder<P, RadioSource> finder = null;
            if (mUseNoMeanNearestFingerprintFinder) {
                //noinspection unchecked
                noMeanfinder = new RadioSourceNoMeanKNearestFinder<>(
                        (Collection<? extends RssiFingerprintLocated<RadioSource,
                                RssiReading<RadioSource>, P>>)mLocatedFingerprints);
            } else {
                //noinspection unchecked
                finder = new RadioSourceKNearestFinder<>(
                        (Collection<? extends RssiFingerprintLocated<RadioSource,
                                RssiReading<RadioSource>, P>>)mLocatedFingerprints);
            }

            mEstimatedPositionCoordinates = null;
            mNearestFingerprints = null;

            int dims = getNumberOfDimensions();
            int max = mMaxNearestFingerprints < 0 ?
                    mLocatedFingerprints.size() :
                    Math.min(mMaxNearestFingerprints, mLocatedFingerprints.size());
            for (int k = mMinNearestFingerprints; k <= max; k++) {

                if (noMeanfinder != null) {
                    //noinspection unchecked
                    mNearestFingerprints = noMeanfinder.findKNearestTo(
                            (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) mFingerprint, k);
                } else {
                    //noinspection unchecked
                    mNearestFingerprints = finder.findKNearestTo(
                            (RssiFingerprint<RadioSource, RssiReading<RadioSource>>) mFingerprint, k);
                }

                //Demonstration in 2D:
                //--------------------
                //Taylor series expansion can be expressed as:
                //f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + ...

                //where f'(x) is the derivative of f respect x, which can also be expressed as:
                //f'(x) = diff(f(x))/diff(x)

                //and f'(a) is the derivative of f respect x evaluated at a, which can be expressed
                //as f'(a) = diff(f(a))/diff(x)

                //consequently f''(a) is the second derivative respect x evaluated at a, which can
                //be expressed as:
                //f''(x) = diff(f(x))/diff(x^2)

                //and:
                //f''(a) = diff(f(a))/diff(x^2)

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
                //have measured received power Pr(pi), we can get the following second-order Taylor
                //approximation:

                //Pr(pi) ~ Pr(p1) + JPtr(p1)*(pi - p1) + 1/2*(pi - p1)^T*HPr(p1)*(pi - p1) + ...

                //where JPr(p1) is the Jacobian of Pr evaluated at p1. Since Pr is a multivariate function
                //with scalar result, the Jacobian has size 1x2 and is equal to the gradient.
                //HPtr(p1) is the Hessian matrix evaluated at p1, which is a symmetric matrix of size 2x2,
                //and (pi-p1)^T is the transposed vector of (pi-p1)

                //Hence, the Jacobian at any point p=(x,y) is equal to:
                //JPr(p = (x,y)) = [diff(Pr(x,y))/diff(x)   diff(Pr(x,y))/diff(y)]

                //And the Heassian matrix is equal to
                //HPr(p = (x,y)) =  [diff(Pr(x,y))/diff(x^2)    diff(Pr(x,y))/diff(x*y)]
                //                  [diff(Pr(x,y))/diff(x*y)    diff(Pr(x,y))/diff(y^2)]

                //Simplifying Taylor expansion to first-order terms to get a linear (but less accurate)
                //solution, we get:
                //Pr(pi) = Pr(p1) + JPtr(p1)*(pi - p1)
                //Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(xi - x1) + diff(Pr(p1))/diff(y)*(yi - y1)

                //where the first order derivatives of Pr(p = (x,y)) are:
                //diff(Pr(x,y))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(x - xa)
                //diff(Pr(x,y))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2))

                //diff(Pr(x,y))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2)*2*(y - ya)
                //diff(Pr(x,y))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2))

                //If we evaluate derivatives at p1 = (x1,y1), we get:
                //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))
                //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2))

                //where square distance from fingerprint 1 to radio source a can be expressed as:
                //d1a^2 = (x1 - xa)^2 + (y1 - ya)^2

                //where both the fingerprint and radio source positions are known, and hence d1a is known.

                //Then derivatives can be expressed as:
                //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)

                //Hence, first order Taylor expansion can be expressed as:
                //Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(xi - x1) + diff(Pr(p1))/diff(y)*(yi - y1)
                //Pr(pi) = Pr(p1) - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1)

                //where the only unknowns are xi,yi.

                //Reordering expression above, we get:
                //10*n*(x1 - xa)/(ln(10)*d1a^2)*xi + 10*n*(y1 - ya)/(ln(10)*d1a^2)*yi = Pr(p1) - Pr(pi) + 10*n*(x1 - xa)/(ln(10)*d1a^2)*x1 + 10*n*(y1 - ya)/(ln(10)*d1a^2)*y1

                //Which can be expressed in matrix form as:
                //[10*n*(x1 - xa)/(ln(10)*d1a^2)    10*n*(y1 - ya)/(ln(10)*d1a^2)]  [xi] = [Pr(p1) - Pr(pi) + 10*n*(x1 - xa)/(ln(10)*d1a^2)*x1 + 10*n*(y1 - ya)/(ln(10)*d1a^2)*y1]
                //                                                                  [yi]

                //which is the equation obtained for fingerprint 1 and radio source a.

                //Having at least 2 linear independent equations for different fingerprints or radio sources allows
                //solving unknown position pi = (xi,yi)
                //Hence we could have either 2 or more located fingerprints with 1 radio sources, 2 or more radio
                //sources on a single located fingerprint, or any combination resulting in enough equations


                //Demonstration in 3D:
                //--------------------
                //Taylor series expansion can be expressed as:
                //f(x) = f(a) + 1/1!*f'(a)*(x - a) + 1/2!*f''(a)*(x - a)^2 + ...

                //where f'(x) is the derivative of f respect x, which can also be expressed as:
                //f'(x) = diff(f(x))/diff(x)

                //and f'(a) is the derivative of f respect x evaluated at a, which can be expressed
                //as f'(a) = diff(f(a))/diff(x)

                //consequently f''(a) is the second derivative respect x evaluated at a, which can
                //be expressed as:
                //f''(x) = diff(f(x))/diff(x^2)

                //and:
                //f''(a) = diff(f(a))/diff(x^2)

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
                //and assuming that the location of the radio source is known and it is located at pa = (xa, ya, za)
                //so that d^2 = (x - xa)^2 + (y - ya)^2 + (z - za)^2 then the received power at an unknown point
                //pi = (xi, yi, zi) is:

                //Pr(pi) = Pr(xi,yi,zi) = K - 5*n*log(d^2) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                //Suppose that received power at point p1=(x1,y1,z1) is known on a located fingerprint
                //containing readings Pr(p1).

                //Then, for an unknown point pi=(xi,yi,zi) close to fingerprint 1 located at p1 where we
                //have measured received power Pr(pi), we can get the following second-order Taylor
                //approximation:

                //Pr(pi) ~ Pr(p1) + JPtr(p1)*(pi - p1) + 1/2*(pi - p1)^T*HPr(p1)*(pi - p1) + ...

                //where JPr(p1) is the Jacobian of Pr evaluated at p1. Since Pr is a multivariate function
                //with scalar result, the Jacobian has size 1x3 and is equal to the gradient.
                //HPtr(p1) is the Hessian matrix evaluated at p1, which is a symmetric matrix of size 3x3,
                //and (pi-p1)^T is the transposed vector of (pi-p1)

                //Hence, the Jacobian at any point p=(x,y,z) is equal to:
                //JPr(p = (x,y,z)) = [diff(Pr(x,y,z))/diff(x)     diff(Pr(x,y,z))/diff(y)     diff(Pr(x,y,z))/diff(z)]

                //And the Hessian matrix is equal to
                //HPr(p = (x,y,z)) = [diff(Pr(x,y,z))/diff(x^2)    diff(Pr(x,y,z))/diff(x*y)     diff(Pr(x,y,z))/diff(x*z)]
                //                   [diff(Pr(x,y,z))/diff(x*y)    diff(Pr(x,y,z))/diff(y^2)     diff(Pr(x,y,z))/diff(y*z)]
                //                   [diff(Pr(x,y,z))/diff(x*z)    diff(Pr(x,y,z))/diff(y*z)     diff(Pr(x,y,z))/diff(z^2)]

                //Simplifying Taylor expansion to first-order terms to get a linear (but less accurate)
                //solution, we get:
                //Pr(pi) = Pr(p1) + JPtr(p1)*(pi - p1)
                //Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(xi - x1) + diff(Pr(p1))/diff(y)*(yi - y1) + diff(Pr(p1))/diff(z)*(zi - z1)

                //where the first order derivatives of Pr(p = (x,y,z)) are:
                //diff(Pr(x,y,z))/diff(x) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(x - xa)
                //diff(Pr(x,y,z))/diff(x) = -10*n*(x - xa)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                //diff(Pr(x,y,z))/diff(y) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(y - ya)
                //diff(Pr(x,y,z))/diff(y) = -10*n*(y - ya)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                //diff(Pr(x,y,z))/diff(z) = -5*n/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2)*2*(z - za)
                //diff(Pr(x,y,z))/diff(z) = -10*n*(z - za)/(ln(10)*((x - xa)^2 + (y - ya)^2 + (z - za)^2))

                //If we evaluate derivatives at p1 = (x1,y1,z1), we get:
                //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))
                //diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))

                //where square distance from fingerprint 1 to radio source a can be expressed as:
                //d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2

                //where both the fingerprint and radio source positions are known, and hence d1a is known.

                //Then derivatives can be expressed as:
                //diff(Pr(p1))/diff(x) = -10*n*(x1 - xa)/(ln(10)*d1a^2)
                //diff(Pr(p1))/diff(y) = -10*n*(y1 - ya)/(ln(10)*d1a^2)
                //diff(Pr(p1))/diff(z) = -10*n*(z1 - za)/(ln(10)*d1a^2)

                //Hence, first order Taylor expansion can be expressed as:
                //Pr(pi) = Pr(p1) + diff(Pr(p1))/diff(x)*(xi - x1) + diff(Pr(p1))/diff(y)*(yi - y1) + diff(Pr(p1))/diff(z)*(zi - z1)
                //Pr(pi) = Pr(p1) - 10*n*(x1 - xa)/(ln(10)*d1a^2)*(xi - x1) - 10*n*(y1 - ya)/(ln(10)*d1a^2)*(yi - y1) - 10*n*(z1 - za)/(ln(10)*d1a^2)*(zi - z1)

                //where the only unknowns are xi,yi,zi.

                //Reordering expression above, we get:
                //10*n*(x1 - xa)/(ln(10)*d1a^2)*xi + 10*n*(y1 - ya)/(ln(10)*d1a^2)*yi + 10*n*(z1 - za)/(ln(10)*d1a^2)*zi = Pr(p1) - Pr(pi) + 10*n*(x1 - xa)/(ln(10)*d1a^2)*x1 + 10*n*(y1 - ya)/(ln(10)*d1a^2)*y1 + 10*n*(z1 - za)/(ln(10)*d1a^2)*z1

                //Which can be expressed in matrix form as:
                //[10*n*(x1 - xa)/(ln(10)*d1a^2)    10*n*(y1 - ya)/(ln(10)*d1a^2)   10*n*(z1 - za)/(ln(10)*d1a^2)]  [xi] = [Pr(p1) - Pr(pi) + 10*n*(x1 - xa)/(ln(10)*d1a^2)*x1 + 10*n*(y1 - ya)/(ln(10)*d1a^2)*y1 + 10*n*(z1 - za)/(ln(10)*d1a^2)*z1]
                //                                                                                                  [yi]
                //                                                                                                  [zi]

                //which is the equation obtained for fingerprint 1 and radio source a.

                //Having at least 3 linear independent equations for different fingerprints or radio sources allows
                //solving unknown position pi = (xi,yi,zi)
                //Hence we could have either 3 or more located fingerprints with 1 radio sources, 3 or more radio
                //sources on a single located fingerprint, or any combination resulting in enough equations


                //build system of equations
                int totalReadings = totalReadings(mNearestFingerprints);

                try {
                    double ln10 = Math.log(10.0);
                    int row = 0;
                    Matrix a = new Matrix(totalReadings, dims);
                    double[] b = new double[totalReadings];
                    for (RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P> locatedFingerprint :
                            mNearestFingerprints) {

                        P fingerprintPosition = locatedFingerprint.getPosition();
                        List<RssiReading<RadioSource>> locatedReadings =
                                locatedFingerprint.getReadings();
                        if (locatedReadings == null) {
                            continue;
                        }

                        double locatedMeanRssi = 0.0;
                        double meanRssi = 0.0;
                        if (mRemoveMeansFromFingerprintReadings) {
                            locatedMeanRssi = locatedFingerprint.getMeanRssi();
                        }


                        for (RssiReading<RadioSource> locatedReading : locatedReadings) {
                            RadioSource source = locatedReading.getSource();

                            //find within the list of located sources the source of
                            //current located fingerprint reading.
                            //Radio sources are compared by their id
                            //regardless of them being located or not
                            @SuppressWarnings("SuspiciousMethodCalls")
                            int pos = mSources.indexOf(source);
                            if (pos < 0) {
                                continue;
                            }

                            RadioSourceLocated<P> locatedSource = mSources.get(pos);
                            double pathLossExponent = mPathLossExponent;
                            if (mUseSourcesPathLossExponentWhenAvailable &&
                                    locatedSource instanceof RadioSourceWithPower) {
                                pathLossExponent = ((RadioSourceWithPower)locatedSource).
                                        getPathLossExponent();
                            }

                            double tmp = 10.0 * pathLossExponent / ln10;

                            P sourcePosition = locatedSource.getPosition();
                            double locatedRssi = locatedReading.getRssi();
                            double sqrDistance = fingerprintPosition.sqrDistanceTo(sourcePosition);
                            if (mRemoveMeansFromFingerprintReadings) {
                                meanRssi = mFingerprint.getMeanRssi();
                            }

                            List<? extends RssiReading<? extends RadioSource>> readings =
                                    mFingerprint.getReadings();
                            for (RssiReading<? extends RadioSource> reading : readings) {
                                if (reading.getSource() == null ||
                                        !reading.getSource().equals(locatedSource)) {
                                    continue;
                                }

                                //only take into account reading for matching sources on located and
                                //non-located readings
                                double rssi = reading.getRssi();

                                //ideally if there was no bias betweeen devices RSSI measures, we should compute:
                                //double diffRssi = locatedRssi - rssi;
                                //However, to account for possible biases, we remove mean of fingerprints from
                                //both readings (ideally both should be equal, but they will only be approximate in
                                // practice).
                                //double diffRssi = (locatedRssi - locatedMeanRssi) - (rssi - meanRssi);
                                double diffRssi;
                                if (mRemoveMeansFromFingerprintReadings) {
                                    diffRssi = (locatedRssi - locatedMeanRssi) - (rssi - meanRssi);
                                } else {
                                    diffRssi = locatedRssi - rssi;
                                }

                                b[row] = diffRssi;
                                for (int i = 0; i < dims; i++) {
                                    double fingerprintCoord = fingerprintPosition.getInhomogeneousCoordinate(i);
                                    double sourceCoord = sourcePosition.getInhomogeneousCoordinate(i);
                                    double diffCoord = fingerprintCoord - sourceCoord;

                                    a.setElementAt(row, i, tmp * diffCoord / sqrDistance);

                                    b[row] += tmp * diffCoord / sqrDistance * fingerprintCoord;
                                }
                                row++;
                            }
                        }
                    }

                    mEstimatedPositionCoordinates = com.irurueta.algebra.Utils.solve(a, b);

                    //a solution was found so we exit loop
                    break;
                } catch (AlgebraException e) {
                    //solution could not be found with current data
                    //Iterate to use additional nearby fingerprints
                    mEstimatedPositionCoordinates = null;
                    mNearestFingerprints = null;
                }
            }

            if (mEstimatedPositionCoordinates == null) {
                //no solution could be found
                throw new FingerprintEstimationException();
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } finally {
            mLocked = false;
        }
    }
}
