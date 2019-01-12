/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;

import java.util.*;

/**
 * Base class for position and radio source estimators based only on located
 * fingerprints containing RSSI readings.
 * All implementations of this class estimate the position of a new fingerprint
 * and the position of all radio sources associated to fingerprints whose location
 * is known.
 * All implementations solve the problem in a non linear way using Levenberg-Marquardt
 * algorithm.
 */
@SuppressWarnings("WeakerAccess")
public abstract class NonLinearFingerprintPositionAndRadioSourceEstimator<P extends Point> extends
        FingerprintPositionAndRadioSourceEstimator<P> {

    /**
     * Default RSSI standard deviation assumed for provided fingerprints as a fallback
     * when none can be determined.
     */
    public static final double FALLBACK_RSSI_STANDARD_DEVIATION = 1e-3;

    /**
     * Indicates that by default measured RSSI standard deviation of closest fingerprint
     * must be propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_FINGERPRINT_RSSI_STANDARD_DEVIATION = true;

    /**
     * Indicates that by default path-loss exponent standard deviation of radio source
     * must be propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_PATHLOSS_EXPONENT_STANDARD_DEVIATION = true;

    /**
     * Indicates that by default covariance of closest fingerprint position must be
     * propagated into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_FINGERPRINT_POSITION_COVARIANCE = true;

    /**
     * Indicates that by default covariance of radio source position must be propagated
     * into measured RSSI reading variance at unknown location.
     */
    public static final boolean DEFAULT_PROPAGATE_RADIO_SOURCE_POSITION_COVARIANCE = true;

    /**
     * Small value to be used as machine precision.
     */
    private static final double TINY = 1e-12;

    /**
     * Initial sources whose location is known.
     * If provided, their location will be used as initial values, but
     * after executing this estimator they will be refined.
     */
    protected List<? extends RadioSourceLocated<P>> mInitialLocatedSources;

    /**
     * Initial position to start the estimation.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     */
    private P mInitialPosition;

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     */
    protected boolean mUseSourcesPathLossExponentWhenAvailable = true;

    /**
     * RSSI standard deviation fallback value to use when none can be
     * determined from provided readings. This fallback value is only used if
     * no variance is propagated or the resulting value is too small to allow
     * convergence to a solution.
     */
    private double mFallbackRssiStandardDeviation =
            FALLBACK_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateFingerprintRssiStandardDeviation =
            DEFAULT_PROPAGATE_FINGERPRINT_RSSI_STANDARD_DEVIATION;

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must
     * be propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagatePathlossExponentStandardDeviation =
            DEFAULT_PROPAGATE_PATHLOSS_EXPONENT_STANDARD_DEVIATION;

    /**
     * Indicates whether covariance of closest fingerprint position must be
     * propagated into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateFingerprintPositionCovariance =
            DEFAULT_PROPAGATE_FINGERPRINT_POSITION_COVARIANCE;

    /**
     * Indicates whether covariance of radio source position must be propagated
     * into measured RSSI reading variance at unknown location.
     */
    private boolean mPropagateRadioSourcePositionCovariance =
            DEFAULT_PROPAGATE_RADIO_SOURCE_POSITION_COVARIANCE;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private LevenbergMarquardtMultiDimensionFitter mFitter = new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Estimated covariance matrix for estimated non-located fingerprint position and
     * estimated located radio sources position.
     */
    private Matrix mCovariance;

    /**
     * Covariance of estimated position for non-located fingerprint.
     */
    private Matrix mEstimatedPositionCovariance;

    /**
     * Estimated chi square value.
     */
    private double mChiSq;

    /**
     * Constructor.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint) {
        super(locatedFingerprints, fingerprint);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            P initialPosition) {
        super(locatedFingerprints, fingerprint);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            P initialPosition,
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        mInitialPosition = initialPosition;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> initialLocatedSources) {
        super(locatedFingerprints, fingerprint);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            List<? extends RadioSourceLocated<P>> initialLocatedSources,
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            P initialPosition,
            List<? extends RadioSourceLocated<P>> initialLocatedSources) {
        this(locatedFingerprints, fingerprint, initialPosition);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Constructor.
     * @param locatedFingerprints located fingerprints containing RSSI readings.
     * @param fingerprint fingerprint containing readings at an unknown location
     *                    for provided located fingerprints.
     * @param initialPosition initial position to be assumed on non located fingerprint or
     *                        null if unknown.
     * @param initialLocatedSources sources containing initial location to be refined or null
     *                              if unknown.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either non located fingerprint or located
     * fingerprints are null.
     */
    public NonLinearFingerprintPositionAndRadioSourceEstimator(List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            P initialPosition,
            List<? extends RadioSourceLocated<P>> initialLocatedSources,
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        this(locatedFingerprints, fingerprint, initialPosition, listener);
        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Gets initial radio sources whose location is known.
     * @return initial radio sources.
     */
    public List<? extends RadioSourceLocated<P>> getInitialLocatedSources() {
        return mInitialLocatedSources;
    }

    /**
     * Sets initial radio sources whose location is known.
     * @param initialLocatedSources initial radio sources.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialLocatedSources(
            List<? extends RadioSourceLocated<P>> initialLocatedSources)
            throws LockedException  {
        if (isLocked()) {
            throw new LockedException();
        }

        mInitialLocatedSources = initialLocatedSources;
    }

    /**
     * Gets initial position to start the solving algorithm.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     * @return initial position to start the solving algorithm or null.
     */
    public P getInitialPosition() {
        return mInitialPosition;
    }

    /**
     * Sets initial position to start the solving algorithm.
     * This should be a value close to the expected solution.
     * If no value is provided, the average position among all selected nearest
     * located fingerprints will be used.
     * @param initialPosition initial position to start the solving algorithm or null.
     * @throws LockedException if estimator is locked.
     */
    public void setInitialPosition(P initialPosition) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mInitialPosition = initialPosition;
    }

    /**
     * Gets estimated covariance matrix for estimated non-located fingerprint position
     * and estimated located radio sources position.
     * @return estimated covariance matrix for estimated non-located fingerprint
     * position and estimated located radio sources position.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Gets covariance of estimated position for non-located fingerprint.
     * @return covariance of estimated position for non-located fingerprint.
     */
    public Matrix getEstimatedPositionCovariance() {
        return mEstimatedPositionCovariance;
    }

    /**
     * Gets estimated chi square value.
     * @return estimated chi square value.
     */
    public double getChiSq() {
        return mChiSq;
    }

    /**
     * Indicates whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     * @return true to use path loss exponent of provided sources when available,
     * false otherwise.
     */
    public boolean getUseSourcesPathLossExponentWhenAvailable() {
        return mUseSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Specifies whether path loss exponent of provided sources must be used when
     * available (if true), or if fallback path loss exponent must be used instead.
     * @param useSourcesPathLossExponentWhenAvailable true to use path loss exponent of
     *                                                provided sources when available,
     *                                                false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setUseSourcesPathLossExponentWhenAvailable(
            boolean useSourcesPathLossExponentWhenAvailable) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseSourcesPathLossExponentWhenAvailable =
                useSourcesPathLossExponentWhenAvailable;
    }

    /**
     * Gets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     * @return RSSI standard deviation fallback.
     */
    public double getFallbackRssiStandardDeviation() {
        return mFallbackRssiStandardDeviation;
    }

    /**
     * Sets RSSI standard deviation fallback value to use when none can be
     * determined from provided readings.
     * @param fallbackRssiStandardDeviation RSSI standard deviation fallback
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if provided value is smaller than
     * {@link #TINY}.
     */
    public void setFallbackRssiStandardDeviation(
            double fallbackRssiStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (fallbackRssiStandardDeviation < TINY) {
            throw new IllegalArgumentException();
        }
        mFallbackRssiStandardDeviation = fallbackRssiStandardDeviation;
    }

    /**
     * Indicates whether measured RSSI standard deviation of closest fingerprint must be
     * propagated into measured RSSI reading variance at unknown location.
     * @return true to propagate RSSI standard deviation of closest fingerprint,
     * false otherwise.
     */
    public boolean isFingerprintRssiStandardDeviationPropagated() {
        return mPropagateFingerprintRssiStandardDeviation;
    }

    /**
     * Specifies whether measured RSSI standard deviation of closest fingerprint must be
     * propagated into measured RSSI reading variance at unknown location.
     * @param propagateFingerprintRssiStandardDeviation true to propagate RSSI standard
     *                                                  deviation of closest fingerprint,
     *                                                  false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprintRssiStandardDeviationPropagated(
            boolean propagateFingerprintRssiStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateFingerprintRssiStandardDeviation =
                propagateFingerprintRssiStandardDeviation;
    }

    /**
     * Indicates whether path-loss exponent standard deviation of radio source must be
     * propagated into measured RSSI reading variance at unknown location.
     * @return true to propagate  path-loss exponent standard deviation of radio source,
     * false otherwise.
     */
    public boolean isPathlossExponentStandardDeviationPropagated() {
        return mPropagatePathlossExponentStandardDeviation;
    }

    /**
     * Specifies whether path-loss exponent standard deviation of radio source must be
     * propagated into measured RSSI reading variance at unknown location.
     * @param propagatePathlossExponentStandardDeviation true to propagate path-loss
     *                                                   exponent standard deviation of
     *                                                   radio source, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPathlossExponentStandardDeviationPropagated(
            boolean propagatePathlossExponentStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagatePathlossExponentStandardDeviation =
                propagatePathlossExponentStandardDeviation;
    }

    /**
     * Indicates whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     * @return true to propagate fingerprint position covariance, false otherwise.
     */
    public boolean isFingerprintPositionCovariancePropagated() {
        return mPropagateFingerprintPositionCovariance;
    }

    /**
     * Specifies whether covariance of closest fingerprint position must be propagated
     * into measured RSSI reading variance at unknown location.
     * @param propagateFingerprintPositionCovariance true to propagate fingerprint
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFingerprintPositionCovariancePropagated(
            boolean propagateFingerprintPositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateFingerprintPositionCovariance =
                propagateFingerprintPositionCovariance;
    }

    /**
     * Indicates whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     * @return true to propagate radio source position covariance, false otherwise.
     */
    public boolean isRadioSourcePositionCovariancePropagated() {
        return mPropagateRadioSourcePositionCovariance;
    }

    /**
     * Specifies whether covariance of radio source position must be propagated into
     * measured RSSI reading variance at unknown location.
     * @param propagateRadioSourcePositionCovariance true to propagate radio source
     *                                               position covariance, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setRadioSourcePositionCovariancePropagated(
            boolean propagateRadioSourcePositionCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPropagateRadioSourcePositionCovariance =
                propagateRadioSourcePositionCovariance;
    }

    /**
     * Estimates position and radio sources based on provided located radio sources and readings of
     * such radio sources at an unknown location.
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
            mCovariance = null;
            mEstimatedPositionCovariance = null;
            mNearestFingerprints = null;
            mEstimatedLocatedSources = null;

            int min = Math.max(1, mMinNearestFingerprints);
            int max = mMaxNearestFingerprints < 0 ?
                    mLocatedFingerprints.size() :
                    Math.min(mMaxNearestFingerprints, mLocatedFingerprints.size());
            for (int k = min; k <= max; k++) {
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

                //The expression of received power expressed in dBm's is:
                //k = (c/(4*pi*f))
                //Pr = Pte*k^n / d^n

                //where c is the speed of light, pi is 3.14159..., f is the frequency of the radio source,
                //Pte is the equivalent transmitted power by the radio source, n is the path-loss exponent
                // (typically 2.0), and d is the distance from a point to the location of the radio source.

                //Hence:
                //Pr(dBm) = 10*log(Pte*k^n/d^n) = 10*n*log(k) + 10*log(Pte) - 10*n*log(d) =
                //          10*n*log(k) + 10*log(Pte) - 5*n*log(d^2)

                //where d^2 = dia^2 = (xi - xa)^2 + (yi - ya)^2 is the squared distance between
                //fingerprint and unknown point pi = (xi, yi) and
                //radio source a = a,b... M
                //10*n*log(k) is constant for a given radio source "a", and
                //Pte is the equivalent transmitted power of radio source "a".

                //We assume that the 2 former terms are constant and known for a given radio source
                //K = 10*n*log(k) + 10*log(Pte), and the only unknown term is
                //the latter one depending on the distance of the
                //measuring point and the radio source.

                //Hence for a given radio source "a" at unknown location "i":
                //Pr(pi) = K - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                //we assume that a known located fingerprint is located at p1 = (x1, y1),
                //Both readings Pr(pi) and Pr(p1) belong to the same radio source "a", hence
                //K term is the same.

                //Pr(p1) = K - 5*n*log((x1 - xa)^2 + (y1 - ya)^2)

                //where d1a^2 = (x1 - xa)^2 + (y1 - ya)^2 is the squared distance between
                //fingerprint 1 and radio source 2

                //To remove possible bias effects on readings, we consider the difference of received
                //power for fingerprint "1" and radio source "a" as:

                //Prdiff1a = Pr(pi) - Pr(p1) = (K - 5*n*log(dia^2)) - (K - 5*n*log(d1a^2)) =
                //  = 5*n*log(d1a^2) - 5*n*log(dia^2)

                //where both d1a^2 and dia^2 are unknown, because location pi=(xi,yi) and pa=(xa,ya) are unknown.
                //Now we have no dependencies on the amount of transmitted power of each radio source
                //contained on constant term K, and we only depend on squared distances d1a^2 and dia^2.

                //Consequently, the difference of received power for fingerprint "2" and radio source "a" is:
                //Prdiff2a = 5*n*log(d2a^2) - 5*n*log(dia^2)

                //the difference of received power for fingerprint "1" and radio source "b" is:
                //Prdiff1b = 5*n*log(d1b^2) - 5*n*log(dib^2)

                //and so on.

                //we want to find unknown location pi, and location of radio source pa, pb,... pM so that Prdiff
                //errors are minimized in LMSE (Least Mean Square Error) terms.

                //Assuming that we have M radio sources and N fingerprints, we have
                //y = [Prdiff1a Prdiff2a Prdiff1b Prdiff2b ... PrdiffNa PrdiffNb ... PrdiffNM]

                //and the unknowns to be found are:
                //x = [xi yi xa ya xb yb ... xM yM], which are the location of the unknown fingerprint
                //pi = (xi, yi) and the locations of the radio sources a, b ... M that we want to find pa = (xa, ya),
                //pb = (xb, yb) ... pM = (xM, yM)

                try {
                    List<RadioSource> sourcesToBeEstimated = setupFitter();
                    if (mMinNearestFingerprints < 0) {
                        //if no limit is set in minimum value, then a minimum of
                        //dims * (1 + numSources) is used
                        int numSources = sourcesToBeEstimated.size();
                        int dims = getNumberOfDimensions();
                        int minNearest = dims * (1 + numSources);
                        if (k < minNearest) {
                            continue;
                        }
                    }

                    mFitter.fit();

                    //estimated position
                    double[] a = mFitter.getA();
                    mCovariance = mFitter.getCovar();
                    mChiSq = mFitter.getChisq();

                    int dims = getNumberOfDimensions();

                    //obtain estimated position coordinates and covariance
                    mEstimatedPositionCoordinates = new double[dims];
                    System.arraycopy(a, 0, mEstimatedPositionCoordinates,
                            0, dims);

                    int dimsMinusOne = dims - 1;
                    mEstimatedPositionCovariance = mCovariance.getSubmatrix(0, 0,
                            dimsMinusOne, dimsMinusOne);

                    //obtain radio sources estimated positions and covariance
                    int totalSources = sourcesToBeEstimated.size();
                    mEstimatedLocatedSources = new ArrayList<>();
                    for (int j = 0; j < totalSources; j++) {
                        P sourcePosition = createPoint();

                        int start = dims * (1 + j);
                        int end = start + dimsMinusOne;
                        for(int i = 0; i < dims; i++) {
                            sourcePosition.setInhomogeneousCoordinate(i, a[start + i]);
                        }

                        Matrix sourceCovariance = mCovariance.getSubmatrix(
                                start, start, end, end);

                        RadioSource source = sourcesToBeEstimated.get(j);
                        mEstimatedLocatedSources.add(
                                createRadioSource(source, sourcePosition,
                                        sourceCovariance));
                    }

                    //a solution was found so we exit loop
                    break;
                } catch (NumericalException e) {
                    //solution could not be found with current data
                    //Iterate to use additional nearby fingerprints
                    mEstimatedPositionCoordinates = null;
                    mCovariance = null;
                    mEstimatedPositionCovariance = null;
                    mEstimatedLocatedSources = null;
                }
            }

            if (mEstimatedPositionCoordinates == null ||
                    mEstimatedLocatedSources == null) {
                //no position could be estimated
                throw new FingerprintEstimationException();
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } finally {
            mLocked = false;
        }
    }

    /**
     * Propagates provided variances into RSSI differences.
     * @param pathlossExponent path-loss exponent.
     * @param fingerprintPosition position of closest located fingerprint.
     * @param radioSourcePosition radio source position associated to fingerprint reading.
     * @param estimatedPosition position to be estimated. Usually this is equal to the
     *                          initial position used by a non linear algorithm.
     * @param pathlossExponentVariance variance of path-loss exponent or null if unknown.
     * @param fingerprintPositionCovariance covariance of fingerprint position or null if
     *                                      unknown.
     * @param radioSourcePositionCovariance covariance of radio source position or null if
     *                                      unknown.
     * @return variance of RSSI difference measured at non located fingerprint reading.
     */
    protected abstract Double propagateVariances(double pathlossExponent,
            P fingerprintPosition, P radioSourcePosition, P estimatedPosition,
            Double pathlossExponentVariance, Matrix fingerprintPositionCovariance,
            Matrix radioSourcePositionCovariance);

    /**
     * Creates a located radio source from provided radio source, position and
     * covariance.
     * @param source radio source.
     * @param sourcePosition radio source position.
     * @param sourceCovariance radio source position covariance.
     * @return located radio source.
     */
    private RadioSourceLocated<P> createRadioSource(RadioSource source,
            P sourcePosition, Matrix sourceCovariance) {

        int dims = getNumberOfDimensions();

        switch (source.getType()) {
            case BEACON:
                Beacon beacon = (Beacon)source;
                if (dims == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                    //2D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>)new BeaconLocated2D(
                            beacon.getIdentifiers(), beacon.getTransmittedPower(),
                            beacon.getFrequency(), beacon.getBluetoothAddress(),
                            beacon.getBeaconTypeCode(), beacon.getManufacturer(),
                            beacon.getServiceUuid(), beacon.getBluetoothName(),
                            (Point2D) sourcePosition, sourceCovariance);
                } else {
                    //3D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>)new BeaconLocated3D(
                            beacon.getIdentifiers(), beacon.getTransmittedPower(),
                            beacon.getFrequency(), beacon.getBluetoothAddress(),
                            beacon.getBeaconTypeCode(), beacon.getManufacturer(),
                            beacon.getServiceUuid(), beacon.getBluetoothName(),
                            (Point3D) sourcePosition, sourceCovariance);
                }
            case WIFI_ACCESS_POINT:
            default:
                WifiAccessPoint accessPoint = (WifiAccessPoint)source;
                if (dims == Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                    //2D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>)new WifiAccessPointLocated2D(
                            accessPoint.getBssid(), accessPoint.getFrequency(),
                            accessPoint.getSsid(), (Point2D) sourcePosition,
                            sourceCovariance);
                } else {
                    //3D

                    //noinspection unchecked
                    return (RadioSourceLocated<P>)new WifiAccessPointLocated3D(
                            accessPoint.getBssid(), accessPoint.getFrequency(),
                            accessPoint.getSsid(), (Point3D) sourcePosition,
                            sourceCovariance);
                }
        }

    }

    /**
     * Builds data required to solve the problem.
     * This method takes into account current nearest fingerprints and discards those
     * readings belonging to radio sources not having enough data to be estimated.
     * @param allPowerDiffs list of received power differences of RSSI readings between a
     *                      located fingerprint and an unknown fingerprint for a given radio
     *                      source.
     * @param allFingerprintPositions positions of all located fingerprints being taken into
     *                                account.
     * @param allInitialSourcesPositions initial positions of all radio sources to be taken
     *                                   into account. If initial located sources where provided,
     *                                   their positions will be used, otherwise the centroid of
     *                                   all located fingerprints associated to a radio source will
     *                                   be used as initial position.
     * @param allSourcesToBeEstimated all radio sources that will be estimated.
     * @param allSourcesIndices indices indicating the position radio source being used
     *                          within the list of sources for current reading.
     * @param allPathLossExponents list of path loss exponents.
     * @param allStandardDeviations list of standard deviations for readings being used.
     * @param nearestFingerprintsCentroid centroid of nearest fingerprints being taken into account.
     */
    @SuppressWarnings("Duplicates")
    private void buildData(List<Double> allPowerDiffs,
                           List<P> allFingerprintPositions,
                           List<P> allInitialSourcesPositions,
                           List<RadioSource> allSourcesToBeEstimated,
                           List<Integer> allSourcesIndices,
                           List<Double> allPathLossExponents,
                           List<Double> allStandardDeviations,
                           P nearestFingerprintsCentroid) {

        int dims = getNumberOfDimensions();
        int num = 0;
        double[] centroidCoords = new double[dims];
        for (RssiFingerprintLocated<? extends RadioSource,
                ? extends RssiReading<? extends RadioSource>, P> fingerprint : mNearestFingerprints) {
            P position = fingerprint.getPosition();
            if (position == null) {
                continue;
            }

            for (int i = 0; i < dims; i++) {
                centroidCoords[i] += position.getInhomogeneousCoordinate(i);
            }
            num++;
        }

        for(int i = 0; i < dims; i++) {
            centroidCoords[i] /= (double)num;
            nearestFingerprintsCentroid.setInhomogeneousCoordinate(i, centroidCoords[i]);
        }

        //maps to keep cached in memory computed values to speed up computations
        HashMap<RadioSource, Integer> numReadingsMap = new HashMap<>();
        HashMap<RadioSource, P> centroidsMap = new HashMap<>();

        for (RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P> locatedFingerprint :
                mNearestFingerprints) {

            List<RssiReading<RadioSource>> locatedReadings =
                    locatedFingerprint.getReadings();
            if (locatedReadings == null) {
                continue;
            }

            P fingerprintPosition = locatedFingerprint.getPosition();
            Matrix fingerprintPositionCovariance = locatedFingerprint.
                    getPositionCovariance();

            for (RssiReading<RadioSource> locatedReading : locatedReadings) {
                RadioSource source = locatedReading.getSource();

                //obtain the total number of readings available for this source and
                //the centroid of all located fingerprints containing readings for
                //such source
                int numReadings;
                if (!numReadingsMap.containsKey(source)) {
                    numReadings = totalReadingsForSource(source, mNearestFingerprints, null);
                    numReadingsMap.put(source, numReadings);
                } else {
                    numReadings = numReadingsMap.get(source);
                }

                if (numReadings < dims) {
                    continue;
                }

                P centroid;
                if (!centroidsMap.containsKey(source)) {
                    centroid = createPoint();

                    //noinspection unchecked
                    totalReadingsForSource(source,
                            (List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>>) mLocatedFingerprints,
                            centroid);

                    centroidsMap.put(source, centroid);
                } else {
                    centroid = centroidsMap.get(source);
                }

                //find within the list of located sources (if available) the source
                //of current located fingerprint
                @SuppressWarnings("SuspiciousMethodCalls")
                int pos = mInitialLocatedSources != null ?
                        mInitialLocatedSources.indexOf(source) : -1;

                double pathLossExponent = mPathLossExponent;
                Double pathLossExponentVariance = null;
                if (mUseSourcesPathLossExponentWhenAvailable &&
                        source instanceof RadioSourceWithPower) {
                    RadioSourceWithPower sourceWithPower = (RadioSourceWithPower)source;
                    pathLossExponent = sourceWithPower.getPathLossExponent();
                    Double std = sourceWithPower.getPathLossExponentStandardDeviation();
                    pathLossExponentVariance = std != null ? std * std : null;
                }


                P sourcePosition;
                Matrix sourcePositionCovariance = null;
                if (pos < 0) {
                    //located source is not available, so we use centroid
                    sourcePosition = centroid;
                } else {
                    RadioSourceLocated<P> locatedSource = mInitialLocatedSources.get(pos);
                    sourcePosition = locatedSource.getPosition();

                    if (mUseSourcesPathLossExponentWhenAvailable &&
                            locatedSource instanceof RadioSourceWithPower &&
                            pathLossExponentVariance == null) {
                        RadioSourceWithPower locatedSourceWithPower =
                                (RadioSourceWithPower)locatedSource;
                        pathLossExponent = locatedSourceWithPower.getPathLossExponent();
                        Double std = locatedSourceWithPower.
                                getPathLossExponentStandardDeviation();
                        pathLossExponentVariance = std != null ? std * std : null;
                    }

                    sourcePositionCovariance = locatedSource.getPositionCovariance();
                }

                int sourceIndex;
                if (!allSourcesToBeEstimated.contains(source)) {
                    sourceIndex = allSourcesToBeEstimated.size();

                    allSourcesToBeEstimated.add(source);
                    allInitialSourcesPositions.add(sourcePosition);
                } else {
                    sourceIndex = allSourcesToBeEstimated.indexOf(source);
                }


                double locatedRssi = locatedReading.getRssi();

                Double locatedRssiStd = locatedReading.getRssiStandardDeviation();
                Double locatedRssiVariance = locatedRssiStd != null ?
                        locatedRssiStd * locatedRssiStd : null;

                List<? extends RssiReading<? extends RadioSource>> readings =
                        mFingerprint.getReadings();
                for (RssiReading<? extends RadioSource> reading : readings) {
                    if (reading.getSource() == null ||
                            !reading.getSource().equals(source)) {
                        continue;
                    }

                    //only take into account reading for matching sources on located
                    //and non-located readings
                    double rssi = reading.getRssi();

                    double powerDiff = rssi - locatedRssi;

                    Double standardDeviation = null;
                    if (mPropagatePathlossExponentStandardDeviation ||
                            mPropagateFingerprintPositionCovariance ||
                            mPropagateRadioSourcePositionCovariance) {

                        //compute initial position
                        P initialPosition = mInitialPosition != null ?
                                mInitialPosition : nearestFingerprintsCentroid;

                        Double variance = propagateVariances(pathLossExponent,
                                fingerprintPosition, sourcePosition, initialPosition,
                                mPropagatePathlossExponentStandardDeviation ? pathLossExponentVariance : null,
                                mPropagateFingerprintPositionCovariance ? fingerprintPositionCovariance : null,
                                mPropagateRadioSourcePositionCovariance ? sourcePositionCovariance : null);
                        if (variance != null) {
                            standardDeviation = Math.sqrt(variance);
                        }
                    }

                    if (standardDeviation == null) {
                        standardDeviation = reading.getRssiStandardDeviation();
                    }

                    if (mPropagateFingerprintRssiStandardDeviation) {
                        if (standardDeviation != null &&
                                reading.getRssiStandardDeviation() != null){
                            //consider propagated variance and reading variance independent, so we
                            //sum them both
                            standardDeviation = standardDeviation * standardDeviation +
                                    reading.getRssiStandardDeviation() * reading.getRssiStandardDeviation();
                            standardDeviation = Math.sqrt(standardDeviation);
                        }

                        if (locatedRssiVariance != null && standardDeviation != null) {
                            //consider propagated variance and located reading variance
                            //independent, so we sum them both
                            standardDeviation = standardDeviation * standardDeviation +
                                    locatedRssiVariance;
                            standardDeviation = Math.sqrt(standardDeviation);
                        }
                    }

                    if (standardDeviation == null || standardDeviation < TINY) {
                        standardDeviation = mFallbackRssiStandardDeviation;
                    }

                    allPowerDiffs.add(powerDiff);
                    allFingerprintPositions.add(fingerprintPosition);
                    allSourcesIndices.add(sourceIndex);
                    allPathLossExponents.add(pathLossExponent);
                    allStandardDeviations.add(standardDeviation);
                }
            }
        }
    }

    /**
     * Setups fitter to solve positions.
     * @return list of radio sources whose location will be estimated.
     * @throws FittingException if Levenberg-Marquardt fitting fails.
     */
    @SuppressWarnings("Duplicates")
    private List<RadioSource> setupFitter() throws FittingException {
        //build lists of data
        final List<Double> allPowerDiffs = new ArrayList<>();
        final List<P> allFingerprintPositions = new ArrayList<>();
        final List<P> allInitialSourcesPositions = new ArrayList<>();
        final List<RadioSource> allSourcesToBeEstimated = new ArrayList<>();
        final List<Integer> allSourcesIndices = new ArrayList<>();
        final List<Double> allPathLossExponents = new ArrayList<>();
        final List<Double> allStandardDeviations = new ArrayList<>();
        final P nearestFingerprintsCentroid = createPoint();
        buildData(allPowerDiffs, allFingerprintPositions, allInitialSourcesPositions,
                allSourcesToBeEstimated, allSourcesIndices, allPathLossExponents,
                allStandardDeviations, nearestFingerprintsCentroid);

        final int totalReadings = allPowerDiffs.size();
        final int totalSources = allSourcesToBeEstimated.size();
        final int dims = getNumberOfDimensions();
        final int n = 1 + dims;

        mFitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                return n;
            }

            @Override
            public double[] createInitialParametersArray() {
                double[] initial = new double[dims * (totalSources + 1)];

                if (mInitialPosition == null) {
                    //use centroid of nearest fingerprints as initial value
                    for (int i = 0; i < dims; i++) {
                        initial[i] = nearestFingerprintsCentroid.getInhomogeneousCoordinate(i);
                    }
                } else {
                    //use provided initial position
                    for(int i = 0; i < dims; i++) {
                        initial[i] = mInitialPosition.getInhomogeneousCoordinate(i);
                    }
                }

                int pos = dims;
                for (int j = 0; j < totalSources; j++) {
                    P initialSourcePosition = allInitialSourcesPositions.get(j);
                    for (int i = 0; i < dims; i++) {
                        initial[pos] = initialSourcePosition.
                                getInhomogeneousCoordinate(i);
                        pos++;
                    }
                }

                return initial;
            }

            @Override
            public double evaluate(int i, double[] point, double[] params, double[] derivatives) {

                //For 2D:
                //-------

                //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)

                //derivatives respect parameters being estimated (xi,yi,xa,ya...,xM,yM)
                //for unknown point pi = (xi, yi)
                //diff(Prdiff1a)/diff(xi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*2*(xi - xa)
                //  = -10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //  = -10*n*(xi - xa)/(log(10)*dia^2)

                //diff(Prdiff1a)/diff(yi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*2*(yi - ya)
                //  = -10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //  = -10*n*(yi - ya)/(log(10)*dia^2)

                //for same radio source pa=(xa,ya)
                //diff(Prdiff1a)/diff(xa) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(x1 - xa) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*-2*(xi - xa) =
                //  = -10*n*(x1 - xa)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //  = 10*n*(-(x1 - xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                //diff(Prdiff1a)/diff(ya) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2))*-2*(y1 - ya) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2))*-2*(yi - ya) =
                //  = -10*n*(y1 - ya)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2)) + 10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2))
                //  = 10*n*(-(y1 - ya)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                //for other radio source pb=(xb,yb)
                //diff(Prdiff1a)/diff(xb) = diff(Prdiff1a)/diff(yb) = 0

                //For 3D:
                //-------

                //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
                //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)

                //derivatives respect parameters being estimated (xi,yi,zi,xa,ya,za...,xM,yM,zM)
                //for unknown point pi = (xi, yi,zi)
                //diff(Prdiff1a)/diff(xi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(xi - xa)
                //  = -10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2) + (zi - za)^2))
                //  = -10*n*(xi - xa)/(log(10)*dia^2)

                //diff(Prdiff1a)/diff(yi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(yi - ya)
                //  = -10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))
                //  = -10*n*(yi - ya)/(log(10)*dia^2)

                //diff(Prdiff1a)/diff(zi) = -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*2*(zi - za)
                //  = -10*n*(zi - za)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))
                //  = -10*n*(zi - za)/(log(10)*dia^2)

                //for same radio source pa=(xa,ya)
                //diff(Prdiff1a)/diff(xa) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(x1 - xa) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(xi - xa) =
                //  = -10*n*(x1 - xa)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(xi - xa)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //  = 10*n*(-(x1 -xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                //diff(Prdiff1a)/diff(ya) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(y1 - ya) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(yi - ya) =
                //  = -10*n*(y1 - ya)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(yi - ya)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //  = 10*n(-(y1 - ya)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2))

                //diff(Prdiff1a)/diff(za) = 5*n/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2))*-2*(z1 - za) -5*n/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2))*-2*(zi - za) =
                //  = -10*n*(z1 - za)/(log(10)*((x1 - xa)^2 + (y1 - ya)^2 + (z1 - za)^2)) + 10*n*(zi - za)/(log(10)*((xi - xa)^2 + (yi - ya)^2 + (zi - za)^2)) =
                //  = 10*n(-(z1 - za)/(log(10)*d1a^2) + (zi - za)/(log(10)*dia^2))

                //for other radio source pb=(xb,yb)
                //diff(Prdiff1a)/diff(xb) = diff(Prdiff1a)/diff(yb) = 0

                int dims = NonLinearFingerprintPositionAndRadioSourceEstimator.this.
                        getNumberOfDimensions();

                //path loss exponent
                double n = point[0];

                double ln10 = Math.log(10.0);

                int sourceIndex = allSourcesIndices.get(i);
                int start = dims * (1 + sourceIndex);

                //d1a^2, d2a^2, ...
                double distanceFingerprint2 = 0.0;

                //dia^2, dib^2, ...
                double distancePoint2 = 0.0;
                for (int j = 0; j < dims; j++) {
                    //fingerprint coordinate p1=(x1,y1,z1), ...
                    double fingerprintCoord = point[1 + j];

                    //unknown point "pi" coordinate
                    double pointCoord = params[j];

                    //radio source coordinate pa=(xa,ya,za), ...
                    double sourceCoord = params[start + j];

                    //x1 - xa, y1 - ya, ...
                    double diffFingerprint = fingerprintCoord - sourceCoord;

                    //xi - xa, yi - ya, ...
                    double diffPoint = pointCoord - sourceCoord;

                    double diffFingerprint2 = diffFingerprint * diffFingerprint;
                    double diffPoint2 = diffPoint * diffPoint;

                    distanceFingerprint2 += diffFingerprint2;
                    distancePoint2 += diffPoint2;
                }

                distanceFingerprint2 = Math.max(distanceFingerprint2, TINY);
                distancePoint2 = Math.max(distancePoint2, TINY);

                double result = 5*n*(Math.log10(distanceFingerprint2) -
                        Math.log10(distancePoint2));


                //we clear derivatives array to ensure that derivatives respect other
                //radio sources are zero
                Arrays.fill(derivatives, 0.0);

                for(int j = 0; j < dims; j++) {
                    //fingerprint coordinate p1=(x1,y1,z1), ...
                    double fingerprintCoord = point[1 + j];

                    //unknown point "pi" coordinate
                    double pointCoord = params[j];

                    //radio source coordinate pa=(xa,ya,za), ...
                    double sourceCoord = params[start + j];

                    //x1 - xa, y1 - ya, ...
                    double diffFingerprint = fingerprintCoord - sourceCoord;

                    //xi - xa, yi - ya, ...
                    double diffPoint = pointCoord - sourceCoord;


                    //Example: diff(Prdiff1a)/diff(xi) =  -10*n*(xi - xa)/(log(10)*dia^2)
                    double derivativePointCoord = -10.0 * n * diffPoint / (ln10 * distancePoint2);

                    //Example: diff(Prdiff1a)/diff(xa) = 10*n*(-(x1 - xa)/(log(10)*d1a^2) + (xi - xa)/(log(10)*dia^2)) =
                    //  -10*n*(x1 - xa)/(log(10)*d1a^2) - diff(Prdiff1a)/diff(xi)
                    double derivativeSameRadioSourceCoord =
                            -10.0 * n * diffFingerprint / (ln10 * distanceFingerprint2)
                            - derivativePointCoord;

                    //derivatives respect point pi = (xi, yi, zi)
                    derivatives[j] = derivativePointCoord;

                    //derivatives respect same radio source pa = (xa, ya, za)
                    derivatives[dims * (1 + sourceIndex) + j] = derivativeSameRadioSourceCoord;
                }

                return result;
            }
        });

        try {
            //In 2D we know that for fingerprint "1" and radio source "a":
            //Prdiff1a = Pr(pi) - Pr(p1) = 5*n*log(d1a^2) - 5*n*log(dia^2) =
            //  = 5*n*log((x1 - xa)^2 + (y1 - ya)^2) - 5*n*log((xi - xa)^2 + (yi - ya)^2)

            //Therefore x must have 1 + dims columns (for path-loss n and fingerprint position (x1,y1)

            Matrix x = new Matrix(totalReadings, n);
            double[] y = new double[totalReadings];
            double[] standardDeviations = new double[totalReadings];
            for (int i = 0; i < totalReadings; i++) {
                //path loss exponent
                x.setElementAt(i, 0, allPathLossExponents.get(i));

                P fingerprintPosition = allFingerprintPositions.get(i);
                int col = 1;
                for (int j = 0; j < dims; j++) {
                    x.setElementAt(i, col,
                            fingerprintPosition.getInhomogeneousCoordinate(j));
                    col++;
                }

                y[i] = allPowerDiffs.get(i);

                standardDeviations[i] = allStandardDeviations.get(i);
            }

            mFitter.setInputData(x, y, standardDeviations);

            return allSourcesToBeEstimated;
        } catch (AlgebraException e) {
            throw new FittingException(e);
        }
    }

    /**
     * Gets the total number of readings associated to provided radio source.
     * This method uses only current nearest fingerprints.
     * @param source radio source to be checked
     * @param centroid centroid to be computed.
     * @param fingerprints fingerprints where search is made.
     * @return total number of readings associated to provided radio source.
     */
    private int totalReadingsForSource(RadioSource source,
            List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P>> fingerprints,
            P centroid) {
        if (source == null) {
            return 0;
        }

        int dims = getNumberOfDimensions();

        int result = 0;
        double[] centroidCoords = centroid != null ?
                new double[dims] : null;

        for (RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, P> fingerprint : fingerprints) {
            List<RssiReading<RadioSource>> readings = fingerprint.getReadings();
            if (readings == null) {
                continue;
            }

            P fingerprintPosition = fingerprint.getPosition();

            for(RssiReading<RadioSource> reading : readings) {
                RadioSource readingSource = reading.getSource();
                if (readingSource != null && readingSource.equals(source)) {
                    result++;

                    if (centroid != null) {
                        for (int i = 0; i < dims; i++) {
                            double coord = fingerprintPosition.getInhomogeneousCoordinate(i);
                            centroidCoords[i] += coord;
                        }
                    }
                }
            }
        }

        if (centroid != null) {
            for (int i = 0; i < dims; i++) {
                centroidCoords[i] /= (double) result;
                centroid.setInhomogeneousCoordinate(i, centroidCoords[i]);
            }
        }

        return result;
    }
}
