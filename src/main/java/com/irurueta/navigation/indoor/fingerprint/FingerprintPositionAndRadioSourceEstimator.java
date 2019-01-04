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

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.indoor.RadioSource;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiFingerprintLocated;
import com.irurueta.navigation.indoor.RssiReading;

import java.util.List;

/**
 * Base class for position and radio source estimators based on located fingerprints
 * containing only RSSI readings.
 * All implementations of this class estimate the position of a new fingerprint
 * and the position of all radio sources associated to fingerprints whose location
 * is known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class FingerprintPositionAndRadioSourceEstimator<P extends Point> extends
        BaseFingerprintPositionAndRadioSourceEstimator<P, FingerprintPositionAndRadioSourceEstimatorListener<P>> {

    /**
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     */
    protected boolean mUseNoMeanNearestFingerprintFinder = true;

    /**
     * Constructor.
     */
    public FingerprintPositionAndRadioSourceEstimator() { }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public FingerprintPositionAndRadioSourceEstimator(
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
    public FingerprintPositionAndRadioSourceEstimator(
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
    public FingerprintPositionAndRadioSourceEstimator(
            List<? extends RssiFingerprintLocated<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>, P>> locatedFingerprints,
            RssiFingerprint<? extends RadioSource,
            ? extends RssiReading<? extends RadioSource>> fingerprint,
            FingerprintPositionAndRadioSourceEstimatorListener<P> listener) {
        super(locatedFingerprints, fingerprint, listener);
    }

    /**
     * Indicates which fingerprint finder is used.
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     * @return indicates which fingerprint finder is used.
     */
    public boolean getUseNoMeanNearestFingerprintFinder() {
        return mUseNoMeanNearestFingerprintFinder;
    }

    /**
     * Specifies which fingerprint finder is used.
     * True indicates that mean effects are removed to find nearest located fingerprints
     * based on RSSI readings. False indicates that RSSI readings are directly used, which
     * might be inaccurate due to bias effects on new fingerprint readings for unknown
     * locations.
     * By default mean effects are removed to remove possible bias effects due to
     * readings measured by different devices with different hardware.
     * @param useNoMeanNearestFingerprintFinder indicates which fingerprint finder is used.
     * @throws LockedException if estimator is locked.
     */
    public void setUseNoMeanNearestFingerprintFinder(
            boolean useNoMeanNearestFingerprintFinder) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mUseNoMeanNearestFingerprintFinder = useNoMeanNearestFingerprintFinder;
    }

}
