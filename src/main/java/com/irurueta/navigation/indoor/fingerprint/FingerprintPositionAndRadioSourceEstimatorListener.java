package com.irurueta.navigation.indoor.fingerprint;

import com.irurueta.geometry.Point;

/**
 * Listener to be notified of events such as when estimation of position and radio
 * sources using fingerprints starts or ends.
 * @param <P> a {@link Point} type.
 */
public interface FingerprintPositionAndRadioSourceEstimatorListener<P extends Point>
        extends BaseFingerprintPositionAndRadioSourceEstimatorListener<FingerprintPositionAndRadioSourceEstimator<P>> { }
