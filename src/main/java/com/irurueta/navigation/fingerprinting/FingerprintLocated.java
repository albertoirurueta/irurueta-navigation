package com.irurueta.navigation.fingerprinting;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point;

/**
 * Contains a located reading from several radio sources.
 * @param <P> a {@link Point} type.
 */
public interface FingerprintLocated<P extends Point> {

    /**
     * Gets position where fingerprint readings were made.
     * @return position where fingerprint readings were made.
     */
    P getPosition();

    /**
     * Gets covariance of inhomogeneous coordinates of current position (if available).
     * @return covariance of position or null.
     */
    Matrix getPositionCovariance();
}
