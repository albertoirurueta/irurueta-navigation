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
package com.irurueta.navigation.indoor;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;

/**
 * Estimates position using WiFi signals indoor and the Weighted k-Nearest
 * Neighbours (WkNN) algorithm.
 * WkNN algorithm is based on https://github.com/ajnas/WiFiPS.
 *
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings({"WeakerAccess", "Duplicates"})
public abstract class WeightedKNearestNeighboursPositionSolver<P extends Point<?>> {

    /**
     * Default minimum allowed distance between received WiFi fingerprints.
     */
    public static final double DEFAULT_EPSILON = 1e-7;

    /**
     * Minimum required number of fingerprints and their distances.
     * If only 1 fingerprint is used, this algorithm will return provided fingerprint
     * position, however, some accuracy might be lost due to numerical computations.
     * For that reason, when only one fingerprint is provided, this algorithm will
     * simply return the fingerprint position.
     */
    public static final int MIN_FINGERPRINTS = 1;

    /**
     * Known located WiFi fingerprints.
     */
    protected RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] mFingerprints;

    /**
     * Euclidean distances between WiFi signal fingerprints (expressed in dB's).
     */
    protected double[] mDistances;

    /**
     * Listener to be notified of events raised by this instance.
     */
    protected WeightedKNearestNeighboursPositionSolverListener<P> mListener;

    /**
     * Estimated inhomogeneous position coordinates.
     */
    protected double[] mEstimatedPositionCoordinates;

    /**
     * Indicates if this instance is locked because indoor is being
     * estimated.
     */
    protected boolean mLocked;

    /**
     * Minimum allowed distance between received WiFi signal strengths.
     */
    private double mEpsilon = DEFAULT_EPSILON;

    /**
     * Constructor.
     */
    public WeightedKNearestNeighboursPositionSolver() {
    }

    /**
     * Constructor.
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) {
        internalSetFingerprintsAndDistances(fingerprints, distances);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events raised by this instance.
     */
    public WeightedKNearestNeighboursPositionSolver(
            final WeightedKNearestNeighboursPositionSolverListener<P> listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @param listener     listener to be notified of events raised by this instance.
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    public WeightedKNearestNeighboursPositionSolver(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances,
            final WeightedKNearestNeighboursPositionSolverListener<P> listener) {
        this(fingerprints, distances);
        mListener = listener;
    }

    /**
     * Gets listener to be notified of events raised by this instance.
     *
     * @return listener to be notified of events raised by this instance.
     */
    public WeightedKNearestNeighboursPositionSolverListener<P> getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events raised by this instance.
     *
     * @param listener listener to be notified of events raised by this instance.
     * @throws LockedException if instance is busy solving the position.
     */
    public void setListener(
            final WeightedKNearestNeighboursPositionSolverListener<P> listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets known located WiFi fingerprints.
     *
     * @return known located WiFi fingerprints.
     */
    public RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] getFingerprints() {
        return mFingerprints;
    }

    /**
     * Gets euclidean distances between WiFi signal fingerprints
     * (expressed in dB's).
     *
     * @return euclidean distances between WiFi signal fingerprints.
     */
    public double[] getDistances() {
        return mDistances;
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    public boolean isReady() {
        return mFingerprints != null && mDistances != null &&
                mFingerprints.length >= MIN_FINGERPRINTS;
    }

    /**
     * Returns boolean indicating if estimator is locked because estimation is under
     * progress.
     *
     * @return true if solver is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     * @throws LockedException          if instance is busy solving the position.
     */
    public void setFingerprintsAndDistances(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetFingerprintsAndDistances(fingerprints, distances);
    }

    /**
     * Gets minimum allowed distance between WiFi signal fingerprints.
     *
     * @return minimum allowed distance between WiFi signal fingerprints.
     */
    public double getEpsilon() {
        return mEpsilon;
    }

    /**
     * Sets minimum allowed distance between WiFi signal fingerprints.
     *
     * @param epsilon minimum allowed distance between WiFi signal fingerprints.
     *                strengths.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if instance is busy solving the indoor problem.
     */
    public void setEpsilon(final double epsilon) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (epsilon <= 0.0) {
            throw new IllegalArgumentException();
        }

        mEpsilon = epsilon;
    }

    /**
     * Estimates position.
     *
     * @throws NotReadyException if solver is not ready.
     * @throws LockedException   if instance is busy solving position.
     */
    public void solve() throws NotReadyException, LockedException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onSolveStart(this);
            }

            final int num = mFingerprints.length;
            final int dims = getNumberOfDimensions();
            if (num == 1) {
                //only one fingerprint available
                mEstimatedPositionCoordinates = new double[dims];
                for (int i = 0; i < dims; i++) {
                    final P p = mFingerprints[0].getPosition();
                    mEstimatedPositionCoordinates[i] = p.getInhomogeneousCoordinate(i);
                }
            } else {
                //multiple fingerprints available
                final double[] coords = new double[dims];
                double sum = 0.0;
                double w;
                for (int i = 0; i < num; i++) {
                    //weighted average and weight summation
                    w = 1.0 / mDistances[i];
                    sum += w;

                    final P p = mFingerprints[i].getPosition();
                    for (int j = 0; j < dims; j++) {
                        coords[j] += w * p.getInhomogeneousCoordinate(j);
                    }
                }

                //normalize by weight summation
                if (sum != 0.0) {
                    for (int j = 0; j < dims; j++) {
                        coords[j] /= sum;
                    }
                }

                mEstimatedPositionCoordinates = coords;
            }

            if (mListener != null) {
                mListener.onSolveEnd(this);
            }
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets estimated inhomogeneous position coordinates.
     *
     * @return estimated inhomogeneous position coordinates.
     */
    public double[] getEstimatedPositionCoordinates() {
        return mEstimatedPositionCoordinates;
    }

    /**
     * Gets estimated position and stores result into provided instance.
     *
     * @param estimatedPosition instance where estimated position will be stored.
     */
    public void getEstimatedPosition(final P estimatedPosition) {
        if (mEstimatedPositionCoordinates != null) {
            for (int i = 0; i < mEstimatedPositionCoordinates.length; i++) {
                estimatedPosition.setInhomogeneousCoordinate(i,
                        mEstimatedPositionCoordinates[i]);
            }
        }
    }

    /**
     * Gets estimated position.
     *
     * @return estimated position.
     */
    public abstract P getEstimatedPosition();

    /**
     * Gets number of dimensions of location points.
     *
     * @return number of dimensions of location points.
     */
    public abstract int getNumberOfDimensions();

    /**
     * Sets known located WiFi fingerprints and euclidean distances between WiFi
     * signal fingerprints.
     *
     * @param fingerprints known located WiFi fingerprints.
     * @param distances    euclidean distances between WiFi signal fingerprints
     *                     (expressed in dB's).
     * @throws IllegalArgumentException if either fingerprints or distances are null,
     *                                  don't have the same length or their length is smaller than 1.
     */
    protected void internalSetFingerprintsAndDistances(
            final RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>[] fingerprints,
            final double[] distances) {
        if (fingerprints == null || distances == null) {
            throw new IllegalArgumentException();
        }

        if (fingerprints.length < MIN_FINGERPRINTS) {
            throw new IllegalArgumentException();
        }

        if (fingerprints.length != distances.length) {
            throw new IllegalArgumentException();
        }

        mFingerprints = fingerprints;
        mDistances = distances;

        //fix distances if needed
        for (int i = 0; i < mDistances.length; i++) {
            if (mDistances[i] < mEpsilon) {
                mDistances[i] = mEpsilon;
            }
        }
    }
}
