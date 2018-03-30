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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * Finds k-nearest WiFi fingerprints based on their signal euclidean distances (not their actual location).
 */
@SuppressWarnings("WeakerAccess")
public class WifiKNearestFinder<P extends Point> {

    /**
     * Collection of fingerprints to match against.
     */
    private Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> mFingerprints;

    /**
     * Constructor.
     * @param fingerprints collection of fingerprints to match against.
     * @throws IllegalArgumentException if collection of fingerprints is null.
     */
    public WifiKNearestFinder(
            Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> fingerprints)
            throws IllegalArgumentException {
        if (fingerprints == null) {
            throw new IllegalArgumentException();
        }
        mFingerprints = fingerprints;
    }

    /**
     * Finds nearest fingerprint to provided one, in terms of signal euclidean distances, within the collection of
     * provided fingerprints.
     * @param fingerprint fingerprint to find the nearest to.
     * @return nearest fingerprint or null if none could be found.
     */
    public RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>
            findNearestTo(RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint) {
        return findNearestTo(fingerprint, mFingerprints);
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal euclidean distances, within the collection of
     * provided fingerprints.
     * @param fingerprint fingerprint to find the k-nearest ones to.
     * @param k number of nearest fingerprints to be found.
     * @return nearest fingerprints ordered from closest to farthest or an empty list if none could be found.
     * @throws IllegalArgumentException if either fingerprint is null or k is less than 1.
     */
    public List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>>
            findKNearestTo(RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint,
            int k) throws IllegalArgumentException {
        return findKNearestTo(fingerprint, mFingerprints, k);
    }

    /**
     * Finds k-nearest fingerprints to provided on, in terms of signal euclidean distances, within the collection
     * of provided fingerprints.
     * @param fingerprint fingerprint to find the k-nearest ones to.
     * @param k number of nearest fingerprints to find.
     * @param nearestFingerprints list where found nearest fingerprints will be stored ordered from closest to farthest
     *                            or an empty list if none could be found.
     * @param nearestSqrDistances list where squared signal euclidean distances corresponding to found fingerprints will
     *                            be stored or an empty list if no fingerprint is found.
     * @throws IllegalArgumentException if any parameter is null or k is less than 1.
     */
    public void findKNearestTo(RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint, int k,
                               List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> nearestFingerprints,
                               List<Double> nearestSqrDistances) throws IllegalArgumentException {
        findKNearestTo(fingerprint, mFingerprints, k, nearestFingerprints, nearestSqrDistances);
    }

    /**
     * Gets collection of fingerprints to match against.
     * @return collection of fingerprints to match against.
     */
    public Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> getFingerprints() {
        return mFingerprints;
    }

    /**
     * Finds nearest fingerprint to provided one, in terms of signal euclidean distances, within the collection of
     * provided fingerprints.
     * @param fingerprint fingerprint to find the nearest to.
     * @param fingerprints collection of fingerprints to make the search for the nearest one.
     * @return nearest fingerprint or null if none could be found.
     * @throws IllegalArgumentException if either fingerprint or collection of fingerprints is null.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point>
    RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>
            findNearestTo(RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint,
            Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> fingerprints)
            throws IllegalArgumentException {
        if (fingerprint == null || fingerprints == null) {
            throw new IllegalArgumentException();
        }

        double bestSqrDist = Double.MAX_VALUE;
        RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P> result = null;
        for(RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P> f : fingerprints) {
            double sqrDist = f.sqrDistanceTo(fingerprint);
            if (sqrDist < bestSqrDist) {
                bestSqrDist = sqrDist;
                result = f;
            }
        }

        return result;
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal euclidean distances, within the collection
     * of provided fingerprints.
     * @param fingerprint fingerprint to find the k-nearest ones to.
     * @param fingerprints collection of fingerprints to make the search for the nearest ones.
     * @param k number of nearest fingerprints to find.
     * @return nearest fingerprints ordered from closest to farthest or an empty list if none could be found.
     * @throws IllegalArgumentException if either fingerprint or collection of fingerprints is null, or k is less than
     * 1.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point> List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>>
            findKNearestTo(RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint,
            Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> fingerprints,
            int k) throws IllegalArgumentException {

        List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> result =
                new ArrayList<>();
        List<Double> nearestSqrDistances = new ArrayList<>();
        findKNearestTo(fingerprint, fingerprints, k, result, nearestSqrDistances);

        return result;
    }

    /**
     * Finds k-nearest fingerprints to provided one, in terms of signal euclidean distances, within the collection
     * of provided fingerprints.
     * @param fingerprint fingerprint to find the k-nearest ones to.
     * @param fingerprints collection of fingerprints ot make the search for the nearest ones.
     * @param k number of nearest fingerprints to find.
     * @param nearestFingerprints list where found nearest fingerprints will be stored ordered from closest to farthest
     *                            or an empty list if none could be found.
     * @param nearestSqrDistances list where squared signal euclidean distances corresponding to found fingerprints will
     *                            be stored or an empty list if no fingerprint is found.
     * @throws IllegalArgumentException if any parameter is null or k is less than 1.
     * @param <P> a {@link Point} type.
     */
    public static <P extends Point> void findKNearestTo(
            RssiFingerprint<WifiAccessPoint, RssiReading<WifiAccessPoint>> fingerprint,
            Collection<? extends RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> fingerprints,
            int k, List<RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P>> nearestFingerprints,
            List<Double> nearestSqrDistances) throws IllegalArgumentException {

        if (fingerprint == null || fingerprints == null || k < 1 || nearestFingerprints == null ||
                nearestSqrDistances == null) {
            throw new IllegalArgumentException();
        }

        nearestSqrDistances.clear();
        nearestFingerprints.clear();

        double maxSqrDist = Double.MAX_VALUE;
        for (RssiFingerprintLocated<WifiAccessPoint, RssiReading<WifiAccessPoint>, P> f : fingerprints) {
            double sqrDist = f.sqrDistanceTo(fingerprint);
            if (sqrDist < maxSqrDist || nearestSqrDistances.size() < k) {

                //find insertion point
                int pos = -1;
                int i = 0;
                for(Double sd : nearestSqrDistances) {
                    if (sqrDist < sd) {
                        //insertion point found
                        pos = i;
                        break;
                    }
                    i++;
                }

                if (pos >= 0) {
                    nearestSqrDistances.add(pos, sqrDist);
                    nearestFingerprints.add(pos, f);
                } else {
                    nearestSqrDistances.add(sqrDist);
                    nearestFingerprints.add(f);
                }

                //remove results exceeding required number of k neighbours to be found
                if (nearestFingerprints.size() > k) {
                    nearestSqrDistances.remove(k);
                    nearestFingerprints.remove(k);
                }

                //update maxSqrDist to the largest squared distance value contained in result list distances
                maxSqrDist = nearestSqrDistances.get(nearestSqrDistances.size() - 1);
            }
        }
    }
}
