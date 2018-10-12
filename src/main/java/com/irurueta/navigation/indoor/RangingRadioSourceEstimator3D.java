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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Accuracy3D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.trilateration.LinearLeastSquaresTrilateration3DSolver;
import com.irurueta.navigation.trilateration.NonLinearLeastSquaresTrilateration3DSolver;

import java.util.List;

/**
 * Estimates position of a radio source (e.g. WiFi access point or bluetooth beacon)
 * by using ranging measurements.
 * Ranging measurements can be obtained by protocols such as ieee 802.11mc (WiFi RTT) which
 * measures travel time of signal and converts the result into distances by taking into
 * account the speed of light as the propagation speed.
 *
 * @param <S> a {@link RadioSource} type.
 */
public class RangingRadioSourceEstimator3D<S extends RadioSource> extends
        RangingRadioSourceEstimator<S, Point3D> {

    /**
     * Constructor.
     */
    public RangingRadioSourceEstimator3D() {
        super();
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings)
            throws IllegalArgumentException {
        super(readings);
    }

    /**
     * Constructor.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RangingRadioSourceEstimator3D(
            RangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            RangingRadioSourceEstimatorListener<S, Point3D> listener)
            throws IllegalArgumentException {
        super(readings, listener);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation or radio
     *                        source position.
     */
    public RangingRadioSourceEstimator3D(Point3D initialPosition) {
        super(initialPosition);
    }

    /**
     * Constructor.
     * Sets radio signal readings belonging to the same radio source.
     * @param readings radio signal readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition) throws IllegalArgumentException {
        super(readings, initialPosition);
    }

    /**
     * Constructor.
     * @param initialPosition initial position to start the estimation of radio
     *                        source position.
     * @param listener listener in charge of attending events raised by this instance.
     */
    public RangingRadioSourceEstimator3D(Point3D initialPosition,
            RangingRadioSourceEstimatorListener<S, Point3D> listener) {
        super(initialPosition, listener);
    }

    /**
     * Constructor.
     * Sets radio signal ranging readings belonging to the same radio source.
     * @param readings radio signal ranging readings belonging to the same radio source.
     * @param initialPosition initial position to start the estimation of radio source
     *                        position.
     * @param listener listener in charge of attending events raised by this instance.
     * @throws IllegalArgumentException if readings are not valid.
     */
    public RangingRadioSourceEstimator3D(
            List<? extends RangingReadingLocated<S, Point3D>> readings,
            Point3D initialPosition,
            RangingRadioSourceEstimatorListener<S, Point3D> listener)
            throws IllegalArgumentException {
        super(readings, initialPosition, listener);
    }

    /**
     * Gets minimum required number of readings to estimate position of radio source,
     * which is 4 readings.
     * @return minimum required number of readings.
     */
    @Override
    public int getMinReadings() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets number of dimensions of position points.
     * This is always 3.
     * @return number of dimensions of position points.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Gets estimated radio source 3D position.
     * @return estimated radio source 3D position.
     */
    @Override
    public Point3D getEstimatedPosition() {
        if (mEstimatedPositionCoordinates == null) {
            return null;
        }

        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        getEstimatedPosition(result);
        return result;
    }

    /**
     * Gets estimated located radio source.
     * @return estimated located radio source or null.
     */
    @Override
    public RadioSourceLocated<Point3D> getEstimatedRadioSource() {
        List<? extends RangingReadingLocated<S, Point3D>> readings = getReadings();
        if (readings == null || readings.isEmpty()) {
            return null;
        }
        S source = readings.get(0).getSource();

        Point3D estimatedPosition = getEstimatedPosition();
        if (estimatedPosition == null) {
            return null;
        }

        Matrix estimatedPositionCovariance = getEstimatedPositionCovariance();

        if (source instanceof WifiAccessPoint) {
            WifiAccessPoint accessPoint = (WifiAccessPoint)source;
            return new WifiAccessPointLocated3D(accessPoint.getBssid(),
                    accessPoint.getFrequency(), accessPoint.getSsid(),
                    estimatedPosition, estimatedPositionCovariance);
        } else if (source instanceof Beacon) {
            Beacon beacon = (Beacon)source;
            return new BeaconLocated3D(beacon.getIdentifiers(),
                    beacon.getTransmittedPower(), beacon.getFrequency(),
                    beacon.getBluetoothAddress(), beacon.getBeaconTypeCode(),
                    beacon.getManufacturer(), beacon.getServiceUuid(),
                    beacon.getBluetoothName(), estimatedPosition,
                    estimatedPositionCovariance);
        } else {
            return null;
        }
    }

    /**
     * Builds an instance of a linear trilateration solver if needed.
     */
    @Override
    protected void buildLinearSolverIfNeeded() {
        if ((mInitialPosition == null || !mNonLinearSolverEnabled) &&
                mLinearSolver == null) {
            mLinearSolver = new LinearLeastSquaresTrilateration3DSolver();
        }
    }

    /**
     * Builds an instance of a non-linear trilateration solver if needed.
     */
    @Override
    protected void buildNonLinearSolverIfNeeded() {
        if (mNonLinearSolver == null && mNonLinearSolverEnabled) {
            mNonLinearSolver = new NonLinearLeastSquaresTrilateration3DSolver();
        }
    }

    /**
     * Build an instance of accuracy if needed.
     */
    @Override
    protected void buildAccuracyIfNeeded() {
        if (mAccuracy == null && mUseReadingPositionCovariances) {
            mAccuracy = new Accuracy3D();

            //to work with standard deviations, we need a unitary factor
            mAccuracy.setStandardDeviationFactor(1.0);
        }
    }

    /**
     * Sets positions, distances and standard deviations of distances on internal
     * trilateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     * @param distanceStandardDeviations standard deviations of distances to be set or
     *                                   null.
     * @throws LockedException if solvers are locked.
     */
    @Override
    protected void setPositionsDistancesAndDistanceStandardDeviations(
            List<Point3D> positions, List<Double> distances,
            List<Double> distanceStandardDeviations) throws LockedException  {

        int size = positions.size();
        Point3D[] positionsArray = new InhomogeneousPoint3D[size];
        positionsArray = positions.toArray(positionsArray);

        double[] distancesArray = new double[size];
        double[] distanceStandardDeviationsArray = new double[size];
        for (int i = 0; i < size; i++) {
            distancesArray[i] = distances.get(i);
            distanceStandardDeviationsArray[i] = distanceStandardDeviations.get(i);
        }

        if (mLinearSolver != null &&
                (mInitialPosition == null || !mNonLinearSolverEnabled)) {
            mLinearSolver.setPositionsAndDistances(positionsArray, distancesArray);
        }
        if (mNonLinearSolver != null && mNonLinearSolverEnabled) {
            mNonLinearSolver.setPositionsDistancesAndStandardDeviations(positionsArray, distancesArray,
                    distanceStandardDeviationsArray);
        }
    }
}
