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
package com.irurueta.navigation.trilateration;

import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;

import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly solve the trilateration problem by
 * finding the best pairs of 3D positions and distances among the provided
 * ones.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("WeakerAccess")
public abstract class RobustTrilateration3DSolver extends RobustTrilaterationSolver<Point3D> {

    /**
     * Linear trilateration solver internally used by a robust algorithm.
     */
    protected LinearLeastSquaresTrilateration3DSolver mLinearSolver =
            new LinearLeastSquaresTrilateration3DSolver();

    /**
     * Non linear trilateration solver internally used to refine solution
     * found by robust algorithm.
     */
    protected NonLinearLeastSquaresTrilateration3DSolver mNonLinearSolver =
            new NonLinearLeastSquaresTrilateration3DSolver();

    /**
     * Positions for linear inner solver used during robust estimation.
     */
    protected Point3D[] mInnerPositions;

    /**
     * Distances for linear inner solver used during robust estimation.
     */
    protected double[] mInnerDistances;

    /**
     * Constructor.
     */
    public RobustTrilateration3DSolver() {
        init();
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public RobustTrilateration3DSolver(
            RobustTrilaterationSolverListener<Point3D> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
        init();
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length is smaller
     * than required (4 points).
     */
    public RobustTrilateration3DSolver(Point3D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
        init();
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public RobustTrilateration3DSolver(Sphere[] spheres) throws IllegalArgumentException {
        this();
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     * than required (4 points) or don't have the same length.
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        this();
        internalSetSpheresAndStandardDeviations(spheres,
                distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null or if length of spheres array
     * is less than required (4 points).
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetSpheres(spheres);
    }

    /**
     * Constructor.
     * @param spheres spheres defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if spheres is null, length of spheres array is less
     * than required (4 points) or don't have the same length.
     */
    public RobustTrilateration3DSolver(Sphere[] spheres,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point3D> listener)
            throws IllegalArgumentException {
        this(listener);
        internalSetSpheresAndStandardDeviations(spheres,
                distanceStandardDeviations);
    }

    /**
     * Gets number of dimensions of provided points.
     * @return always returns 2 dimensions.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Minimum required number of positions and distances.
     * At least 3 positions will be required.
     * @return minimum required number of positions and distances.
     */
    @Override
    public int getMinRequiredPositionsAndDistances() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1;
    }

    /**
     * Gets spheres defined by provided positions and distances.
     * @return spheres defined by provided positions and distances.
     */
    public Sphere[] getSpheres() {
        if (mPositions == null) {
            return null;
        }

        Sphere[] result = new Sphere[mPositions.length];

        for (int i = 0; i < mPositions.length; i++) {
            result[i] = new Sphere(mPositions[i], mDistances[i]);
        }
        return result;
    }

    /**
     * Sets spheres defining positions and euclidean distances.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if sphere is null or length of array of spheres
     * is less than 4.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setSpheres(Sphere[] spheres) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheres(spheres);
    }

    /**
     * Sets spheres defining positions and euclidean distances along with the standard
     * deviations of provided spheres radii.
     * @param spheres spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     * 4 or don' thave the same length.
     * @throws LockedException if instance is busy solving the trilateration problem.
     */
    public void setSpheresAndStandardDeviations(Sphere[] spheres, double[] radiusStandardDeviations)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSpheresAndStandardDeviations(spheres, radiusStandardDeviations);
    }

    /**
     * Attempts to refine estimated position if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     * @param position position estimated by a robust estimator without refinement.
     * @return solution after refinement (if requested) or provided non-refined
     * estimated position if not requested or refinement failed.
     */
    protected Point3D attemptRefine(Point3D position) {
        if (mRefineResult && mInliersData != null) {
            BitSet inliers = mInliersData.getInliers();
            int nSamples = mDistances.length;
            int nInliers = mInliersData.getNumInliers();
            Point3D[] inlierPositions = new Point3D[nInliers];
            double[] inlierDistances = new double[nInliers];
            double[] inlierStandardDeviations = null;
            if (mDistanceStandardDeviations != null) {
                inlierStandardDeviations = new double[nInliers];
            }
            int pos = 0;
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    //sample is inlier
                    inlierPositions[pos] = mPositions[i];
                    inlierDistances[pos] = mDistances[i];
                    if (inlierStandardDeviations != null) {
                        inlierStandardDeviations[pos] = mDistanceStandardDeviations[i];
                    }
                    pos++;
                }
            }

            try {
                mNonLinearSolver.setInitialPosition(position);
                if (inlierStandardDeviations != null) {
                    mNonLinearSolver.setPositionsDistancesAndStandardDeviations(
                            inlierPositions, inlierDistances, inlierStandardDeviations);
                } else {
                    mNonLinearSolver.setPositionsAndDistances(
                            inlierPositions, inlierDistances);
                }
                mNonLinearSolver.solve();

                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = mNonLinearSolver.getCovariance();
                }

                return mEstimatedPosition = mNonLinearSolver.getEstimatedPosition();
            } catch (Exception e) {
                //refinement failed, so we return input value
                return mEstimatedPosition = position;
            }
        } else {
            return mEstimatedPosition = position;
        }
    }

    /**
     * Solves a preliminar solution for a subset of samples picked by a robust estimator.
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions list where estimated preliminar solution will be stored.
     */
    protected void solvePreliminarSolutions(int[] samplesIndices, List<Point3D> solutions) {
        try {
            int length = samplesIndices.length;
            int index;
            for (int i = 0; i < length; i++) {
                index = samplesIndices[i];
                mInnerPositions[i] = mPositions[index];
                mInnerDistances[i] = mDistances[index];
            }

            mLinearSolver.setPositionsAndDistances(mInnerPositions, mInnerDistances);
            mLinearSolver.solve();
            solutions.add(mLinearSolver.getEstimatedPosition());
        } catch (NavigationException ignore) {
            //if anything fails, no solution is added
        }
    }

    /**
     * Internally sets spheres defining positions and euclidean distances.
     * @param spheres spheres defining positions and distances.
     * @throws IllegalArgumentException if spheres is null or length of array of spheres
     * is less than {@link #getMinRequiredPositionsAndDistances()}
     */
    private void internalSetSpheres(Sphere[] spheres) throws IllegalArgumentException {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        Point3D[] positions = new Point3D[spheres.length];
        double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsAndDistances(positions, distances);
    }

    /**
     * Internally sets spheres defining positions and euclidean distances along
     * with the standard deviations of provided spheres radii.
     * @param spheres spheres defining positions and distances.
     * @param radiusStandardDeviations standard deviations of spheres radii.
     * @throws IllegalArgumentException if spheres is null, length of arrays is less than
     * 4 or don't have the same length.
     */
    private void internalSetSpheresAndStandardDeviations(Sphere[] spheres,
            double[] radiusStandardDeviations) throws IllegalArgumentException {
        if (spheres == null || spheres.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations == null) {
            throw new IllegalArgumentException();
        }

        if (radiusStandardDeviations.length != spheres.length) {
            throw new IllegalArgumentException();
        }

        Point3D[] positions = new Point3D[spheres.length];
        double[] distances = new double[spheres.length];
        for (int i = 0; i < spheres.length; i++) {
            Sphere sphere = spheres[i];
            positions[i] = sphere.getCenter();
            distances[i] = sphere.getRadius();
        }

        internalSetPositionsDistancesAndStandardDeviations(positions, distances,
                radiusStandardDeviations);
    }

    /**
     * Setup inner positions and distances.
     */
    private void init() {
        int points = getMinRequiredPositionsAndDistances();
        mInnerPositions = new Point3D[points];
        mInnerDistances = new double[points];

        mLinearSolver = new LinearLeastSquaresTrilateration3DSolver();
        mNonLinearSolver = new NonLinearLeastSquaresTrilateration3DSolver();
    }
}
