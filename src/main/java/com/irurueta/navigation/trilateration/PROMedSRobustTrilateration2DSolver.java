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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Robustly solves the trilateration problem by finding the best pairs of 2D
 * positions and distances among the provided ones using PROMedS algorithm to
 * discard outliers.
 */
public class PROMedSRobustTrilateration2DSolver extends RobustTrilateration2DSolver {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-5;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustTrilateration2DSolver() {
        super();
    }

    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSRobustTrilateration2DSolver(
            RobustTrilaterationSolverListener<Point2D> listener) {
        super(listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length of their length is smaller than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Point2D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation stats,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when estimation stats,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions or distances are null,
     * don't have the same length or their length is smaller than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Point2D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if circles is null or if length or circles array
     * is less than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Circle[] circles)
            throws IllegalArgumentException {
        super(circles);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     * than required (3 points) or don't have the same length.
     */
    public PROMedSRobustTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations) throws IllegalArgumentException {
        super(circles, distanceStandardDeviations);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null or if length of circles array
     * is less than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(Circle[] circles,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(circles, listener);
    }

    /**
     * Constructor.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if circles is null, length of circles array is less
     * than required (3 points) or don't have the same length.
     */
    public PROMedSRobustTrilateration2DSolver(Circle[] circles,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(circles, distanceStandardDeviations, listener);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @throws IllegalArgumentException if quality scores is null, length
     * of quality scores is less than required minimum (3 samples).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores)
            throws IllegalArgumentException {
        super();
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if quality scores is null, length
     * of quality scores is less than required minimum (3 samples).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @throws IllegalArgumentException if either positions, distances or quality
     * scores are null, don't have the same length of their length is smaller
     * than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Point2D[] positions, double[] distances)
            throws IllegalArgumentException {
        super(positions, distances);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node to be
     *                  estimated.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either positions, distances, quality scores or
     * standard deviations are null, don't have the same length or their length is
     * smaller than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances or
     * standard deviations are null, don't have the same length or their length is smaller
     * than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Point2D[] positions, double[] distances,
            double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, distanceStandardDeviations, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param positions known positions of static nodes.
     * @param distances euclidean distances from static nodes to mobile node.
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if either positions, distances,
     * quality scores or standard deviations are null, don't have the same
     * length or their length is smaller than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Point2D[] positions, double[] distances,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(positions, distances, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param circles circles defining positions and distances.
     * @throws IllegalArgumentException if either circles or quality scores
     * are null don't have the same length or their length is less than
     * required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Circle[] circles) throws IllegalArgumentException {
        super(circles);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @throws IllegalArgumentException if either circles, quality scores or
     * standard deviations are null, don't have the same length or their
     * length is less than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations)
            throws IllegalArgumentException {
        super(circles, distanceStandardDeviations);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param circles circles defining positions and distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either circles or quality scores
     * are null, don't have the same length or their length is less than
     * required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Circle[] circles,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(circles, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided
     *                     sample. The larger the score value the better
     *                      the quality of the sample.
     * @param circles circles defining positions and distances.
     * @param distanceStandardDeviations standard deviations of provided measured distances.
     * @param listener listener to be notified of events such as when estimation starts,
     *                 ends or its progress significantly changes.
     * @throws IllegalArgumentException if either circles, quality scores
     * or standard deviations are null, don't have the same length or their
     * length is less than required (3 points).
     */
    public PROMedSRobustTrilateration2DSolver(double[] qualityScores,
            Circle[] circles, double[] distanceStandardDeviations,
            RobustTrilaterationSolverListener<Point2D> listener)
            throws IllegalArgumentException {
        super(circles, distanceStandardDeviations, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algrithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if this solver is locked.
     */
    public void setStopThreshold(double stopThreshold)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        mStopThreshold = stopThreshold;
    }

    /**
     * Returns quality scores corresponding to each pair of
     * positions and distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }

    /**
     * Sets quality scores corresponding to each pair of positions and
     * distances (i.e. sample).
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than minimum required samples.
     * @throws LockedException if robust solver is locked because an
     * estimation is already in progress.
     */
    public void setQualityScores(double[] qualityScores)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null &&
                mQualityScores.length == mDistances.length;
    }

    /**
     * Solves the trilateration problem.
     * @return estimated position.
     * @throws LockedException if instance is busy solving the trilateration problem.
     * @throws NotReadyException is solver is not ready.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point2D solve() throws LockedException, NotReadyException,
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        PROMedSRobustEstimator<Point2D> innerEstimator =
                new PROMedSRobustEstimator<>(new PROMedSRobustEstimatorListener<Point2D>() {
                    @Override
                    public double[] getQualityScores() {
                        return mQualityScores;
                    }

                    @Override
                    public double getThreshold() {
                        return mStopThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return mDistances.length;
                    }

                    @Override
                    public int getSubsetSize() {
                        return getMinRequiredPositionsAndDistances();
                    }

                    @Override
                    public void estimatePreliminarSolutions(int[] samplesIndices, List<Point2D> solutions) {
                        solvePreliminarSolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(Point2D currentEstimation, int i) {
                        return Math.abs(currentEstimation.distanceTo(mPositions[i]) - mDistances[i]);
                    }

                    @Override
                    public boolean isReady() {
                        return PROMedSRobustTrilateration2DSolver.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(RobustEstimator<Point2D> estimator) {
                        if (mListener != null) {
                            mListener.onSolveStart(PROMedSRobustTrilateration2DSolver.this);
                        }
                    }

                    @Override
                    public void onEstimateEnd(RobustEstimator<Point2D> estimator) {
                        if (mListener != null) {
                            mListener.onSolveEnd(PROMedSRobustTrilateration2DSolver.this);
                        }
                    }

                    @Override
                    public void onEstimateNextIteration(RobustEstimator<Point2D> estimator, int iteration) {
                        if (mListener != null) {
                            mListener.onSolveNextIteration(PROMedSRobustTrilateration2DSolver.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(RobustEstimator<Point2D> estimator, float progress) {
                        if (mListener != null) {
                            mListener.onSolveProgressChange(PROMedSRobustTrilateration2DSolver.this,
                                    progress);
                        }
                    }
                });

        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            Point2D result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMedS;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     * is smaller than 3 samples.
     */
    private void internalSetQualityScores(double[] qualityScores)
            throws IllegalArgumentException {
        if (qualityScores == null ||
                qualityScores.length < getMinRequiredPositionsAndDistances()) {
            throw new IllegalArgumentException();
        }

        mQualityScores = qualityScores;
    }
}
