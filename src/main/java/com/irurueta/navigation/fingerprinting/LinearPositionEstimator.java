package com.irurueta.navigation.fingerprinting;

import com.irurueta.geometry.Point;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.trilateration.LinearLeastSquaresTrilaterationSolver;
import com.irurueta.navigation.trilateration.TrilaterationException;
import com.irurueta.navigation.trilateration.TrilaterationSolver;
import com.irurueta.navigation.trilateration.TrilaterationSolverListener;

import java.util.ArrayList;
import java.util.List;

/**
 * Linearly estimates position using located radio sources and their readings at unknown
 * locations.
 * These kind of estimators can be used to determine the position of a given device by
 * getting readings at an unknown location of different radio source whose locations are
 * known.
 * @param <P> a {@link Point} type.
 */
@SuppressWarnings("WeakerAccess")
public abstract class LinearPositionEstimator<P extends Point> extends PositionEstimator<P> {

    /**
     * A linear trilateration solver to solve position.
     */
    protected LinearLeastSquaresTrilaterationSolver<P> mTrilaterationSolver;

    /**
     * Listener for the trilateration solver.
     */
    protected TrilaterationSolverListener<P> mTrilaterationSolverListener;

    /**
     * Constructor.
     */
    public LinearPositionEstimator() { }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided sources is null or the number of provided sources is less
     * than the required minimum.
     */
    public LinearPositionEstimator(List<? extends RadioSourceLocated<P>> sources)
            throws IllegalArgumentException {
        super(sources);
        init();
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearPositionEstimator(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        super(fingerprint);
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or the number of provided
     * sources is less than the required minimum.
     */
    public LinearPositionEstimator(List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException{
        super(sources, fingerprint);
        init();
    }

    /**
     * Constructor.
     * @param listener listener in charge of handling events.
     */
    public LinearPositionEstimator(PositionEstimatorListener<P> listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided sources is null or the number of provided sources is less
     * than the required minimum.
     */
    public LinearPositionEstimator(List<? extends RadioSourceLocated<P>> sources,
            PositionEstimatorListener<P> listener) throws IllegalArgumentException {
        super(sources, listener);
        init();
    }

    /**
     * Constructor.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if provided fingerprint is null.
     */
    public LinearPositionEstimator(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            PositionEstimatorListener<P> listener) throws IllegalArgumentException {
        super(fingerprint, listener);
        init();
    }

    /**
     * Constructor.
     * @param sources located radio sources used for trilateration.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @param listener listener in charge of handling events.
     * @throws IllegalArgumentException if either provided sources or fingerprint is null or the number of provided
     * sources is less than the required minimum.
     */
    public LinearPositionEstimator(List<? extends RadioSourceLocated<P>> sources,
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint,
            PositionEstimatorListener<P> listener) throws IllegalArgumentException {
        super(sources, fingerprint, listener);
        init();
    }

    /**
     * Gets minimum required number of located radio sources to perform trilateration.
     * @return minimum required number of located radio sources to perform trilateration.
     */
    @Override
    public int getMinRequiredSources() {
        return mTrilaterationSolver.getMinRequiredPositionsAndDistances();
    }

    /**
     * Indicates whether estimator is ready to find a solution.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mTrilaterationSolver.isReady();
    }

    /**
     * Returns boolean indicating whether this estimator is locked because an estimation is already in progress.
     * @return true if estimator is locked, false otherwise.
     */
    @Override
    public boolean isLocked() {
        return mTrilaterationSolver.isLocked();
    }

    /**
     * Estimates position based on provided located radio sources and readings of such radio sources at
     * an unknown location.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws PositionEstimationException if estimation fails for some other reason.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException,
            PositionEstimationException {
        try {
            mTrilaterationSolver.solve();
            mEstimatedPositionCoordinates =
                    mTrilaterationSolver.getEstimatedPositionCoordinates();
        } catch (TrilaterationException e) {
            throw new PositionEstimationException(e);
        }
    }

    /**
     * Internally sets located radio sources used for trilateration.
     * @param sources located radio sources used for trilateration.
     * @throws IllegalArgumentException if provided value is null or the number of provided sources is less
     * than the required minimum.
     */
    protected void internalSetSources(List<? extends RadioSourceLocated<P>> sources)
            throws IllegalArgumentException {
        super.internalSetSources(sources);
        buildPositionsAndDistances();
    }

    /**
     * Internally sets fingerprint containing readings at an unknown location for provided located radio sources.
     * @param fingerprint fingerprint containing readings at an unknown location for provided located radio sources.
     * @throws IllegalArgumentException if provided value is null.
     */
    protected void internalSetFingerprint(
            Fingerprint<? extends RadioSource, ? extends Reading<? extends RadioSource>> fingerprint)
            throws IllegalArgumentException {
        super.internalSetFingerprint(fingerprint);
        buildPositionsAndDistances();
    }

    /**
     * Sets positions and distances on internal trilateration solver.
     * @param positions positions to be set.
     * @param distances distances to be set.
     */
    protected abstract void setPositionsAndDistances(List<P> positions,
            List<Double> distances);

    /**
     * Initializes trilateration solver listener.
     */
    private void init() {
        mTrilaterationSolverListener = new TrilaterationSolverListener<P>() {
            @Override
            public void onSolveStart(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateStart(LinearPositionEstimator.this);
                }
            }

            @Override
            public void onSolveEnd(TrilaterationSolver<P> solver) {
                if (mListener != null) {
                    mListener.onEstimateEnd(LinearPositionEstimator.this);
                }
            }
        };
    }

    /**
     * Build positions and distances for the internal trilateration solver.
     */
    @SuppressWarnings("unchecked")
    private void buildPositionsAndDistances() {
        if (mTrilaterationSolver == null) {
            return;
        }

        int min = getMinRequiredSources();
        if (mSources == null || mFingerprint == null ||
                mSources.size() < min ||
                mFingerprint.getReadings() == null ||
                mFingerprint.getReadings().size() < min) {
            return;
        }

        List<? extends Reading<? extends RadioSource>> readings = mFingerprint.getReadings();
        List<P> positions = new ArrayList<>();
        List<Double> distances = new ArrayList<>();
        for (Reading<? extends RadioSource> reading : readings) {
            //noinspection all
            int index = mSources.indexOf(reading.getSource());
            if (index >= 0) {
                RadioSourceLocated<P> locatedSource = mSources.get(index);
                P position = locatedSource.getPosition();

                //compute distance
                Double distance1 = null, distance2 = null;
                switch (reading.getType()) {
                    case RANGING_READING:
                        distance1 = computeDistanceRanging((RangingReading<? extends RadioSource>)reading);
                        break;
                    case RSSI_READING:
                        distance1 =  computeDistanceRssi(locatedSource,
                                (RssiReading<? extends RadioSource>)reading);
                        break;
                    case RANGING_AND_RSSI_READING:
                        //in this case two positions and distance might be added to
                        //the trilateration solver
                        distance1 = computeDistanceRanging(
                                (RangingAndRssiReading<? extends RadioSource>)reading);
                        distance2 =  computeDistanceRssi(locatedSource,
                                (RangingAndRssiReading<? extends RadioSource>)reading);
                }

                if (position != null) {
                    if (distance1 != null) {
                        positions.add(position);
                        distances.add(distance1);
                    }
                    if (distance2 != null) {
                        positions.add(position);
                        distances.add(distance2);
                    }
                }
            }
        }

        setPositionsAndDistances(positions, distances);
    }

    /**
     * Obtain distance for a ranging reading.
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private Double computeDistanceRanging(RangingReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtain distance for a ranging reading.
     * @param reading a ranging reading.
     * @return distance to reading source or null if not available.
     */
    private Double computeDistanceRanging(RangingAndRssiReading<? extends RadioSource> reading) {
        return reading.getDistance();
    }

    /**
     * Obtain distance for an RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param reading an RSSI reading.
     * @return estimated distance or null if not available.
     */
    private Double computeDistanceRssi(RadioSourceLocated<P> locatedSource,
            RssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtain distance for an RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param reading a randing and RSSI reading.
     * @return estimated distance or null if not available.
     */
    private Double computeDistanceRssi(RadioSourceLocated<P> locatedSource,
            RangingAndRssiReading<? extends RadioSource> reading) {
        return computeDistanceRssi(locatedSource, reading.getRssi());
    }

    /**
     * Obtain distance for an RSSI reading.
     * @param locatedSource a located source, that must also have power information.
     * @param rxPower received power expressed in dBm's.
     * @return estimated distance or null if not available.
     */
    private Double computeDistanceRssi(RadioSourceLocated<P> locatedSource,
                                       double rxPower) {
        if(!(locatedSource instanceof RadioSourceWithPower)) {
            return null;
        }

        RadioSourceWithPower poweredSource = (RadioSourceWithPower)locatedSource;

        //source related parameters:

        //transmitted power in dBm's
        double txPower = poweredSource.getTransmittedPower();

        //path loss exponent
        double pathLossExponent = poweredSource.getPathLossExponent();

        double frequency = poweredSource.getFrequency();
        double k = RssiRadioSourceEstimator.SPEED_OF_LIGHT / (4.0 * Math.PI * frequency);
        double kdB = 10.0 * Math.log10(k);


        //received power in dBm's follows the equation:
        //rxPower = pathLossExponent * kdB + txPower - 5.0 * pathLossExponent * logSqrDistance

        //hence:
        //5.0 * pathLossExponent * logSqrDistance = pathLossExponent * kdB + txPower - rxPower

        double logSqrDistance = (pathLossExponent * kdB + txPower - rxPower) / (5.0 * pathLossExponent);

        //where logSqrDistance = Math.log10(sqrDistance)
        //and sqrDistance = distance * distance, hence
        //logSqrDistance = Math.log10(distance * distance) = 2 * Math.log10(distance)

        return Math.pow(10.0,logSqrDistance / 2.0);
    }
}
