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
package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.units.*;

import java.io.Serializable;
import java.util.Objects;

/**
 * Contains GNSS configuration parameters.
 */
public class GNSSConfig implements Serializable, Cloneable {

    public static final int MIN_SATELLITES = 4;

    /**
     * Minimum interval between GNSS epochs expressed in seconds (s).
     */
    private double mEpochInterval;

    /**
     * X coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     */
    private double mInitialEstimatedEcefPositionX;

    /**
     * Y coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     */
    private double mInitialEstimatedEcefPositionY;

    /**
     * Z coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     */
    private double mInitialEstimatedEcefPositionZ;

    /**
     * Number of satellites in constellation.
     */
    private int mNumberOfSatellites = MIN_SATELLITES;

    /**
     * Orbital radius of satellites expressed in meters (m).
     */
    private double mOrbitalRadiusOfSatellites;

    /**
     * Inclination angle of satellites expressed in degrees (deg).
     */
    private double mSatellitesInclinationDegrees;

    /**
     * Longitude offset of constellation expressed in degrees (deg).
     */
    private double mConstellationLongitudeOffsetDegrees;

    /**
     * Timing offset of constellation expressed in seconds (s).
     */
    private double mConstellationTimingOffset;

    /**
     * Mask angle expressed in degrees (deg).
     */
    private double mMaskAngleDegrees;

    /**
     * Signal In Space (SIS) error Standard Deviation (SD) expressed in meters (m).
     */
    private double mSISErrorSD;

    /**
     * Zenith ionosphere error Standard Deviation (SD) expressed in meters (m).
     */
    private double mZenithIonosphereErrorSD;

    /**
     * Zenith troposphere error Standard Deviation (SD) expressed in meters (m).
     */
    private double mZenithTroposphereErrorSD;

    /**
     * Code tracking error Standard Deviation (SD) expressed in meters per second (m/s).
     */
    private double mCodeTrackingErrorSD;

    /**
     * Range rate tracking error Standard Deviation (SD) expressed in meters per second (m/s).
     */
    private double mRangeRateTrackingErrorSD;

    /**
     * Initial receiver clock offset at time = 0 expressed in meters (m).
     */
    private double mInitialReceiverClockOffset;

    /**
     * Initial receiver clock drift at time = 0 expressed in meters per second (m/s).
     */
    private double mInitialReceiverClockDrift;

    /**
     * Constructor.
     */
    public GNSSConfig() {
    }

    /**
     * Constructor.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedEcefPositionX       x coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param initialEstimatedEcefPositionY       y coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param initialEstimatedEcefPositionZ       z coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param numberOfSatellites                  number of satellites in constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final double epochInterval,
                      final double initialEstimatedEcefPositionX,
                      final double initialEstimatedEcefPositionY,
                      final double initialEstimatedEcefPositionZ,
                      final int numberOfSatellites,
                      final double orbitalRadiusOfSatellites,
                      final double satellitesInclinationDegrees,
                      final double constellationLongitudeOffsetDegrees,
                      final double constellationTimingOffset,
                      final double maskAngleDegrees,
                      final double sisErrorSD,
                      final double zenithIonosphereErrorSD,
                      final double zenithTroposphereErrorSD,
                      final double codeTrackingErrorSD,
                      final double rangeRateTrackingErrorSD,
                      final double initialReceiverClockOffset,
                      final double initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Constructor
     *
     * @param epochInterval                 minimum interval between GNSS epochs.
     * @param initialEstimatedEcefPositionX x coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param initialEstimatedEcefPositionY y coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param initialEstimatedEcefPositionZ z coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param numberOfSatellites            number of satellites in constellation.
     * @param orbitalRadiusOfSatellites     orbital radius of satellites.
     * @param satellitesInclination         inclination angle of satellites.
     * @param constellationLongitudeOffset  longitude offset of constellation.
     * @param constellationTimingOffset     timing offset of constellation.
     * @param maskAngle                     mask angle.
     * @param sisErrorSD                    Signal In Space (SIS) error Standard
     *                                      Deviation (SD).
     * @param zenithIonosphereErrorSD       zenith ionosphere error Standard
     *                                      Deviation (SD).
     * @param zenithTroposphereErrorSD      zenith troposphere error Standard
     *                                      Deviation (SD).
     * @param codeTrackingErrorSD           code tracking error Standard Deviation
     *                                      (SD).
     * @param rangeRateTrackingErrorSD      range rate tracking error Standard
     *                                      Deviation (SD).
     * @param initialReceiverClockOffset    initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift     initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final Time epochInterval,
                      final Distance initialEstimatedEcefPositionX,
                      final Distance initialEstimatedEcefPositionY,
                      final Distance initialEstimatedEcefPositionZ,
                      final int numberOfSatellites,
                      final Distance orbitalRadiusOfSatellites,
                      final Angle satellitesInclination,
                      final Angle constellationLongitudeOffset,
                      final Time constellationTimingOffset,
                      final Angle maskAngle,
                      final Distance sisErrorSD,
                      final Distance zenithIonosphereErrorSD,
                      final Distance zenithTroposphereErrorSD,
                      final Speed codeTrackingErrorSD,
                      final Speed rangeRateTrackingErrorSD,
                      final Distance initialReceiverClockOffset,
                      final Speed initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedEcefPosition        initial estimated position
     *                                            resolved along ECEF axes.
     * @param numberOfSatellites                  number of satellites in
     *                                            constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final double epochInterval,
                      final ECEFPosition initialEstimatedEcefPosition,
                      final int numberOfSatellites,
                      final double orbitalRadiusOfSatellites,
                      final double satellitesInclinationDegrees,
                      final double constellationLongitudeOffsetDegrees,
                      final double constellationTimingOffset,
                      final double maskAngleDegrees,
                      final double sisErrorSD,
                      final double zenithIonosphereErrorSD,
                      final double zenithTroposphereErrorSD,
                      final double codeTrackingErrorSD,
                      final double rangeRateTrackingErrorSD,
                      final double initialReceiverClockOffset,
                      final double initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param epochInterval                minimum interval between GNSS epochs.
     * @param initialEstimatedEcefPosition initial estimated position
     *                                     resolved along ECEF axes.
     * @param numberOfSatellites           number of satellites in constellation.
     * @param orbitalRadiusOfSatellites    orbital radius of satellites.
     * @param satellitesInclination        inclination angle of satellites.
     * @param constellationLongitudeOffset longitude offset of constellation.
     * @param constellationTimingOffset    timing offset of constellation.
     * @param maskAngle                    mask angle.
     * @param sisErrorSD                   Signal In Space (SIS) error Standard
     *                                     Deviation (SD).
     * @param zenithIonosphereErrorSD      zenith ionosphere error Standard
     *                                     Deviation (SD).
     * @param zenithTroposphereErrorSD     zenith troposphere error Standard
     *                                     Deviation (SD).
     * @param codeTrackingErrorSD          code tracking error Standard Deviation
     *                                     (SD).
     * @param rangeRateTrackingErrorSD     range rate tracking error Standard
     *                                     Deviation (SD).
     * @param initialReceiverClockOffset   initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift    initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final Time epochInterval,
                      final ECEFPosition initialEstimatedEcefPosition,
                      final int numberOfSatellites,
                      final Distance orbitalRadiusOfSatellites,
                      final Angle satellitesInclination,
                      final Angle constellationLongitudeOffset,
                      final Time constellationTimingOffset,
                      final Angle maskAngle,
                      final Distance sisErrorSD,
                      final Distance zenithIonosphereErrorSD,
                      final Distance zenithTroposphereErrorSD,
                      final Speed codeTrackingErrorSD,
                      final Speed rangeRateTrackingErrorSD,
                      final Distance initialReceiverClockOffset,
                      final Speed initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Constructor.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedPosition            initial estimated position
     *                                            resolved along ECEF axes and expressed
     *                                            in meters (m).
     * @param numberOfSatellites                  number of satellites in
     *                                            constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final double epochInterval,
                      final Point3D initialEstimatedPosition,
                      final int numberOfSatellites,
                      final double orbitalRadiusOfSatellites,
                      final double satellitesInclinationDegrees,
                      final double constellationLongitudeOffsetDegrees,
                      final double constellationTimingOffset,
                      final double maskAngleDegrees,
                      final double sisErrorSD,
                      final double zenithIonosphereErrorSD,
                      final double zenithTroposphereErrorSD,
                      final double codeTrackingErrorSD,
                      final double rangeRateTrackingErrorSD,
                      final double initialReceiverClockOffset,
                      final double initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                minimum interval between GNSS epochs.
     * @param initialEstimatedPosition     initial estimated position
     *                                     resolved along ECEF axes and expressed
     *                                     in meters (m).
     * @param numberOfSatellites           number of satellites in constellation.
     * @param orbitalRadiusOfSatellites    orbital radius of satellites.
     * @param satellitesInclination        inclination angle of satellites.
     * @param constellationLongitudeOffset longitude offset of constellation.
     * @param constellationTimingOffset    timing offset of constellation.
     * @param maskAngle                    mask angle.
     * @param sisErrorSD                   Signal In Space (SIS) error Standard
     *                                     Deviation (SD).
     * @param zenithIonosphereErrorSD      zenith ionosphere error Standard
     *                                     Deviation (SD).
     * @param zenithTroposphereErrorSD     zenith troposphere error Standard
     *                                     Deviation (SD).
     * @param codeTrackingErrorSD          code tracking error Standard Deviation
     *                                     (SD).
     * @param rangeRateTrackingErrorSD     range rate tracking error Standard
     *                                     Deviation (SD).
     * @param initialReceiverClockOffset   initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift    initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public GNSSConfig(final Time epochInterval,
                      final Point3D initialEstimatedPosition,
                      final int numberOfSatellites,
                      final Distance orbitalRadiusOfSatellites,
                      final Angle satellitesInclination,
                      final Angle constellationLongitudeOffset,
                      final Time constellationTimingOffset,
                      final Angle maskAngle,
                      final Distance sisErrorSD,
                      final Distance zenithIonosphereErrorSD,
                      final Distance zenithTroposphereErrorSD,
                      final Speed codeTrackingErrorSD,
                      final Speed rangeRateTrackingErrorSD,
                      final Distance initialReceiverClockOffset,
                      final Speed initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public GNSSConfig(final GNSSConfig input) {
        copyFrom(input);
    }

    /**
     * Gets minimum interval between GNSS epochs expressed in seconds (s).
     *
     * @return minimum interval between GNSS epochs.
     */
    public double getEpochInterval() {
        return mEpochInterval;
    }

    /**
     * Sets minimum interval between GNSS epochs expressed in seconds (s).
     *
     * @param epochInterval minimum interval between GNSS epochs.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setEpochInterval(final double epochInterval) {
        if (epochInterval < 0.0) {
            throw new IllegalArgumentException();
        }
        mEpochInterval = epochInterval;
    }

    /**
     * Gets minimum interval between GNSS epochs.
     *
     * @param result instance where minimum interval between GNSS epochs
     *               will be stored.
     */
    public void getEpochIntervalTime(final Time result) {
        result.setValue(mEpochInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets minimum interval between GNSS epochs.
     *
     * @return minimum interval between GNSS epochs.
     */
    public Time getEpochIntervalTime() {
        return new Time(mEpochInterval, TimeUnit.SECOND);
    }

    /**
     * Sets minimum interval between GNSS epochs.
     *
     * @param epochInterval minimum interval between GNSS epochs.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setEpochIntervalTime(final Time epochInterval) {
        setEpochInterval(TimeConverter.convert(
                epochInterval.getValue().doubleValue(),
                epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Gets x coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @return x coordinate of initial estimated position.
     */
    public double getInitialEstimatedEcefPositionX() {
        return mInitialEstimatedEcefPositionX;
    }

    /**
     * Sets x coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @param initialEstimatedEcefPositionX x coordinate of initial estimated position.
     */
    public void setInitialEstimatedEcefPositionX(
            final double initialEstimatedEcefPositionX) {
        mInitialEstimatedEcefPositionX = initialEstimatedEcefPositionX;
    }

    /**
     * Gets x coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param result instance where x coordinate of initial estimated position will
     *               be stored.
     */
    public void getInitialEstimatedEcefPositionXDistance(final Distance result) {
        result.setValue(mInitialEstimatedEcefPositionX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of initial estimated position resolved along ECEF axes.
     *
     * @return x coordinate of initial estimated position.
     */
    public Distance getInitialEstimatedEcefPositionXDistance() {
        return new Distance(mInitialEstimatedEcefPositionX,
                DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param initialEstimatedEcefPositionX x coordinate of initial estimated
     *                                      position.
     */
    public void setInitialEstimatedEcefPositionXDistance(
            final Distance initialEstimatedEcefPositionX) {
        mInitialEstimatedEcefPositionX = DistanceConverter.convert(
                initialEstimatedEcefPositionX.getValue().doubleValue(),
                initialEstimatedEcefPositionX.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @return x coordinate of initial estimated position.
     */
    public double getInitialEstimatedEcefPositionY() {
        return mInitialEstimatedEcefPositionY;
    }

    /**
     * Sets y coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @param initialEstimatedEcefPositionY y coordinate of initial estimated
     *                                      position.
     */
    public void setInitialEstimatedEcefPositionY(
            final double initialEstimatedEcefPositionY) {
        mInitialEstimatedEcefPositionY = initialEstimatedEcefPositionY;
    }

    /**
     * Gets y coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param result instance where y coordinate of initial estimated position will
     *               be stored.
     */
    public void getInitialEstimatedEcefPositionYDistance(final Distance result) {
        result.setValue(mInitialEstimatedEcefPositionY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of initial estimated position resolved along ECEF axes.
     *
     * @return y coordinate of initial estimated position.
     */
    public Distance getInitialEstimatedEcefPositionYDistance() {
        return new Distance(mInitialEstimatedEcefPositionY,
                DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param initialEstimatedEcefPositionY y coordinate of initial estimated
     *                                      position.
     */
    public void setInitialEstimatedEcefPositionYDistance(
            final Distance initialEstimatedEcefPositionY) {
        mInitialEstimatedEcefPositionY = DistanceConverter.convert(
                initialEstimatedEcefPositionY.getValue().doubleValue(),
                initialEstimatedEcefPositionY.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @return z coordinate of initial estimated position.
     */
    public double getInitialEstimatedEcefPositionZ() {
        return mInitialEstimatedEcefPositionZ;
    }

    /**
     * Sets z coordinate of initial estimated position resolved along ECEF axes and
     * expressed in meters (m).
     *
     * @param initialEstimatedEcefPositionZ z coordinate of initial estimated
     *                                      position.
     */
    public void setInitialEstimatedEcefPositionZ(
            final double initialEstimatedEcefPositionZ) {
        mInitialEstimatedEcefPositionZ = initialEstimatedEcefPositionZ;
    }

    /**
     * Gets z coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param result instance where z coordinate of initial estimated position will
     *               be stored.
     */
    public void getInitialEstimatedEcefPositionZDistance(final Distance result) {
        result.setValue(mInitialEstimatedEcefPositionZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of initial estimated position resolved along ECEF axes.
     *
     * @return z coordinate of initial estimated position.
     */
    public Distance getInitialEstimatedEcefPositionZDistance() {
        return new Distance(mInitialEstimatedEcefPositionZ,
                DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of initial estimated position resolved along ECEF axes.
     *
     * @param initialEstimatedEcefPositionZ z coordinate of initial estimated
     *                                      position.
     */
    public void setInitialEstimatedEcefPositionZDistance(
            final Distance initialEstimatedEcefPositionZ) {
        mInitialEstimatedEcefPositionZ = DistanceConverter.convert(
                initialEstimatedEcefPositionZ.getValue().doubleValue(),
                initialEstimatedEcefPositionZ.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets initial estimated position resolved along ECEF axes.
     *
     * @param result instance where initial estimated position will be stored.
     */
    public void getInitialEstimatedEcefPosition(final ECEFPosition result) {
        result.setCoordinates(mInitialEstimatedEcefPositionX,
                mInitialEstimatedEcefPositionY,
                mInitialEstimatedEcefPositionZ);
    }

    /**
     * Gets initial estimated position resolved along ECEF axes.
     *
     * @return initial estimated position.
     */
    public ECEFPosition getInitialEstimatedEcefPosition() {
        return new ECEFPosition(mInitialEstimatedEcefPositionX,
                mInitialEstimatedEcefPositionY,
                mInitialEstimatedEcefPositionZ);
    }

    /**
     * Sets initial estimated position resolved along ECEF axes.
     *
     * @param initialEstimatedEcefPosition initial estimated position resolved
     *                                     along ECEF axes.
     */
    public void setInitialEstimatedEcefPosition(
            final ECEFPosition initialEstimatedEcefPosition) {
        mInitialEstimatedEcefPositionX = initialEstimatedEcefPosition.getX();
        mInitialEstimatedEcefPositionY = initialEstimatedEcefPosition.getY();
        mInitialEstimatedEcefPositionZ = initialEstimatedEcefPosition.getZ();
    }

    /**
     * Gets initial estimated position resolved along ECEF axes and expressed in
     * meters (m).
     *
     * @param result instance where initial estimated position will be stored.
     */
    public void getInitialEstimatedPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mInitialEstimatedEcefPositionX,
                mInitialEstimatedEcefPositionY,
                mInitialEstimatedEcefPositionZ);
    }

    /**
     * Gets initial estimated position resolved along ECEF axes and expressed in
     * meters (m).
     *
     * @return initial estimated position.
     */
    public Point3D getInitialEstimatedPosition() {
        return new InhomogeneousPoint3D(mInitialEstimatedEcefPositionX,
                mInitialEstimatedEcefPositionY,
                mInitialEstimatedEcefPositionZ);
    }

    /**
     * Sets initial estimated position resolved along ECEF axes and expressed in
     * meters (m).
     *
     * @param initialEstimatedPosition initial estimated position.
     */
    public void setInitialEstimatedPosition(final Point3D initialEstimatedPosition) {
        mInitialEstimatedEcefPositionX = initialEstimatedPosition.getInhomX();
        mInitialEstimatedEcefPositionY = initialEstimatedPosition.getInhomY();
        mInitialEstimatedEcefPositionZ = initialEstimatedPosition.getInhomZ();
    }

    /**
     * Gets number of satellites in constellation.
     *
     * @return number of satellites in constellation.
     */
    public int getNumberOfSatellites() {
        return mNumberOfSatellites;
    }

    /**
     * Sets number of satellites in constellation.
     *
     * @param numberOfSatellites number of satellites in constellation.
     * @throws IllegalArgumentException if number of satellites is less than 4, which
     *                                  prevents proper position estimation.
     */
    public void setNumberOfSatellites(final int numberOfSatellites) {
        if (numberOfSatellites < MIN_SATELLITES) {
            throw new IllegalArgumentException();
        }
        mNumberOfSatellites = numberOfSatellites;
    }

    /**
     * Gets orbital radius of satellites expressed in meters (m).
     *
     * @return orbital radius of satellites.
     */
    public double getOrbitalRadiusOfSatellites() {
        return mOrbitalRadiusOfSatellites;
    }

    /**
     * Sets orbital radius of satellites expressed in meters (m).
     *
     * @param orbitalRadiusOfSatellites orbital radius of satellites.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setOrbitalRadiusOfSatellites(final double orbitalRadiusOfSatellites) {
        if (orbitalRadiusOfSatellites < 0.0) {
            throw new IllegalArgumentException();
        }
        mOrbitalRadiusOfSatellites = orbitalRadiusOfSatellites;
    }

    /**
     * Gets orbital radius of satellites.
     *
     * @param result instance where orbital radius of satellites will be stored.
     */
    public void getOrbitalRadiusOfSatellitesDistance(final Distance result) {
        result.setValue(mOrbitalRadiusOfSatellites);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets orbital radius of satellites.
     *
     * @return orbital radius of satellites.
     */
    public Distance getOrbitalRadiusOfSatellitesDistance() {
        return new Distance(mOrbitalRadiusOfSatellites,
                DistanceUnit.METER);
    }

    /**
     * Sets orbital radius of satellites.
     *
     * @param orbitalRadiusOfSatellites orbital radius of satellites.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setOrbitalRadiusOfSatellitesDistance(
            final Distance orbitalRadiusOfSatellites) {
        setOrbitalRadiusOfSatellites(DistanceConverter.convert(
                orbitalRadiusOfSatellites.getValue().doubleValue(),
                orbitalRadiusOfSatellites.getUnit(), DistanceUnit.METER));
    }

    /**
     * Gets inclination angle of satellites expressed in degrees (deg).
     *
     * @return inclination angle of satellites.
     */
    public double getSatellitesInclinationDegrees() {
        return mSatellitesInclinationDegrees;
    }

    /**
     * Sets inclination angle of satellites expressed in degrees (deg).
     *
     * @param satellitesInclinationDegrees inclination angle of satellites.
     */
    public void setSatellitesInclinationDegrees(
            final double satellitesInclinationDegrees) {
        mSatellitesInclinationDegrees = satellitesInclinationDegrees;
    }

    /**
     * Gets inclination angle of satellites.
     *
     * @param result instance where inclination angle of satellites will be
     *               stored.
     */
    public void getSatellitesInclinationAngle(final Angle result) {
        result.setValue(mSatellitesInclinationDegrees);
        result.setUnit(AngleUnit.DEGREES);
    }

    /**
     * Gets inclination angle of satellites.
     *
     * @return inclination angle of satellites.
     */
    public Angle getSatellitesInclinationAngle() {
        return new Angle(mSatellitesInclinationDegrees,
                AngleUnit.DEGREES);
    }

    /**
     * Sets inclination angle of satellites.
     *
     * @param satellitesInclination inclination angle of satellites.
     */
    public void setSatellitesInclinationAngle(final Angle satellitesInclination) {
        mSatellitesInclinationDegrees = AngleConverter.convert(
                satellitesInclination.getValue().doubleValue(),
                satellitesInclination.getUnit(), AngleUnit.DEGREES);
    }

    /**
     * Gets longitude offset of constellation expressed in degrees (deg).
     *
     * @return longitude offset of constellation.
     */
    public double getConstellationLongitudeOffsetDegrees() {
        return mConstellationLongitudeOffsetDegrees;
    }

    /**
     * Sets longitude offset of constellation expressed in degrees (deg).
     *
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation.
     */
    public void setConstellationLongitudeOffsetDegrees(
            final double constellationLongitudeOffsetDegrees) {
        mConstellationLongitudeOffsetDegrees = constellationLongitudeOffsetDegrees;
    }

    /**
     * Gets longitude offset of constellation.
     *
     * @param result instance where longitude offset of constellation will be stored.
     */
    public void getConstellationLongitudeOffsetAngle(final Angle result) {
        result.setValue(mConstellationLongitudeOffsetDegrees);
        result.setUnit(AngleUnit.DEGREES);
    }

    /**
     * Gets longitude offset of constellation.
     *
     * @return longitude offset of constellation.
     */
    public Angle getConstellationLongitudeOffsetAngle() {
        return new Angle(mConstellationLongitudeOffsetDegrees,
                AngleUnit.DEGREES);
    }

    /**
     * Sets longitude offset of constellation.
     *
     * @param constellationLongitudeOffset longitude offset of constellation.
     */
    public void setConstellationLongitudeOffsetAngle(
            final Angle constellationLongitudeOffset) {
        mConstellationLongitudeOffsetDegrees = AngleConverter.convert(
                constellationLongitudeOffset.getValue().doubleValue(),
                constellationLongitudeOffset.getUnit(), AngleUnit.DEGREES);
    }

    /**
     * Gets timing offset of constellation expressed in seconds (s).
     *
     * @return timing offset of constellation.
     */
    public double getConstellationTimingOffset() {
        return mConstellationTimingOffset;
    }

    /**
     * Sets timing offset of constellation expressed in seconds (s).
     *
     * @param constellationTimingOffset timing offset of constellation.
     */
    public void setConstellationTimingOffset(
            final double constellationTimingOffset) {
        mConstellationTimingOffset = constellationTimingOffset;
    }

    /**
     * Gets timing offset of constellation.
     *
     * @param result instance where timing offset of constellation will be stored.
     */
    public void getConstellationTimingOffsetTime(final Time result) {
        result.setValue(mConstellationTimingOffset);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets timing offset of constellation.
     *
     * @return timing offset of constellation.
     */
    public Time getConstellationTimingOffsetTime() {
        return new Time(mConstellationTimingOffset, TimeUnit.SECOND);
    }

    /**
     * Sets timing offset of constellation.
     *
     * @param constellationTimingOffset timing offset of constellation.
     */
    public void setConstellationTimingOffsetTime(final Time constellationTimingOffset) {
        mConstellationTimingOffset = TimeConverter.convert(
                constellationTimingOffset.getValue().doubleValue(),
                constellationTimingOffset.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Gets mask angle expressed in degrees (deg).
     *
     * @return mask angle.
     */
    public double getMaskAngleDegrees() {
        return mMaskAngleDegrees;
    }

    /**
     * Sets mask angle expressed in degrees (deg).
     *
     * @param maskAngleDegrees mask angle.
     */
    public void setMaskAngleDegrees(final double maskAngleDegrees) {
        mMaskAngleDegrees = maskAngleDegrees;
    }

    /**
     * Gets mask angle.
     *
     * @param result instance where mask angle will be stored.
     */
    public void getMaskAngle(final Angle result) {
        result.setValue(mMaskAngleDegrees);
        result.setUnit(AngleUnit.DEGREES);
    }

    /**
     * Gets mask angle.
     *
     * @return mask angle.
     */
    public Angle getMaskAngle() {
        return new Angle(mMaskAngleDegrees, AngleUnit.DEGREES);
    }

    /**
     * Sets mask angle.
     *
     * @param maskAngle mask angle.
     */
    public void setMaskAngle(final Angle maskAngle) {
        mMaskAngleDegrees = AngleConverter.convert(
                maskAngle.getValue().doubleValue(),
                maskAngle.getUnit(), AngleUnit.DEGREES);
    }

    /**
     * Gets Signal In Space (SIS) error Standard Deviation (SD) expressed in
     * meters (m).
     *
     * @return Signal In Space error Standard Deviation.
     */
    public double getSISErrorSD() {
        return mSISErrorSD;
    }

    /**
     * Sets Signal In Space (SIS) error Standard Deviation (SD) expressed in
     * meters (m).
     *
     * @param sisErrorSD Signal In Space (SIS) error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setSISErrorSD(final double sisErrorSD) {
        if (sisErrorSD < 0.0) {
            throw new IllegalArgumentException();
        }
        mSISErrorSD = sisErrorSD;
    }

    /**
     * Gets Signal In Space (SIS) error Standard Deviation (SD).
     *
     * @param result instance where Signal In Space (SIS) error
     *               Standard Deviation (SD) will be stored.
     */
    public void getSISErrorSDDistance(final Distance result) {
        result.setValue(mSISErrorSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets Signal In Space (SIS) error Standard Deviation (SD).
     *
     * @return Signal In Space (SIS) error Standard Deviation (SD).
     */
    public Distance getSISErrorSDDistance() {
        return new Distance(mSISErrorSD, DistanceUnit.METER);
    }

    /**
     * Sets Signal In Space (SIS) error Standard Deviation (SD).
     *
     * @param sisErrorSD Signal In Space (SIS) error Standard Deviation (SD).
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setSISErrorSDDistance(final Distance sisErrorSD) {
        setSISErrorSD(DistanceConverter.convert(
                sisErrorSD.getValue().doubleValue(),
                sisErrorSD.getUnit(), DistanceUnit.METER));
    }

    /**
     * Gets Zenith ionosphere error Standard Deviation (SD) expressed in meters (m).
     *
     * @return Zenith ionosphere error Standard Deviation.
     */
    public double getZenithIonosphereErrorSD() {
        return mZenithIonosphereErrorSD;
    }

    /**
     * Sets Zenith ionosphere error Standard Deviation (SD) expressed in meters (m).
     *
     * @param zenithIonosphereErrorSD Zenith ionosphere error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setZenithIonosphereErrorSD(final double zenithIonosphereErrorSD) {
        if (zenithIonosphereErrorSD < 0.0) {
            throw new IllegalArgumentException();
        }
        mZenithIonosphereErrorSD = zenithIonosphereErrorSD;
    }

    /**
     * Gets Zenith ionosphere error Standard Deviation (SD).
     *
     * @param result instance where Zenith ionosphere error Standard Deviation
     *               will be stored.
     */
    public void getZenithIonosphereErrorSDDistance(final Distance result) {
        result.setValue(mZenithIonosphereErrorSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets Zenith ionosphere error Standard Deviation (SD).
     *
     * @return Zenith ionosphere error Standard Deviation.
     */
    public Distance getZenithIonosphereErrorSDDistance() {
        return new Distance(mZenithIonosphereErrorSD,
                DistanceUnit.METER);
    }

    /**
     * Sets Zenith ionosphere error Standard Deviation (SD).
     *
     * @param zenithIonosphereErrorSD Zenith ionosphere error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setZenithIonosphereErrorSDDistance(
            final Distance zenithIonosphereErrorSD) {
        setZenithIonosphereErrorSD(DistanceConverter.convert(
                zenithIonosphereErrorSD.getValue().doubleValue(),
                zenithIonosphereErrorSD.getUnit(),
                DistanceUnit.METER));
    }

    /**
     * Gets Zenith troposphere error Standard Deviation (SD) expressed in meters (m).
     *
     * @return Zenith troposphere error Standard Deviation.
     */
    public double getZenithTroposphereErrorSD() {
        return mZenithTroposphereErrorSD;
    }

    /**
     * Sets Zenith troposphere error Standard Deviation (SD) expressed in meters (m).
     *
     * @param zenithTroposphereErrorSD Zenith troposphere error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setZenithTroposphereErrorSD(final double zenithTroposphereErrorSD) {
        if (zenithTroposphereErrorSD < 0.0) {
            throw new IllegalArgumentException();
        }
        mZenithTroposphereErrorSD = zenithTroposphereErrorSD;
    }

    /**
     * Gets Zenith troposphere error Standard Deviation (SD).
     *
     * @param result instance where Zenith troposphere error Standard Deviation (SD)
     *               will be stored.
     */
    public void getZenithTroposphereErrorSDDistance(final Distance result) {
        result.setValue(mZenithTroposphereErrorSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets Zenith troposphere error Standard Deviation (SD).
     *
     * @return Zenith troposphere error Standard Deviation (SD).
     */
    public Distance getZenithTroposphereErrorSDDistance() {
        return new Distance(mZenithTroposphereErrorSD,
                DistanceUnit.METER);
    }

    /**
     * Sets Zenith troposphere error Standard Deviation (SD).
     *
     * @param zenithTroposphereErrorSD Zenith troposphere error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setZenithTroposphereErrorSDDistance(
            final Distance zenithTroposphereErrorSD) {
        setZenithTroposphereErrorSD(DistanceConverter.convert(
                zenithTroposphereErrorSD.getValue().doubleValue(),
                zenithTroposphereErrorSD.getUnit(),
                DistanceUnit.METER));
    }

    /**
     * Gets code tracking error Standard Deviation (SD) expressed in meters per
     * second (m/s).
     *
     * @return code tracking error Standard Deviation (SD).
     */
    public double getCodeTrackingErrorSD() {
        return mCodeTrackingErrorSD;
    }

    /**
     * Sets code tracking error Standard Deviation (SD) expressed in meters per
     * second (m/s).
     *
     * @param codeTrackingErrorSD code tracking error Standard Deviation (SD).
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setCodeTrackingErrorSD(final double codeTrackingErrorSD) {
        if (codeTrackingErrorSD < 0.0) {
            throw new IllegalArgumentException();
        }
        mCodeTrackingErrorSD = codeTrackingErrorSD;
    }

    /**
     * Gets code tracking error Standard Deviation (SD).
     *
     * @param result instance where code tracking error Standard Deviation will
     *               be stored.
     */
    public void getCodeTrackingErrorSDSpeed(final Speed result) {
        result.setValue(mCodeTrackingErrorSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets code tracking error Standard Deviation (SD).
     *
     * @return code tracking error Standard Deviation.
     */
    public Speed getCodeTrackingErrorSDSpeed() {
        return new Speed(mCodeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets code tracking error Standard Deviation (SD).
     *
     * @param codeTrackingErrorSD code tracking error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setCodeTrackingErrorSDSpeed(final Speed codeTrackingErrorSD) {
        setCodeTrackingErrorSD(SpeedConverter.convert(
                codeTrackingErrorSD.getValue().doubleValue(),
                codeTrackingErrorSD.getUnit(), SpeedUnit.METERS_PER_SECOND));
    }

    /**
     * Gets range rate tracking error Standard Deviation (SD) expressed in meters
     * per second (m/s).
     *
     * @return range rate tracking error.
     */
    public double getRangeRateTrackingErrorSD() {
        return mRangeRateTrackingErrorSD;
    }

    /**
     * Sets range rate tracking error Standard Deviation (SD) expressed in meters
     * per second (m/s).
     *
     * @param rangeRateTrackingErrorSD range rate tracking error Standard Deviation.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setRangeRateTrackingErrorSD(final double rangeRateTrackingErrorSD) {
        if (rangeRateTrackingErrorSD < 0.0) {
            throw new IllegalArgumentException();
        }
        mRangeRateTrackingErrorSD = rangeRateTrackingErrorSD;
    }

    /**
     * Gets range rate tracking error Standard Deviation (SD).
     *
     * @param result instance where range rate tracking error Standard Deviation will
     *               be stored.
     */
    public void getRangeRateTrackingErrorSDSpeed(final Speed result) {
        result.setValue(mRangeRateTrackingErrorSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets range rate tracking error Standard Deviation (SD).
     *
     * @return range rate tracking error.
     */
    public Speed getRangeRateTrackingErrorSDSpeed() {
        return new Speed(mRangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets range rate tracking error Standard Deviation (SD).
     *
     * @param rangeRateTrackingErrorSD range rate tracking error.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setRangeRateTrackingErrorSDSpeed(
            final Speed rangeRateTrackingErrorSD) {
        setRangeRateTrackingErrorSD(SpeedConverter.convert(
                rangeRateTrackingErrorSD.getValue().doubleValue(),
                rangeRateTrackingErrorSD.getUnit(), SpeedUnit.METERS_PER_SECOND));
    }

    /**
     * Gets initial receiver clock offset at time = 0 expressed in meters (m).
     *
     * @return initial receiver clock offset.
     */
    public double getInitialReceiverClockOffset() {
        return mInitialReceiverClockOffset;
    }

    /**
     * Sets initial receiver clock offset at time = 0 expressed in meters (m).
     *
     * @param initialReceiverClockOffset initial receiver clock offset.
     */
    public void setInitialReceiverClockOffset(
            final double initialReceiverClockOffset) {
        mInitialReceiverClockOffset = initialReceiverClockOffset;
    }

    /**
     * Gets initial receiver clock offset at time = 0.
     *
     * @param result instance where initial receiver clock offset will be stored.
     */
    public void getInitialReceiverClockOffsetDistance(final Distance result) {
        result.setValue(mInitialReceiverClockOffset);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets initial receiver clock offset at time = 0.
     *
     * @return initial receiver clock offset.
     */
    public Distance getInitialReceiverClockOffsetDistance() {
        return new Distance(mInitialReceiverClockOffset, DistanceUnit.METER);
    }

    /**
     * Sets initial receiver clock offset at time = 0.
     *
     * @param initialReceiverClockOffset initial receiver clock offset.
     */
    public void setInitialReceiverClockOffsetDistance(
            final Distance initialReceiverClockOffset) {
        mInitialReceiverClockOffset = DistanceConverter.convert(
                initialReceiverClockOffset.getValue().doubleValue(),
                initialReceiverClockOffset.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets initial receiver clock drift at time = 0 expressed in meters per
     * second (m/s).
     *
     * @return initial receiver clock drift.
     */
    public double getInitialReceiverClockDrift() {
        return mInitialReceiverClockDrift;
    }

    /**
     * Sets initial receiver clock drift at time = 0 expressed in meters per
     * second (m/s).
     *
     * @param initialReceiverClockDrift initial receiver clock drift.
     */
    public void setInitialReceiverClockDrift(
            final double initialReceiverClockDrift) {
        mInitialReceiverClockDrift = initialReceiverClockDrift;
    }

    /**
     * Gets initial receiver clock drift at time = 0.
     *
     * @param result instance where initial receiver clock drift at time = 0 will
     *               be stored.
     */
    public void getInitialReceiverClockDriftSpeed(final Speed result) {
        result.setValue(mInitialReceiverClockDrift);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets initial receiver clock drift at time = 0.
     *
     * @return initial receiver clock drift at time = 0.
     */
    public Speed getInitialReceiverClockDriftSpeed() {
        return new Speed(mInitialReceiverClockDrift,
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets initial receiver clock drift at time = 0.
     *
     * @param initialReceiverClockDrift initial receiver clock drift at time = 0.
     */
    public void setInitialReceiverClockDriftSpeed(
            final Speed initialReceiverClockDrift) {
        mInitialReceiverClockDrift = SpeedConverter.convert(
                initialReceiverClockDrift.getValue().doubleValue(),
                initialReceiverClockDrift.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedEcefPositionX       x coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param initialEstimatedEcefPositionY       y coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param initialEstimatedEcefPositionZ       z coordinate of initial estimated
     *                                            position resolved along ECEF axes
     *                                            and expressed in meters (m).
     * @param numberOfSatellites                  number of satellites in constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final double epochInterval,
                          final double initialEstimatedEcefPositionX,
                          final double initialEstimatedEcefPositionY,
                          final double initialEstimatedEcefPositionZ,
                          final int numberOfSatellites,
                          final double orbitalRadiusOfSatellites,
                          final double satellitesInclinationDegrees,
                          final double constellationLongitudeOffsetDegrees,
                          final double constellationTimingOffset,
                          final double maskAngleDegrees,
                          final double sisErrorSD,
                          final double zenithIonosphereErrorSD,
                          final double zenithTroposphereErrorSD,
                          final double codeTrackingErrorSD,
                          final double rangeRateTrackingErrorSD,
                          final double initialReceiverClockOffset,
                          final double initialReceiverClockDrift) {
        setEpochInterval(epochInterval);
        mInitialEstimatedEcefPositionX = initialEstimatedEcefPositionX;
        mInitialEstimatedEcefPositionY = initialEstimatedEcefPositionY;
        mInitialEstimatedEcefPositionZ = initialEstimatedEcefPositionZ;
        setNumberOfSatellites(numberOfSatellites);
        setOrbitalRadiusOfSatellites(orbitalRadiusOfSatellites);
        mSatellitesInclinationDegrees = satellitesInclinationDegrees;
        mConstellationLongitudeOffsetDegrees = constellationLongitudeOffsetDegrees;
        mConstellationTimingOffset = constellationTimingOffset;
        mMaskAngleDegrees = maskAngleDegrees;
        setSISErrorSD(sisErrorSD);
        setZenithIonosphereErrorSD(zenithIonosphereErrorSD);
        setZenithTroposphereErrorSD(zenithTroposphereErrorSD);
        setCodeTrackingErrorSD(codeTrackingErrorSD);
        setRangeRateTrackingErrorSD(rangeRateTrackingErrorSD);
        mInitialReceiverClockOffset = initialReceiverClockOffset;
        mInitialReceiverClockDrift = initialReceiverClockDrift;
    }

    /**
     * Sets values.
     *
     * @param epochInterval                 minimum interval between GNSS epochs.
     * @param initialEstimatedEcefPositionX x coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param initialEstimatedEcefPositionY y coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param initialEstimatedEcefPositionZ z coordinate of initial estimated
     *                                      position resolved along ECEF axes.
     * @param numberOfSatellites            number of satellites in constellation.
     * @param orbitalRadiusOfSatellites     orbital radius of satellites.
     * @param satellitesInclination         inclination angle of satellites.
     * @param constellationLongitudeOffset  longitude offset of constellation.
     * @param constellationTimingOffset     timing offset of constellation.
     * @param maskAngle                     mask angle.
     * @param sisErrorSD                    Signal In Space (SIS) error Standard
     *                                      Deviation (SD).
     * @param zenithIonosphereErrorSD       zenith ionosphere error Standard
     *                                      Deviation (SD).
     * @param zenithTroposphereErrorSD      zenith troposphere error Standard
     *                                      Deviation (SD).
     * @param codeTrackingErrorSD           code tracking error Standard Deviation
     *                                      (SD).
     * @param rangeRateTrackingErrorSD      range rate tracking error Standard
     *                                      Deviation (SD).
     * @param initialReceiverClockOffset    initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift     initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final Time epochInterval,
                          final Distance initialEstimatedEcefPositionX,
                          final Distance initialEstimatedEcefPositionY,
                          final Distance initialEstimatedEcefPositionZ,
                          final int numberOfSatellites,
                          final Distance orbitalRadiusOfSatellites,
                          final Angle satellitesInclination,
                          final Angle constellationLongitudeOffset,
                          final Time constellationTimingOffset,
                          final Angle maskAngle,
                          final Distance sisErrorSD,
                          final Distance zenithIonosphereErrorSD,
                          final Distance zenithTroposphereErrorSD,
                          final Speed codeTrackingErrorSD,
                          final Speed rangeRateTrackingErrorSD,
                          final Distance initialReceiverClockOffset,
                          final Speed initialReceiverClockDrift) {
        setEpochIntervalTime(epochInterval);
        setInitialEstimatedEcefPositionXDistance(initialEstimatedEcefPositionX);
        setInitialEstimatedEcefPositionYDistance(initialEstimatedEcefPositionY);
        setInitialEstimatedEcefPositionZDistance(initialEstimatedEcefPositionZ);
        internalSetValues(numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedEcefPosition        initial estimated position
     *                                            resolved along ECEF axes.
     * @param numberOfSatellites                  number of satellites in
     *                                            constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final double epochInterval,
                          final ECEFPosition initialEstimatedEcefPosition,
                          final int numberOfSatellites,
                          final double orbitalRadiusOfSatellites,
                          final double satellitesInclinationDegrees,
                          final double constellationLongitudeOffsetDegrees,
                          final double constellationTimingOffset,
                          final double maskAngleDegrees,
                          final double sisErrorSD,
                          final double zenithIonosphereErrorSD,
                          final double zenithTroposphereErrorSD,
                          final double codeTrackingErrorSD,
                          final double rangeRateTrackingErrorSD,
                          final double initialReceiverClockOffset,
                          final double initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedEcefPosition.getX(),
                initialEstimatedEcefPosition.getY(),
                initialEstimatedEcefPosition.getZ(), numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees,
                constellationTimingOffset,
                maskAngleDegrees,
                sisErrorSD,
                zenithIonosphereErrorSD,
                zenithTroposphereErrorSD,
                codeTrackingErrorSD,
                rangeRateTrackingErrorSD,
                initialReceiverClockOffset,
                initialReceiverClockDrift);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                minimum interval between GNSS epochs.
     * @param initialEstimatedEcefPosition initial estimated position
     *                                     resolved along ECEF axes.
     * @param numberOfSatellites           number of satellites in constellation.
     * @param orbitalRadiusOfSatellites    orbital radius of satellites.
     * @param satellitesInclination        inclination angle of satellites.
     * @param constellationLongitudeOffset longitude offset of constellation.
     * @param constellationTimingOffset    timing offset of constellation.
     * @param maskAngle                    mask angle.
     * @param sisErrorSD                   Signal In Space (SIS) error Standard
     *                                     Deviation (SD).
     * @param zenithIonosphereErrorSD      zenith ionosphere error Standard
     *                                     Deviation (SD).
     * @param zenithTroposphereErrorSD     zenith troposphere error Standard
     *                                     Deviation (SD).
     * @param codeTrackingErrorSD          code tracking error Standard Deviation
     *                                     (SD).
     * @param rangeRateTrackingErrorSD     range rate tracking error Standard
     *                                     Deviation (SD).
     * @param initialReceiverClockOffset   initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift    initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final Time epochInterval,
                          final ECEFPosition initialEstimatedEcefPosition,
                          final int numberOfSatellites,
                          final Distance orbitalRadiusOfSatellites,
                          final Angle satellitesInclination,
                          final Angle constellationLongitudeOffset,
                          final Time constellationTimingOffset,
                          final Angle maskAngle,
                          final Distance sisErrorSD,
                          final Distance zenithIonosphereErrorSD,
                          final Distance zenithTroposphereErrorSD,
                          final Speed codeTrackingErrorSD,
                          final Speed rangeRateTrackingErrorSD,
                          final Distance initialReceiverClockOffset,
                          final Speed initialReceiverClockDrift) {
        setEpochIntervalTime(epochInterval);
        setInitialEstimatedEcefPosition(initialEstimatedEcefPosition);
        internalSetValues(numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                       minimum interval between GNSS
     *                                            epochs expressed in seconds (s).
     * @param initialEstimatedPosition            initial estimated position
     *                                            resolved along ECEF axes and expressed
     *                                            in meters (m).
     * @param numberOfSatellites                  number of satellites in
     *                                            constellation.
     * @param orbitalRadiusOfSatellites           orbital radius of satellites
     *                                            expressed in meters (m).
     * @param satellitesInclinationDegrees        inclination angle of satellites
     *                                            expressed in degrees (deg).
     * @param constellationLongitudeOffsetDegrees longitude offset of constellation
     *                                            expressed in degrees (deg).
     * @param constellationTimingOffset           timing offset of constellation
     *                                            expressed in seconds (s).
     * @param maskAngleDegrees                    mask angle expressed in degrees
     *                                            (deg).
     * @param sisErrorSD                          Signal In Space (SIS) error
     *                                            Standard Deviation (SD) expressed
     *                                            in meters (m).
     * @param zenithIonosphereErrorSD             zenith ionosphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param zenithTroposphereErrorSD            zenith troposphere error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            (m).
     * @param codeTrackingErrorSD                 code tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param rangeRateTrackingErrorSD            range rate tracking error Standard
     *                                            Deviation (SD) expressed in meters
     *                                            per second (m/s).
     * @param initialReceiverClockOffset          initial receiver clock offset at
     *                                            time = 0 expressed in meters (m).
     * @param initialReceiverClockDrift           initial receiver clock drift at
     *                                            time = 0 expressed in meters per
     *                                            second (m/s).
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final double epochInterval,
                          final Point3D initialEstimatedPosition,
                          final int numberOfSatellites,
                          final double orbitalRadiusOfSatellites,
                          final double satellitesInclinationDegrees,
                          final double constellationLongitudeOffsetDegrees,
                          final double constellationTimingOffset,
                          final double maskAngleDegrees,
                          final double sisErrorSD,
                          final double zenithIonosphereErrorSD,
                          final double zenithTroposphereErrorSD,
                          final double codeTrackingErrorSD,
                          final double rangeRateTrackingErrorSD,
                          final double initialReceiverClockOffset,
                          final double initialReceiverClockDrift) {
        setValues(epochInterval, initialEstimatedPosition.getInhomX(),
                initialEstimatedPosition.getInhomY(),
                initialEstimatedPosition.getInhomZ(),
                numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees,
                constellationTimingOffset,
                maskAngleDegrees,
                sisErrorSD,
                zenithIonosphereErrorSD,
                zenithTroposphereErrorSD,
                codeTrackingErrorSD,
                rangeRateTrackingErrorSD,
                initialReceiverClockOffset,
                initialReceiverClockDrift);
    }

    /**
     * Sets values.
     *
     * @param epochInterval                minimum interval between GNSS epochs.
     * @param initialEstimatedPosition     initial estimated position
     *                                     resolved along ECEF axes and expressed
     *                                     in meters (m).
     * @param numberOfSatellites           number of satellites in constellation.
     * @param orbitalRadiusOfSatellites    orbital radius of satellites.
     * @param satellitesInclination        inclination angle of satellites.
     * @param constellationLongitudeOffset longitude offset of constellation.
     * @param constellationTimingOffset    timing offset of constellation.
     * @param maskAngle                    mask angle.
     * @param sisErrorSD                   Signal In Space (SIS) error Standard
     *                                     Deviation (SD).
     * @param zenithIonosphereErrorSD      zenith ionosphere error Standard
     *                                     Deviation (SD).
     * @param zenithTroposphereErrorSD     zenith troposphere error Standard
     *                                     Deviation (SD).
     * @param codeTrackingErrorSD          code tracking error Standard Deviation
     *                                     (SD).
     * @param rangeRateTrackingErrorSD     range rate tracking error Standard
     *                                     Deviation (SD).
     * @param initialReceiverClockOffset   initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift    initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if provided epoch interval is negative or
     *                                  if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    public void setValues(final Time epochInterval,
                          final Point3D initialEstimatedPosition,
                          final int numberOfSatellites,
                          final Distance orbitalRadiusOfSatellites,
                          final Angle satellitesInclination,
                          final Angle constellationLongitudeOffset,
                          final Time constellationTimingOffset,
                          final Angle maskAngle,
                          final Distance sisErrorSD,
                          final Distance zenithIonosphereErrorSD,
                          final Distance zenithTroposphereErrorSD,
                          final Speed codeTrackingErrorSD,
                          final Speed rangeRateTrackingErrorSD,
                          final Distance initialReceiverClockOffset,
                          final Speed initialReceiverClockDrift) {
        setEpochIntervalTime(epochInterval);
        setInitialEstimatedPosition(initialEstimatedPosition);
        internalSetValues(numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclination,
                constellationLongitudeOffset, constellationTimingOffset, maskAngle,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final GNSSConfig output) {
        output.mEpochInterval = mEpochInterval;
        output.mInitialEstimatedEcefPositionX = mInitialEstimatedEcefPositionX;
        output.mInitialEstimatedEcefPositionY = mInitialEstimatedEcefPositionY;
        output.mInitialEstimatedEcefPositionZ = mInitialEstimatedEcefPositionZ;
        output.mNumberOfSatellites = mNumberOfSatellites;
        output.mOrbitalRadiusOfSatellites = mOrbitalRadiusOfSatellites;
        output.mSatellitesInclinationDegrees = mSatellitesInclinationDegrees;
        output.mConstellationLongitudeOffsetDegrees =
                mConstellationLongitudeOffsetDegrees;
        output.mConstellationTimingOffset = mConstellationTimingOffset;
        output.mMaskAngleDegrees = mMaskAngleDegrees;
        output.mSISErrorSD = mSISErrorSD;
        output.mZenithIonosphereErrorSD = mZenithIonosphereErrorSD;
        output.mZenithTroposphereErrorSD = mZenithTroposphereErrorSD;
        output.mCodeTrackingErrorSD = mCodeTrackingErrorSD;
        output.mRangeRateTrackingErrorSD = mRangeRateTrackingErrorSD;
        output.mInitialReceiverClockOffset = mInitialReceiverClockOffset;
        output.mInitialReceiverClockDrift = mInitialReceiverClockDrift;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final GNSSConfig input) {
        mEpochInterval = input.mEpochInterval;
        mInitialEstimatedEcefPositionX = input.mInitialEstimatedEcefPositionX;
        mInitialEstimatedEcefPositionY = input.mInitialEstimatedEcefPositionY;
        mInitialEstimatedEcefPositionZ = input.mInitialEstimatedEcefPositionZ;
        mNumberOfSatellites = input.mNumberOfSatellites;
        mOrbitalRadiusOfSatellites = input.mOrbitalRadiusOfSatellites;
        mSatellitesInclinationDegrees = input.mSatellitesInclinationDegrees;
        mConstellationLongitudeOffsetDegrees =
                input.mConstellationLongitudeOffsetDegrees;
        mConstellationTimingOffset = input.mConstellationTimingOffset;
        mMaskAngleDegrees = input.mMaskAngleDegrees;
        mSISErrorSD = input.mSISErrorSD;
        mZenithIonosphereErrorSD = input.mZenithIonosphereErrorSD;
        mZenithTroposphereErrorSD = input.mZenithTroposphereErrorSD;
        mCodeTrackingErrorSD = input.mCodeTrackingErrorSD;
        mRangeRateTrackingErrorSD = input.mRangeRateTrackingErrorSD;
        mInitialReceiverClockOffset = input.mInitialReceiverClockOffset;
        mInitialReceiverClockDrift = input.mInitialReceiverClockDrift;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mEpochInterval, mInitialEstimatedEcefPositionX,
                mInitialEstimatedEcefPositionY, mInitialEstimatedEcefPositionZ,
                mNumberOfSatellites, mOrbitalRadiusOfSatellites,
                mSatellitesInclinationDegrees, mConstellationLongitudeOffsetDegrees,
                mConstellationTimingOffset, mMaskAngleDegrees, mSISErrorSD,
                mZenithIonosphereErrorSD, mZenithTroposphereErrorSD,
                mCodeTrackingErrorSD, mRangeRateTrackingErrorSD,
                mInitialReceiverClockOffset, mInitialReceiverClockDrift);
    }

    /**
     * Checks if provided object is a GNSSConfig having exactly the same contents
     * as this instance.
     *
     * @param o object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }

        final GNSSConfig other = (GNSSConfig) o;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final GNSSConfig other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final GNSSConfig other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mEpochInterval - other.mEpochInterval) <= threshold
                && Math.abs(mInitialEstimatedEcefPositionX - other.mInitialEstimatedEcefPositionX) <= threshold
                && Math.abs(mInitialEstimatedEcefPositionY - other.mInitialEstimatedEcefPositionY) <= threshold
                && Math.abs(mInitialEstimatedEcefPositionZ - other.mInitialEstimatedEcefPositionZ) <= threshold
                && Math.abs(mNumberOfSatellites - other.mNumberOfSatellites) <= threshold
                && Math.abs(mOrbitalRadiusOfSatellites - other.mOrbitalRadiusOfSatellites) <= threshold
                && Math.abs(mSatellitesInclinationDegrees - other.mSatellitesInclinationDegrees) <= threshold
                && Math.abs(mConstellationLongitudeOffsetDegrees - other.mConstellationLongitudeOffsetDegrees)
                <= threshold
                && Math.abs(mConstellationTimingOffset - other.mConstellationTimingOffset) <= threshold
                && Math.abs(mMaskAngleDegrees - other.mMaskAngleDegrees) <= threshold
                && Math.abs(mSISErrorSD - other.mSISErrorSD) <= threshold
                && Math.abs(mZenithIonosphereErrorSD - other.mZenithIonosphereErrorSD) <= threshold
                && Math.abs(mZenithTroposphereErrorSD - other.mZenithTroposphereErrorSD) <= threshold
                && Math.abs(mCodeTrackingErrorSD - other.mCodeTrackingErrorSD) <= threshold
                && Math.abs(mRangeRateTrackingErrorSD - other.mRangeRateTrackingErrorSD) <= threshold
                && Math.abs(mInitialReceiverClockOffset - other.mInitialReceiverClockOffset) <= threshold
                && Math.abs(mInitialReceiverClockDrift - other.mInitialReceiverClockDrift) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final GNSSConfig result = (GNSSConfig)super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Method used internally to set values.
     *
     * @param numberOfSatellites           number of satellites in constellation.
     * @param orbitalRadiusOfSatellites    orbital radius of satellites.
     * @param satellitesInclination        inclination angle of satellites.
     * @param constellationLongitudeOffset longitude offset of constellation.
     * @param constellationTimingOffset    timing offset of constellation.
     * @param maskAngle                    mask angle.
     * @param sisErrorSD                   Signal In Space (SIS) error Standard
     *                                     Deviation (SD).
     * @param zenithIonosphereErrorSD      zenith ionosphere error Standard
     *                                     Deviation (SD).
     * @param zenithTroposphereErrorSD     zenith troposphere error Standard
     *                                     Deviation (SD).
     * @param codeTrackingErrorSD          code tracking error Standard Deviation
     *                                     (SD).
     * @param rangeRateTrackingErrorSD     range rate tracking error Standard
     *                                     Deviation (SD).
     * @param initialReceiverClockOffset   initial receiver clock offset at time = 0.
     * @param initialReceiverClockDrift    initial receiver clock drift at time = 0.
     * @throws IllegalArgumentException if number of satellites in constellation is
     *                                  less than 4 or if orbital radius of
     *                                  satellites is negative or if SIS error SD is
     *                                  negative, or if zenith ionosphere error SD
     *                                  is negative, or if zenith troposphere error
     *                                  SD is negative, or if code tracking error SD
     *                                  is negative, or if range rate tracking error
     *                                  SD is negative.
     */
    private void internalSetValues(final int numberOfSatellites,
                                   final Distance orbitalRadiusOfSatellites,
                                   final Angle satellitesInclination,
                                   final Angle constellationLongitudeOffset,
                                   final Time constellationTimingOffset,
                                   final Angle maskAngle,
                                   final Distance sisErrorSD,
                                   final Distance zenithIonosphereErrorSD,
                                   final Distance zenithTroposphereErrorSD,
                                   final Speed codeTrackingErrorSD,
                                   final Speed rangeRateTrackingErrorSD,
                                   final Distance initialReceiverClockOffset,
                                   final Speed initialReceiverClockDrift) {
        setNumberOfSatellites(numberOfSatellites);
        setOrbitalRadiusOfSatellitesDistance(orbitalRadiusOfSatellites);
        setSatellitesInclinationAngle(satellitesInclination);
        setConstellationLongitudeOffsetAngle(constellationLongitudeOffset);
        setConstellationTimingOffsetTime(constellationTimingOffset);
        setMaskAngle(maskAngle);
        setSISErrorSDDistance(sisErrorSD);
        setZenithIonosphereErrorSDDistance(zenithIonosphereErrorSD);
        setZenithTroposphereErrorSDDistance(zenithTroposphereErrorSD);
        setCodeTrackingErrorSDSpeed(codeTrackingErrorSD);
        setRangeRateTrackingErrorSDSpeed(rangeRateTrackingErrorSD);
        setInitialReceiverClockOffsetDistance(initialReceiverClockOffset);
        setInitialReceiverClockDriftSpeed(initialReceiverClockDrift);
    }
}
