package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.SerializationHelper;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class GNSSConfigTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstants() {
        assertEquals(GNSSConfig.MIN_SATELLITES, MIN_SATELLITES);
    }

    @Test
    void testConstructor() {
        // test empty constructor
        var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(-1.0,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, 3,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                -1.0, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, -1.0, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, -1.0,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0, initialReceiverClockOffset,
                initialReceiverClockDrift));

        // test constructor with unit values
        final var epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final var initialEstimatedEcefPositionXDistance = new Distance(initialEstimatedEcefPositionX,
                DistanceUnit.METER);
        final var initialEstimatedEcefPositionYDistance = new Distance(initialEstimatedEcefPositionY,
                DistanceUnit.METER);
        final var initialEstimatedEcefPositionZDistance = new Distance(initialEstimatedEcefPositionZ,
                DistanceUnit.METER);
        final var orbitalRadiusOfSatellitesDistance = new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final var satellitesInclinationAngle = new Angle(satellitesInclinationDegrees, AngleUnit.DEGREES);
        final var constellationLongitudeOffsetAngle = new Angle(constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final var constellationTimingOffsetTime = new Time(constellationTimingOffset, TimeUnit.SECOND);
        final var maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final var sisErrorSDDistance = new Distance(sisErrorSD, DistanceUnit.METER);
        final var zenithIonosphereErrorSDDistance = new Distance(zenithIonosphereErrorSD, DistanceUnit.METER);
        final var zenithTroposphereErrorSDDistance = new Distance(zenithTroposphereErrorSD, DistanceUnit.METER);
        final var codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var initialReceiverClockOffsetDistance = new Distance(initialReceiverClockOffset, DistanceUnit.METER);
        final var initialReceiverClockDriftSpeed = new Speed(initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance, initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        final var time = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(time, initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance, initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, 3, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, distance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance, distance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        final var speed = new Speed(-1.0, SpeedUnit.METERS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                speed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, speed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));

        // test constructor with values and ECEF position
        final var initialEstimatedEcefPosition = new ECEFPosition(initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(-1.0,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                3, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, -1.0, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, -1.0,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                -1.0, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0,
                initialReceiverClockOffset, initialReceiverClockDrift));

        // test constructor with unit values and ECEF position
        config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(time, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, 3, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, distance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, distance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                speed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, speed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));

        // test constructor with values and position point
        final var initialEstimatedPosition = new InhomogeneousPoint3D(
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        config = new GNSSConfig(epochInterval, initialEstimatedPosition, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(-1.0, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                3, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, -1.0, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, -1.0,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                -1.0, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0,
                initialReceiverClockOffset, initialReceiverClockDrift));

        // test constructor with unit values and position point
        config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(time, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                3, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, distance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, distance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                speed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                speed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));

        // test copy constructor
        config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
        final var config2 = new GNSSConfig(config);

        // check default values
        assertEquals(epochInterval, config2.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config2.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config2.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config2.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config2.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config2.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config2.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config2.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config2.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config2.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config2.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config2.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config2.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config2.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config2.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config2.getInitialReceiverClockDrift(), 0.0);
    }

    @Test
    void testGetSetEpochInterval() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getEpochInterval(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setEpochInterval(epochInterval);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setEpochInterval(-1.0));
    }

    @Test
    void testGetSetEpochIntervalTime() {
        final var config = new GNSSConfig();

        // check default value
        final var epochInterval1 = config.getEpochIntervalTime();
        assertEquals(0.0, epochInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, epochInterval1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var epochInterval2 = new Time(epochInterval, TimeUnit.SECOND);
        config.setEpochIntervalTime(epochInterval2);

        // check
        final var epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        config.getEpochIntervalTime(epochInterval3);
        final var epochInterval4 = config.getEpochIntervalTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);

        // Force IllegalArgumentException
        final var time = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setEpochIntervalTime(time));
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionX() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionX(initialEstimatedEcefPositionX);

        // check
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionXDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var initialEstimatedEcefPositionX1 = config.getInitialEstimatedEcefPositionXDistance();

        assertEquals(0.0, initialEstimatedEcefPositionX1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialEstimatedEcefPositionX1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX2 = new Distance(initialEstimatedEcefPositionX, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionXDistance(initialEstimatedEcefPositionX2);

        // check
        final var initialEstimatedEcefPositionX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionXDistance(initialEstimatedEcefPositionX3);
        final var initialEstimatedEcefPositionX4 = config.getInitialEstimatedEcefPositionXDistance();

        assertEquals(initialEstimatedEcefPositionX2, initialEstimatedEcefPositionX3);
        assertEquals(initialEstimatedEcefPositionX2, initialEstimatedEcefPositionX4);
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionY() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionY(initialEstimatedEcefPositionY);

        // check
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionYDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var initialEstimatedEcefPositionY1 = config.getInitialEstimatedEcefPositionYDistance();

        assertEquals(0.0, initialEstimatedEcefPositionY1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialEstimatedEcefPositionY1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY2 = new Distance(initialEstimatedEcefPositionY, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionYDistance(initialEstimatedEcefPositionY2);

        // check
        final var initialEstimatedEcefPositionY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionYDistance(initialEstimatedEcefPositionY3);
        final var initialEstimatedEcefPositionY4 = config.getInitialEstimatedEcefPositionYDistance();

        assertEquals(initialEstimatedEcefPositionY2, initialEstimatedEcefPositionY3);
        assertEquals(initialEstimatedEcefPositionY2, initialEstimatedEcefPositionY4);
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionZ() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionZ(initialEstimatedEcefPositionZ);

        // check
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
    }

    @Test
    void testGetSetInitialEstimatedEcefPositionZDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var initialEstimatedEcefPositionZ1 = config.getInitialEstimatedEcefPositionZDistance();

        assertEquals(0.0, initialEstimatedEcefPositionZ1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialEstimatedEcefPositionZ1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ2 = new Distance(initialEstimatedEcefPositionZ, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionZDistance(initialEstimatedEcefPositionZ2);

        // check
        final var initialEstimatedEcefPositionZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionZDistance(initialEstimatedEcefPositionZ3);
        final var initialEstimatedEcefPositionZ4 = config.getInitialEstimatedEcefPositionZDistance();

        assertEquals(initialEstimatedEcefPositionZ2, initialEstimatedEcefPositionZ3);
        assertEquals(initialEstimatedEcefPositionZ2, initialEstimatedEcefPositionZ4);
    }

    @Test
    void testGetSetInitialEstimatedEcefPosition() {
        final var config = new GNSSConfig();

        // check default value
        final var position1 = config.getInitialEstimatedEcefPosition();

        assertEquals(0.0, position1.getX(), 0.0);
        assertEquals(0.0, position1.getY(), 0.0);
        assertEquals(0.0, position1.getZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var position2 = new ECEFPosition(x, y, z);
        config.setInitialEstimatedEcefPosition(position2);

        // check
        final var position3 = new ECEFPosition();
        config.getInitialEstimatedEcefPosition(position3);
        final var position4 = config.getInitialEstimatedEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetInitialEstimatedPosition() {
        final var config = new GNSSConfig();

        // check default value
        final var position1 = config.getInitialEstimatedPosition();

        assertEquals(position1, Point3D.create());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var position2 = new InhomogeneousPoint3D(x, y, z);
        config.setInitialEstimatedPosition(position2);

        // check
        final var position3 = new InhomogeneousPoint3D();
        config.getInitialEstimatedPosition(position3);
        final var position4 = config.getInitialEstimatedPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    void testGetSetNumberOfSatellites() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(GNSSConfig.MIN_SATELLITES, config.getNumberOfSatellites());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);

        config.setNumberOfSatellites(numberOfSatellites);

        // check
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setNumberOfSatellites(3));
    }

    @Test
    void testGetSetOrbitalRadiusOfSatellites() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setOrbitalRadiusOfSatellites(orbitalRadiusOfSatellites);

        // check
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setOrbitalRadiusOfSatellites(-1.0));
    }

    @Test
    void testGetSetOrbitalRadiusOfSatellitesDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var orbitalRadiusOfSatellites1 = config.getOrbitalRadiusOfSatellitesDistance();

        assertEquals(0.0, orbitalRadiusOfSatellites1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, orbitalRadiusOfSatellites1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var orbitalRadiusOfSatellites2 = new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        config.setOrbitalRadiusOfSatellitesDistance(orbitalRadiusOfSatellites2);

        // check
        final var orbitalRadiusOfSatellites3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getOrbitalRadiusOfSatellitesDistance(orbitalRadiusOfSatellites3);
        final var orbitalRadiusOfSatellites4 = config.getOrbitalRadiusOfSatellitesDistance();

        assertEquals(orbitalRadiusOfSatellites2, orbitalRadiusOfSatellites3);
        assertEquals(orbitalRadiusOfSatellites2, orbitalRadiusOfSatellites4);

        // Force IllegalArgumentException
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setOrbitalRadiusOfSatellitesDistance(distance));
    }

    @Test
    void testGetSetSatellitesInclinationDegrees() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setSatellitesInclinationDegrees(satellitesInclinationDegrees);

        // check
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
    }

    @Test
    void testGetSetSatellitesInclinationAngle() {
        final var config = new GNSSConfig();

        // check default value
        final var satellitesInclinationAngle1 = config.getSatellitesInclinationAngle();

        assertEquals(0.0, satellitesInclinationAngle1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.DEGREES, satellitesInclinationAngle1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationAngle2 = new Angle(satellitesInclinationDegrees, AngleUnit.DEGREES);
        config.setSatellitesInclinationAngle(satellitesInclinationAngle2);

        // check
        final var satellitesInclinationAngle3 = new Angle(0.0, AngleUnit.RADIANS);
        config.getSatellitesInclinationAngle(satellitesInclinationAngle3);
        final var satellitesInclinationAngle4 = config.getSatellitesInclinationAngle();

        assertTrue(satellitesInclinationAngle2.equals(satellitesInclinationAngle3, ABSOLUTE_ERROR));
        assertTrue(satellitesInclinationAngle2.equals(satellitesInclinationAngle4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetConstellationLongitudeOffsetDegrees() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setConstellationLongitudeOffsetDegrees(constellationLongitudeOffsetDegrees);

        // check
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
    }

    @Test
    void testGetSetConstellationLongitudeOffsetAngle() {
        final var config = new GNSSConfig();

        // check default value
        final var constellationLongitudeOffset1 = config.getConstellationLongitudeOffsetAngle();

        assertEquals(0.0, constellationLongitudeOffset1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.DEGREES, constellationLongitudeOffset1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffset2 = new Angle(constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        config.setConstellationLongitudeOffsetAngle(constellationLongitudeOffset2);

        // check
        final var constellationLongitudeOffset3 = new Angle(0.0, AngleUnit.RADIANS);
        config.getConstellationLongitudeOffsetAngle(constellationLongitudeOffset3);
        final var constellationLongitudeOffset4 = config.getConstellationLongitudeOffsetAngle();

        assertTrue(constellationLongitudeOffset2.equals(constellationLongitudeOffset3, ABSOLUTE_ERROR));
        assertTrue(constellationLongitudeOffset2.equals(constellationLongitudeOffset4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetConstellationTimingOffset() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setConstellationTimingOffset(constellationTimingOffset);

        // check
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
    }

    @Test
    void testGetSetConstellationTimingOffsetTime() {
        final var config = new GNSSConfig();

        // check default value
        final var constellationTimingOffset1 = config.getConstellationTimingOffsetTime();

        assertEquals(0.0, constellationTimingOffset1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, constellationTimingOffset1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset2 = new Time(constellationTimingOffset, TimeUnit.SECOND);

        config.setConstellationTimingOffsetTime(constellationTimingOffset2);

        // check
        final var constellationTimingOffset3 = new Time(0.0, TimeUnit.MILLISECOND);
        config.getConstellationTimingOffsetTime(constellationTimingOffset3);
        final var constellationTimingOffset4 = config.getConstellationTimingOffsetTime();

        assertEquals(constellationTimingOffset2, constellationTimingOffset3);
        assertEquals(constellationTimingOffset2, constellationTimingOffset4);
    }

    @Test
    void testGetSetMaskAngleDegrees() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setMaskAngleDegrees(maskAngleDegrees);

        // check
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
    }

    @Test
    void testGetSetMaskAngle() {
        final var config = new GNSSConfig();

        // check default value
        final var maskAngle1 = config.getMaskAngle();

        assertEquals(0.0, maskAngle1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.DEGREES, maskAngle1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngle2 = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        config.setMaskAngle(maskAngle2);

        // check
        final var maskAngle3 = new Angle(0.0, AngleUnit.RADIANS);
        config.getMaskAngle(maskAngle3);
        final var maskAngle4 = config.getMaskAngle();

        assertTrue(maskAngle2.equals(maskAngle3, ABSOLUTE_ERROR));
        assertTrue(maskAngle2.equals(maskAngle4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetSISErrorSD() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getSISErrorSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setSISErrorSD(sisErrorSD);

        // check
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setSISErrorSD(-1.0));
    }

    @Test
    void testGetSetSISErrorSDDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var sisErrorSD1 = config.getSISErrorSDDistance();

        assertEquals(0.0, sisErrorSD1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, sisErrorSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD2 = new Distance(sisErrorSD, DistanceUnit.METER);
        config.setSISErrorSDDistance(sisErrorSD2);

        // check
        final var sisErrorSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getSISErrorSDDistance(sisErrorSD3);
        final var sisErrorSD4 = config.getSISErrorSDDistance();

        assertEquals(sisErrorSD2, sisErrorSD3);
        assertEquals(sisErrorSD2, sisErrorSD4);

        // Force IllegalArgumentException
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setSISErrorSDDistance(distance));
    }

    @Test
    void testGetSetZenithIonosphereErrorSD() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setZenithIonosphereErrorSD(zenithIonosphereErrorSD);

        // check
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setZenithIonosphereErrorSD(-1.0));
    }

    @Test
    void testGetSetZenithIonosphereErrorSDDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var zenithIonosphereErrorSD1 = config.getZenithIonosphereErrorSDDistance();

        assertEquals(0.0, zenithIonosphereErrorSD1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, zenithIonosphereErrorSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD2 = new Distance(zenithIonosphereErrorSD, DistanceUnit.METER);
        config.setZenithIonosphereErrorSDDistance(zenithIonosphereErrorSD2);

        // check
        final var zenithIonosphereErrorSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getZenithIonosphereErrorSDDistance(zenithIonosphereErrorSD3);
        final var zenithIonosphereErrorSD4 = config.getZenithIonosphereErrorSDDistance();

        assertEquals(zenithIonosphereErrorSD2, zenithIonosphereErrorSD3);
        assertEquals(zenithIonosphereErrorSD2, zenithIonosphereErrorSD4);

        // Force IllegalArgumentException
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setZenithIonosphereErrorSDDistance(distance));
    }

    @Test
    void testGetSetZenithTroposphereErrorSD() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setZenithTroposphereErrorSD(zenithTroposphereErrorSD);

        // check
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setZenithTroposphereErrorSD(-1.0));
    }

    @Test
    void testGetSetZenithTroposphereErrorSDDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var zenithTroposphereErrorSD1 = config.getZenithTroposphereErrorSDDistance();

        assertEquals(0.0, zenithTroposphereErrorSD1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, zenithTroposphereErrorSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD2 = new Distance(zenithTroposphereErrorSD, DistanceUnit.METER);
        config.setZenithTroposphereErrorSDDistance(zenithTroposphereErrorSD2);

        // check
        final var zenithTroposphereErrorSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getZenithTroposphereErrorSDDistance(zenithTroposphereErrorSD3);
        final var zenithTroposphereErrorSD4 = config.getZenithTroposphereErrorSDDistance();

        assertEquals(zenithTroposphereErrorSD2, zenithTroposphereErrorSD3);
        assertEquals(zenithTroposphereErrorSD2, zenithTroposphereErrorSD4);

        // Force IllegalArgumentException
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setZenithTroposphereErrorSDDistance(distance));
    }

    @Test
    void testGetSetCodeTrackingErrorSD() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setCodeTrackingErrorSD(codeTrackingErrorSD);

        // check
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setCodeTrackingErrorSD(-1.0));
    }

    @Test
    void testGetSetCodeTrackingErrorSpeed() {
        final var config = new GNSSConfig();

        // check default value
        final var codeTrackingErrorSD1 = config.getCodeTrackingErrorSDSpeed();

        assertEquals(0.0, codeTrackingErrorSD1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, codeTrackingErrorSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD2 = new Speed(codeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        config.setCodeTrackingErrorSDSpeed(codeTrackingErrorSD2);

        // check
        final var codeTrackingErrorSD3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getCodeTrackingErrorSDSpeed(codeTrackingErrorSD3);
        final var codeTrackingErrorSD4 = config.getCodeTrackingErrorSDSpeed();

        assertEquals(codeTrackingErrorSD2, codeTrackingErrorSD3);
        assertEquals(codeTrackingErrorSD2, codeTrackingErrorSD4);
    }

    @Test
    void testGetSetRangeRateTrackingErrorSD() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setRangeRateTrackingErrorSD(rangeRateTrackingErrorSD);

        // check
        assertEquals(rangeRateTrackingErrorSD, config.getRangeRateTrackingErrorSD(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setRangeRateTrackingErrorSD(-1.0));
    }

    @Test
    void testGetSetRangeRateTrackingErrorSDSpeed() {
        final var config = new GNSSConfig();

        // check default value
        final var rangeRateTrackingErrorSD1 = config.getRangeRateTrackingErrorSDSpeed();

        assertEquals(0.0, rangeRateTrackingErrorSD1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, rangeRateTrackingErrorSD1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD2 = new Speed(rangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        config.setRangeRateTrackingErrorSDSpeed(rangeRateTrackingErrorSD2);

        // check
        final var rangeRateTrackingErrorSD3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getRangeRateTrackingErrorSDSpeed(rangeRateTrackingErrorSD3);
        final var rangeRateTrackingErrorSD4 = config.getRangeRateTrackingErrorSDSpeed();

        assertEquals(rangeRateTrackingErrorSD2, rangeRateTrackingErrorSD3);
        assertEquals(rangeRateTrackingErrorSD2, rangeRateTrackingErrorSD4);

        // Force IllegalArgumentException
        final var speed = new Speed(-1.0, SpeedUnit.METERS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setRangeRateTrackingErrorSDSpeed(speed));
    }

    @Test
    void testGetSetInitialReceiverClockOffset() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialReceiverClockOffset(initialReceiverClockOffset);

        // check
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
    }

    @Test
    void testGetSetInitialReceiverClockOffsetDistance() {
        final var config = new GNSSConfig();

        // check default value
        final var initialReceiverClockOffset1 = config.getInitialReceiverClockOffsetDistance();

        assertEquals(0.0, initialReceiverClockOffset1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialReceiverClockOffset1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset2 = new Distance(initialReceiverClockOffset, DistanceUnit.METER);
        config.setInitialReceiverClockOffsetDistance(initialReceiverClockOffset2);

        // check
        final var initialReceiverClockOffset3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialReceiverClockOffsetDistance(initialReceiverClockOffset3);
        final var initialReceiverClockOffset4 = config.getInitialReceiverClockOffsetDistance();

        assertEquals(initialReceiverClockOffset2, initialReceiverClockOffset3);
        assertEquals(initialReceiverClockOffset2, initialReceiverClockOffset4);
    }

    @Test
    void testGetSetInitialReceiverClockDrift() {
        final var config = new GNSSConfig();

        // check default value
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialReceiverClockDrift(initialReceiverClockDrift);

        // check
        assertEquals(config.getInitialReceiverClockDrift(), initialReceiverClockDrift, 0.0);
    }

    @Test
    void testGetSetInitialReceiverClockDriftSpeed() {
        final var config = new GNSSConfig();

        final var initialReceiverClockDrift1 = config.getInitialReceiverClockDriftSpeed();

        assertEquals(0.0, initialReceiverClockDrift1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialReceiverClockDrift1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift2 = new Speed(initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);
        config.setInitialReceiverClockDriftSpeed(initialReceiverClockDrift2);

        // check
        final var initialReceiverClockDrift3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialReceiverClockDriftSpeed(initialReceiverClockDrift3);
        final var initialReceiverClockDrift4 = config.getInitialReceiverClockDriftSpeed();

        assertEquals(initialReceiverClockDrift2, initialReceiverClockDrift3);
        assertEquals(initialReceiverClockDrift2, initialReceiverClockDrift4);
    }

    @Test
    void testSetValues1() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setValues(-1.0,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                3, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, -1.0, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, -1.0,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                -1.0, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0,
                initialReceiverClockOffset, initialReceiverClockDrift));
    }

    @Test
    void testSetValues2() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);


        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final var initialEstimatedEcefPositionXDistance = new Distance(initialEstimatedEcefPositionX,
                DistanceUnit.METER);
        final var initialEstimatedEcefPositionYDistance = new Distance(initialEstimatedEcefPositionY,
                DistanceUnit.METER);
        final var initialEstimatedEcefPositionZDistance = new Distance(initialEstimatedEcefPositionZ,
                DistanceUnit.METER);
        final var orbitalRadiusOfSatellitesDistance = new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final var satellitesInclinationAngle = new Angle(satellitesInclinationDegrees, AngleUnit.DEGREES);
        final var constellationLongitudeOffsetAngle = new Angle(constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final var constellationTimingOffsetTime = new Time(constellationTimingOffset, TimeUnit.SECOND);
        final var maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final var sisErrorSDDistance = new Distance(sisErrorSD, DistanceUnit.METER);
        final var zenithIonosphereErrorSDDistance = new Distance(zenithIonosphereErrorSD, DistanceUnit.METER);
        final var zenithTroposphereErrorSDDistance = new Distance(zenithTroposphereErrorSD, DistanceUnit.METER);
        final var codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var initialReceiverClockOffsetDistance = new Distance(initialReceiverClockOffset, DistanceUnit.METER);
        final var initialReceiverClockDriftSpeed = new Speed(initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime, initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance, initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        final var time = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(time, initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance, initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, 3, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, distance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                distance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        final var speed = new Speed(-1.0, SpeedUnit.METERS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, speed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance, initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, speed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
    }

    @Test
    void testSetValues3() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialEstimatedEcefPosition = new ECEFPosition(initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);

        config.setValues(epochInterval, initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setValues(-1.0,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                3, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, -1.0, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, -1.0,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                -1.0, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0,
                initialReceiverClockOffset, initialReceiverClockDrift));
    }

    @Test
    void testSetValues4() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final var initialEstimatedEcefPosition = new ECEFPosition(initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        final var orbitalRadiusOfSatellitesDistance = new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final var satellitesInclinationAngle = new Angle(satellitesInclinationDegrees, AngleUnit.DEGREES);
        final var constellationLongitudeOffsetAngle = new Angle(constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final var constellationTimingOffsetTime = new Time(constellationTimingOffset, TimeUnit.SECOND);
        final var maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final var sisErrorSDDistance = new Distance(sisErrorSD, DistanceUnit.METER);
        final var zenithIonosphereErrorSDDistance = new Distance(zenithIonosphereErrorSD, DistanceUnit.METER);
        final var zenithTroposphereErrorSDDistance = new Distance(zenithTroposphereErrorSD, DistanceUnit.METER);
        final var codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var initialReceiverClockOffsetDistance = new Distance(initialReceiverClockOffset, DistanceUnit.METER);
        final var initialReceiverClockDriftSpeed = new Speed(initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime, initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees,  config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals( sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        final var time = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(time, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, 3, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, distance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, distance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        final var speed = new Speed(-1.0, SpeedUnit.METERS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                speed, rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                sisErrorSDDistance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, speed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
    }

    @Test
    void testSetValues5() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialEstimatedPosition = new InhomogeneousPoint3D(initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);

        config.setValues(epochInterval, initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees,
                sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> config.setValues(-1.0, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                3, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, -1.0, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, -1.0,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                -1.0, zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, -1.0, codeTrackingErrorSD, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, -1.0, rangeRateTrackingErrorSD,
                initialReceiverClockOffset, initialReceiverClockDrift));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD, -1.0,
                initialReceiverClockOffset, initialReceiverClockDrift));
    }

    @Test
    void testSetValues6() {
        final var config = new GNSSConfig();

        // check default values
        assertEquals(0.0, config.getEpochInterval(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(0.0, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(MIN_SATELLITES, config.getNumberOfSatellites());
        assertEquals(0.0, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(0.0, config.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(0.0, config.getConstellationTimingOffset(), 0.0);
        assertEquals(0.0, config.getMaskAngleDegrees(), 0.0);
        assertEquals(0.0, config.getSISErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(0.0, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(0.0, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getRangeRateTrackingErrorSD(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(0.0, config.getInitialReceiverClockDrift(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final var initialEstimatedPosition = new InhomogeneousPoint3D(initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        final var orbitalRadiusOfSatellitesDistance = new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final var satellitesInclinationAngle = new Angle(satellitesInclinationDegrees, AngleUnit.DEGREES);
        final var constellationLongitudeOffsetAngle = new Angle(constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final var constellationTimingOffsetTime = new Time(constellationTimingOffset, TimeUnit.SECOND);
        final var maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final var sisErrorSDDistance = new Distance(sisErrorSD, DistanceUnit.METER);
        final var zenithIonosphereErrorSDDistance = new Distance(zenithIonosphereErrorSD, DistanceUnit.METER);
        final var zenithTroposphereErrorSDDistance = new Distance(zenithTroposphereErrorSD, DistanceUnit.METER);
        final var codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD, SpeedUnit.METERS_PER_SECOND);
        final var initialReceiverClockOffsetDistance = new Distance(initialReceiverClockOffset, DistanceUnit.METER);
        final var initialReceiverClockDriftSpeed = new Speed(initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime, initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check
        assertEquals(epochInterval, config.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config.getSatellitesInclinationDegrees(), ABSOLUTE_ERROR);
        assertEquals(constellationLongitudeOffsetDegrees, config.getConstellationLongitudeOffsetDegrees(),
                ABSOLUTE_ERROR);
        assertEquals(constellationTimingOffset, config.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config.getMaskAngleDegrees(), ABSOLUTE_ERROR);
        assertEquals(sisErrorSD, config.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config.getInitialReceiverClockDrift(), 0.0);

        // Force IllegalArgumentException
        final var time = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(time, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                3, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        final var distance = new Distance(-1.0, DistanceUnit.METER);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, distance, satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle,
                distance, zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                distance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, distance, codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        final var speed = new Speed(-1.0, SpeedUnit.METERS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, speed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
        assertThrows(IllegalArgumentException.class, () -> config.setValues(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed, speed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed));
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD, rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final var config2 = new GNSSConfig();

        config1.copyTo(config2);

        assertEquals(epochInterval, config2.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config2.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config2.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config2.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config2.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config2.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config2.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config2.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config2.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config2.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config2.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config2.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config2.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config2.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config2.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config2.getInitialReceiverClockDrift(), 0.0);
        assertEquals(config1, config2);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config2 = new GNSSConfig();

        config2.copyFrom(config1);

        assertEquals(epochInterval, config2.getEpochInterval(), 0.0);
        assertEquals(initialEstimatedEcefPositionX, config2.getInitialEstimatedEcefPositionX(), 0.0);
        assertEquals(initialEstimatedEcefPositionY, config2.getInitialEstimatedEcefPositionY(), 0.0);
        assertEquals(initialEstimatedEcefPositionZ, config2.getInitialEstimatedEcefPositionZ(), 0.0);
        assertEquals(numberOfSatellites, config2.getNumberOfSatellites());
        assertEquals(orbitalRadiusOfSatellites, config2.getOrbitalRadiusOfSatellites(), 0.0);
        assertEquals(satellitesInclinationDegrees, config2.getSatellitesInclinationDegrees(), 0.0);
        assertEquals(constellationLongitudeOffsetDegrees, config2.getConstellationLongitudeOffsetDegrees(), 0.0);
        assertEquals(constellationTimingOffset, config2.getConstellationTimingOffset(), 0.0);
        assertEquals(maskAngleDegrees, config2.getMaskAngleDegrees(), 0.0);
        assertEquals(sisErrorSD, config2.getSISErrorSD(), 0.0);
        assertEquals(zenithIonosphereErrorSD, config2.getZenithIonosphereErrorSD(), 0.0);
        assertEquals(zenithTroposphereErrorSD, config2.getZenithTroposphereErrorSD(), 0.0);
        assertEquals(codeTrackingErrorSD, config2.getCodeTrackingErrorSD(), 0.0);
        assertEquals(initialReceiverClockOffset, config2.getInitialReceiverClockOffset(), 0.0);
        assertEquals(initialReceiverClockDrift, config2.getInitialReceiverClockDrift(), 0.0);
        assertEquals(config1, config2);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config2 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config3 = new GNSSConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config2 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config3 = new GNSSConfig();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(config1.equals((Object)config1));
        //noinspection EqualsWithItself
        assertTrue(config1.equals(config1));
        assertTrue(config1.equals(config2));
        assertFalse(config1.equals(config3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(config1.equals((Object)null));
        assertFalse(config1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), config1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config2 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
        final var config3 = new GNSSConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        final var config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(config1);
        final var config2 = SerializationHelper.<GNSSConfig>deserialize(bytes);

        // check
        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }
}
