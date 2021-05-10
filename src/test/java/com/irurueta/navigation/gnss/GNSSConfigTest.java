package com.irurueta.navigation.gnss;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class GNSSConfigTest {

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(GNSSConfig.MIN_SATELLITES, MIN_SATELLITES);
    }

    @Test
    public void testConstructor() {
        // test empty constructor
        GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(-1.0, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test constructor with unit values
        final Time epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final Distance initialEstimatedEcefPositionXDistance =
                new Distance(initialEstimatedEcefPositionX, DistanceUnit.METER);
        final Distance initialEstimatedEcefPositionYDistance =
                new Distance(initialEstimatedEcefPositionY, DistanceUnit.METER);
        final Distance initialEstimatedEcefPositionZDistance =
                new Distance(initialEstimatedEcefPositionZ, DistanceUnit.METER);
        final Distance orbitalRadiusOfSatellitesDistance =
                new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final Angle satellitesInclinationAngle = new Angle(satellitesInclinationDegrees,
                AngleUnit.DEGREES);
        final Angle constellationLongitudeOffsetAngle = new Angle(
                constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final Time constellationTimingOffsetTime = new Time(constellationTimingOffset,
                TimeUnit.SECOND);
        final Angle maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final Distance sisErrorSDDistance = new Distance(sisErrorSD,
                DistanceUnit.METER);
        final Distance zenithIonosphereErrorSDDistance = new Distance(
                zenithIonosphereErrorSD, DistanceUnit.METER);
        final Distance zenithTroposphereErrorSDDistance = new Distance(
                zenithTroposphereErrorSD, DistanceUnit.METER);
        final Speed codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Speed rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialReceiverClockOffsetDistance = new Distance(
                initialReceiverClockOffset, DistanceUnit.METER);
        final Speed initialReceiverClockDriftSpeed = new Speed(
                initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config = new GNSSConfig(epochIntervalTime,
                initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, 3,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test constructor with values and ECEF position
        final ECEFPosition initialEstimatedEcefPosition =
                new ECEFPosition(initialEstimatedEcefPositionX,
                        initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(-1.0, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test constructor with unit values and ECEF position
        config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    3, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test constructor with values and position point
        final Point3D initialEstimatedPosition = new InhomogeneousPoint3D(
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ);
        config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(-1.0, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test constructor with unit values and position point
        config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);

        // check default values
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        config = null;
        try {
            config = new GNSSConfig(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    3, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config = new GNSSConfig(epochIntervalTime, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        assertNull(config);


        // test copy constructor
        config = new GNSSConfig(epochIntervalTime, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellitesDistance,
                satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
        final GNSSConfig config2 = new GNSSConfig(config);

        // check default values
        assertEquals(config2.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config2.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config2.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config2.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config2.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config2.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config2.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config2.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config2.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config2.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config2.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config2.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config2.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);
    }

    @Test
    public void testGetSetEpochInterval() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getEpochInterval(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setEpochInterval(epochInterval);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);

        // Force IllegalArgumentException
        try {
            config.setEpochInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetEpochIntervalTime() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Time epochInterval1 = config.getEpochIntervalTime();
        assertEquals(epochInterval1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(epochInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Time epochInterval2 = new Time(epochInterval, TimeUnit.SECOND);
        config.setEpochIntervalTime(epochInterval2);

        // check
        final Time epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        config.getEpochIntervalTime(epochInterval3);
        final Time epochInterval4 = config.getEpochIntervalTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);

        // Force IllegalArgumentException
        try {
            config.setEpochIntervalTime(new Time(-1.0, TimeUnit.SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionX() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionX(initialEstimatedEcefPositionX);

        // check
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionXDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance initialEstimatedEcefPositionX1 = config
                .getInitialEstimatedEcefPositionXDistance();

        assertEquals(initialEstimatedEcefPositionX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialEstimatedEcefPositionX1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance initialEstimatedEcefPositionX2 = new Distance(
                initialEstimatedEcefPositionX, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionXDistance(initialEstimatedEcefPositionX2);

        // check
        final Distance initialEstimatedEcefPositionX3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionXDistance(initialEstimatedEcefPositionX3);
        final Distance initialEstimatedEcefPositionX4 = config
                .getInitialEstimatedEcefPositionXDistance();

        assertEquals(initialEstimatedEcefPositionX2, initialEstimatedEcefPositionX3);
        assertEquals(initialEstimatedEcefPositionX2, initialEstimatedEcefPositionX4);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionY() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionY(initialEstimatedEcefPositionY);

        // check
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionYDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance initialEstimatedEcefPositionY1 = config
                .getInitialEstimatedEcefPositionYDistance();

        assertEquals(initialEstimatedEcefPositionY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialEstimatedEcefPositionY1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final Distance initialEstimatedEcefPositionY2 = new Distance(
                initialEstimatedEcefPositionY, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionYDistance(initialEstimatedEcefPositionY2);

        // check
        final Distance initialEstimatedEcefPositionY3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionYDistance(initialEstimatedEcefPositionY3);
        final Distance initialEstimatedEcefPositionY4 = config
                .getInitialEstimatedEcefPositionYDistance();

        assertEquals(initialEstimatedEcefPositionY2, initialEstimatedEcefPositionY3);
        assertEquals(initialEstimatedEcefPositionY2, initialEstimatedEcefPositionY4);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionZ() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialEstimatedEcefPositionZ(initialEstimatedEcefPositionZ);

        // check
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPositionZDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance initialEstimatedEcefPositionZ1 = config
                .getInitialEstimatedEcefPositionZDistance();

        assertEquals(initialEstimatedEcefPositionZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialEstimatedEcefPositionZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final Distance initialEstimatedEcefPositionZ2 = new Distance(
                initialEstimatedEcefPositionZ, DistanceUnit.METER);
        config.setInitialEstimatedEcefPositionZDistance(initialEstimatedEcefPositionZ2);

        // check
        final Distance initialEstimatedEcefPositionZ3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getInitialEstimatedEcefPositionZDistance(initialEstimatedEcefPositionZ3);
        final Distance initialEstimatedEcefPositionZ4 = config
                .getInitialEstimatedEcefPositionZDistance();

        assertEquals(initialEstimatedEcefPositionZ2, initialEstimatedEcefPositionZ3);
        assertEquals(initialEstimatedEcefPositionZ2, initialEstimatedEcefPositionZ4);
    }

    @Test
    public void testGetSetInitialEstimatedEcefPosition() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final ECEFPosition position1 = config.getInitialEstimatedEcefPosition();

        assertEquals(position1.getX(), 0.0, 0.0);
        assertEquals(position1.getY(), 0.0, 0.0);
        assertEquals(position1.getZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        config.setInitialEstimatedEcefPosition(position2);

        // check
        final ECEFPosition position3 = new ECEFPosition();
        config.getInitialEstimatedEcefPosition(position3);
        final ECEFPosition position4 = config.getInitialEstimatedEcefPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetInitialEstimatedPosition() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Point3D position1 = config.getInitialEstimatedPosition();

        assertEquals(position1, Point3D.create());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double y = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double z = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Point3D position2 = new InhomogeneousPoint3D(x, y, z);
        config.setInitialEstimatedPosition(position2);

        // check
        final Point3D position3 = new InhomogeneousPoint3D();
        config.getInitialEstimatedPosition(position3);
        final Point3D position4 = config.getInitialEstimatedPosition();

        assertEquals(position2, position3);
        assertEquals(position2, position4);
    }

    @Test
    public void testGetSetNumberOfSatellites() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getNumberOfSatellites(), GNSSConfig.MIN_SATELLITES);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);

        config.setNumberOfSatellites(numberOfSatellites);

        // check
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);

        // Force IllegalArgumentException
        try {
            config.setNumberOfSatellites(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetOrbitalRadiusOfSatellites() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        config.setOrbitalRadiusOfSatellites(orbitalRadiusOfSatellites);

        // check
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);

        // Force IllegalArgumentException
        try {
            config.setOrbitalRadiusOfSatellites(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetOrbitalRadiusOfSatellitesDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance orbitalRadiusOfSatellites1 = config
                .getOrbitalRadiusOfSatellitesDistance();

        assertEquals(orbitalRadiusOfSatellites1.getValue().doubleValue(), 0.0,
                0.0);
        assertEquals(orbitalRadiusOfSatellites1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final Distance orbitalRadiusOfSatellites2 = new Distance(
                orbitalRadiusOfSatellites, DistanceUnit.METER);
        config.setOrbitalRadiusOfSatellitesDistance(orbitalRadiusOfSatellites2);

        // check
        final Distance orbitalRadiusOfSatellites3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getOrbitalRadiusOfSatellitesDistance(orbitalRadiusOfSatellites3);
        final Distance orbitalRadiusOfSatellites4 = config
                .getOrbitalRadiusOfSatellitesDistance();

        assertEquals(orbitalRadiusOfSatellites2, orbitalRadiusOfSatellites3);
        assertEquals(orbitalRadiusOfSatellites2, orbitalRadiusOfSatellites4);

        // Force IllegalArgumentException
        try {
            config.setOrbitalRadiusOfSatellitesDistance(
                    new Distance(-1.0, DistanceUnit.METER));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetSatellitesInclinationDegrees() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        config.setSatellitesInclinationDegrees(satellitesInclinationDegrees);

        // check
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
    }

    @Test
    public void testGetSetSatellitesInclinationAngle() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Angle satellitesInclinationAngle1 = config
                .getSatellitesInclinationAngle();

        assertEquals(satellitesInclinationAngle1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(satellitesInclinationAngle1.getUnit(), AngleUnit.DEGREES);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final Angle satellitesInclinationAngle2 = new Angle(satellitesInclinationDegrees,
                AngleUnit.DEGREES);
        config.setSatellitesInclinationAngle(satellitesInclinationAngle2);

        // check
        final Angle satellitesInclinationAngle3 = new Angle(0.0, AngleUnit.RADIANS);
        config.getSatellitesInclinationAngle(satellitesInclinationAngle3);
        final Angle satellitesInclinationAngle4 = config.getSatellitesInclinationAngle();

        assertTrue(satellitesInclinationAngle2.equals(satellitesInclinationAngle3,
                ABSOLUTE_ERROR));
        assertTrue(satellitesInclinationAngle2.equals(satellitesInclinationAngle4,
                ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetConstellationLongitudeOffsetDegrees() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setConstellationLongitudeOffsetDegrees(
                constellationLongitudeOffsetDegrees);

        // check
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
    }

    @Test
    public void testGetSetConstellationLongitudeOffsetAngle() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Angle constellationLongitudeOffset1 = config
                .getConstellationLongitudeOffsetAngle();

        assertEquals(constellationLongitudeOffset1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(constellationLongitudeOffset1.getUnit(),
                AngleUnit.DEGREES);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Angle constellationLongitudeOffset2 = new Angle(
                constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        config.setConstellationLongitudeOffsetAngle(constellationLongitudeOffset2);

        // check
        final Angle constellationLongitudeOffset3 = new Angle(0.0,
                AngleUnit.RADIANS);
        config.getConstellationLongitudeOffsetAngle(constellationLongitudeOffset3);
        final Angle constellationLongitudeOffset4 = config
                .getConstellationLongitudeOffsetAngle();

        assertTrue(constellationLongitudeOffset2.equals(constellationLongitudeOffset3, ABSOLUTE_ERROR));
        assertTrue(constellationLongitudeOffset2.equals(constellationLongitudeOffset4, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetConstellationTimingOffset() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double constellationTimingOffset = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setConstellationTimingOffset(constellationTimingOffset);

        // check
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
    }

    @Test
    public void testGetSetConstellationTimingOffsetTime() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Time constellationTimingOffset1 = config
                .getConstellationTimingOffsetTime();

        assertEquals(constellationTimingOffset1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(constellationTimingOffset1.getUnit(), TimeUnit.SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final Time constellationTimingOffset2 = new Time(constellationTimingOffset,
                TimeUnit.SECOND);

        config.setConstellationTimingOffsetTime(constellationTimingOffset2);

        // check
        final Time constellationTimingOffset3 = new Time(0.0, TimeUnit.MILLISECOND);
        config.getConstellationTimingOffsetTime(constellationTimingOffset3);
        final Time constellationTimingOffset4 = config
                .getConstellationTimingOffsetTime();

        assertEquals(constellationTimingOffset2, constellationTimingOffset3);
        assertEquals(constellationTimingOffset2, constellationTimingOffset4);
    }

    @Test
    public void testGetSetMaskAngleDegrees() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setMaskAngleDegrees(maskAngleDegrees);

        // check
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
    }

    @Test
    public void testGetSetMaskAngle() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Angle maskAngle1 = config.getMaskAngle();

        assertEquals(maskAngle1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(maskAngle1.getUnit(), AngleUnit.DEGREES);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Angle maskAngle2 = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        config.setMaskAngle(maskAngle2);

        // check
        final Angle maskAngle3 = new Angle(0.0, AngleUnit.RADIANS);
        config.getMaskAngle(maskAngle3);
        final Angle maskAngle4 = config.getMaskAngle();

        assertTrue(maskAngle2.equals(maskAngle3, ABSOLUTE_ERROR));
        assertTrue(maskAngle2.equals(maskAngle4, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetSISErrorSD() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setSISErrorSD(sisErrorSD);

        // check
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);

        // Force IllegalArgumentException
        try {
            config.setSISErrorSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetSISErrorSDDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance sisErrorSD1 = config.getSISErrorSDDistance();

        assertEquals(sisErrorSD1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(sisErrorSD1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance sisErrorSD2 = new Distance(sisErrorSD, DistanceUnit.METER);
        config.setSISErrorSDDistance(sisErrorSD2);

        // check
        final Distance sisErrorSD3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getSISErrorSDDistance(sisErrorSD3);
        final Distance sisErrorSD4 = config.getSISErrorSDDistance();

        assertEquals(sisErrorSD2, sisErrorSD3);
        assertEquals(sisErrorSD2, sisErrorSD4);

        // Force IllegalArgumentException
        try {
            config.setSISErrorSDDistance(new Distance(-1.0, DistanceUnit.METER));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetZenithIonosphereErrorSD() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setZenithIonosphereErrorSD(zenithIonosphereErrorSD);

        // check
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);

        // Force IllegalArgumentException
        try {
            config.setZenithIonosphereErrorSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetZenithIonosphereErrorSDDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance zenithIonosphereErrorSD1 = config
                .getZenithIonosphereErrorSDDistance();

        assertEquals(zenithIonosphereErrorSD1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(zenithIonosphereErrorSD1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance zenithIonosphereErrorSD2 = new Distance(
                zenithIonosphereErrorSD, DistanceUnit.METER);
        config.setZenithIonosphereErrorSDDistance(zenithIonosphereErrorSD2);

        // check
        final Distance zenithIonosphereErrorSD3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getZenithIonosphereErrorSDDistance(zenithIonosphereErrorSD3);
        final Distance zenithIonosphereErrorSD4 = config
                .getZenithIonosphereErrorSDDistance();

        assertEquals(zenithIonosphereErrorSD2, zenithIonosphereErrorSD3);
        assertEquals(zenithIonosphereErrorSD2, zenithIonosphereErrorSD4);

        // Force IllegalArgumentException
        try {
            config.setZenithIonosphereErrorSDDistance(
                    new Distance(-1.0, DistanceUnit.METER));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetZenithTroposphereErrorSD() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double zenithTroposphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setZenithTroposphereErrorSD(zenithTroposphereErrorSD);

        // check
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);

        // Force IllegalArgumentException
        try {
            config.setZenithTroposphereErrorSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetZenithTroposphereErrorSDDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance zenithTroposphereErrorSD1 = config
                .getZenithTroposphereErrorSDDistance();

        assertEquals(zenithTroposphereErrorSD1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(zenithTroposphereErrorSD1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double zenithTroposphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance zenithTroposphereErrorSD2 = new Distance(
                zenithTroposphereErrorSD, DistanceUnit.METER);
        config.setZenithTroposphereErrorSDDistance(zenithTroposphereErrorSD2);

        // check
        final Distance zenithTroposphereErrorSD3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getZenithTroposphereErrorSDDistance(zenithTroposphereErrorSD3);
        final Distance zenithTroposphereErrorSD4 = config
                .getZenithTroposphereErrorSDDistance();

        assertEquals(zenithTroposphereErrorSD2, zenithTroposphereErrorSD3);
        assertEquals(zenithTroposphereErrorSD2, zenithTroposphereErrorSD4);

        // Force IllegalArgumentException
        try {
            config.setZenithTroposphereErrorSDDistance(
                    new Distance(-1.0, DistanceUnit.METER));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCodeTrackingErrorSD() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setCodeTrackingErrorSD(codeTrackingErrorSD);

        // check
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);

        // Force IllegalArgumentException
        try {
            config.setCodeTrackingErrorSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetCodeTrackingErrorSpeed() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Speed codeTrackingErrorSD1 = config.getCodeTrackingErrorSDSpeed();

        assertEquals(codeTrackingErrorSD1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(codeTrackingErrorSD1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double codeTrackingErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Speed codeTrackingErrorSD2 = new Speed(codeTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        config.setCodeTrackingErrorSDSpeed(codeTrackingErrorSD2);

        // check
        final Speed codeTrackingErrorSD3 = new Speed(0.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        config.getCodeTrackingErrorSDSpeed(codeTrackingErrorSD3);
        final Speed codeTrackingErrorSD4 = config.getCodeTrackingErrorSDSpeed();

        assertEquals(codeTrackingErrorSD2, codeTrackingErrorSD3);
        assertEquals(codeTrackingErrorSD2, codeTrackingErrorSD4);
    }

    @Test
    public void testGetSetRangeRateTrackingErrorSD() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setRangeRateTrackingErrorSD(rangeRateTrackingErrorSD);

        // check
        assertEquals(config.getRangeRateTrackingErrorSD(),
                rangeRateTrackingErrorSD, 0.0);

        // Force IllegalArgumentException
        try {
            config.setRangeRateTrackingErrorSD(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetRangeRateTrackingErrorSDSpeed() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Speed rangeRateTrackingErrorSD1 = config.getRangeRateTrackingErrorSDSpeed();

        assertEquals(rangeRateTrackingErrorSD1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(rangeRateTrackingErrorSD1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Speed rangeRateTrackingErrorSD2 = new Speed(rangeRateTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        config.setRangeRateTrackingErrorSDSpeed(rangeRateTrackingErrorSD2);

        // check
        final Speed rangeRateTrackingErrorSD3 = new Speed(0.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        config.getRangeRateTrackingErrorSDSpeed(rangeRateTrackingErrorSD3);
        final Speed rangeRateTrackingErrorSD4 = config
                .getRangeRateTrackingErrorSDSpeed();

        assertEquals(rangeRateTrackingErrorSD2, rangeRateTrackingErrorSD3);
        assertEquals(rangeRateTrackingErrorSD2, rangeRateTrackingErrorSD4);

        // Force IllegalArgumentException
        try {
            config.setRangeRateTrackingErrorSDSpeed(
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testGetSetInitialReceiverClockOffset() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialReceiverClockOffset = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialReceiverClockOffset(initialReceiverClockOffset);

        // check
        assertEquals(config.getInitialReceiverClockOffset(), initialReceiverClockOffset,
                0.0);
    }

    @Test
    public void testGetSetInitialReceiverClockOffsetDistance() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        final Distance initialReceiverClockOffset1 = config
                .getInitialReceiverClockOffsetDistance();

        assertEquals(initialReceiverClockOffset1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(initialReceiverClockOffset1.getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialReceiverClockOffset = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Distance initialReceiverClockOffset2 = new Distance(initialReceiverClockOffset,
                DistanceUnit.METER);
        config.setInitialReceiverClockOffsetDistance(initialReceiverClockOffset2);

        // check
        final Distance initialReceiverClockOffset3 = new Distance(0.0,
                DistanceUnit.KILOMETER);
        config.getInitialReceiverClockOffsetDistance(initialReceiverClockOffset3);
        final Distance initialReceiverClockOffset4 = config
                .getInitialReceiverClockOffsetDistance();

        assertEquals(initialReceiverClockOffset2, initialReceiverClockOffset3);
        assertEquals(initialReceiverClockOffset2, initialReceiverClockOffset4);
    }

    @Test
    public void testGetSetInitialReceiverClockDrift() {
        final GNSSConfig config = new GNSSConfig();

        // check default value
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialReceiverClockDrift = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        config.setInitialReceiverClockDrift(initialReceiverClockDrift);

        // check
        assertEquals(config.getInitialReceiverClockDrift(), initialReceiverClockDrift,
                0.0);
    }

    @Test
    public void testGetSetInitialReceiverClockDriftSpeed() {
        final GNSSConfig config = new GNSSConfig();

        final Speed initialReceiverClockDrift1 = config.getInitialReceiverClockDriftSpeed();

        assertEquals(initialReceiverClockDrift1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(initialReceiverClockDrift1.getUnit(), SpeedUnit.METERS_PER_SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialReceiverClockDrift = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final Speed initialReceiverClockDrift2 = new Speed(initialReceiverClockDrift,
                SpeedUnit.METERS_PER_SECOND);
        config.setInitialReceiverClockDriftSpeed(initialReceiverClockDrift2);

        // check
        final Speed initialReceiverClockDrift3 = new Speed(0.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialReceiverClockDriftSpeed(initialReceiverClockDrift3);
        final Speed initialReceiverClockDrift4 = config
                .getInitialReceiverClockDriftSpeed();

        assertEquals(initialReceiverClockDrift2, initialReceiverClockDrift3);
        assertEquals(initialReceiverClockDrift2, initialReceiverClockDrift4);
    }

    @Test
    public void testSetValues1() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        config.setValues(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(-1.0, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPositionX,
                    initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testSetValues2() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final Time epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final Distance initialEstimatedEcefPositionXDistance =
                new Distance(initialEstimatedEcefPositionX, DistanceUnit.METER);
        final Distance initialEstimatedEcefPositionYDistance =
                new Distance(initialEstimatedEcefPositionY, DistanceUnit.METER);
        final Distance initialEstimatedEcefPositionZDistance =
                new Distance(initialEstimatedEcefPositionZ, DistanceUnit.METER);
        final Distance orbitalRadiusOfSatellitesDistance =
                new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final Angle satellitesInclinationAngle = new Angle(satellitesInclinationDegrees,
                AngleUnit.DEGREES);
        final Angle constellationLongitudeOffsetAngle = new Angle(
                constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final Time constellationTimingOffsetTime = new Time(constellationTimingOffset,
                TimeUnit.SECOND);
        final Angle maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final Distance sisErrorSDDistance = new Distance(sisErrorSD,
                DistanceUnit.METER);
        final Distance zenithIonosphereErrorSDDistance = new Distance(
                zenithIonosphereErrorSD, DistanceUnit.METER);
        final Distance zenithTroposphereErrorSDDistance = new Distance(
                zenithTroposphereErrorSD, DistanceUnit.METER);
        final Speed codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Speed rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialReceiverClockOffsetDistance = new Distance(
                initialReceiverClockOffset, DistanceUnit.METER);
        final Speed initialReceiverClockDriftSpeed = new Speed(
                initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime,
                initialEstimatedEcefPositionXDistance,
                initialEstimatedEcefPositionYDistance,
                initialEstimatedEcefPositionZDistance, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees,
                ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, 3,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPositionXDistance,
                    initialEstimatedEcefPositionYDistance,
                    initialEstimatedEcefPositionZDistance, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testSetValues3() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final ECEFPosition initialEstimatedEcefPosition =
                new ECEFPosition(initialEstimatedEcefPositionX,
                        initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);

        config.setValues(epochInterval, initialEstimatedEcefPosition,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(-1.0, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedEcefPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testSetValues4() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final Time epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final ECEFPosition initialEstimatedEcefPosition =
                new ECEFPosition(initialEstimatedEcefPositionX,
                        initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ);
        final Distance orbitalRadiusOfSatellitesDistance =
                new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final Angle satellitesInclinationAngle = new Angle(satellitesInclinationDegrees,
                AngleUnit.DEGREES);
        final Angle constellationLongitudeOffsetAngle = new Angle(
                constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final Time constellationTimingOffsetTime = new Time(constellationTimingOffset,
                TimeUnit.SECOND);
        final Angle maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final Distance sisErrorSDDistance = new Distance(sisErrorSD,
                DistanceUnit.METER);
        final Distance zenithIonosphereErrorSDDistance = new Distance(
                zenithIonosphereErrorSD, DistanceUnit.METER);
        final Distance zenithTroposphereErrorSDDistance = new Distance(
                zenithTroposphereErrorSD, DistanceUnit.METER);
        final Speed codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Speed rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialReceiverClockOffsetDistance = new Distance(
                initialReceiverClockOffset, DistanceUnit.METER);
        final Speed initialReceiverClockDriftSpeed = new Speed(
                initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime,
                initialEstimatedEcefPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, 3,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedEcefPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testSetValues5() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final Point3D initialEstimatedPosition = new InhomogeneousPoint3D(
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ);

        config.setValues(epochInterval, initialEstimatedPosition,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(-1.0, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    3, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, -1.0,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, -1.0,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    -1.0, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, -1.0,
                    codeTrackingErrorSD, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    -1.0, rangeRateTrackingErrorSD,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochInterval, initialEstimatedPosition,
                    numberOfSatellites, orbitalRadiusOfSatellites,
                    satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                    constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                    zenithIonosphereErrorSD, zenithTroposphereErrorSD,
                    codeTrackingErrorSD, -1.0,
                    initialReceiverClockOffset, initialReceiverClockDrift);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testSetValues6() {
        final GNSSConfig config = new GNSSConfig();

        // check default values
        assertEquals(config.getEpochInterval(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(), 0.0, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(), 0.0, 0.0);
        assertEquals(config.getNumberOfSatellites(), MIN_SATELLITES);
        assertEquals(config.getOrbitalRadiusOfSatellites(), 0.0, 0.0);
        assertEquals(config.getSatellitesInclinationDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(), 0.0, 0.0);
        assertEquals(config.getConstellationTimingOffset(), 0.0, 0.0);
        assertEquals(config.getMaskAngleDegrees(), 0.0, 0.0);
        assertEquals(config.getSISErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), 0.0, 0.0);
        assertEquals(config.getCodeTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getRangeRateTrackingErrorSD(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(), 0.0, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(), 0.0, 0.0);


        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final Time epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        final Point3D initialEstimatedPosition = new InhomogeneousPoint3D(
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ);
        final Distance orbitalRadiusOfSatellitesDistance =
                new Distance(orbitalRadiusOfSatellites, DistanceUnit.METER);
        final Angle satellitesInclinationAngle = new Angle(satellitesInclinationDegrees,
                AngleUnit.DEGREES);
        final Angle constellationLongitudeOffsetAngle = new Angle(
                constellationLongitudeOffsetDegrees, AngleUnit.DEGREES);
        final Time constellationTimingOffsetTime = new Time(constellationTimingOffset,
                TimeUnit.SECOND);
        final Angle maskAngle = new Angle(maskAngleDegrees, AngleUnit.DEGREES);
        final Distance sisErrorSDDistance = new Distance(sisErrorSD,
                DistanceUnit.METER);
        final Distance zenithIonosphereErrorSDDistance = new Distance(
                zenithIonosphereErrorSD, DistanceUnit.METER);
        final Distance zenithTroposphereErrorSDDistance = new Distance(
                zenithTroposphereErrorSD, DistanceUnit.METER);
        final Speed codeTrackingErrorSDSpeed = new Speed(codeTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Speed rangeRateTrackingErrorSDSpeed = new Speed(rangeRateTrackingErrorSD,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialReceiverClockOffsetDistance = new Distance(
                initialReceiverClockOffset, DistanceUnit.METER);
        final Speed initialReceiverClockDriftSpeed = new Speed(
                initialReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);

        config.setValues(epochIntervalTime,
                initialEstimatedPosition, numberOfSatellites,
                orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                initialReceiverClockDriftSpeed);

        // check
        assertEquals(config.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config.getMaskAngleDegrees(), maskAngleDegrees, ABSOLUTE_ERROR);
        assertEquals(config.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);

        // Force IllegalArgumentException
        try {
            config.setValues(new Time(-1.0, TimeUnit.SECOND),
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, 3,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    new Distance(-1.0, DistanceUnit.METER),
                    satellitesInclinationAngle, constellationLongitudeOffsetAngle,
                    constellationTimingOffsetTime, maskAngle, sisErrorSDDistance,
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, new Distance(-1.0, DistanceUnit.METER),
                    zenithIonosphereErrorSDDistance, zenithTroposphereErrorSDDistance,
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    new Distance(-1.0, DistanceUnit.METER),
                    codeTrackingErrorSDSpeed, rangeRateTrackingErrorSDSpeed,
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    rangeRateTrackingErrorSDSpeed, initialReceiverClockOffsetDistance,
                    initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
        try {
            config.setValues(epochIntervalTime,
                    initialEstimatedPosition, numberOfSatellites,
                    orbitalRadiusOfSatellitesDistance, satellitesInclinationAngle,
                    constellationLongitudeOffsetAngle, constellationTimingOffsetTime,
                    maskAngle, sisErrorSDDistance, zenithIonosphereErrorSDDistance,
                    zenithTroposphereErrorSDDistance, codeTrackingErrorSDSpeed,
                    new Speed(-1.0, SpeedUnit.METERS_PER_SECOND),
                    initialReceiverClockOffsetDistance, initialReceiverClockDriftSpeed);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) { }
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config2 = new GNSSConfig();

        config1.copyTo(config2);


        assertEquals(config2.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config2.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config2.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config2.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config2.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config2.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config2.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config2.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config2.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config2.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config2.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config2.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config2.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);
        assertEquals(config1, config2);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config2 = new GNSSConfig();

        config2.copyFrom(config1);


        assertEquals(config2.getEpochInterval(), epochInterval, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionX(),
                initialEstimatedEcefPositionX, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionY(),
                initialEstimatedEcefPositionY, 0.0);
        assertEquals(config2.getInitialEstimatedEcefPositionZ(),
                initialEstimatedEcefPositionZ, 0.0);
        assertEquals(config2.getNumberOfSatellites(), numberOfSatellites);
        assertEquals(config2.getOrbitalRadiusOfSatellites(), orbitalRadiusOfSatellites,
                0.0);
        assertEquals(config2.getSatellitesInclinationDegrees(),
                satellitesInclinationDegrees, 0.0);
        assertEquals(config2.getConstellationLongitudeOffsetDegrees(),
                constellationLongitudeOffsetDegrees, 0.0);
        assertEquals(config2.getConstellationTimingOffset(), constellationTimingOffset,
                0.0);
        assertEquals(config2.getMaskAngleDegrees(), maskAngleDegrees, 0.0);
        assertEquals(config2.getSISErrorSD(), sisErrorSD, 0.0);
        assertEquals(config2.getZenithIonosphereErrorSD(), zenithIonosphereErrorSD,
                0.0);
        assertEquals(config2.getZenithTroposphereErrorSD(), zenithTroposphereErrorSD,
                0.0);
        assertEquals(config2.getCodeTrackingErrorSD(), codeTrackingErrorSD, 0.0);
        assertEquals(config2.getInitialReceiverClockOffset(),
                initialReceiverClockOffset, 0.0);
        assertEquals(config2.getInitialReceiverClockDrift(),
                initialReceiverClockDrift, 0.0);
        assertEquals(config1, config2);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config2 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config3 = new GNSSConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config2 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config3 = new GNSSConfig();

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
        assertFalse(config1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config2 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
        final GNSSConfig config3 = new GNSSConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        final GNSSConfig config1 = new GNSSConfig(epochInterval,
                initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites,
                orbitalRadiusOfSatellites, satellitesInclinationDegrees,
                constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD,
                zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }
}
