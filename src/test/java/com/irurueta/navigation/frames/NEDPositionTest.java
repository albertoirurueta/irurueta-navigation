package com.irurueta.navigation.frames;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class NEDPositionTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    @Test
    public void testConstructor() {

        // test empty constructor
        com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check
        assertEquals(position.getLatitude(), 0.0, 0.0);
        assertEquals(position.getLongitude(), 0.0, 0.0);
        assertEquals(position.getHeight(), 0.0, 0.0);

        assertEquals(position.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);


        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);

        // check
        assertEquals(position.getLatitude(), latitude, 0.0);
        assertEquals(position.getLongitude(), longitude, 0.0);
        assertEquals(position.getHeight(), height, 0.0);

        assertEquals(position.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(position.getLatitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(position.getLongitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position.getHeightDistance().getValue().doubleValue(), height, 0.0);
        assertEquals(position.getHeightDistance().getUnit(), DistanceUnit.METER);


        // test constructor with angles and distance coordinates
        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        position = new com.irurueta.navigation.frames.NEDPosition(latitudeAngle, longitudeAngle, heightDistance);

        // check
        assertEquals(position.getLatitude(), latitude, 0.0);
        assertEquals(position.getLongitude(), longitude, 0.0);
        assertEquals(position.getHeight(), height, 0.0);

        assertEquals(position.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(position.getLatitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(position.getLongitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position.getHeightDistance().getValue().doubleValue(), height, 0.0);
        assertEquals(position.getHeightDistance().getUnit(), DistanceUnit.METER);


        // test copy constructor
        final com.irurueta.navigation.frames.NEDPosition position2 = new com.irurueta.navigation.frames.NEDPosition(position);

        // check
        assertEquals(position2.getLatitude(), latitude, 0.0);
        assertEquals(position2.getLongitude(), longitude, 0.0);
        assertEquals(position2.getHeight(), height, 0.0);

        assertEquals(position2.getLatitudeAngle().getValue().doubleValue(), latitude, 0.0);
        assertEquals(position2.getLatitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position2.getLongitudeAngle().getValue().doubleValue(), longitude, 0.0);
        assertEquals(position2.getLongitudeAngle().getUnit(), AngleUnit.RADIANS);

        assertEquals(position2.getHeightDistance().getValue().doubleValue(), height, 0.0);
        assertEquals(position2.getHeightDistance().getUnit(), DistanceUnit.METER);
    }

    @Test
    public void testGetSetLatitude() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default value
        assertEquals(position.getLatitude(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLatitude(latitude);

        // check
        assertEquals(position.getLatitude(), latitude, 0.0);
    }

    @Test
    public void testGetSetLongitude() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default value
        assertEquals(position.getLongitude(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        position.setLongitude(longitude);

        // check
        assertEquals(position.getLongitude(), longitude, 0.0);
    }

    @Test
    public void testGetSetHeight() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default value
        assertEquals(position.getHeight(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setHeight(height);

        // check
        assertEquals(position.getHeight(), height, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default values
        assertEquals(position.getLatitude(), 0.0, 0.0);
        assertEquals(position.getLongitude(), 0.0, 0.0);
        assertEquals(position.getHeight(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        position.setCoordinates(latitude, longitude, height);

        // check
        assertEquals(position.getLatitude(), latitude, 0.0);
        assertEquals(position.getLongitude(), longitude, 0.0);
        assertEquals(position.getHeight(), height, 0.0);
    }

    @Test
    public void testGetSetLatitudeAngle() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default values
        assertEquals(position.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getLatitudeAngle().getUnit(), AngleUnit.RADIANS);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Angle latitudeAngle1 = new Angle(latitude, AngleUnit.RADIANS);
        position.setLatitudeAngle(latitudeAngle1);

        // check
        final Angle latitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLatitudeAngle(latitudeAngle2);
        final Angle latitudeAngle3 = position.getLatitudeAngle();

        assertEquals(latitudeAngle2.getValue().doubleValue(), latitude, 0.0);
        assertEquals(latitudeAngle2.getUnit(), AngleUnit.RADIANS);
        assertEquals(latitudeAngle2, latitudeAngle3);
    }

    @Test
    public void testGetSetLongitudeAngle() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default values
        assertEquals(position.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getLongitudeAngle().getUnit(), AngleUnit.RADIANS);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Angle longitudeAngle1 = new Angle(longitude, AngleUnit.RADIANS);
        position.setLongitudeAngle(longitudeAngle1);

        // check
        final Angle longitudeAngle2 = new Angle(0.0, AngleUnit.DEGREES);
        position.getLongitudeAngle(longitudeAngle2);
        final Angle longitudeAngle3 = position.getLongitudeAngle();

        assertEquals(longitudeAngle2.getValue().doubleValue(), longitude, 0.0);
        assertEquals(longitudeAngle2.getUnit(), AngleUnit.RADIANS);
        assertEquals(longitudeAngle2, longitudeAngle3);
    }

    @Test
    public void testGetSetHeightDistance() {
        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default values
        assertEquals(position.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getHeightDistance().getUnit(), DistanceUnit.METER);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Distance heightDistance1 = new Distance(height, DistanceUnit.METER);
        position.setHeightDistance(heightDistance1);

        // check
        final Distance heightDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getHeightDistance(heightDistance2);
        final Distance heightDistance3 = position.getHeightDistance();

        assertEquals(heightDistance2.getValue().doubleValue(), height, 0.0);
        assertEquals(heightDistance2.getUnit(), DistanceUnit.METER);
        assertEquals(heightDistance2, heightDistance3);
    }

    @Test
    public void testSetAngleAndDistanceCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final com.irurueta.navigation.frames.NEDPosition position = new com.irurueta.navigation.frames.NEDPosition();

        // check default values
        assertEquals(position.getLatitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getLatitudeAngle().getUnit(), AngleUnit.RADIANS);
        assertEquals(position.getLongitudeAngle().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getLongitudeAngle().getUnit(), AngleUnit.RADIANS);
        assertEquals(position.getHeightDistance().getValue().doubleValue(), 0.0, 0.0);
        assertEquals(position.getHeightDistance().getUnit(), DistanceUnit.METER);

        // set new values
        position.setCoordinates(latitudeAngle, longitudeAngle, heightDistance);

        // check
        final Angle latitudeAngle2 = position.getLatitudeAngle();
        final Angle longitudeAngle2 = position.getLongitudeAngle();
        final Distance heightDistance2 = position.getHeightDistance();

        assertEquals(latitudeAngle2.getValue().doubleValue(), latitude, 0.0);
        assertEquals(latitudeAngle2.getUnit(), AngleUnit.RADIANS);
        assertEquals(longitudeAngle2.getValue().doubleValue(), longitude, 0.0);
        assertEquals(longitudeAngle2.getUnit(), AngleUnit.RADIANS);
        assertEquals(heightDistance2.getValue().doubleValue(), height, 0.0);
        assertEquals(heightDistance2.getUnit(), DistanceUnit.METER);
        assertEquals(latitudeAngle, latitudeAngle2);
        assertEquals(longitudeAngle, longitudeAngle2);
        assertEquals(heightDistance, heightDistance2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition
            position1 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition position2 = new com.irurueta.navigation.frames.NEDPosition();
        position1.copyTo(position2);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition
            position1 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition position2 = new com.irurueta.navigation.frames.NEDPosition();
        position2.copyFrom(position1);

        // check
        assertEquals(position1.getLatitude(), position2.getLatitude(), 0.0);
        assertEquals(position1.getLongitude(), position2.getLongitude(), 0.0);
        assertEquals(position1.getHeight(), position2.getHeight(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition
            position1 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition
            position2 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition position3 = new com.irurueta.navigation.frames.NEDPosition();

        assertEquals(position1.hashCode(), position2.hashCode());
        assertNotEquals(position1.hashCode(), position3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition
            position1 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition
            position2 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition position3 = new com.irurueta.navigation.frames.NEDPosition();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(position1.equals((Object) position1));
        assertTrue(position1.equals(position1));
        assertTrue(position1.equals(position2));
        assertFalse(position1.equals(position3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertFalse(position1.equals((Object) null));
        assertFalse(position1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(position1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition
            position1 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition
            position2 = new com.irurueta.navigation.frames.NEDPosition(latitude, longitude, height);
        final com.irurueta.navigation.frames.NEDPosition position3 = new com.irurueta.navigation.frames.NEDPosition();

        assertTrue(position1.equals(position1, THRESHOLD));
        assertTrue(position1.equals(position2, THRESHOLD));
        assertFalse(position1.equals(position3, THRESHOLD));
        assertFalse(position1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final com.irurueta.navigation.frames.NEDPosition position1 = new NEDPosition(latitude, longitude, height);

        final Object position2 = position1.clone();

        assertEquals(position1, position2);
    }
}
