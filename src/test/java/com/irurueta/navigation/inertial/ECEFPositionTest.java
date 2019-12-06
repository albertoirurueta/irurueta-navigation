package com.irurueta.navigation.inertial;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ECEFPositionTest {

    private static final double THRESHOLD = 1e-6;

    private static final double MIN_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_POSITION_VALUE =
            Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructor() {

        // test empty constructor
        ECEFPosition position = new ECEFPosition();

        // check
        assertEquals(position.getX(), 0.0, 0.0);
        assertEquals(position.getY(), 0.0, 0.0);
        assertEquals(position.getZ(), 0.0, 0.0);

        Distance distanceX = position.getDistanceX();
        Distance distanceY = position.getDistanceY();
        Distance distanceZ = position.getDistanceZ();

        assertEquals(distanceX.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX.getUnit(), DistanceUnit.METER);
        assertEquals(distanceY.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY.getUnit(), DistanceUnit.METER);
        assertEquals(distanceZ.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ.getUnit(), DistanceUnit.METER);

        // test constructor with coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        position = new ECEFPosition(x, y, z);

        // check
        assertEquals(position.getX(), x, 0.0);
        assertEquals(position.getY(), y, 0.0);
        assertEquals(position.getZ(), z, 0.0);

        distanceX = position.getDistanceX();
        distanceY = position.getDistanceY();
        distanceZ = position.getDistanceZ();

        assertEquals(distanceX.getValue().doubleValue(), x, 0.0);
        assertEquals(distanceX.getUnit(), DistanceUnit.METER);
        assertEquals(distanceY.getValue().doubleValue(), y, 0.0);
        assertEquals(distanceY.getUnit(), DistanceUnit.METER);
        assertEquals(distanceZ.getValue().doubleValue(), z, 0.0);
        assertEquals(distanceZ.getUnit(), DistanceUnit.METER);


        // test constructor with coordinate distances
        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);

        position = new ECEFPosition(distanceX2, distanceY2, distanceZ2);

        // check
        assertEquals(position.getX(), x, 0.0);
        assertEquals(position.getY(), y, 0.0);
        assertEquals(position.getZ(), z, 0.0);

        distanceX = position.getDistanceX();
        distanceY = position.getDistanceY();
        distanceZ = position.getDistanceZ();

        assertEquals(distanceX, distanceX2);
        assertEquals(distanceY, distanceY2);
        assertEquals(distanceZ, distanceZ2);


        // test copy constructor
        final ECEFPosition position2 = new ECEFPosition(position);

        // check
        assertEquals(position.getX(), position2.getX(), 0.0);
        assertEquals(position.getY(), position2.getY(), 0.0);
        assertEquals(position.getZ(), position2.getZ(), 0.0);
        assertEquals(position, position2);
    }

    @Test
    public void testGetSetX() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        assertEquals(position.getX(), 0.0, 0.0);

        // set new value
        position.setX(x);

        // check
        assertEquals(position.getX(), x, 0.0);
    }

    @Test
    public void testGetSetY() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        assertEquals(position.getY(), 0.0, 0.0);

        // set new value
        position.setY(y);

        // check
        assertEquals(position.getY(), y, 0.0);
    }

    @Test
    public void testGetSetZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        assertEquals(position.getZ(), 0.0, 0.0);

        // set new value
        position.setZ(z);

        // check
        assertEquals(position.getZ(), z, 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default values
        assertEquals(position.getX(), 0.0, 0.0);
        assertEquals(position.getY(), 0.0, 0.0);
        assertEquals(position.getZ(), 0.0, 0.0);

        // set new values
        position.setCoordinates(x, y, z);

        // check
        assertEquals(position.getX(), x, 0.0);
        assertEquals(position.getY(), y, 0.0);
        assertEquals(position.getZ(), z, 0.0);
    }

    @Test
    public void testGetSetPosition() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        assertEquals(position.getPosition(), Point3D.create());

        // set new vale
        final Point3D pos1 = new InhomogeneousPoint3D(x, y, z);
        position.setPosition(pos1);

        // check
        final Point3D pos2 = position.getPosition();
        final Point3D pos3 = new InhomogeneousPoint3D();
        position.getPosition(pos3);

        assertEquals(pos1, pos2);
        assertEquals(pos1, pos3);
    }

    @Test
    public void testGetSetDistanceX() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        final Distance distanceX1 = position.getDistanceX();

        assertEquals(distanceX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceX1.getUnit(), DistanceUnit.METER);

        // set new value
        final Distance distanceX2 = new Distance(x, DistanceUnit.METER);
        position.setX(distanceX2);

        // check
        final Distance distanceX3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceX(distanceX3);
        final Distance distanceX4 = position.getDistanceX();

        assertEquals(distanceX2, distanceX3);
        assertEquals(distanceX2, distanceX4);
    }

    @Test
    public void testGetSetDistanceY() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default value
        final Distance distanceY1 = position.getDistanceY();

        assertEquals(distanceY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceY1.getUnit(), DistanceUnit.METER);

        // set new value
        final Distance distanceY2 = new Distance(y, DistanceUnit.METER);
        position.setY(distanceY2);

        // check
        final Distance distanceY3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceY(distanceY3);
        final Distance distanceY4 = position.getDistanceY();

        assertEquals(distanceY2, distanceY3);
        assertEquals(distanceY2, distanceY4);
    }

    @Test
    public void testGetSetDistanceZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default values
        final Distance distanceZ1 = position.getDistanceZ();

        assertEquals(distanceZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(distanceZ1.getUnit(), DistanceUnit.METER);

        // set new value
        final Distance distanceZ2 = new Distance(z, DistanceUnit.METER);
        position.setZ(distanceZ2);

        // check
        final Distance distanceZ3 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getDistanceZ(distanceZ3);
        final Distance distanceZ4 = position.getDistanceZ();

        assertEquals(distanceZ2, distanceZ3);
        assertEquals(distanceZ2, distanceZ4);
    }

    @Test
    public void testSetDistanceCoordinates() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition();

        // check default values
        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);
        position.setCoordinates(distanceX, distanceY, distanceZ);

        // check
        assertEquals(position.getDistanceX(), distanceX);
        assertEquals(position.getDistanceY(), distanceY);
        assertEquals(position.getDistanceZ(), distanceZ);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position = new ECEFPosition(x, y, z);

        final double norm = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0)
                + Math.pow(z, 2.0));
        assertEquals(position.getNorm(), norm, ABSOLUTE_ERROR);

        final Distance normDistance1 = new Distance(0.0, DistanceUnit.KILOMETER);
        position.getNormAsDistance(normDistance1);
        final Distance normDistance2 = position.getNormAsDistance();

        assertEquals(normDistance1.getValue().doubleValue(), norm, ABSOLUTE_ERROR);
        assertEquals(normDistance1.getUnit(), DistanceUnit.METER);
        assertEquals(normDistance1, normDistance2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);
        final ECEFPosition position2 = new ECEFPosition();

        position1.copyTo(position2);

        // check
        assertEquals(position2.getX(), x, 0.0);
        assertEquals(position2.getY(), y, 0.0);
        assertEquals(position2.getZ(), z, 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);
        final ECEFPosition position2 = new ECEFPosition();

        position2.copyFrom(position1);

        // check
        assertEquals(position2.getX(), x, 0.0);
        assertEquals(position2.getY(), y, 0.0);
        assertEquals(position2.getZ(), z, 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);
        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        final ECEFPosition position3 = new ECEFPosition();

        assertEquals(position1.hashCode(), position2.hashCode());
        assertNotEquals(position1.hashCode(), position3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);
        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        final ECEFPosition position3 = new ECEFPosition();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(position1.equals((Object)position1));
        assertTrue(position1.equals(position1));
        assertTrue(position1.equals(position2));
        assertFalse(position1.equals(position3));
        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertFalse(position1.equals((Object)null));
        assertFalse(position1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertFalse(position1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);
        final ECEFPosition position2 = new ECEFPosition(x, y, z);
        final ECEFPosition position3 = new ECEFPosition();

        assertTrue(position1.equals(position1, THRESHOLD));
        assertTrue(position1.equals(position2, THRESHOLD));
        assertFalse(position1.equals(position3, THRESHOLD));
        assertFalse(position1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double x = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double y = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);
        final double z = randomizer.nextDouble(MIN_POSITION_VALUE, MAX_POSITION_VALUE);

        final ECEFPosition position1 = new ECEFPosition(x, y, z);

        final Object position2 = position1.clone();

        assertEquals(position1, position2);
    }
}
