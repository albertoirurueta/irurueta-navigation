package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class TimedBodyKinematicsAndMagneticFluxDensityTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.
    private static final double MIN_MAGNETIC_FLUX_VALUE = 30e-6;
    private static final double MAX_MAGNETIC_FLUX_VALUE = 70e-6;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor1() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), 0.0, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() {
        final BodyKinematics kinematics = new BodyKinematics();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), 0.0, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), 0.0, 0.0);
    }

    @Test
    public void testConstructor3() {
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), 0.0, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), 0.0, 0.0);
    }

    @Test
    public void testConstructor4() {
        final BodyKinematics kinematics = new BodyKinematics();
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), 0.0, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), 0.0, 0.0);
    }

    @Test
    public void testConstructor5() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                timestampSeconds);

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor6() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                time);

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor7() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = new BodyKinematics();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, timestampSeconds);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor8() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final BodyKinematics kinematics = new BodyKinematics();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, time);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor9() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b,
                timestampSeconds);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor10() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b,
                time);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor11() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, timestampSeconds);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor12() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, time);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
        Time time1 = tkb.getTimestamp();
        assertEquals(time1.getValue().doubleValue(), timestampSeconds, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final TimedBodyKinematics tk = tkb.getTimedKinematics();
        assertSame(tk.getKinematics(), kinematics);
        assertEquals(tk.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testConstructor13() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, timestampSeconds);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(tkb1);

        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testGetSetKinematics() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb =
                new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(tkb.getKinematics());

        // set new value
        final BodyKinematics kinematics = new BodyKinematics();
        tkb.setKinematics(kinematics);

        // check
        assertSame(tkb.getKinematics(), kinematics);
    }

    @Test
    public void testGetSetMagneticFluxDensity() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb =
                new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(tkb.getMagneticFluxDensity());

        // set new value
        final BodyMagneticFluxDensity b = new BodyMagneticFluxDensity();
        tkb.setMagneticFluxDensity(b);

        // check
        assertSame(tkb.getMagneticFluxDensity(), b);
    }

    @Test
    public void testGetSetTimestampSeconds() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb =
                new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertEquals(tkb.getTimestampSeconds(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        tkb.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(tkb.getTimestampSeconds(), timestampSeconds, 0.0);
    }

    @Test
    public void testGetSetTimestamp() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb =
                new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        final Time time1 = tkb.getTimestamp();

        assertEquals(time1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final Time time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        tkb.setTimestamp(time2);

        final Time time3 = tkb.getTimestamp();
        final Time time4 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    public void testGetSetTimedKinematics() {
        final TimedBodyKinematicsAndMagneticFluxDensity tkb =
                new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        final TimedBodyKinematics tk1 = tkb.getTimedKinematics();
        assertNull(tk1.getKinematics());
        assertEquals(tk1.getTimestampSeconds(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = createKinematics();
        final TimedBodyKinematics tk2 = new TimedBodyKinematics(kinematics, timestampSeconds);
        tkb.setTimedKinematics(tk2);

        // check
        final TimedBodyKinematics tk3 = tkb.getTimedKinematics();
        final TimedBodyKinematics tk4 = new TimedBodyKinematics();
        tkb.getTimedKinematics(tk4);
        assertEquals(tk2, tk3);
        assertNotSame(tk2, tk3);
        assertEquals(tk2, tk4);
        assertNotSame(tk2, tk4);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, timestampSeconds);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity();

        tkb2.copyFrom(tkb1);

        // check
        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();
        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, timestampSeconds);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity();

        tkb1.copyTo(tkb2);

        // check
        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics2, b2, timestampSeconds2);

        assertEquals(tkb1.hashCode(), tkb2.hashCode());
        assertNotEquals(tkb1.hashCode(), tkb3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics2, b2, timestampSeconds2);

        //noinspection ConstantConditions,SimplifiableAssertion
        assertTrue(tkb1.equals((Object) tkb1));
        assertTrue(tkb1.equals(tkb1));
        assertTrue(tkb1.equals(tkb2));
        assertFalse(tkb1.equals(tkb3));
        //noinspection ConstantConditions,SimplifiableAssertion
        assertFalse(tkb1.equals((Object) null));
        assertFalse(tkb1.equals(null));
        //noinspection SimplifiableAssertion
        assertFalse(tkb1.equals(new Object()));
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics1 = createKinematics();
        final BodyMagneticFluxDensity b1 = createMagneticFlux();

        final BodyKinematics kinematics2 = createKinematics();
        final BodyMagneticFluxDensity b2 = createMagneticFlux();

        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics1, b1, timestampSeconds1);

        final TimedBodyKinematicsAndMagneticFluxDensity tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics2, b2, timestampSeconds2);

        assertTrue(tkb1.equals(tkb1, THRESHOLD));
        assertTrue(tkb1.equals(tkb2, THRESHOLD));
        assertFalse(tkb1.equals(tkb3, THRESHOLD));
        assertFalse(tkb1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final BodyKinematics kinematics = createKinematics();
        final BodyMagneticFluxDensity b = createMagneticFlux();

        final TimedBodyKinematicsAndMagneticFluxDensity tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(
                kinematics, b, timestampSeconds);

        final Object tkb2 = tkb1.clone();

        // check
        assertEquals(tkb1, tkb2);
    }

    private BodyKinematics createKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE,
                MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }


    private BodyMagneticFluxDensity createMagneticFlux() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double bx = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final double by = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final double bz = randomizer.nextDouble(
                MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);

        return new BodyMagneticFluxDensity(bx, by,bz);
    }
}
