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
package com.irurueta.navigation.lateration;

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Sphere;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import static org.junit.Assert.*;

public class RobustLateration3DSolverTest implements
        RobustLaterationSolverListener<Point3D> {

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustLateration3DSolver solver = RobustLateration3DSolver.create(
                RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.LMedS);

        // check
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.PROMedS);

        // check
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(this,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with positions, distances and method
        final Point3D[] positions = {
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D()
        };
        final double[] distances = {1.0, 1.0, 1.0, 1.0};

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                RobustEstimatorMethod.LMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with positions, distances, standard deviations and method
        final double[] standardDeviations = new double[4];

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, RobustEstimatorMethod.LMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with positions, distances, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                this, RobustEstimatorMethod.LMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with positions, distances, standard deviations, listener and
        // method

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this, RobustEstimatorMethod.LMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with spheres and method
        final Sphere[] spheres = {
                new Sphere(positions[0], distances[0]),
                new Sphere(positions[1], distances[1]),
                new Sphere(positions[2], distances[2]),
                new Sphere(positions[3], distances[3])
        };

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres,
                RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres,
                RobustEstimatorMethod.LMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres,
                RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres,
                RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres,
                RobustEstimatorMethod.PROMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with spheres, standard deviations and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                RobustEstimatorMethod.LMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                RobustEstimatorMethod.PROMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with spheres, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, this,
                RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, this,
                RobustEstimatorMethod.LMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, this,
                RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, this,
                RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, this,
                RobustEstimatorMethod.PROMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with spheres, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this, RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this, RobustEstimatorMethod.LMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this, RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this, RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this, RobustEstimatorMethod.PROMedS);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores and method
        final double[] qualityScores = new double[4];

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, positions, distances and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, positions, distances, standard deviations
        // and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, positions, distance, standard deviations,
        // listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, positions, distances, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, spheres and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, spheres, standard deviations and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with quality scores, spheres, standard deviations, listener
        // and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof RANSACRobustLateration3DSolver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this, RobustEstimatorMethod.LMedS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof LMedSRobustLateration3DSolver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof MSACRobustLateration3DSolver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROSACRobustLateration3DSolver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this, RobustEstimatorMethod.PROMedS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);


        // create with default method
        solver = RobustLateration3DSolver.create();

        // check
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with listener and default method
        solver = RobustLateration3DSolver.create(this);

        // check
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with positions, distances and default method
        solver = RobustLateration3DSolver.create(positions, distances);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with positions, distances, standard deviations and default method
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with positions, distances, listener and default method
        solver = RobustLateration3DSolver.create(positions, distances, this);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with positions, distances, standard deviations, listener and
        // default method
        solver = RobustLateration3DSolver.create(positions, distances,
                standardDeviations, this);

        // check
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with spheres and default method
        solver = RobustLateration3DSolver.create(spheres);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with spheres, standard deviations and default method
        solver = RobustLateration3DSolver.create(spheres, standardDeviations);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with spheres, listener and default method
        solver = RobustLateration3DSolver.create(spheres, this);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with spheres, standard deviations, listener and default method
        solver = RobustLateration3DSolver.create(spheres, standardDeviations,
                this);

        // check
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores
        solver = RobustLateration3DSolver.create(qualityScores);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, this);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, positions, distances and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, positions, distances, standard deviations
        // and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, positions, distances, standard deviations,
        // listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, standardDeviations, this);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, positions, distances, listener and
        // default method
        solver = RobustLateration3DSolver.create(qualityScores, positions,
                distances, this);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        assertSame(solver.getPositions(), positions);
        assertSame(solver.getDistances(), distances);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, spheres and default method
        solver = RobustLateration3DSolver.create(qualityScores, spheres);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, spheres, standard deviations and default
        // method
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);

        // create with quality scores, spheres, standard deviations, listener and
        // default method
        solver = RobustLateration3DSolver.create(qualityScores, spheres,
                standardDeviations, this);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < spheres.length; i++) {
            assertEquals(solver.getPositions()[i], spheres[i].getCenter());
            assertEquals(solver.getDistances()[i], spheres[i].getRadius(), 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertSame(solver.getListener(), this);
        assertTrue(solver instanceof PROMedSRobustLateration3DSolver);
    }

    @Override
    public void onSolveStart(final RobustLaterationSolver<Point3D> solver) {
    }

    @Override
    public void onSolveEnd(final RobustLaterationSolver<Point3D> solver) {
    }

    @Override
    public void onSolveNextIteration(
            final RobustLaterationSolver<Point3D> solver,
            final int iteration) {
    }

    @Override
    public void onSolveProgressChange(
            final RobustLaterationSolver<Point3D> solver,
            final float progress) {
    }
}
