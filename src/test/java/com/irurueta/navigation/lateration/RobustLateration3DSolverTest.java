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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class RobustLateration3DSolverTest implements RobustLaterationSolverListener<Point3D> {

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var solver = RobustLateration3DSolver.create(RobustEstimatorMethod.RANSAC);

        // check
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.LMEDS);

        // check
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.MSAC);

        // check
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.PROSAC);

        // check
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances and method
        final Point3D[] positions = {
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D(),
                new InhomogeneousPoint3D()
        };
        final double[] distances = {1.0, 1.0, 1.0, 1.0};

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances, RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, standard deviations and method
        final var standardDeviations = new double[4];

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances, this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances, this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances, this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, standard deviations, listener and
        // method

        // RANSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres and method
        final Sphere[] spheres = {
                new Sphere(positions[0], distances[0]),
                new Sphere(positions[1], distances[1]),
                new Sphere(positions[2], distances[2]),
                new Sphere(positions[3], distances[3])
        };

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, RobustEstimatorMethod.RANSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, RobustEstimatorMethod.LMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, RobustEstimatorMethod.MSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, RobustEstimatorMethod.PROSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, RobustEstimatorMethod.PROMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, standard deviations and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, RobustEstimatorMethod.RANSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, RobustEstimatorMethod.LMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, RobustEstimatorMethod.PROMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, this, RobustEstimatorMethod.RANSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, this, RobustEstimatorMethod.LMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, this, RobustEstimatorMethod.MSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, this, RobustEstimatorMethod.PROSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, this, RobustEstimatorMethod.PROMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this, RobustEstimatorMethod.MSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores and method
        final var qualityScores = new double[4];

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances, standard deviations and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distance, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres, standard deviations and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(RANSACRobustLateration3DSolver.class, solver);

        // LMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(LMedSRobustLateration3DSolver.class, solver);

        // MSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(MSACRobustLateration3DSolver.class, solver);

        // PROSAC
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROSACRobustLateration3DSolver.class, solver);

        // PROMedS
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with default method
        solver = RobustLateration3DSolver.create();

        // check
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with listener and default method
        solver = RobustLateration3DSolver.create(this);

        // check
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances and default method
        solver = RobustLateration3DSolver.create(positions, distances);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, standard deviations and default method
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, listener and default method
        solver = RobustLateration3DSolver.create(positions, distances, this);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with positions, distances, standard deviations, listener and default method
        solver = RobustLateration3DSolver.create(positions, distances, standardDeviations, this);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres and default method
        solver = RobustLateration3DSolver.create(spheres);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, standard deviations and default method
        solver = RobustLateration3DSolver.create(spheres, standardDeviations);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, listener and default method
        solver = RobustLateration3DSolver.create(spheres, this);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with spheres, standard deviations, listener and default method
        solver = RobustLateration3DSolver.create(spheres, standardDeviations, this);

        // check
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores
        solver = RobustLateration3DSolver.create(qualityScores);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances, standard deviations and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances, standard deviations, listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, standardDeviations, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, positions, distances, listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, positions, distances, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres and default method
        solver = RobustLateration3DSolver.create(qualityScores, spheres);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres, standard deviations and default
        // method
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);

        // create with quality scores, spheres, standard deviations, listener and default method
        solver = RobustLateration3DSolver.create(qualityScores, spheres, standardDeviations, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        for (var i = 0; i < spheres.length; i++) {
            assertEquals(spheres[i].getCenter(), solver.getPositions()[i]);
            assertEquals(spheres[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertInstanceOf(PROMedSRobustLateration3DSolver.class, solver);
    }

    @Override
    public void onSolveStart(final RobustLaterationSolver<Point3D> solver) {
        // no action needed
    }

    @Override
    public void onSolveEnd(final RobustLaterationSolver<Point3D> solver) {
        // no action needed
    }

    @Override
    public void onSolveNextIteration(final RobustLaterationSolver<Point3D> solver, final int iteration) {
        // no action needed
    }

    @Override
    public void onSolveProgressChange(final RobustLaterationSolver<Point3D> solver, final float progress) {
        // no action needed
    }
}
