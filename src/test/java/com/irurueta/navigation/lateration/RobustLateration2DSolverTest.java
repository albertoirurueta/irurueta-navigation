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

import com.irurueta.geometry.Circle;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import static org.junit.Assert.*;

public class RobustLateration2DSolverTest implements RobustLaterationSolverListener<Point2D> {

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustLateration2DSolver solver = RobustLateration2DSolver.create(RobustEstimatorMethod.RANSAC);

        // check
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(RobustEstimatorMethod.LMEDS);

        // check
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(RobustEstimatorMethod.MSAC);

        // check
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(RobustEstimatorMethod.PROSAC);

        // check
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(RobustEstimatorMethod.PROMEDS);

        // check
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(this, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(this, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances and method
        final Point2D[] positions = {
                new InhomogeneousPoint2D(),
                new InhomogeneousPoint2D(),
                new InhomogeneousPoint2D()
        };
        final double[] distances = {1.0, 1.0, 1.0};

        // RANSAC
        solver = RobustLateration2DSolver.create(positions, distances, RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(positions, distances, RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(positions, distances, RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(positions, distances, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(positions, distances, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, standard deviations and method
        final double[] standardDeviations = new double[3];

        // RANSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(positions, distances, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(positions, distances, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(positions, distances,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(positions, distances, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(positions, distances, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, standard deviations, listener and
        // method

        // RANSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles and method
        final Circle[] circles = {
                new Circle(positions[0], distances[0]),
                new Circle(positions[1], distances[1]),
                new Circle(positions[2], distances[2]),
        };

        // RANSAC
        solver = RobustLateration2DSolver.create(circles, RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(circles, RobustEstimatorMethod.LMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(circles, RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(circles, RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(circles, RobustEstimatorMethod.PROMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, standard deviations and method

        // RANSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(circles, standardDeviations, RobustEstimatorMethod.LMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(circles, standardDeviations, RobustEstimatorMethod.PROMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(circles, this, RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(circles, this, RobustEstimatorMethod.LMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(circles, this, RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(circles, this, RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(circles, this, RobustEstimatorMethod.PROMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores and method
        final double[] qualityScores = new double[3];

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);


        // create with quality scores, listener and method

        // ANSAC
        solver = RobustLateration2DSolver.create(qualityScores, this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances, standard deviations and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions,
                distances, standardDeviations, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distance, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles, standard deviations and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(solver.getDistanceStandardDeviations(), standardDeviations);
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles, standard deviations, listener and method

        // RANSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this,
                RobustEstimatorMethod.RANSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof RANSACRobustLateration2DSolver);

        // LMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this,
                RobustEstimatorMethod.LMEDS);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof LMedSRobustLateration2DSolver);

        // MSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this,
                RobustEstimatorMethod.MSAC);

        // check
        assertNull(solver.getQualityScores());
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof MSACRobustLateration2DSolver);

        // PROSAC
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this,
                RobustEstimatorMethod.PROSAC);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROSACRobustLateration2DSolver);

        // PROMedS
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this,
                RobustEstimatorMethod.PROMEDS);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with default method
        solver = RobustLateration2DSolver.create();

        // check
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with listener and default method
        solver = RobustLateration2DSolver.create(this);

        // check
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances and default method
        solver = RobustLateration2DSolver.create(positions, distances);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, standard deviations and default method
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, listener and default method
        solver = RobustLateration2DSolver.create(positions, distances, this);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with positions, distances, standard deviations, listener and
        // default method
        solver = RobustLateration2DSolver.create(positions, distances, standardDeviations, this);

        // check
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles and default method
        solver = RobustLateration2DSolver.create(circles);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, standard deviations and default method
        solver = RobustLateration2DSolver.create(circles, standardDeviations);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, listener and default method
        solver = RobustLateration2DSolver.create(circles, this);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with circles, standard deviations, listener and default method
        solver = RobustLateration2DSolver.create(circles, standardDeviations, this);

        // check
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores
        solver = RobustLateration2DSolver.create(qualityScores);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, listener and default method
        solver = RobustLateration2DSolver.create(qualityScores, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances and default method
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances, standard deviations and default method
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances, standard deviations,
        // listener and default method
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, standardDeviations,
                this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, positions, distances, listener and default method
        solver = RobustLateration2DSolver.create(qualityScores, positions, distances, this);

        // check
        assertSame(qualityScores, solver.getQualityScores());
        assertSame(positions, solver.getPositions());
        assertSame(distances, solver.getDistances());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles and default method
        solver = RobustLateration2DSolver.create(qualityScores, circles);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles, standard deviations and default
        // method
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);

        // create with quality scores, circles, standard deviations, listener and
        // default method
        solver = RobustLateration2DSolver.create(qualityScores, circles, standardDeviations, this);

        // check
        assertSame(solver.getQualityScores(), qualityScores);
        for (int i = 0; i < circles.length; i++) {
            assertEquals(circles[i].getCenter(), solver.getPositions()[i]);
            assertEquals(circles[i].getRadius(), solver.getDistances()[i], 0.0);
        }
        assertSame(standardDeviations, solver.getDistanceStandardDeviations());
        assertSame(this, solver.getListener());
        assertTrue(solver instanceof PROMedSRobustLateration2DSolver);
    }

    @Override
    public void onSolveStart(final RobustLaterationSolver<Point2D> solver) {
    }

    @Override
    public void onSolveEnd(final RobustLaterationSolver<Point2D> solver) {
    }

    @Override
    public void onSolveNextIteration(final RobustLaterationSolver<Point2D> solver, final int iteration) {
    }

    @Override
    public void onSolveProgressChange(final RobustLaterationSolver<Point2D> solver, final float progress) {
    }
}
