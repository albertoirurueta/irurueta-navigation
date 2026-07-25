# irurueta-navigation

A GNSS/INS navigation library

[![Maven Central](https://img.shields.io/maven-central/v/com.irurueta/irurueta-navigation.svg)](https://search.maven.org/artifact/com.irurueta/irurueta-navigation)

[![Build Status](https://github.com/albertoirurueta/irurueta-navigation/actions/workflows/master.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation/actions/workflows/master.yml)
[![Build Status](https://github.com/albertoirurueta/irurueta-navigation/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation/actions/workflows/develop.yml)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)

## Project Status

| | |
| --- | --- |
| Language | Java 21 |
| Build tool | Maven |
| Current development version | 1.8.1-SNAPSHOT |
| Latest release | 1.7.1 |
| License | Apache License, Version 2.0 |
| CI | GitHub Actions — `develop.yml` (build, test, SonarCloud scan, Antora + Maven site docs, snapshot deploy) and `master.yml` (same pipeline on release, deploying to Maven Central) |
| Quality | SonarCloud, JaCoCo coverage, Checkstyle, SpotBugs, PMD |

## Documentation

- [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation)
- [Maven Site Report](https://albertoirurueta.github.io/irurueta-navigation/mvn-site) — Javadoc, unit test, coverage, and static-analysis reports
- [SonarCloud Dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation)
- [Changelog](CHANGELOG.md)

## Installation

irurueta-navigation is published to Maven Central. Add the following dependency to your project:

Latest release:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation</artifactId>
    <version>1.7.1</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation</artifactId>
    <version>1.8.1-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

Snapshot artifacts are published from the `develop` branch to the Sonatype snapshots repository. Make sure your
project's repositories include the Sonatype snapshots repository if you want to consume a snapshot version.

## How It Works

irurueta-navigation is a Java library for GNSS/INS navigation: estimating position, velocity, and attitude from
satellite and inertial measurements, and working with the coordinate frames and geodetic calculations that
navigation algorithms depend on. Rather than one monolithic solver, it packages this as composable building
blocks — GNSS position/velocity estimators (`gnss`), reference frame types and converters (`frames`),
trilateration solvers (`lateration`), and WGS84 geodesic calculations (`geodesic`) — so each piece can be used on
its own or combined into a full navigation pipeline.

A pattern common across the library's estimators is least squares followed by Kalman filtering: a least-squares
step turns raw measurements into a position/velocity estimate, and a Kalman filter step fuses that estimate with
the previous state over time. For example, estimating a GNSS receiver's position and velocity from raw
pseudorange measurements:

```java
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;

import java.util.List;

// each measurement needs pseudo-range (m), pseudo-rate (m/s), and the satellite's ECEF
// position/velocity at the time of the measurement; at least 4 measurements are required
final List<GNSSMeasurement> measurements = List.of(
        new GNSSMeasurement(pseudoRange1, pseudoRate1, satX1, satY1, satZ1, satVx1, satVy1, satVz1),
        new GNSSMeasurement(pseudoRange2, pseudoRate2, satX2, satY2, satZ2, satVx2, satVy2, satVz2),
        new GNSSMeasurement(pseudoRange3, pseudoRate3, satX3, satY3, satZ3, satVx3, satVy3, satVz3),
        new GNSSMeasurement(pseudoRange4, pseudoRate4, satX4, satY4, satZ4, satVx4, satVy4, satVz4));

final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements);
final GNSSEstimation estimation = estimator.estimate();

final var position = estimation.getEcefPosition();
final var velocity = estimation.getEcefVelocity();
```

See the [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation) for a full walkthrough
of the library's core concepts (GNSS, frames, lateration, geodesic, sensor calibration, indoor positioning) and
further examples.

## License

This library is licensed under the [Apache License, Version 2.0](LICENSE.txt).
