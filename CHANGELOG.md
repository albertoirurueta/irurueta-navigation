# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

No user-facing changes yet — only a version bump to `1.8.1-SNAPSHOT` on `develop`.

## [1.7.1] - 2026-03-04

### Changed

- Updated dependencies `irurueta-numerical` and `irurueta-geometry` from 1.4.0 to 1.5.0. No library source changes
  in this release (dependency maintenance only; there is no final `1.7.0` tag — development went straight from a
  `1.7.0-SNAPSHOT` to this `1.7.1` patch release).

## [1.6.0] - 2025-12-18

### Changed

- Updated dependencies `irurueta-numerical` and `irurueta-geometry` from 1.3.2 to 1.4.0. No library source changes
  in this release (dependency maintenance only).

## [1.5.2] - 2025-09-22

### Changed

- Updated dependencies `irurueta-numerical`, `irurueta-geometry`, and `irurueta-algebra` from 1.3.1 to 1.3.2. No
  library source changes in this release (dependency maintenance only).

## [1.5.1] - 2025-09-20

### Changed

- Build tooling and dependency maintenance only: cleaned up `pom.xml`, updated Maven plugins and GitHub Actions
  workflows, and removed the `junit-vintage` test dependency. No library source changes in this release.

## [1.5.0] - 2025-01-08

### Changed

- Raised the minimum/target Java version from 1.7 to Java 17 — consumers must now build and run with at least
  JDK 17.
- Migrated the test suite to JUnit 5.
- Large internal, non-behavioral refactor across nearly every source file: renamed Hungarian-notation fields
  (`mFoo` → `foo`), adopted `var` type inference, and converted `switch` statements to `switch` expressions.

## [1.4.1] - 2023-12-24

### Fixed

- `ECEFFrame.equals()` and `ECIFrame.equals()` (inherited from `ECIorECEFFrame`) now correctly override
  `Object.equals()`. Previously the base class only declared `equals(T other)`, which did not satisfy the
  `Object.equals()` contract, so equality comparisons (e.g. in collections or assertions) could silently fall back
  to reference equality.

## [1.4.0] - 2023-12-24

### Changed

- Updated dependencies `irurueta-numerical` (1.1.0 → 1.2.1), `irurueta-geometry` (1.1.0 → 1.2.0), `irurueta-units`
  (1.1.0 → 1.2.0), and `irurueta-algebra` (1.1.0 → 1.2.0), and adapted the `LaterationSolver`/
  `RobustLateration*Solver` classes to the renamed `RobustEstimatorMethod` enum constants `LMedS` → `LMEDS` and
  `PROMedS` → `PROMEDS` introduced by `irurueta-numerical`, which changes the values these methods return/accept.
- Widespread Javadoc wording/formatting fixes across the `frames`, `geodesic`, `gnss`, and `lateration` packages;
  no behavioral changes.

## [1.3.1] - 2022-10-07

### Fixed

- Corrected the sign convention of the rotation matrix produced by
  `CoordinateTransformation.ecefToEciMatrixFromAngle`, which had been given the wrong sign in 1.3.0.
- `eciToEcefMatrixFromAngle` is now computed directly instead of via the transpose of `ecefToEciMatrixFromAngle`,
  fixing incorrect results that the previous sign fix had introduced there.

## [1.3.0] - 2022-10-06

### Added

- Added convenience methods on `Frame` (and its `ECEFFrame`/`ECIFrame`/`NEDFrame` implementations) to get/set the
  coordinate transformation directly as a matrix (`getCoordinateTransformationMatrix`/
  `setCoordinateTransformationMatrix`) or as a `Rotation3D` (`getCoordinateTransformationRotation`/
  `setCoordinateTransformationRotation`), avoiding the overhead of copying a full `CoordinateTransformation`
  instance.

### Fixed

- Fixed `CoordinateTransformation.ecefToEciMatrixFromAngle`, which built the 3x3 rotation matrix with row/column
  indices swapped, producing an incorrectly transposed matrix instead of the intended rotation.

## [1.2.0] - 2022-07-30

### Added

- `CoordinateTransformation` gained a new constructor accepting a `Rotation3D`
  (`CoordinateTransformation(Rotation3D, FrameType, FrameType)`) and a new public `fromRotation(Rotation3D)` method
  to set the transformation directly from a 3D rotation.

## [1.1.0] - 2021-12-11

### Changed

- CI/tooling only: migrated the GitHub Actions setup and removed the legacy Travis CI configuration. No library
  source changes in this release.

## [1.0.0] - 2021-12-09

Initial public release. The library shipped with:

### Added

- **Reference frames** (`frames`): ECEF, ECI, and NED frame/position/velocity representations, converters between
  them, and `CoordinateTransformation` support.
- **Inertial navigation and kinematics**: body kinematics estimators/generators, ECEF/ECI/NED navigators,
  gravity/gravitation estimators, IMU noise and bias estimators, and Kalman-filter based drift/random-walk
  estimators.
- **GNSS**: least-squares and Kalman-filtered position/velocity estimators, GNSS measurement/bias generators, and
  satellite position/velocity generation.
- **Sensor calibration**: accelerometer, gyroscope, and magnetometer calibrators (known-frame, known-bias,
  known-position/gravity-norm, turntable, and "easy" gyroscope calibrators), each with robust-estimator variants
  (RANSAC, LMedS, MSAC, PROSAC, PROMedS) and factory methods.
- **Indoor positioning**: trilateration/lateration solvers (linear, non-linear, robust variants in 2D/3D),
  fingerprint-based position estimators, and RSSI/ranging radio-source (WiFi access point, beacon) position and
  power estimators, with robust variants and covariance/accuracy propagation support.
- **Supporting utilities**: geodesic calculations (Karney's algorithms), accuracy classes (`Accuracy`,
  `Accuracy2D`, `Accuracy3D`), and location utilities.

[Unreleased]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.7.1...develop
[1.7.1]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.6.0...1.7.1
[1.6.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.5.2...1.6.0
[1.5.2]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.5.1...1.5.2
[1.5.1]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.5.0...1.5.1
[1.5.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.4.1...1.5.0
[1.4.1]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.4.0...1.4.1
[1.4.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.3.1...1.4.0
[1.3.1]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.2.0b...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.1.0...1.2.0b
[1.1.0]: https://github.com/albertoirurueta/irurueta-navigation/compare/1.0.0...1.1.0
[1.0.0]: https://github.com/albertoirurueta/irurueta-navigation/releases/tag/1.0.0
