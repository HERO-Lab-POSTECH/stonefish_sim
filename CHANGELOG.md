# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- TF broadcasting to odometry publisher
- SetWaveHeight ROS2 service (0-10m range)
- SetWindVelocity ROS2 service for runtime wind control
- SetOceanCurrent ROS2 service for current control
- BlueBoat surface vehicle support
- Comprehensive README documentation

### Changed
- Simplified main README with essential information only
- Replaced hardcoded absolute paths with package-relative paths
- Renamed service fields for consistency
- Updated BlueROV2 scenario and configurations

### Removed
- Legacy robot definitions (moved to _legacy folder)

### Fixed
- Corrected hardcoded workspace paths in launch files
- Path following restart issue with trajectory re-reception
