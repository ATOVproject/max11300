# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.5.3](https://github.com/ATOVproject/max11300/compare/v0.5.2...v0.5.3) - 2026-07-14

### Added

- skip redundant DAC data writes via shadow registers

## [0.5.2](https://github.com/ATOVproject/max11300/compare/v0.5.1...v0.5.2) - 2026-04-11

### Added

- add methods to update multiple GPOs simultaneously

## [0.5.1](https://github.com/ATOVproject/max11300/compare/v0.5.0...v0.5.1) - 2025-09-11

### Fixed

- gpo addressing for ports > 15

### Other

- *(deps)* bump actions/checkout from 4 to 5
- *(deps)* update heapless requirement from 0.8.0 to 0.9.1
- add dependabot config
- fix GH actions permissions
- Update CHANGELOG.md

## [0.5.0](https://github.com/ATOVproject/max11300/compare/v0.4.3...v0.5.0) - 2025-08-09

### Added

- [**breaking**] return actual Mode from get_mode function

## [0.4.3](https://github.com/ATOVproject/max11300/compare/v0.4.2...v0.4.3) - 2025-07-21

### Added

- update all dependencies
- fix release-plz workflow

## [0.4.2](https://github.com/ATOVproject/max11300/compare/v0.4.1...v0.4.2) - 2025-03-19

### Other

- clippy and cleanup

## [0.4.1](https://github.com/AtoVproject/max11300/compare/v0.4.0...v0.4.1) - 2024-12-07

### Other

- temporarily disable clippy

## [0.4.0](https://github.com/AtoVproject/max11300/compare/v0.3.0...v0.4.0) - 2024-11-08

### Other

- remove NOTES

## [0.3.0](https://github.com/AtoVproject/max11300/compare/v0.2.0...v0.3.0) - 2024-08-02

### Added
- add method to read raw interrupt register
- add temperature sensor interrupt definitions
- add interrupts to exposed Parts after split()

## [0.2.0](https://github.com/AtoVproject/max11300/compare/v0.1.1...v0.2.0) - 2024-04-10

### Other
- update github actions
