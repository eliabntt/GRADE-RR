# Changelog

## [1.5.0] - 2023-03-13
### Added
- pointcloud_include_unlabelled parameter for pointcloud data

## [1.4.2] - 2023-02-22
### Fixed
- added wait_until_complete for S3 bucket writing one frame less (OM-82465)
- S3 accepting None values for s3_region and s3_endpoint

## [1.4.1] - 2023-02-14
### Fixed
- Synthetic recorder should only subscribe to the type of stage event it needs

## [1.4.0] - 2023-02-07
### Added
- Timeline Control

### Fixed
- Incremental folder naming not supported with S3 (OM-80864)

## [1.3.0] - 2023-01-25
### Removed
- Manual Control UI

### Changed
- Switched to async functions from Replicator 1.7.1 API

## [1.2.2] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [1.2.1] - 2023-01-04
### Fixed
- clean-ups on on_shutdown

## [1.2.0] - 2022-12-09
### Fixed
- Empty strings are no loger saved to config files

### Added
- Refresh path strings to default


## [1.1.1] - 2022-12-06
### Fixed
- Menu toggle value when user closes the window

## [1.1.0] - 2022-11-30
### Added
- Support for loading custom writers

### Changed 
- renamed extension.py to synthetic_recorder_extension.py
- renamed extension class from Extension to SyntheticRecorderExtension

### Fixed
- Annotators blocking other annotators of writing data if their requirements are not met

## [1.0.0] - 2022-11-14
### Added
- version using Replicator OV API

## [0.1.2] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.1.1] - 2022-08-02

### Fixed

- Error message when there was no instance data to write

## [0.1.0] - 2021-08-11

### Added
- Initial version of Isaac Sim Synthetic Data Recorder Extension
- Records RGB, Depth, Semantic and Instance segmentation, 2D Tight and Loose bounding box
- Supports multi-viewport recording
