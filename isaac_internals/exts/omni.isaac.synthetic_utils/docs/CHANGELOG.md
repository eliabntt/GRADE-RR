# Changelog

## [0.4.3] - 2023-01-19
### Fixed
- test errors due to missing button

## [0.4.2] - 2022-10-17
### Fixed
- pass viewport api to next_sensor_data_async

## [0.4.1] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.4.0] - 2022-09-01

### Changed
- remove non synthetic data related legacy viewport calls

## [0.3.5] - 2022-08-12

### Removed
- removed isaac replicator style DOPE Writer

## [0.3.4] - 2022-08-11

### Removed
- removed isaac replicator style YCB Video writer
- YCB Video writer using OV Replicator style added to omni.replicator.isaac

## [0.3.3] - 2022-08-08

### Changed
- Raise exception in DOPE writer when s3 bucket name is invalid

## [0.3.2] - 2022-08-04

### Added
- Change output folder structure for DOPE writer

## [0.3.1] - 2022-07-29

### Added
- Write to s3 bucket for DOPE Writer

## [0.3.0] - 2022-07-11

### Added
- DOPE Writer
- Occlusion sensor in SyntheticDataHelper
- initialize_async

### Fixed
- get_groundtruth works in an async function

## [0.2.1] - 2022-05-05

### Changed
- Modify the initialize() function to wait until sensor data is available

## [0.2.0] - 2022-04-05

### Added
- YCB Video writer

## [0.1.7] - 2022-03-16

### Changed
- Replaced find_nucleus_server() with get_assets_root_path()

## [0.1.6] - 2022-01-24

### Changed
- updated code to match API changes in omni.syntheticdata

## [0.1.5] - 2021-11-01

### Added
- get_mapped_semantic_data
- get_semantic_label_map
- get_semantic_id_map

## [0.1.4] - 2021-10-18

### Added
- kitti writer supports both loose and tight 2d bounding boxes for labels

## [0.1.3] - 2021-10-09

### Changed
- Restructure files in extension

## [0.1.2] - 2021-08-13

### Removed
- Removed domain randomization helper file. Use commands directly.
- Moved shapenet utility file to omni.isaac.shapenet.

## [0.1.1] - 2021-08-02

### Added
- Unit tests
- Updated API

## [0.1.0] - 2021-07-08

### Added
- Initial version of Isaac Sim Synthetic Utils Extension
