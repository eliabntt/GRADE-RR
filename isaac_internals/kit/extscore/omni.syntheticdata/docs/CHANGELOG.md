# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [0.2.4] - 2022-09-22
### Changed
- Update icon to match Viewport 2.0 style

## [0.2.3] - 2021-08-16
### Fixed
- Call dict.discard instead of non extistant dict.remove.

## [0.2.2] - 2021-05-18
### Changed
- Add dependency on omni.kit.viewport.utility

## [0.2.1] - 2022-03-23
### Changed
- Support Legacy and Modern Viewport

## [0.1.8] - 2021-12-10
### Changed
- Deprecated Depth and DepthLinear sensors and added DistanceToImagePlane and DistanceToCamera
### Added
- Cross Correspondence Sensor

## [0.1.7] - 2021-10-16
### Changed
- Move synthetic data sensors to AOV outputs that can be specified in USD and used in OmniGraph nodes

## [0.1.6] - 2021-06-18
### Fixed
- Instance Segmentation is now always returned as uint32
- Fixed parsed segmentation mode
- Fixed Pinhole projection which incorrectly used the camera's local transform instead of its world transform
### Added
- Linear depth sensor mode

## [0.1.5] - 2021-03-11
### Added
- Motion Vector visualization and helper function

### Changed
- BBox3D corners axis order to be Y-Up for consistency with USD API
- All parsed data return uniqueId field, along with list of instanceIds
- `instanceId` field removed from parsed output to avoid confusion with renderer instanceId
- Add `get_instance` function to extension
- Improve returned data of `get_occlusion_quadrant` for consistency with other sensors

### Fixed
- Fix BBox3D parsed mode when dealing with nested transforms
- Fix BBox3D camera_frame mode, previously returned incorrect values
- Use seeded random generator for shuffling colours during visualization

## [0.1.4] - 2021-02-10
### Changed
- Moved to internal extension
- Minor bug fixes
## [0.1.3] - 2021-02-05
### Added
- Python 3.7 support

### Changed
- Bug fixes
## [0.1.2] - 2021-01-28
### Added
- Occlusion Quadrant Sensor
- Metadata Sensor

### Changed
- Metadata (SemanticSchemas of Type != 'class') added to sensor outputs
- UI changed to better suit multi-viewport scenarios
- Misc. sensor fixes and improvements
## [0.1.1] - 2021-01-25
- Linux support

## [0.1.0] - 2021-01-18
- Initial version
