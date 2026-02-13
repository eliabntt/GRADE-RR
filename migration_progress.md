## 2026-02-13: USD Crash & Asset Compatibility Plan

### Problem
- Isaac Sim 5.1+ crashes with a segmentation fault after attempting to load GRADE-RR/usds/env_base.usd.
- Log: "Unable to re-read @GRADE-RR/usds/env_base.usd@" followed by a segmentation fault.
- All extension/ROS issues are now guarded; crash is likely due to USD asset incompatibility or corruption.

### Smart Migration Plan
1. **Diagnose USD Asset Compatibility**
	- Check if env_base.usd and other USDs were created with an older Omniverse/Isaac Sim version.
	- Use Isaac Sim 5.1+ USD tools to validate and attempt to open the file.
	- Check for known breaking changes in USD schema or asset format between Isaac Sim versions.
2. **Automated USD Repair/Conversion**
	- Use usdcat, usdview, or Isaac Sim's built-in conversion tools to re-save or upgrade the USD file.
	- If conversion fails, attempt to export a new compatible USD from an older Isaac Sim or Omniverse Create.
3. **Minimal Repro Test**
	- Write a minimal script to load env_base.usd in isolation with Isaac Sim 5.1+ Python API.
	- If this also crashes, the file is definitely incompatible/corrupt.
4. **Iterative Asset Fix**
	- If conversion is possible, replace the old USD in GRADE-RR/usds/ with the new one.
	- If not, document the minimal changes needed for manual repair (e.g., removing deprecated prims, updating schema).
5. **Document and Automate**
	- Add all findings, steps, and scripts to migration_progress.md for reproducibility.
	- Automate asset validation as part of the migration pipeline.

### Next Steps
- [ ] Step 1: Try to load env_base.usd with a minimal script using Isaac Sim 5.1+ Python API and capture error details.
- [ ] Step 2: Attempt automated conversion/repair if loading fails.
- [ ] Step 3: Replace or repair asset and re-test full pipeline.
# Migration Progress

## Isaac Sim 5.1+ Migration Tasks

- [x] Update all imports from omni.isaac to isaacsim where needed
- [x] Rename simulator/utils to simulator/grade_utils and update all references
- [x] Update SimulationApp initialization for Isaac Sim 5.1+
- [x] Update URDF importer extension references
- [x] Update config.yaml asset paths to local

## 2024-02-13: Runtime Error Fixes

- [x] Patched zebra_datagen.py to handle empty/invalid fix_env by selecting a random environment.
- [x] Resolved ValueError for fix_env: zebra_datagen.py now robust to missing/empty fix_env.
- [x] Fixed confuse.exceptions.NotFoundError: env_path not found by updating config.yaml to use env_path: "../usds" (local USD environment folder).

Pending:
- [ ] Re-test zebra_datagen.py for further runtime/config errors.
- [x] Remove all ROS 1 (rospy) imports and usage
- [x] Add ROS 2 (rclpy) import block and compatibility
- [x] Guard all ROS/extension imports with try/except for cross-platform compatibility
- [x] Guard omni.isaac.synthetic_recorder import for Isaac Sim 5.1+
- [ ] Test zebra_datagen.py for ROS 2 compatibility
- [ ] Document any further issues or next steps

**Latest action:** Guarded omni.isaac.synthetic_recorder import in misc_utils.py for Isaac Sim 5.1+ compatibility. All extension/ROS imports are now robust for Ubuntu 24.04 and Isaac Sim 5.1+.
