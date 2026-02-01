# AAE5303 Environment Setup Report

---

## 1. System Information

**Laptop model:**  TX Air FA401KM

**CPU / RAM:**  AMD Ryzen AI 7 H 350 w, 32GB RAM

**Host OS:**  Ubuntu 24.04

**Linux/ROS environment type:**  
- [ ] Dual-boot Ubuntu
- [x] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

**Describe briefly how you created/activated your Python environment:**  
In WSL2 Ubuntu 24.04, I created an isolated Python virtual environment named `.venv` in the project root directory using Python's built-in `venv` module. The environment was activated via the `source` command, with the `(.venv)` terminal prefix confirming successful activation. All required Python packages were installed following the specifications in `requirements.txt` after activation.

**Tool used:**  venv

**Key commands I ran:**
```bash
sudo apt install -y python3-venv
sudo apt list --installed | grep python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
Manually created the project directory structure/empty files; installed dependencies via Alibaba Cloud PyPI mirror

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
(.venv) lorelleh@HOOSLAPTOP:/mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check$ python scripts/test_python_env.py
=== AAE5303 Python Environment Test ===
Python executable: /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/.venv/bin/python

✅ Python 3.12.3 (≥3.10)
✅ numpy 2.4.1 (≥2.0)
✅ scipy 1.17.0 (≥1.12)
✅ matplotlib 3.10.8 (≥3.8)
✅ opencv-python 4.13.0.90 (≥4.9)
✅ open3d 0.19.0 (exact match)
✅ ROS2 rclpy imported successfully
✅ ROS2 std_msgs imported successfully

🎉 All Python environment checks PASSED!
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
(.venv) lorelleh@HOOSLAPTOP:/mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check$ python scripts/test_open3d_pointcloud.py
=== AAE5303 Open3D Point Cloud & Image Test ===
Data directory: /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/data
Open3D version: 0.19.0 (required: 0.19.0)
NumPy version: 2.4.1 (required: ≥2.0)
OpenCV version: 4.13.0 (required: ≥4.9)

=== Testing Point Cloud (PCD) Operations ===
⚠️  Creating dummy point cloud (no real data): /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/data/sample_pointcloud.pcd
✅ Read PCD: 10 test points
✅ Converted PCD to NumPy 2.x: (10, 3) (points, 3D coordinates)
✅ Wrote PCD copy: sample_pointcloud_copy.pcd

=== Testing Image (PNG) Operations ===
⚠️  Creating dummy image (no real data): /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/data/sample_image.png
✅ Read PNG: (100, 100, 3) (height, width, channels)
✅ Image NumPy dtype: uint8 (uint8, correct for image data)
✅ Wrote PNG copy: sample_image_out.png
✅ Resized & wrote tiny PNG: sample_image_tiny.png (50x50)

=== Testing Open3D ↔ NumPy 2.x Compatibility ===
✅ Open3D 0.19.0 ↔ NumPy 2.x conversion: PASSED (100% compatible)

🎉 All Open3D point cloud/image checks PASSED!
```

**Screenshot:**  
![Python Tests Passing](https://raw.githubusercontent.com/lorelleh/assignment1/cdd7452a8a3b80400a9bfe5aa52abb638c405f94/images/a1-1.png)
![Open3d_Pointcloud Tests Passing](https://raw.githubusercontent.com/lorelleh/assignment1/cdd7452a8a3b80400a9bfe5aa52abb638c405f94/images/a1-2.png)
---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/jazzy/setup.bash  
cd ros2_wst/ros/jazzy/setup.bash
colcon build                                                             
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Actual output:**
```
(.venv) lorelleh@HOOSLAPTOP:/mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check$
source /opt/ros/jazzy/setup.bash  
cd ros2_wst/ros/jazzy/setup.bash
colcon build                                                             
Starting >>> env_check_pkg
[0.884s] WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'env_check_pkg' because it has no 'install' target
Finished <<< env_check_pkg [0.42s]

Summary: 1 package finished [0.78s]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[INFO] [1769704527.795947467] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #50'
[INFO] [1769704528.292883602] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #51'
[INFO] [1769704528.798259520] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #52'
[INFO] [1769704529.294923713] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #53'
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[INFO] [1769704527.796227730] [env_check_pkg_listener]: I heard: 'AAE5303 hello #50'
[INFO] [1769704528.293257669] [env_check_pkg_listener]: I heard: 'AAE5303 hello #51'
[INFO] [1769704528.798673022] [env_check_pkg_listener]: I heard: 'AAE5303 hello #52'
[INFO] [1769704529.295189986] [env_check_pkg_listener]: I heard: 'AAE5303 hello #53'
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
![Talker and Listener Running](https://github.com/lorelleh/assignment1/blob/a39fabedeef690728deafa988c26123648f0756d/images/a1-3.png)

---

## 4. Problems Encountered and How I Solved Them

### Issue 1: [Package 'env_check_pkg' not found]

**Cause / diagnosis:**  
The ROS2 runtime environment did not load the compiled workspace configuration (install/setup.bash);
The package was not compiled successfully, so the install directory (which contains package metadata) was missing;
The package configuration files (package.xml/setup.py) did not comply with ROS2 ament_python standards.

**Fix:**  
Recompile the ROS2 workspace with Python-specific options to generate valid install directory;
Reload the system ROS2 environment first, then the compiled workspace environment;
Verify the package is recognized by ROS2 with ros2 pkg list.

```bash
# Compile workspace with Python-specific option
colcon build --symlink-install
# Reload environment in correct order
source /opt/ros/jazzy/setup.bash
# Use absolute path for workspace setup.bash
source /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/install/setup.bash
# Verify package is recognized by ROS2
ros2 pkg list | grep env_check_pkg
```

**Reference:**  
ROS 2 Official Documentation (Working with Workspaces), Stack Overflow (ROS2 package not found issues)

---

### Issue 2: [Permission denied when running Python script directly]

**Cause / diagnosis:**  
The Python script files (talker.py/listener.py) did not have executable permissions in the Linux filesystem, which blocked direct execution;
The default file creation mode in WSL2 does not grant executable permissions to regular files (e.g., .py scripts), leading to this restriction by default.

**Fix:**  
Add executable permissions to the Python scripts;
Use the ROS2 standard ros2 run command.

```bash
# Add executable permission
cd /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/src/env_check_pkg
chmod +x talker.py listener.py
```

**Reference:**  
Linux File Permission Basics, ROS 2 ros2 run Command Documentation

---

### Issue 3: [No explicit error, but new changes (e.g., fixed package.xml) do not take effect]

**Cause / diagnosis:**  
Residual files in build/install/log directories from previous failed builds overwrite new configuration;
The ROS2 build system uses cached data instead of reprocessing updated files.

**Fix:**  
Delete all build artifacts before recompiling to ensure a clean build environment.

```bash
# Clean all build artifacts
rm -rf /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/build
rm -rf /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/install
rm -rf /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/log
# Change directory to your ROS2 workspace root 
cd /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws
# Recompile with clean state
colcon build --symlink-install
```

**Reference:**  
ROS 2 Troubleshooting Guide (Clean Build), ROS 2 Colcon Cache Documentation

---

## 5. Use of Generative AI

### 5.1 Exact prompt I asked

**My prompt:**
```
I created the env_check_pkg package in ROS2 Jazzy, compiled it with colcon build, and sourced /opt/ros/jazzy/setup.bash and install/setup.bash. However, when I run "ros2 run env_check_pkg talker", it still prompts "Package 'env_check_pkg' not found". Why can't ROS2 recognize my package?
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
The core cause of the "Package 'env_check_pkg' not found" error is that the package was not compiled to generate valid installation targets, or old build artifacts are interfering with the new configuration. The key fixes are:
1. Clean up old build artifacts to avoid residual file conflicts: rm -rf build install log
2. Compile Python packages with the --symlink-install option (mandatory for ament_python packages): colcon build --symlink-install
3. Source the environment in the correct order (system ROS2 first, then workspace):
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
4. Verify if the package is recognized by ROS2: ros2 pkg list | grep env_check_pkg
```

### 5.3 What I changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**My explanation:**  
The AI did not recommend any unsafe operations; all core steps aligned with ROS2 official best practices.
I modified the solution by prioritizing the cleanup of old build artifacts (build/install/log) before compiling—while the AI mentioned this step, it did not emphasize its criticality. My previous repeated failed builds left residual files that overwrote new configurations, so manual cleanup became a mandatory first step (rather than an optional one).
I double-checked the AI’s advice against the ROS2 Jazzy official documentation to confirm that --symlink-install is required for Python packages (ament_python) to generate valid installation targets in the install directory—this validation ensured I did not blindly copy the AI’s suggestion.
I ignored no part of the AI’s core guidance but adapted the step order to address my specific issue (residual build files).

### 5.4 Final solution I applied

The exact command or file edit that fixed the problem:

```bash
# Step 1: Clean up old build artifacts to eliminate residual conflicts
rm -rf build install log
# Step 2: Compile the workspace with Python-specific option
colcon build --symlink-install
# Step 3: Reload environment in correct order
source /opt/ros/jazzy/setup.bash
# Step 4:Use absolute path for workspace setup.bash
source /mnt/e/PolyU/SEM2/AAE5303/test/a1/aae5303-env-check/ros2_ws/install/setup.bash
# Step 5: Verify package is recognized by ROS2
ros2 pkg list | grep env_check_pkg
# Step 6: Run the talker node
ros2 run env_check_pkg talker
```

**Why this worked:**  
Cleaning old build artifacts removed invalid/cached files from previous failed builds, ensuring the new compilation used only up-to-date configurations.
The --symlink-install option created symbolic links for Python scripts in the install directory (generic colcon build does not generate these for Python packages), which ROS2 requires to detect the package’s installation targets.
Sourcing the system ROS2 environment first (then the workspace) ensured the locally compiled env_check_pkg took precedence over system-level packages, allowing ROS2 to locate the package correctly.

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**My reflection:**

_[Write your 3-5 sentence reflection here]_

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
HU Yiduo

**Student ID:**  
25133786G

**Date:**  
_[Date of submission]_

---

## Submission Checklist

Before submitting, ensure you have:

- [ ] Filled in all system information
- [ ] Included actual terminal outputs (not just screenshots)
- [ ] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [ ] Documented 2–3 real problems with solutions
- [ ] Completed the AI usage section with exact prompts
- [ ] Written a thoughtful reflection (3–5 sentences)
- [ ] Signed the declaration

---

**End of Report**
