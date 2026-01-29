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

**Key commands you ran:**
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
![test_python](https://raw.githubusercontent.com/lorelleh/assignment1/cdd7452a8a3b80400a9bfe5aa52abb638c405f94/images/a1-1.png)
![test_open3d_pointcloud](https://raw.githubusercontent.com/lorelleh/assignment1/cdd7452a8a3b80400a9bfe5aa52abb638c405f94/images/a1-2.png)
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

**Your actual output:**
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
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[Paste 3-4 lines of talker output here]
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[Paste 3-4 lines of listener output here]
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

![Talker and Listener Running](path/to/your/screenshot.png)

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: [Write the exact error message or problem]

**Cause / diagnosis:**  
_[Explain what you think caused it]_

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
[Your fix command/code here]
```

**Reference:**  
_[Official ROS docs? StackOverflow? AI assistant? Something else?]_

---

### Issue 2: [Another real error or roadblock]

**Cause / diagnosis:**  
_[Explain what you think caused it]_

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
[Your fix command/code here]
```

**Reference:**  
_[Official ROS docs? StackOverflow? AI assistant? Something else?]_

---

### Issue 3 (Optional): [Title]

**Cause / diagnosis:**  
_[Explain what you think caused it]_

**Fix:**  
_[The exact command/config change you used to solve it]_

```bash
[Your fix command/code here]
```

**Reference:**  
_[Official ROS docs? StackOverflow? AI assistant? Something else?]_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
[Copy-paste your actual message to the AI, not a summary]
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
[Quote only the relevant part of the AI's answer]
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
_[Write your analysis here]_

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
[Your final command/code here]
```

**Why this worked:**  
_[Brief explanation]_

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

_[Write your 3-5 sentence reflection here]_

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[Your name]_

**Student ID:**  
_[Your student ID]_

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
