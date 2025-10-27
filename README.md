

## Installation & Configuration

## 1. Create a Virtual Environment

It is recommended to run training or deployment inside a virtual environment, preferably using Conda.

### 1.1 Create a New Environment

```bash
conda create -n robomimic python=3.8
```

### 1.2 Activate Environment

```bash
conda activate robomimic
```

## 2. Install Dependencies

### 2.1 Install PyTorch

PyTorch is a deep learning framework used for model training and inference. Install using:

```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

#### 2.2.2 Install Components

Enter the project directory and install required packages:

```bash
cd RoboMimicDeploy_G1
pip install numpy==1.20.0
pip install onnx onnxruntime
pip install hydra-core
```

#### 2.2.3 Install unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

---

## Run Code

## 1. Run Mujoco Simulation Code

```bash
python deploy_mujoco/deploy_mujoco.py
```

---

## 2. Policy Description

| Mode Name         | Description                                                                                                             |
| ----------------- | ----------------------------------------------------------------------------------------------------------------------- |
| **PassiveMode**   | Damping protection mode                                                                                                 |
| **FixedPose**     | Position control recovery to default joint angles                                                                       |
| **STANDMODE**     | Stand up from lying posture                                                                                             |
| **LocoMode**      | Walking stability control mode                                                                                          |
| **Dance**         | Charleston dancing                                                                                                      |
| **SKILL_ASAP**    | “Ronaldo-style” jump motion                                                                                             |
| **KungFu**        | Martial arts motion                                                                                                     |
| **KungFu2**       | Failed martial arts motion                                                                                              |
| **Kick**          | Simple kicking motion                                                                                                   |
| **SkillCast**     | Lower body + waist stable standing, upper body moves to skill-specific poses — usually executed **before** Mimic policy |
| **SkillCooldown** | Lower body + waist balance maintenance, upper body resets — usually executed **after** Mimic policy                     |

---

## 3. Simulation Operation Instructions

1. Connect an Xbox controller
2. Start the simulation:

```bash
python deploy_mujoco/deploy_mujoco.py
```

To make the robot stand up from lying posture and enter LocoMode:

```bash
python deploy_mujoco/deploy_mujoco.py xml_path=g1_description/g1_29dof_LieDown.xml
```

3. Press **Start** to enter position control mode

4. Hold **R1 + A** to enter **LocoMode**, press **BACKSPACE** to make robot stand up; afterwards use joysticks to walk
   (If starting from lying posture: first press **L1 + X** to stand up → then **R1 + A** into LocoMode)

5. Hold **R1 + X** to enter **Dance mode**
   (Charleston dance). Press **L1** anytime for PassiveMode. Can switch back to LocoMode or position mode but **not recommended**.

6. Terminal will show progress bar; after dance ends you can press **R1 + A** to return to walking mode

7. In **LocoMode**, press **R1 + Y** for martial arts performance — **simulation only**

8. In **LocoMode**, press **L1 + Y** for failed martial arts — **simulation only**

9. In **LocoMode**, press **R1 + B** for kicking — **simulation only**

10. In **LocoMode**, press **L1 + A** for ASAP jump — **simulation only**

---

## 4. Real Robot Operation

1. Hang the robot first (for safety)
2. Run real deployment program:

```bash
python deploy_real/deploy_real.py
```

3. Press **Start** to enter position control mode
4. If starting from lying posture: **L1 + X** to stand → **R1 + A** to enter walking mode
5. Other actions are the same as simulation

---

## Notes & Safety Reminders

### 1. Framework Compatibility Notes

The current framework **does not support** direct deployment on **G1 robots with Orin-NX onboard computer**.

Root cause suspected: compatibility issues with `unitree_python_sdk`.

Recommended alternative:

* Use **unitree_sdk2** instead
* Use ROS dual-node architecture:

  * **C++ node**: robot communication + remote control
  * **Python node**: policy inference

---

### 2. Mimic Policy Reliability Warning

Success rate is **not guaranteed**, especially on slippery/uneven ground.

If robot becomes unstable:

* Press **F1** to activate **PassiveMode**
* Press **Select** to immediately stop program

---

### 3. Charleston Dance Stability Notice

This is currently the **only** behavior confirmed safe and stable **on the real robot**.

⚠️ Important:

* **Remove hand palms** (hands were not included in training dataset → collision risk)
* **Start/end stability may require brief human support**
* After dance:

  * Prefer switching first to **position** or **damping** protection modes
  * Provide **manual stabilization** during transitions

---

### 4. Other Actions

All other advanced motions are **NOT recommended** for real-world deployment yet.

---

### 5. Strong Recommendation

✅ Practice thoroughly in simulation **before any real-robot execution**


