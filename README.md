# Parameter-Estimation-For-Accurate-Model-based-control-Featuring-3DOF-RRR-Manipulator-PoC-Baseline

An end-to-end parameter estimation framework designed as a pure, naked mathematical Proof of Concept (PoC). This repository establishes a completely friction-free and noise-free baseline where the **MATLAB Robotics System Toolbox serves as a strict, one-to-one validation ground truth** for our data-driven symbolic math.

## 📝 Project Overview
 Because standard robotics toolbox blocks are strictly limited to pure rigid-body spatial mechanics(Just contains the inertial Dynamics of the robot arm the mass and inertia of each link no friction part is inculded in the toolbox matrices ). This repository isolates the core identification pipeline from these disturbances. By stripping away all environmental noise and non-linear joint friction, the **Robotics System Toolbox blocks function as an absolute, one-to-one verification benchmark**, proving the mathematical integrity of our symbolic equations before scaling to friction, noise-heavy robot arm models.

### Why the 1:1 Toolbox Validation Baseline is Essential

* **Definitive One-to-One Benchmarking**: Enforces an idealized environment where the outputs of our data-driven math and the native toolbox blocks (`M(qm)`, `C(qm,qmdot)`, `G(qm)`) match **1:1 with zero numerical deviation**.
* **Decoupling Core Math from Chaos**: Proves that the symbolic regressor matrices ($Y_b$), D-optimality trajectory excitation, and convex positive-definite optimization loops are structurally flawless before the toolbox becomes unviable due to real-world chaos.
* **Proving Physical Realism**: Demonstrates that the parameter estimation pipeline accurately reconstructs symmetric positive-definite mass matrices ($M(q) > 0$) using pure, uncorrupted input-output data verified directly by native MATLAB blocks.

<img width="1525" height="718" alt="image" src="https://github.com/user-attachments/assets/81495670-a90e-4039-a9b5-80639f7c14b8" />

<img width="1917" height="927" alt="image" src="https://github.com/user-attachments/assets/a2d909d6-ba24-419b-ad0f-d3efe0d9a880" />

<img width="1280" height="704" alt="Step_3_ParameterExcitation-ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/0d65c915-5845-4457-a9e8-20640900602f" />

<img width="1912" height="927" alt="image" src="https://github.com/user-attachments/assets/6b49358f-d3c7-40dd-b91a-37c3c3df0d11" />

### Against The excitation trajectory
| Joint Component | Accuracy Metric |
| :--- | :--- |
| **Joint 1 (Base Yaw)** | 99.99 % |
| **Joint 2 (Shoulder)** | 99.98 % |
| **Joint 3 (Elbow)** | 99.90 % |
| **OVERALL CONSTRAINED ACCURACY** | **99.99 %** |

<img width="1917" height="921" alt="image" src="https://github.com/user-attachments/assets/6525912d-469e-4f71-bc7e-d814effcc4bd" />

### Validation Experiment 1

| Joint Component | Fit Percentage / Accuracy |
| :--- | :--- |
| **Joint 1 (Base Yaw)** | 99.98 % |
| **Joint 2 (Shoulder)** | 99.92 % |
| **Joint 3 (Elbow)** | 99.63 % |
| **OVERALL CROSS-VALIDATION ACCURACY** | **99.95 %** |

<img width="1917" height="923" alt="image" src="https://github.com/user-attachments/assets/c37820dd-86ff-4311-96d0-5a48bdcd1de7" />

### Validation Experiment 2

| Joint Component | Fit Percentage |
| :--- | ---: |
| **Joint 1 (Base Yaw)** | 99.98 % |
| **Joint 2 (Shoulder)** | 99.97 % |
| **Joint 3 (Elbow)** | 99.79 % |
| **OVERALL CROSS-VALIDATION ACCURACY** | **99.96 %** |


<img width="1861" height="723" alt="image" src="https://github.com/user-attachments/assets/0edecf16-d7a5-47c3-9324-8103fc49817e" />

<img width="1013" height="633" alt="image" src="https://github.com/user-attachments/assets/0842467b-e584-43f5-8cce-eee311ad972d" />


<img width="1280" height="704" alt="ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/8342ef91-0c2b-46b2-8ffe-5764c68bf511" />



<img width="1917" height="640" alt="image" src="https://github.com/user-attachments/assets/7d4456e6-f8d5-4e25-b851-c39761ab1421" />


<img width="1117" height="711" alt="image" src="https://github.com/user-attachments/assets/0eaaa606-f8fb-46b3-889c-31f4fa821479" />

<img width="1917" height="602" alt="image" src="https://github.com/user-attachments/assets/3ff1657c-f3d4-4449-a786-3553c6b74b7f" />

<img width="1280" height="704" alt="ezgif com-video-to-gif-converter (2)" src="https://github.com/user-attachments/assets/5b7aea7d-7432-4885-96b3-d1e9515b2921" />

<img width="1917" height="711" alt="image" src="https://github.com/user-attachments/assets/3525bd75-fc44-4fd1-b457-7bff6ba7eff5" />


<img width="1917" height="666" alt="image" src="https://github.com/user-attachments/assets/5ec85441-9ebc-48d9-a080-6fbb206bfd91" />







## 🛠 Pipeline & File Architecture

The repository is structured sequentially to transition the manipulator from raw symbolic modeling to complete 1:1 toolbox cross-validation:

### 1. Kinematic & Symbolic Modeling
* **Step_0_RRR_import.m**: Imports the kinematic structure, links, and baseline geometric data of the RRR robot.
* **Step_1_TheRegressorModel.m**: Derives the pure data-driven symbolic regressor matrix and base parameter vector.

### 2. Trajectory Generation & Clean Data Extraction
* **Step_2_Excitation_Trajectory.m**: Runs a non-linear D-optimality trajectory optimization loop via `fmincon` using an analytical Finite Fourier Series (FFS) to enforce absolute zero boundary conditions ($q(0)=0, \dot{q}(0)=0, \ddot{q}(0)=0$).
* **Step_3_ParameterExcitation.slx**: Simulates the trajectory in a noise-free, friction-free environment to generate ideal tracking torques.
* **Step_4_Data_Extraction.slx**: Extracts the resulting noise-free joint kinematic states (Position, Velocity, Acceleration) and perfect torque profiles.


### 3. Dynamics & Convex Identification
* **Step_5_Parameter_Estimation.m**: Executes a constrained interior-point optimization algorithm to estimate physical parameters while guaranteeing a positive-definite mass matrix across a workspace coordinate grid.
* **Step_6_TestingTheEstimatedParameters.slx**: Runs verification tracking simulations using the newly identified parameters.
* **Step_7_Data_Extraction_2.slx**: Extracts the post-estimation validation tracking profiles.

### 4. Verification & Control Implementation
* **Step_8_Validation_of_Estimation.m**: Computes RMSE tracking accuracy percentages to confirm parameter validity.
* **Step_9_ReformTheEstimatedMatrices.m**: Reconstructs the full symbolic dynamic matrices from the identified parameters.
* **Step_10_CheckPositive.m**: Double-checks that the estimated joint-space mass matrix is strictly positive-definite ($M(q) > 0$) across the workspace boundaries.
* **Step_11_TheEstimated_MatricesCTC.slx**: Implements the custom model-based Calculated Torque Controller (CTC) utilizing the estimated plant parameters.
* **Step_12_The_Estimated_Matrices_CTC_Results.m**: Extracts, analyzes, and plots the tracking performance of the estimated matrix controller.

### 5. Toolbox One-to-One Cross-Validation
* **Step_13_MatlabRobotics_Toolbox_CTC.slx**: Runs an identical CTC tracking loop driven entirely by built-in rigid-body tree mechanics blocks for direct 1:1 benchmark matching.
* **Step_14_MatlabRoboticsToolbox_CTC_Results.m**: Executes the final comparative analysis, proving that the data-driven mathematical model perfectly tracks the native toolbox blocks.

## 📋 Real-Time Matrix S-Functions

Real-time feedback linearization within the custom CTC tracking loops is handled cleanly by standalone matrix generators, bypassing symbolic RAM bottlenecks:
* **get_RRR_M_sfun.m**: Generates the identified bounded joint space Inertia Matrix $M(q)$.
* **get_RRR_C_sfun.m**: Evaluates the Coriolis and Centripetal matrix $C(q, \dot{q})$.
* **get_RRR_G_sfun.m**: Computes the isolated joint space Gravity torque vector $G(q)$.

## ⚠ System Validation Architecture Note

This repository is intentionally deployed as a pure mathematical PoC. Do not add joint friction terms, sensor noise, or actuator variances into this specific codebase. Introducing non-linear physical parameters here breaks the validation mechanics against standard rigid-body toolbox blocks, destroying the 1:1 baseline verification step required for the advanced multi-body thesis pipeline.
