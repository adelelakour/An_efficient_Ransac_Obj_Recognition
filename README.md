**3D Object Recognition and Grasping System
**This repository contains the implementation of my master’s thesis project, "Designing a System for Learning, Representing, Selecting, and Executing Robot Grasps of Point Clouds of Household Objects", submitted to the Department of Informatics at Technische Universität München.

**Overview**
The system integrates 3D object recognition and robotic grasping, addressing challenges in cluttered and occluded environments. Inspired by the paper "An Efficient RANSAC for 3D Object Recognition in Noisy and Occluded Scenes", the project combines efficient point cloud processing and grasp planning to enable robust robotic manipulation.

**Key features include:
**
Object Recognition: Identifies objects in occluded and cluttered scenes using a RANSAC-based algorithm.
Pose Estimation: Estimates 3D poses of objects relative to a global reference frame.
Grasp Planning: Leverages precomputed grasps for YCB objects using the Simox platform.
The system was tested on a Franka Emika robotic arm with an Intel RealSense D435i camera, achieving precision: 0.536, recall: 0.5415, and F1-score: 0.515 across 12 scenes with 32 YCB models.

**Highlights**
Offline Preprocessing: Precomputed 10 grasps per object using Simox, optimizing for robust manipulation scenarios.
Efficient Recognition: Uses a novel RANSAC variant for reduced computational complexity in noisy and occluded scenes.
Scalable Design: Supports integration with diverse robotic hardware and applications.
Future Work
Planned improvements include:

Refining optimal grasp selection criteria.
Enhancing robustness in grasp execution.
Expanding the object library and adapting to additional 3D sensors.
**Usage**
Clone the repository and install dependencies.
Precompute grasps for your model set or load precomputed data.
Run the object recognition and grasp execution pipeline to test with a robotic arm.
**Acknowledgments**
This project was conducted under the supervision of Prof. Darius Burschka and Andrei Costinescu, with access to state-of-the-art resources at Technische Universität München.
