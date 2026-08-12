# 4-DOF Robotic Arm: Forward Kinematics

## Overview
This repository contains the Forward Kinematics (FK) mathematical model and Python implementation for a 4-Degree of Freedom (DOF) articulated robotic arm. The goal of this task is to calculate the final 3D spatial coordinates (x, y, z) of the arm's end-effector based on the input angles of its four rotational joints.

## System Parameters

### Constants (Link Lengths)
Based on the physical design, the lengths of the links are defined as follows:
* **L1 (Shoulder Link):** 100 mm
* **L2 (Elbow Link):** 65 mm
* **L3 (Wrist Link):** 15 mm

### Variables (Joint Angles)
In robotics, the angle of each motor is measured relative to the preceding link. The variables are:
* **Theta 1:** Base rotation angle (controls horizontal span)
* **Theta 2:** Shoulder motor angle
* **Theta 3:** Elbow motor angle
* **Theta 4:** Wrist motor angle

---

## Kinematics Model (Trigonometric Approach)

Since this specific arm design features three upper motors moving up and down in the same plane, we use a direct trigonometric approach to solve the kinematics.

**Step 1: Calculate Total Horizontal Reach (r) and Vertical Height (z)**
We calculate the extension of the arm before accounting for the base rotation:
* `r = L1*cos(Theta_2) + L2*cos(Theta_2 + Theta_3) + L3*cos(Theta_2 + Theta_3 + Theta_4)`
* `z = L1*sin(Theta_2) + L2*sin(Theta_2 + Theta_3) + L3*sin(Theta_2 + Theta_3 + Theta_4)`

**Step 2: Project Reach onto 3D Coordinates (x, y)**
We use the base rotation angle (Theta 1) to distribute the total horizontal reach (r) across the X and Y axes:
* `x = r * cos(Theta_1)`
* `y = r * sin(Theta_1)`

---

## Python Implementation

The following function translates the mathematical model into code, returning the final coordinates in millimeters.

```python
import math

def forward_kinematics(theta1, theta2, theta3, theta4):
    # Convert angles from degrees to radians to match the math library 
    t1, t2, t3, t4 = map(math.radians, [theta1, theta2, theta3, theta4]) 
    
    # Arm link lengths (in mm) 
    L1, L2, L3 = 100, 65, 15 
    
    # Calculate horizontal reach and vertical height 
    r = L1*math.cos(t2) + L2*math.cos(t2+t3) + L3*math.cos(t2+t3+t4) 
    z = L1*math.sin(t2) + L2*math.sin(t2+t3) + L3*math.sin(t2+t3+t4) 
    
    # Calculate final X and Y coordinates 
    x = r * math.cos(t1) 
    y = r * math.sin(t1) 
    
    # Return result rounded to the nearest two decimal places 
    return round(x, 2), round(y, 2), round(z, 2)
