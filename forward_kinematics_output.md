# Forward Kinematics — Modified DH Method

## DH Parameter Table

| Joint i | a_{i-1} | α_{i-1} (°) | d_i | θ_i |
|---------|----------|-------------|-----|-----|
| 1 | 0 | 0 | 114 | θ1 |
| 2 | 0 | 90 | 0 | θ2 |
| 3 | 400 | 0 | 0 | θ3 |
| 4 | 300 | 0 | 165.4 | θ4 |
| 5 | 0 | 90 | 136 | θ5 |
| 6 | 0 | -90 | 233.5 | θ6 |

> **Note:** Angles are in degrees. `θ1`–`θ6` are the joint variables.

## Individual Transformation Matrices

### $T_{0}^{1}$

$$
\left[\begin{matrix}\cos{\left(θ_{1} \right)} & - \sin{\left(θ_{1} \right)} & 0 & 0\\\sin{\left(θ_{1} \right)} & \cos{\left(θ_{1} \right)} & 0 & 0\\0 & 0 & 1 & 114\\0 & 0 & 0 & 1\end{matrix}\right]
$$


### $T_{1}^{2}$

$$
\left[\begin{matrix}\cos{\left(θ_{2} \right)} & - \sin{\left(θ_{2} \right)} & 0 & 0\\0 & 0 & -1 & 0\\\sin{\left(θ_{2} \right)} & \cos{\left(θ_{2} \right)} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
$$


### $T_{2}^{3}$

$$
\left[\begin{matrix}\cos{\left(θ_{3} \right)} & - \sin{\left(θ_{3} \right)} & 0 & 400\\\sin{\left(θ_{3} \right)} & \cos{\left(θ_{3} \right)} & 0 & 0\\0 & 0 & 1 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
$$


### $T_{3}^{4}$

$$
\left[\begin{matrix}\cos{\left(θ_{4} \right)} & - \sin{\left(θ_{4} \right)} & 0 & 300\\\sin{\left(θ_{4} \right)} & \cos{\left(θ_{4} \right)} & 0 & 0\\0 & 0 & 1 & 165.4\\0 & 0 & 0 & 1\end{matrix}\right]
$$


### $T_{4}^{5}$

$$
\left[\begin{matrix}\cos{\left(θ_{5} \right)} & - \sin{\left(θ_{5} \right)} & 0 & 0\\0 & 0 & -1 & -136\\\sin{\left(θ_{5} \right)} & \cos{\left(θ_{5} \right)} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
$$


### $T_{5}^{6}$

$$
\left[\begin{matrix}\cos{\left(θ_{6} \right)} & - \sin{\left(θ_{6} \right)} & 0 & 0\\0 & 0 & 1 & 233.5\\- \sin{\left(θ_{6} \right)} & - \cos{\left(θ_{6} \right)} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]
$$


## Overall Transformation Matrix $T_0^7$

$$
\left[\begin{matrix}\left(\sin{\left(θ_{1} \right)} \sin{\left(θ_{5} \right)} + \cos{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)}\right) \cos{\left(θ_{6} \right)} - \sin{\left(θ_{6} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{1} \right)} & - \left(\sin{\left(θ_{1} \right)} \sin{\left(θ_{5} \right)} + \cos{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)}\right) \sin{\left(θ_{6} \right)} - \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{1} \right)} \cos{\left(θ_{6} \right)} & \sin{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} - \sin{\left(θ_{5} \right)} \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} & 233.5 \sin{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} + 165.4 \sin{\left(θ_{1} \right)} - 233.5 \sin{\left(θ_{5} \right)} \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 136.0 \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{1} \right)} + 400.0 \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} \right)} + 300.0 \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} \right)}\\\left(\sin{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} - \sin{\left(θ_{5} \right)} \cos{\left(θ_{1} \right)}\right) \cos{\left(θ_{6} \right)} - \sin{\left(θ_{1} \right)} \sin{\left(θ_{6} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} & \left(- \sin{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + \sin{\left(θ_{5} \right)} \cos{\left(θ_{1} \right)}\right) \sin{\left(θ_{6} \right)} - \sin{\left(θ_{1} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{6} \right)} & - \sin{\left(θ_{1} \right)} \sin{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} - \cos{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} & - 233.5 \sin{\left(θ_{1} \right)} \sin{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 136.0 \sin{\left(θ_{1} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 400.0 \sin{\left(θ_{1} \right)} \cos{\left(θ_{2} \right)} + 300.0 \sin{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} \right)} - 233.5 \cos{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} - 165.4 \cos{\left(θ_{1} \right)}\\\sin{\left(θ_{6} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{5} \right)} \cos{\left(θ_{6} \right)} & - \sin{\left(θ_{6} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{5} \right)} + \cos{\left(θ_{6} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} & - \sin{\left(θ_{5} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} & 400.0 \sin{\left(θ_{2} \right)} - 233.5 \sin{\left(θ_{5} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 300.0 \sin{\left(θ_{2} + θ_{3} \right)} - 136.0 \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 114.0\\0 & 0 & 0 & 1\end{matrix}\right]
$$


## End-Effector Position

$$P_x = 233.5 \sin{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} + 165.4 \sin{\left(θ_{1} \right)} - 233.5 \sin{\left(θ_{5} \right)} \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 136.0 \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} \cos{\left(θ_{1} \right)} + 400.0 \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} \right)} + 300.0 \cos{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} \right)}$$

$$P_y = - 233.5 \sin{\left(θ_{1} \right)} \sin{\left(θ_{5} \right)} \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 136.0 \sin{\left(θ_{1} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 400.0 \sin{\left(θ_{1} \right)} \cos{\left(θ_{2} \right)} + 300.0 \sin{\left(θ_{1} \right)} \cos{\left(θ_{2} + θ_{3} \right)} - 233.5 \cos{\left(θ_{1} \right)} \cos{\left(θ_{5} \right)} - 165.4 \cos{\left(θ_{1} \right)}$$

$$P_z = 400.0 \sin{\left(θ_{2} \right)} - 233.5 \sin{\left(θ_{5} \right)} \sin{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 300.0 \sin{\left(θ_{2} + θ_{3} \right)} - 136.0 \cos{\left(θ_{2} + θ_{3} + θ_{4} \right)} + 114.0$$
