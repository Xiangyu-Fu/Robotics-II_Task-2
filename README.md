### 1.Kinematic Model
We can split the solution (**inverse kinematic model**)

$$\begin{bmatrix}\omega_1\\\ \omega_2\\\ \omega_3\\\ \omega_4 \end{bmatrix}=\frac{1}{R}\begin{bmatrix}1&1&-(l_1+l_2)\\\ 1&-1&l_1+l_2 \\\ 1&-1(l_1+l_2) \\\ 1&1&l_1+l_2 \end{bmatrix}\begin{bmatrix}v_x \\\ x_y \end{bmatrix}$$

into two controllers,

- position cotroller

$$\begin{bmatrix}\omega_1\\\ \omega_2\\\ \omega_3\\\ \omega_4\end{bmatrix}=\frac{1}{R}\begin{bmatrix}1&1\\\ 1&-1\\\ 1&-1\\\ 1&1 \end{bmatrix} \begin{bmatrix}v_x\\\ v_y\end{bmatrix}$$

- orientation cotroller

$$\begin{bmatrix}\omega_1\\\omega_2\\\omega_3\\\omega_4\end{bmatrix}=\frac{1}{R}\begin{bmatrix}-(a+b)&a+b&-(a+b)&a+b\end{bmatrix}^T\omega_z$$

if we can kown absolute position and orientation $p=[x \quad y\quad \theta]^T$,we can still simplfy the modle:
- position cotroller

$$\begin{bmatrix}\omega_1\\\omega_2\\\omega_3\\\omega_4\end{bmatrix}=\frac{1}{R}\begin{bmatrix}v_x\end{bmatrix}$$

- orientation cotroller

$$\begin{bmatrix}\omega_1\\\omega_2\\\omega_3\\\omega_4\end{bmatrix}=\frac{1}{R}\begin{bmatrix}-(a+b)&a+b&-(a+b)&a+b\end{bmatrix}^T\omega_z$$

we can assume that $v_x$ and$\omega_z$are constant. 

Therefore, if we kown there are two points，$A(x_A, y_A)$,$B(x_B, y_B)$,

![Y8xHCq.md.png](https://imgconvert.csdnimg.cn/aHR0cHM6Ly9zMS5heDF4LmNvbS8yMDIwLzA1LzExL1k4eEhDcS5tZC5wbmc?x-oss-process=image/format,png)
we can get the angle $\omega$ and the distance $x$ between A and B.

$$Angle\quad \omega = \theta(absolate\ angle)-\ arctan\frac{y_B-y_A}{x_B-x_A} $$

$$distance \quad x = \sqrt{(y_B-y_A)^2·(x_B-x_A)^2}$$

So this model can be simplified into two steps，

**step1**:according to the angle $\omega$,call the orientation controller to steer the model.The condition for the end of the loop is that the angle error is less than 0.01.

**step2**:Make the model move forward at a fixed speed and cyclically detect the distance x between the actual position of the car and the target.The condition for the end of the loop is that the angle error is less than 0.01.

---
In our task, point A is the absolute position of the model,we can use `simxGetObjectPosition`function to get the correct position without any error.Point B is the target we want the vehicle to drive to.

##### pseudo code

![YYudE9.png](https://imgconvert.csdnimg.cn/aHR0cHM6Ly9zMS5heDF4LmNvbS8yMDIwLzA1LzExL1lZdWRFOS5wbmc?x-oss-process=image/format,png)
### 2. PID controller

In the discrete time-case, the integral can be approximated as simply summing rectangles,

$$\int_0^t e(\tau) d\tau \approx \sum_{k=1}^n e_kΔt$$

It is possible then to approximate the differential partwith a **single-step backwards difference formula**,
$$f'(x_k) \approx \frac{f(x_k)-(x_{k-1})}{\Delta t}\implies \frac{e_k-e_{k-1}}{\Delta t}$$

### 3.Orientation Correction
In order to live the correct angle error, the absolute angle obtained must be corrected and converted.
```
            if(sign(real_ornt(3)) ~= sign(theta_1))
                if(theta_1>=pi/2 && real_ornt(3)<= 0)
                    theta_1_fix = theta_1 - pi;
                    real_ornt_fix = real_ornt(3)+ pi;
                elseif(theta_1<= -pi/2 && real_ornt(3)>= 0)
                    theta_1_fix = theta_1 + pi;
                    real_ornt_fix = real_ornt(3) - pi;
                end
            else
                theta_1_fix = theta_1;
                real_ornt_fix = real_ornt(3);

                
            end
            rotation_angle = real_ornt_fix - theta_1_fix;
            fprintf('       Rotation_angle:%d\n',rotation_angle);
            
            if(abs(rotation_angle)>pi)
                rotation_angle = 2*pi-abs(rotation_angle);
            end
```
