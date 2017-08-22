
# The model #
The model is entirely based on the Global kinematic model (Lesson 18. Vehicle Models. 5 Global Kinematic Model)
## State ##
The state takes into account 6 vehicle parameters. x, y, v, psi, cte and epsi.
## Actuators ##
Two actuators. Steering and acceleration.
Steering rules the vehicle direction. Its allowed values go between -0.43, 0.43 (-25, 25 degrees).
Acceleration rules the vehicle speed and rules directly the throttle.
## Update Equation ##
The update equation are the follow:

x<sub>t+1</sub>​​=x​<sub>t</sub>​​+v​<sub>t</sub>​​​​∗cos(ψ​t​<sub>t</sub>​​)∗dt

y​<sub>t+1</sub>​​=y​<sub>t</sub>​​​​+v​<sub>t</sub>​​​​∗sin(ψ​t​<sub>t</sub>​​)∗dt

ψ​<sub>t+1</sub>​​=ψ​<sub>t</sub>​​​​+​​​​​v​<sub>t</sub>​​​​​​/L<sub>f</sub>​​∗δ​<sub>t</sub>​​∗dt

v​<sub>t+1</sub>=v​t​<sub>t</sub>​​+a​<sub>t</sub>​​​​∗dt

cte​​<sub>t+1</sub>=cte​​<sub>t</sub>​​+v​<sub>t</sub>​∗sin(eψ​<sub>t</sub>)∗dt

eψ​​<sub>t+1</sub>​​=ψ​<sub>t</sub>-ψdest<sub>t</sub>​​+​​​​​​v​<sub>t</sub>​​​​​​/L<sub>f</sub>​​∗δ​<sub>t</sub>​​∗dt

Being:

cte​​<sub>t</sub>=f(x​​<sub>t</sub>​​)−y​​<sub>t</sub>

​​
f(x​​<sub>t</sub>​​) is a polynomial fitting the waypoints.

ψdest<sub>t</sub> = atan(f'(x​​<sub>t</sub>​​))

# Timestep Length and Elapsed Duration (N & dt) #
At the beginning, I started with N = 25 and dt = 0.05. The problem with this was that only Time = 25*0.05 = 1.25 seconds were simulated. It was a problem mostly in the curves.
Then I tried with bigger time, N = 25 and dt = 0.5. The result quite improved, but a very weird behavior appeared.
[Forum link](https://discussions.udacity.com/t/predicted-path-does-weird-loops-in-some-frames/344163)
Apparently the problem was that the model was going farther than the waypoints.
Reducing the number of points and dt solved the problem.
Finally, I use N = 20 and dt = 0.20

Edit Submission 2:
I have managed to increase the vehicle speed. Because of this, I needed to reduce T in order to avoid predecing farther than the waypoints. Now, N = 10, dt = 0.1

# Polynomial Fitting and MPC Preprocessing #
A 3th grade polynomial is calculated by polyfit function.
I have develop a method to derive this polynomial, in order to calculate epsi.

All the calculation is done in vehicle coordinates, so ptsx and ptsy are transformed to vehicle coordinates, and x, y and psi are equal to zero.

This transformation has proved to be very important, because I didn't manage to complete a full lap until I did the conversion.

# Model Predictive Control with Latency #
There were two possible approaches. Predicting the state of the vehicle after the latency and solving that state, or introducing the latency inside the model constraints.

I have chosen the first approach. So, the final flow would be:

- I get the state from the Json (x, y, psi, speed, steering and throttle).
- I update all the states using the update equations and the latency (0.1 seconds)
- I transform all the coordinates from world to vehicle local.
- I calculate the polynomial fitting the waypoints and its derivative
- I calculate cte and epsi
- I call the the minimizing cost solver. The solver has been modeled entirely in mpc.cpp
- I get the result. I apply the first acceleration value to the throttle and the first steering value to the steer.
- Steer needs to be transformed in order to keep it between -1 and 1 and to match what the simulator is expecting
