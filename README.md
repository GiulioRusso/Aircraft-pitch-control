# Aircraft pitch control
Control System for an aircraft in longitudinal motion. Pitch movement control is implemented with state feedback, Luenberger and Kalman observers.

<img src="./output/0_plane.png" width="50%"> <br>

# Tasks

1. Characterize the system in terms of eigenvalues, time constants and according to its reachability and observability property.

    <img src="./output/1_autovalori_tc.jpg" width="50%"> <br>

2. A state feedback controller with time constants of about 5s is designed.

    <img src="./output/4_stato_uscita_retroazione_stato.jpg" width="80%"> <br>

3. We design an output feedback to be validated with a step reference of 0.05rad, initial conditions
x(0) = [0 0 0]' and sampling step of 100ms.

    <img src="./output/6_stato_uscita_retroazione_uscita.jpg" width="80%"> <br>

4. Design two observers, one determinsitic and one at Kalman to be validated in a controller-observer structure. Design the two observers such that they have similar behaviors and with initial conditions x(0) = [0.05 0 0.01]'.

    Luenberger: <br>
    <img src="./output/8_luenberger.jpg" width="80%"> <br>

    Kalman: <br>
    <img src="./output/10_kalman.jpg" width="80%"> <br>

# File organization

The repository contains all the files to run the project with MATLAB.

- **output**: output plot.
- *A380.m*: airplane 3D model
- *ALLFUNCS.m*: all implemented functions.
- *Ciclo_Aperto.slx*: open loop Simulink file.
- *log.rtf*: MATLAB console output.
- *main.m*: starting point of the project.