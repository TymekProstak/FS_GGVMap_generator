# FS_GGVMap_generator

Implementation of GGV map calculation using CasADi/Ipopt and the approach presented in *A free-trajectory quasi-steady-state optimal-control method for minimum
lap-time of race vehicles”* (2019) Matteo Massaro et al.  
Implementation is for an all-wheel-driven (AWD/4WD) Formula Student vehicle with an electric (EV) powertrain. 

Two alternative models are implemented:  
   - single-track model (e.g. dynamic bicycle model) with MF 1996 and partial load transfer (longitudinal),  
   - double-track model with MF 1996 and simplified full load transfer (quasi-static roll + longitudinal and lateral), omitting full kinetamitc Ackermann geometry, in favour of virtual rack        angle ( delta/steer) .  

---

**Formulation of the problem:**  

Following Matteo Massaro et al. (2019), the state of the vehicle in QSS is uniquely determined by: [delta, beta, kappa_fl, kappa_fr, kappa_rr, kappa_rl] if V (center of mass speed) and ax, ay (ax, ay in vehicle frame) are given → yaw rate is simply V/ay, and the rest is given by QSS equations. Optionally, box constraints on the state can be added to ensure safety of the output (for example to exclude powerslide solutions, or high tire slippage).  

The problem is formulated as a maximizaiton of rho ( " adherance radius") which is defined as rho = sqrt ( ax^2 + ay^2). For each alpha = atan(ax/ay) a SPO is solved based on previous solutions, constraints and with cost function J = -rho. Staring solutions is found by Newton's bisection for ay = 0 ( pure longitudal acceleration).
  
Important to note that the result  it is NOT acceleration in Vehicle frame( due to its rotation by a slip angle of vehicle and due to non-zero yaw rate of vehicle) but acceleration in interial frame aligned with center mass velocity V.

---

**User-specified parameters:**  

**Vehicle dynamics:**  
- g : gravity  
- m : mass  
- h : height of center of mass  
- w : wheelbase  
- b : longitudinal distance of the center of mass from rear axle  
- a : w - b  
- T : vehicle track  
- K1 : roll stiffness of front axle  
- K2 : roll stiffness of rear axle  
- Cd : reduced drag coefficient  
- Cl1 : reduced downforce coefficient for front axle  
- Cl2 : reduced downforce coefficient for rear axle  
- Pmax_drive : maximum power of powertrain  
- Pmin_recuperation : maximum power the powertrain can recuperate (engine braking)  
- pCx1 : longitudinal shape factor  
- pDx1 : longitudinal friction coefficient  
- pDx2 : longitudinal friction load dependency factor  
- pEx1 : longitudinal curvature factor  
- pKx1 : longitudinal stiffness coefficient  
- pKx3 : longitudinal stiffness load coefficient  
- lambda_x : longitudinal scaling factor  
- pCy1 : lateral shape factor  
- pDy1 : lateral friction coefficient  
- pDy2 : lateral friction load dependency factor  
- pEy1 : lateral curvature factor  
- pKy1 : cornering stiffness coefficient  
- pKy2 : cornering stiffness load coefficient  
- lambda_y : lateral scaling factor  
- N0 : nominal load (where dfz = 0)
- Cr : roll friction factor

**Vehicle state constraints:**  
- delta_u :  
- delta_d :  
- beta_u :  
- beta_d :  
- kappa_u :  
- kappa_d :  
- slip_angle_u :  
- slip_angle_d :  

// some more advanced parameterization so, for example, the wheel is not spinning or the slip angle stays within the linear range of tire operation can be added here  

Parameters are assumed to be read from a JSON file into the ParamBank structure and translated into a symbolic (SX) vector of parameters for usage in solver/problem formulation.  

---

**Form of output:**  

  
