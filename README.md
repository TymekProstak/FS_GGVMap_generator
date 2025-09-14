# FS_GGVMap_generator

Implementation of GGV map calculation using CasADi and the approach presented in *“An optimal control approach to the computation of g-g diagrams”* (2023) Matteo Massaro et al.  
Implementation is for an all-wheel-driven (AWD/4WD) Formula Student vehicle with an electric (EV) powertrain.

Two alternative models are implemented:  
   - single-track model (e.g. dynamic bicycle model) with MF 1996 and partial load transfer (longitudinal),  
   - double-track model with MF 1996 and simplified full load transfer (quasi-static roll + longitudinal and lateral), omitting Ackermann geometry.  

---

**Formulation of the problem:**  

Following Matteo Massaro et al. (2023), the state of the vehicle in QSS is uniquely determined by: [delta, beta, kappa_fl, kappa_fr, kappa_rr, kappa_rl] if V (center of mass speed) and ax, ay (ax, ay in vehicle frame) are given → yaw rate is simply V/ay, and the rest is given by QSS equations. Optionally, box constraints on the state can be added to ensure safety of the output (for example to exclude powerslide solutions, or high tire slippage).  

The problem is formulated as a maximization of the area under the GG envelope for a given center of mass speed (V). The independent variable is alpha (where ay/ax = tan(alpha)). Rho (radius = sqrt(ax² + ay²)) is simply the magnitude of the acceleration vector. For smoothness of the resultant solution, a small regularization term on the control input is added to the cost function.  

Control inputs are: [u_rho, u_delta, u_beta, u_kappa_fl, u_kappa_fr, u_kappa_rr, u_kappa_rl] with box constraints. They represent the vehicle state + rho derivatives with respect to alpha.  

To solve the OCP problem in CasADi, a warm start using results from SPO is utilized, and all control inputs + vehicle state are comprehended into the **X vector** – treated by the solver as the decision vector, **Z vector** – constraints (dynamic equations, box for state/control, and differential formulation for control inputs), and **J** as the cost function.  

ax and ay are given in a "velocity V frame" (x-axis along V, and y-axis perpendicular).  

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

**OCP formulation:**  
- u_rho_u :  
- u_rho_d :  
- u_delta_u :  
- u_delta_d :  
- u_beta_u :  
- u_beta_d :  
- u_kappafl_u :  
- u_kappafl_d :  
- u_kappafr_u :  
- u_kappafr_d :  
- u_kapparr_u :  
- u_kapparr_d :  
- u_kapparl_u :  
- u_kapparl_d :  

**Cost function:**  
- epsilon : regularization term weight  

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

  
