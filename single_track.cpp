#include <iostream>
#include<cmath>
#include<vector>
#include<string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <casadi/casadi.hpp>
#include "ParamBank.hpp"

using json = nlohmann::json;
using namespace casadi;

// -------------------- Util --------------------

static json load_json(const std::string& path){
  std::ifstream f(path);
  if(!f) throw std::runtime_error("Cannot open JSON: "+path);
  json J; f >> J; return J;
}
// -------------------- Main code --------------------


int main(){
    // Buduje bank parametrów na podstawie JSON-a :  P
    // oraz udostępnia adapter do wygodnej pracy z CasADi : A
    auto J = load_json("config/example_config.json");
    ParamBank P = build_param_bank(J);
    auto A = make_casadi_adapter(P);


    const int N = 181;                 // liczba węzłów po α -> MUSI BYĆ NIEPARZYSTA by sztucznie domknąć pętle 
    const double alpha_min = -M_PI/2, alpha_max =  3* M_PI/2; // zakładmy  

    // by domknać pętle i wymusić periodczynosć warunków brzegowych zakładamy sztucznie że alpha[0]  jest geometrycznie tożsame z alpha[N-1] 
    const double dA = (alpha_max - alpha_min) / (N-1); // przez N-1 bo trzeba domknać pętle ->  mamy N punktów ale dwa są koincydentne by zamknąc pętle więc N-1 odcinków zamiast pełnych N


     // 1) Zmienne  solvera
    std::vector<SX> vars; //  symboliczny wektor decyzyjny
    std::vector<SX> cons; // symboliczny wektor ograniczeń 
    std::vector<double> lbg, ubg;  // lowe/upper bounds na wartość ograniczeń( jeśli miękkie/nierówność a nie równość)
    std::vector<double> lbx, ubx; // lower/upper bounds na stany (
    std::vector<double> x0; // startowy wektor deczyzjny ( start solvera)

   
    // Stany i sterowania w każdym węźle ( zarownó stany jak i sterowania są funkcją α oraz są traktowane jako stany OCP)

    std::vector<SX> rho(N), urho(N); // acc magitude and derative wr to alpha
    std::vector<SX> beta(N), ubeta(N); // slip angle of vehicle and its derivative wr to alpha
    std::vector<SX> delta(N), udelta(N); //  virtual steering angle and its derivative wr to alpha
    std::vector<SX> kf(N), ukf(N); // front wheels slip ratio and its derivative wr to alpha
    std::vector<SX> kr(N), ukr(N); //  rear wheels slip ratio and its derivative wr to alpha

    for (int i = 0; i<N;i++){

    rho[i] = SX::sym("rho_"+std::to_string(i));
    urho[i] = SX::sym("urho_"+std::to_string(i));
    beta[i] = SX::sym("beta_"+std::to_string(i));
    ubeta[i] = SX::sym("ubeta_"+std::to_string(i));
    delta[i] = SX::sym("delta_"+std::to_string(i));
    udelta[i] = SX::sym("udelta_"+std::to_string(i));
    kf[i] = SX::sym("kf_"+std::to_string(i));
    ukf[i] = SX::sym("ukf_"+std::to_string(i));
    kr[i] = SX::sym("kr_"+std::to_string(i));
    ukr[i] = SX::sym("ukr_"+std::to_string(i));

    // Dodajemy zmienne do wektora decyzyjnego
    vars.insert(vars.end(), {rho[i], urho[i], beta[i], ubeta[i], delta[i], udelta[i], kf[i], ukf[i], kr[i], ukr[i]});
    
    // lower/upper bounds na stany + sterowania( traktowane przez OCP zbiorczo jako stany)
     // Dodajemy ograniczenia dolne (lbx) na podstawie parametrów z P
    lbx.insert(lbx.end(), {
          0,                              // rho >= 0
          P.get("u_rho_d"),              // urho >= -u_rho_d
          P.get("beta_d"),                // beta >= beta_d
          P.get("u_beta_d"),             // ubeta >= -u_beta_d
          P.get("delta_d"),               // delta >= delta_d
          P.get("u_delta_d"),            // udelta >= -u_delta_d
          P.get("kappa_d"),               // kfr >= kappa_d
          P.get("u_kappafr_d"),          // ukfr >= -u_kappafr_d
          P.get("kappa_d"),               // kfl >= kappa_d
          P.get("u_kappafl_d"),          // ukfl >= -u_kappafl_d
      });
  
      // Dodajemy ograniczenia górne (ubx) na podstawie parametrów z P
      ubx.insert(ubx.end(), {
          P.get("rho_u"),          // rho <= 
          P.get("u_rho_u"),               // urho <= u_rho_u
          P.get("beta_u"),                // beta <= beta_u
          P.get("u_beta_u"),              // ubeta <= u_beta_u
          P.get("delta_u"),               // delta <= delta_u
          P.get("u_delta_u"),             // udelta <= u_delta_u
          P.get("kappa_u"),               // kfr <= kappa_u
          P.get("u_kappafr_u"),           // ukfr <= u_kappafr_u
          P.get("kappa_u"),               // kfl <= kappa_u
          
      });

    }

  // Startowy wektor decyzyjny (x0) ? 

    
  // 2) Równania transkrypcji i QSS
  auto axay = [&](SX rho_i, double a){
    SX ax = rho_i * cos(a);
    SX ay = rho_i * sin(a);
    return std::make_pair(ax, ay);
    // "a" to tutaj jest kąt alfa 
  };

  float V = 15; // stała prędkość pojazdu [m/s] -> tutaj to jest do przemyśleni ale to gdy będzie cała pętla i zwracanie

  for (int i=0; i<N ;i++){
    double alpha_i = alpha_min + i*dA;
    auto [ax, ay] = axay(rho[i], alpha_i);

    SX ay_vehframe = ay * cos(beta[i]) + ax * sin(beta[i]) ;
    SX ax_vehframe = ax * cos(beta[i]) - ay * sin(beta[i]) ;

    SX omega =  ay_vehframe/ V * cos(beta[i]) ; // yaw rate assuming QSS conditions

    SX v = V * sin(beta[i]) ; // lateral velocity
    SX u = V * cos(beta[i]) ; // longitudinal velocity

    // normal forces and load transfer caculation:

    SX F_normal_f = A.P("m") * A.P("g") * A.P("a") / A.P("w") + A.P("Cl1") * V *V;
    SX F_normal_r = A.P("m") * A.P("g") * A.P("b") / A.P("w") + A.P("Cl2") * V *V;

    F_normal_f -= ax_vehframe * A.P("m") * A.P("h") / A.P("w") ;
    F_normal_r += ax_vehframe * A.P("m") * A.P("h") / A.P("w") ;

    // Pacejka 1996 -  tire forces

    // slip angles and slip ratios calculation:

    SX lambda_f = delta[i] - atan( (v + omega * A.P("a"))/ u); // front slip angle 
    SX lambda_r =  - atan( v - omega * A.P("b")) ; // rear slip angle
    
    SX slip_x_f = kf[i]/(1 + kf[i]); // front simga x ( following Pacejka 1996 notatnion)
    SX slip_x_r =  kr[i]/(1 + kr[i]); // rear simga x ( following Pacejka 1996 notatnion)
    SX slip_y_f = lambda_f/(1 + kf[i]); // front simga y ( following Pacejka 1996 notatnion)
    SX slip_y_r = lambda_r/(1 + kr[i]); // rear simga y ( following Pacejka 1996 notatnion)
    
    SX slip_f = sqrt(slip_x_f*slip_x_f + slip_y_f*slip_y_f); // combined slip front (Pacejka 1996)
    SX slip_r = sqrt(slip_x_r*slip_x_r + slip_y_r*slip_y_r); // combined slip rear (Pacejka 1996)
    
    // Front tire force calculation

    // makro parameters D, C, B, E calculation based on Pacejka 1996
    
    SX dfz_f = ( F_normal_f - A.P("N0"))/ A.P("N0");
    SX Kx_f =  F_normal_f  * A.P("pKx1") * exp( dfz_f * A.P("pKx3"));
    SX Ex_f = A.P("pEx1");
    SX Dx_f = ( A.P("pDx1") + A.P("pDx2") *dfz_f ) * A.P("lambda_x");
    SX Cx_f = A.P("pCx1");
    SX Bx_f = Kx_f/Cx_f/Dx_f/F_normal_f;

    SX Ky_f = A.P("N0") * A.P("pKy1") * sin(2 * atan( F_normal_f/A.P("pKy2")/A.P("N0")) );
    SX Ey_f = A.P("pEy1");
    SX Dy_f = (A.P("pDy1") + A.P("pDy2") * dfz_f ) * A.P("lambda_y");
    SX Cy_f = A.P("pCy1");
    SX By_f = Ky_f/Cy_f/Dy_f/F_normal_f;
    //

    SX F_x_f = F_normal_f * slip_x_f/slip_f * Dx_f * sin( Cx_f * atan(Bx_f * slip_f  - Ex_f *(Bx_f * slip_f - atan(Bx_f * slip_f)))) - F_normal_f * A.P("Cr") ;
    SX F_y_f = F_normal_f * slip_y_f/slip_f * Dy_f * sin( Cy_f * atan(By_f * slip_f  - Ey_f *(By_f * slip_f - atan(By_f * slip_f)))) ;
   


    // Rear tire force calcuation 

    // makro parameters D, C, B, E calculation based on Pacejka 1996 

    SX dfz_r = ( F_normal_r - A.P("N0"))/ A.P("N0");
    SX Kx_r =  F_normal_r  * A.P("pKx1") * exp( dfz_r * A.P("pKx3"));
    SX Ex_r = A.P("pEx1");
    SX Dx_r = ( A.P("pDx1") + A.P("pDx2") *dfz_r ) * A.P("lambda_x");
    SX Cx_r = A.P("pCx1");
    SX Bx_r= Kx_r/Cx_r/Dx_r/F_normal_r;

    SX Ky_r = A.P("N0") * A.P("pKy1") * sin(2 * atan( F_normal_r/A.P("pKy2")/A.P("N0")) );
    SX Ey_r = A.P("pEy1");
    SX Dy_r = (A.P("pDy1") + A.P("pDy2") * dfz_r ) * A.P("lambda_y");
    SX Cy_r = A.P("pCy1");
    SX By_r = Ky_r/Cy_r/Dy_r/F_normal_r;  

    //

    SX F_x_r =  F_normal_r * slip_x_r/slip_r * Dx_r * sin( Cx_r * atan(Bx_r * slip_r  - Ex_r *(Bx_r * slip_r - atan(Bx_r * slip_r)))) - F_normal_r * A.P("Cr")  ;
    SX F_y_r = F_normal_r * slip_y_r/slip_r * Dy_r * sin( Cy_r * atan(By_r * slip_r  - Ey_f *(By_r * slip_r - atan(By_r * slip_r)))) ;
   

    // Drag Forces

    SX FD = V * V * A.P("Cd") ;
    // Resulting force / moment

    SX Fx = F_x_r + F_x_f * cos(delta[i]) - FD  - F_y_f * sin(delta[i]);
    SX Fy = F_y_r + F_y_f * cos(delta[i]) + F_x_f * sin( delta[i]);
    SX Mz = - F_y_r * A.P("b") + F_y_f * cos(delta[i]) * A.P("a") + F_x_f * sin(delta[i]) * A.P("a");

    cons.push_back(Fx - A.P("m") * ax_vehframe); lbg.push_back(0); ubg.push_back(0); // F = m*a
    cons.push_back(Fy - A.P("m") * ay_vehframe); lbg.push_back(0); ubg.push_back(0); // F = m*a
    cons.push_back(Mz);      lbg.push_back(0); ubg.push_back(0); // moment equilibrium -> QSS cornering assumption

    // physical state augmented by its derative formulation equations
    
   

    if( i < N-1)
    {

    SX drho = rho[i+1] - rho[i] - urho[i]*dA;
    SX dbeta = beta[i+1] - beta[i] - ubeta[i]*dA;
    SX ddelta = delta[i+1] - delta[i] - udelta[i]*dA;
    SX dkf = kf[i+1] - kf[i] - ukf[i]*dA;
    SX dkr = kr[i+1] - kr[i] - ukr[i]*dA;

    cons.push_back(drho); lbg.push_back(0); ubg.push_back(0);
    cons.push_back(dbeta); lbg.push_back(0); ubg.push_back(0);
    cons.push_back(ddelta); lbg.push_back(0); ubg.push_back(0);
    cons.push_back(dkf); lbg.push_back(0); ubg.push_back(0);
    cons.push_back(dkr); lbg.push_back(0); ubg.push_back(0);
    }

    // Power limit 

    SX P_motor = (F_x_r + F_x_f ) * u ; // simple model of powertrain consumption - power at wheels = force * velocity
    double  P_max = P.get("Pmax_drive"); // max power of motor
    double  P_min = P.get("Pmin_recuperation"); // max power of motor in recuperation ( negative value)
    cons.push_back( P_motor  ); lbg.push_back(P_min); ubg.push_back(P_max); // power limit ->  P <= P_max


    
  }

  //  Wymuszenie domknięcie pętli ( warunki brzegowe)
  cons.push_back( beta[ N-1] -  beta[0]); lbg.push_back(0); ubg.push_back(0); 
  cons.push_back( rho[N-1] -  rho[0]); lbg.push_back(0); ubg.push_back(0);  
  cons.push_back( delta[ N-1] +  delta[0]); lbg.push_back(0); ubg.push_back(0); 
  cons.push_back( kf[ N-1] -  kf[0]); lbg.push_back(0); ubg.push_back(0); 
  cons.push_back( kr[ N-1] -  kr[0]); lbg.push_back(0); ubg.push_back(0); 


  
  




}