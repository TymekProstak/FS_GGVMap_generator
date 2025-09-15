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


    const int N = 181;                 // liczba węzłów po α
    const double alpha_min = -M_PI/2, alpha_max =  M_PI/2; // zakładmy symetryczną charekterstykę auta/opony 
    const double dA = (alpha_max - alpha_min) / (N-1); // przez N-1 bo trzeba domknać pętle 


     // 1) Zmienne
    std::vector<SX> vars; //  symboliczny wektor decyzyjny
    std::vector<SX> cons; // symboliczny wektor ograniczeń 
    std::vector<double> lbg, ubg;  // lowe/upper bounds na wartość ograniczeń( jeśli miękkie/nierówność a nie równość)
    std::vector<double> lbx, ubx; // lower/upper bounds na stany (
    std::vector<double> x0; // startowy wektor deczyzjny ( start solvera)




}