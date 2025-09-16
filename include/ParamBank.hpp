#pragma once
#include <nlohmann/json.hpp>
#include <casadi/casadi.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

struct ParamBank {
  std::vector<std::string> names;                 // nazwy w kolejności
  std::unordered_map<std::string,int> idx;        // nazwa -> index
  std::vector<double> values;                     // wartości liczbowe

  int add(const std::string& name, double val){
    auto it = idx.find(name);
    if (it != idx.end()){
      // nadpisz istniejącą wartość
      values[it->second] = val;
      return it->second;
    }
    int k = (int)names.size();
    names.push_back(name);
    idx[name] = k;
    values.push_back(val);
    return k;
  }

  int i(const std::string& name) const {
    auto it = idx.find(name);
    if (it == idx.end()) throw std::runtime_error("ParamBank: missing key '"+name+"'");
    return it->second;
  }

  double get(const std::string& name) const { return values.at(i(name)); }
  void   set(const std::string& name, double v){ values.at(i(name)) = v; }
  size_t size() const { return values.size(); }
};

// bezpieczne pobieranie z JSON (wymagane)
inline double JgetReq(const nlohmann::json& J, const std::string& path){
  // path w formacie "vehicle.m" albo "bounds.delta_u"
  auto pos = path.find('.');
  if (pos == std::string::npos) {
    if (!J.contains(path)) throw std::runtime_error("JSON: missing required key '"+path+"'");
    return J.at(path).get<double>();
  }
  std::string head = path.substr(0,pos);
  std::string tail = path.substr(pos+1);
  if (!J.contains(head)) throw std::runtime_error("JSON: missing object '"+head+"'");
  return JgetReq(J.at(head), tail);
}

// opcjonalne z domyślną
inline double JgetOpt(const nlohmann::json& J, const std::string& path, double def){
  try { return JgetReq(J, path); } catch(...) { return def; }
}

// budowa banku parametrów na podstawie Twojej listy + JSON
inline ParamBank build_param_bank(const nlohmann::json& J){
  ParamBank P;
  // --- Vehicle dynamics ---
  P.add("g",   JgetReq(J,"vehicle.g"));
  P.add("m",   JgetReq(J,"vehicle.m"));
  P.add("h",   JgetReq(J,"vehicle.h"));
  P.add("w",   JgetReq(J,"vehicle.w"));
  P.add("b",   JgetReq(J,"vehicle.b"));
  // a = w - b (jeśli nie podano, licz)
  double a_val = J.contains("vehicle") && J["vehicle"].contains("a")
                   ? J["vehicle"]["a"].get<double>()
                   : (P.get("w") - P.get("b"));
  P.add("a", a_val);
  P.add("T",   JgetReq(J,"vehicle.T"));
  P.add("K1",  JgetReq(J,"vehicle.K1"));
  P.add("K2",  JgetReq(J,"vehicle.K2"));
  P.add("Cd",  JgetReq(J,"vehicle.Cd"));
  P.add("Cl1", JgetReq(J,"vehicle.Cl1"));
  P.add("Cl2", JgetReq(J,"vehicle.Cl2"));
  P.add("Pmax_drive",       JgetReq(J,"vehicle.Pmax_drive"));
  P.add("Pmin_recuperation",JgetReq(J,"vehicle.Pmin_recuperation"));
  P.add("Cr",  JgetReq(J,"vehicle.Cr"));

  // --- Tire (MF makro) ---
  P.add("pCx1",    JgetReq(J,"tire.pCx1"));
  P.add("pDx1",    JgetReq(J,"tire.pDx1"));
  P.add("pDx2",    JgetReq(J,"tire.pDx2"));
  P.add("pEx1",    JgetReq(J,"tire.pEx1"));
  P.add("pKx1",    JgetReq(J,"tire.pKx1"));
  P.add("pKx3",    JgetReq(J,"tire.pKx3"));
  P.add("lambda_x",JgetReq(J,"tire.lambda_x"));
  P.add("pCy1",    JgetReq(J,"tire.pCy1"));
  P.add("pDy1",    JgetReq(J,"tire.pDy1"));
  P.add("pDy2",    JgetReq(J,"tire.pDy2"));
  P.add("pEy1",    JgetReq(J,"tire.pEy1"));
  P.add("pKy1",    JgetReq(J,"tire.pKy1"));
  P.add("pKy2",    JgetReq(J,"tire.pKy2"));
  P.add("lambda_y",JgetReq(J,"tire.lambda_y"));
  P.add("N0",      JgetReq(J,"tire.N0"));
  // --- OCP (limity pochodnych = sterowania) ---
  const char* U[] = {
    "u_rho_u","u_rho_d","u_delta_u","u_delta_d","u_beta_u","u_beta_d",
    "u_kappafl_u","u_kappafl_d","u_kappafr_u","u_kappafr_d",
    "u_kapparr_u","u_kapparr_d","u_kapparl_u","u_kapparl_d"
  };
  for (auto k: U) P.add(k, JgetReq(J,std::string("ocp.")+k));

  // --- Cost ---
  P.add("epsilon", JgetReq(J,"cost.epsilon"));

  // --- Bounds (boxy na stany) ---
  const char* B[] = {
    "rho_u",
    "delta_u","delta_d","beta_u","beta_d","kappa_u","kappa_d",
    "slip_angle_u","slip_angle_d"
  };
  for (auto k: B) P.add(k, JgetReq(J,std::string("bounds.")+k));

  return P;
}

// Adapter: symboliczny wektor p i wygodny dostęp P("nazwa")
struct CasadiParamAdapter {
  const ParamBank* bank{};
  casadi::SX p; // wektor symboliczny o rozmiarze bank->size()

  casadi::SX P(const std::string& name) const {
    return p(bank->i(name));
  }
};

// Tworzy adapter z gotowym p = SX::sym("p", N)
inline CasadiParamAdapter make_casadi_adapter(const ParamBank& bank){
  CasadiParamAdapter A;
  A.bank = &bank;
  A.p = casadi::SX::sym("p", (int)bank.size());
  return A;
}

// Przykładowa symboliczna funkcja ax, ay = g * rho * [cos α, sin α]
inline casadi::Function make_axay_fun(){
  using namespace casadi;
  SX rho   = SX::sym("rho");
  SX alpha = SX::sym("alpha");
  SX p     = SX::sym("p", 1); // tu wprost 1 parametr: g
  SX g     = p(0);
  SX ax = g * rho * cos(alpha);
  SX ay = g * rho * sin(alpha);
  return Function("axay", {rho, alpha, p}, {ax, ay});
}
