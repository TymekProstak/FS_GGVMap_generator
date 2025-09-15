#include <gtest/gtest.h>
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

// -------------------- Test 1 --------------------

TEST(ParamBank, LoadsAndIndexes){
  auto J = load_json("config/example_config.json");
  ParamBank P = build_param_bank(J);

  EXPECT_NEAR(P.get("g"), 9.81, 1e-9);
  EXPECT_NEAR(P.get("w") - P.get("b"), P.get("a"), 1e-12);

  // kilka przykładowych kluczy
  EXPECT_NO_THROW( P.i("pCx1") );
  EXPECT_NO_THROW( P.i("u_rho_u") );
  EXPECT_NO_THROW( P.i("delta_d") );
  EXPECT_NO_THROW( P.i("epsilon") );
}

// -------------------- Test 2 --------------------

TEST(CasADi, AxAyWithParams){
  auto J = load_json("config/example_config.json");
  ParamBank P = build_param_bank(J);
  auto A = make_casadi_adapter(P);

  casadi::SX rho   = casadi::SX::sym("rho");
  casadi::SX alpha = casadi::SX::sym("alpha");
  casadi::SX g     = A.P("g");

  casadi::SX ax = g * rho * cos(alpha);
  casadi::SX ay = g * rho * sin(alpha);
  

  casadi::Function axay("axay_ext", {rho, alpha, A.p}, {ax, ay});

  // Przygotuj wejścia
  casadi::DM rho_val   = 1.0;
  casadi::DM alpha0    = 0.0;
  casadi::DM alpha30   = M_PI / 6.0;
  casadi::DM pvals     = casadi::DM(P.values); // wartości parametrów

  // Case 1: α = 0
  auto out0 = axay(std::vector<casadi::DM>{rho_val, alpha0, pvals});
  double ax0 = static_cast<double>(out0.at(0).scalar());
  double ay0 = static_cast<double>(out0.at(1).scalar());
  EXPECT_NEAR(ax0, P.get("g") * 1.0, 1e-9);
  EXPECT_NEAR(ay0, 0.0, 1e-12);

  // Case 2: α = 30°
  auto out1 = axay(std::vector<casadi::DM>{0.5, alpha30, pvals});
  double ax1 = static_cast<double>(out1.at(0).scalar());
  double ay1 = static_cast<double>(out1.at(1).scalar());
  EXPECT_NEAR(ax1, P.get("g") * 0.5 * std::cos(static_cast<double>(alpha30)), 1e-9);
  EXPECT_NEAR(ay1, P.get("g") * 0.5 * std::sin(static_cast<double>(alpha30)), 1e-9);
}

// -------------------- Test 3 --------------------

TEST(CasADi, SimpleStandaloneAxAy){
  auto F = make_axay_fun(); // ta funkcja zwraca casadi::Function

  casadi::DM rho   = 1.0;
  casadi::DM alpha = 0.0;
  casadi::DM p     = casadi::DM(std::vector<double>{9.81}); // zakładamy, że [g]

  auto out = F(std::vector<casadi::DM>{rho, alpha, p});
  double ax = static_cast<double>(out.at(0).scalar());
  double ay = static_cast<double>(out.at(1).scalar());

  EXPECT_NEAR(ax, 9.81, 1e-12);
  EXPECT_NEAR(ay, 0.0, 1e-12);
}
