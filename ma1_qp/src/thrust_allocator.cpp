#include "ma1_qp/thrust_allocator.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <limits>

ThrustAllocator::ThrustAllocator(double /*vesselLength*/, double /*vesselWidth*/, double maxThrust)
  : maxThrust(maxThrust)
{
    // Hardcode thruster positions per the new specification:
    // Thruster1: (1.8, -0.8) --> front-left or right depending on coordinate frame.
    // Thruster2: (1.8, 0.8)
    // Thruster3: (-1.8, 0.8)
    // Thruster4: (-1.8, -0.8)
    thrusters.resize(4);
    thrusters[0] = { 1.8, -0.8 };
    thrusters[1] = { 1.8,  0.8 };
    thrusters[2] = { -1.8, 0.8 };
    thrusters[3] = { -1.8, -0.8 };
}

std::vector<ThrusterCommand> ThrustAllocator::allocate(double Fx, double Fy, double Mz)
{
  const int nVars = 8; // [f1x, f1y, f2x, f2y, f3x, f3y, f4x, f4y]
  const int numEq = 2; // net Fx and Fy equality
  const int numBox = nVars; // box constraints on each variable
  const int numDir = 8; // directional constraints (2 per thruster)
  const int numCons = numEq + numBox + numDir;
  
  // Weight for the soft moment penalty.
  double w = 1e4; // Tune this as needed.
  
  // Moment mapping row vector.
  // For thruster i at position (rₓ, r_y), moment contribution is rₓ * f_y - r_y * fₓ.
  // With our positions:
  // Thruster1 (1.8, -0.8): 0.8 for f₁x? Actually, rearranging:
  //   Contribution = 1.8*f1y - (-0.8)*f1x = 1.8*f1y + 0.8*f1x.
  // Thruster2 (1.8, 0.8):  1.8*f2y - 0.8*f2x.
  // Thruster3 (-1.8, 0.8): -1.8*f3y - 0.8*f3x.
  // Thruster4 (-1.8, -0.8): -1.8*f4y - (-0.8)*f4x = -1.8*f4y + 0.8*f4x.
  // So we set up M_row accordingly:
  Eigen::RowVectorXd M_row(nVars);
  M_row << 0.8,  1.8,   // Thruster1
          -0.8,  1.8,   // Thruster2
          -0.8, -1.8,   // Thruster3
           0.8, -1.8;   // Thruster4
  
  // Cost function: 1/2 * fᵀ*(I + 2w*Mᵀ*M)*f - 2w*Mz*(M_row*f)
  Eigen::SparseMatrix<double> P(nVars, nVars);
  std::vector<Eigen::Triplet<double>> P_triplets;
  // Start with the identity.
  for (int i = 0; i < nVars; i++) {
      P_triplets.push_back({i, i, 1.0});
  }
  // Add 2w*Mᵀ*M.
  for (int i = 0; i < nVars; i++) {
      for (int j = 0; j < nVars; j++) {
          double value = 2 * w * M_row(i) * M_row(j);
          if (value != 0.0)
              P_triplets.push_back({i, j, value});
      }
  }
  P.setFromTriplets(P_triplets.begin(), P_triplets.end());
  
  Eigen::VectorXd q(nVars);
  for (int i = 0; i < nVars; i++) {
      q(i) = -2 * w * Mz * M_row(i);
  }
  
  // Build constraint matrix A and vectors l, u.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numCons, nVars);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(numCons);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(numCons);
  int row = 0;
  
  // (1) Equality constraints: net Fx and Fy.
  // Fx: f1x + f2x + f3x + f4x = Fx.
  A.row(row) << 1, 0, 1, 0, 1, 0, 1, 0;
  l(row) = Fx; u(row) = Fx;
  row++;
  // Fy: f1y + f2y + f3y + f4y = Fy.
  A.row(row) << 0, 1, 0, 1, 0, 1, 0, 1;
  l(row) = Fy; u(row) = Fy;
  row++;
  
  // (2) Box constraints.
  // Normally, each variable is bounded by [-maxThrust, maxThrust].
  // But if the commanded Fy has a sign, we disable thrusters that are not allowed.
  // Allowed f_y sign per thruster:
  //   Thruster1: f_y ≥ 0, Thruster2: f_y ≤ 0,
  //   Thruster3: f_y ≤ 0, Thruster4: f_y ≥ 0.
  // So if Fy < 0, then thrusters 1 and 4 must be inactive (force them to 0);
  // if Fy > 0, then thrusters 2 and 3 must be inactive.
  for (int i = 0; i < 4; i++) {
    bool forceInactive = false;
    if (Fy < 0) {
      if (i == 0 || i == 3)
         forceInactive = true;
    } else if (Fy > 0) {
      if (i == 1 || i == 2)
         forceInactive = true;
    }
    // For thruster i, the two decision variables:
    for (int j = 0; j < 2; j++) {
      A.row(row).setZero();
      A(row, 2*i + j) = 1.0;
      if (forceInactive) {
         l(row) = 0;
         u(row) = 0;
      } else {
         l(row) = -maxThrust;
         u(row) = maxThrust;
      }
      row++;
    }
  }
  
  // (3) Directional constraints to enforce the allowed force cone per thruster.
  const double INF = std::numeric_limits<double>::infinity();
  // Thruster1 (index 0): allowed [π/2, π] → f_x ≤ 0, f_y ≥ 0.
  A.row(row).setZero(); A(row, 0) = 1.0;
  l(row) = -INF; u(row) = 0; row++;
  A.row(row).setZero(); A(row, 1) = -1.0;
  l(row) = -INF; u(row) = 0; row++;
  
  // Thruster2 (index 1): allowed [-π, -π/2] → f_x ≤ 0, f_y ≤ 0.
  A.row(row).setZero(); A(row, 2) = 1.0;
  l(row) = -INF; u(row) = 0; row++;
  A.row(row).setZero(); A(row, 3) = 1.0;
  l(row) = -INF; u(row) = 0; row++;
  
  // Thruster3 (index 2): allowed [-π/2, 0] → f_x ≥ 0, f_y ≤ 0.
  A.row(row).setZero(); A(row, 4) = -1.0;
  l(row) = -INF; u(row) = 0; row++;
  A.row(row).setZero(); A(row, 5) = 1.0;
  l(row) = -INF; u(row) = 0; row++;
  
  // Thruster4 (index 3): allowed [0, π/2] → f_x ≥ 0, f_y ≥ 0.
  A.row(row).setZero(); A(row, 6) = -1.0;
  l(row) = -INF; u(row) = 0; row++;
  A.row(row).setZero(); A(row, 7) = -1.0;
  l(row) = -INF; u(row) = 0; row++;
  
  // Convert A to sparse format.
  Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  
  // Set up and solve the QP using OSQP-Eigen.
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(nVars);
  solver.data()->setNumberOfConstraints(numCons);
  solver.data()->setHessianMatrix(P);
  solver.data()->setGradient(q);
  solver.data()->setLinearConstraintsMatrix(A_sparse);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(u);
  
  if (!solver.initSolver()) {
      throw std::runtime_error("Failed to initialize OSQP solver");
  }
  OsqpEigen::ErrorExitFlag flag = solver.solveProblem();
  if (flag != OsqpEigen::ErrorExitFlag::NoError) {
      throw std::runtime_error("Failed to solve QP problem");
  }
  
  Eigen::VectorXd sol = solver.getSolution();
  
  // Final step: compute nominal (inward) angles at rest.
  double nominal[4] = { 3*M_PI/4, -3*M_PI/4, -M_PI/4, M_PI/4 };
  
  std::vector<ThrusterCommand> commands;
  commands.reserve(4);
  for (int i = 0; i < 4; i++) {
      double fx = sol(2*i);
      double fy = sol(2*i+1);
      double mag = std::sqrt(fx*fx + fy*fy);
      double angle;
      if (mag < 1e-6) {
          // At rest: use nominal inward angle.
          angle = nominal[i];
      } else {
          angle = std::atan2(fy, fx);
          // Clamp to allowed range.
          if (i == 0) { // Thruster1: allowed [π/2, π]
              if (angle < M_PI/2) angle = M_PI/2;
              if (angle > M_PI) angle = M_PI;
          } else if (i == 1) { // Thruster2: allowed [-π, -π/2]
              if (angle > -M_PI/2) angle = -M_PI/2;
              if (angle < -M_PI) angle = -M_PI;
          } else if (i == 2) { // Thruster3: allowed [-π/2, 0]
              if (angle < -M_PI/2) angle = -M_PI/2;
              if (angle > 0) angle = 0;
          } else if (i == 3) { // Thruster4: allowed [0, π/2]
              if (angle < 0) angle = 0;
              if (angle > M_PI/2) angle = M_PI/2;
          }
      }
      commands.push_back({fx, fy, mag, angle});
  }
  
  return commands;
}