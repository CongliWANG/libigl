// This file is part of libigl, a simple C++ geometry processing library.
//
// Copyright (C) 2020 Xiangyu Kong <xiangyu.kong@mail.utoronto.ca>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "direct_delta_mush.h"
#include "cotmatrix.h"
#include "diag.h"


// ===== DEBUG: START
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SparseQR>
#include <Eigen/QR>
#include <iostream>
#include "Timer.h"

using namespace std;
// ===== DEBUG: END

// TODOs:

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedC,
  typename DerivedE,
  typename DerivedW,
  typename DerivedT,
  typename DerivedTAlloc,
  typename DerivedU>
IGL_INLINE void igl::direct_delta_mush(
  const Eigen::MatrixBase<DerivedV> &V,
  const Eigen::MatrixBase<DerivedF> &F,
  const Eigen::MatrixBase<DerivedC> &C,
  const Eigen::MatrixBase<DerivedE> &E,
  const Eigen::SparseMatrix<DerivedW> &W,
  const std::vector<DerivedT, DerivedTAlloc> &T,
  Eigen::PlainObjectBase<DerivedU> &U)
{
  cout << "START DDM" << endl;
  cout << "END DDM" << endl;
}

template <
  typename DerivedV,
  typename DerivedF,
  typename DerivedC,
  typename DerivedE,
  typename DerivedW,
  typename DerivedOmega>
IGL_INLINE void igl::direct_delta_mush_precomputation(
  const Eigen::MatrixBase<DerivedV> &V,
  const Eigen::MatrixBase<DerivedF> &F,
  const Eigen::MatrixBase<DerivedC> &C,
  const Eigen::MatrixBase<DerivedE> &E,
  const Eigen::SparseMatrix<DerivedW> &W,
  const int p,
  const typename DerivedV::Scalar lambda,
  const typename DerivedV::Scalar kappa,
  const typename DerivedV::Scalar alpha,
  Eigen::PlainObjectBase<DerivedOmega> &Omega)
{
  assert(kappa < lambda &&
    "kappa needs to be smaller than lambda so that optimization for R_i is well defined");
  cout << "START DDM Precomputation" << endl;
  cout << "Using params:"
       << "\np: " << p
       << "\nlambda: " << lambda
       << "\nkappa: " << kappa
       << endl;
  cout << "V: " << V.rows() << " x " << V.cols() << " Sum: " << V.sum()
       << "\nF: " << F.rows() << " x " << F.cols() << " Sum: " << F.sum()
       << "\nC: " << C.rows() << " x " << C.cols() << " Sum: " << C.sum()
       << "\nE: " << E.rows() << " x " << E.cols() << " Sum: " << E.sum()
       << "\nW: " << W.rows() << " x " << W.cols() << " Sum: " << W.sum()
       << endl;

  const int n = V.rows();
  const int m = C.rows();

  // U: 4 by #V homogeneous transposed version of V
  Eigen::MatrixXd U(4, n);
  U.block(0, 0, 3, n) = V.transpose();
  Eigen::VectorXd ones(n);
  for (int i = 0; i < n; i++)
  {
      ones(i) = 1;
  }
  U.row(U.rows() - 1) = ones;
  cout << "U: " << U.rows() << " x " << U.cols() << " Sum: " << U.sum() << endl;
  // sparse version of U
  Eigen::SparseMatrix<double> U_sparse = U.sparseView();
  cout << "U_sparse: " << U_sparse.rows() << " x " << U_sparse.cols() << " Sum: " << U_sparse.sum() << endl;

  // Identity of #V by #V
  Eigen::SparseMatrix<double> I(n, n);
  I.setIdentity();
  cout << "I: " << I.rows() << " x " << I.cols() << " Sum: " << I.sum() << endl;

  // Laplacian: L_bar = L \times D_L^{-1}
  Eigen::SparseMatrix<double> L;
  igl::cotmatrix(V, F, L);
  cout << "L: " << L.rows() << " x " << L.cols() << " Sum: " << L.sum() << endl;
  Eigen::MatrixXd D_L = L.diagonal().asDiagonal();
  cout << "D_L: " << D_L.rows() << " x " << D_L.cols() << " Sum: " << D_L.sum() << endl;
  Eigen::SparseMatrix<double> D_L_sparse = D_L.sparseView();
  cout << "D_L_sparse: " << D_L_sparse.rows() << " x " << D_L_sparse.cols() << " Sum: " << D_L_sparse.sum() << endl;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
  ldlt.compute(D_L_sparse);
  Eigen::SparseMatrix<double> L_bar = ldlt.solve(L).transpose();
  cout << "L_bar: " << L_bar.rows() << " x " << L_bar.cols() << " Sum: " << L_bar.sum() << endl;

  // Implicitly and iteratively solve
  // w'_{ij} = \sum_{k=1}^{n}{C_{ki} w_{kj}}, C = (I + kappa L_bar)^{-p}:
  // W' = C^T \times W
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt_W_prime;
  Eigen::SparseMatrix<double> C_calc((I + kappa * L_bar).transpose());
  cout << "C_calc: " << C_calc.rows() << " x " << C_calc.cols() << " Sum: " << C_calc.sum() << endl;
  Eigen::SparseMatrix<double> W_prime(W);
  cout << "W_prime: " << W_prime.rows() << " x " << W_prime.cols() << " Sum: " << W_prime.sum() << endl;
  ldlt_W_prime.compute(C_calc);
  cout << "computing W'" << endl;
  for (int iter = 0; iter < p; iter++) {
    cout << "iter:" << iter << endl;
    W_prime.makeCompressed();
    W_prime = ldlt_W_prime.solve(W_prime);
  }
  cout << "W_prime: " << W_prime.rows() << " x " << W_prime.cols() << " Sum: " << W_prime.sum() << endl;

  // Psi was hard to solve iteratively since i couldnt express u_k \times u_k^T as matrix form
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt_U_tilde;
  Eigen::SparseMatrix<double> B_calc((I + lambda * L_bar).transpose());
  cout << "B_calc: " << B_calc.rows() << " x " << B_calc.cols() << " Sum: " << B_calc.sum() << endl;
  ldlt_U_tilde.compute(B_calc);

  Eigen::SparseMatrix<double> U_tilde(U_sparse.transpose());
  cout << "U_tilde: " << U_tilde.rows() << " x " << U_tilde.cols() << " Sum: " << U_tilde.sum() << endl;
  cout << "computing U_tilde'" << endl;
  for (int i = 0; i < p; i++) {
    cout << "i = " << i << endl;
    U_tilde.makeCompressed();
    U_tilde = ldlt_U_tilde.solve(U_tilde);
  }
  U_tilde = U_tilde.transpose();
  cout << "U_tilde: " << U_tilde.rows() << " x " << U_tilde.cols() << " Sum: " << U_tilde.sum() << endl;

  // TODO: sparse didn't work here
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr_B;
  Eigen::MatrixXd U_dense(U_sparse);
  Eigen::MatrixXd U_tilde_dense(U_tilde);
  Eigen::MatrixXd B_inv_dense = U_dense.householderQr().solve(U_tilde_dense);
  Eigen::SparseMatrix<double> B_inv = B_inv_dense.sparseView();
  // U_sparse.makeCompressed();
  // Eigen::SparseMatrix<double> V_sparse_transpose(U_sparse), V_tilde_transpose(U_tilde);
  // qr_B.compute(V_sparse_transpose);
  // Eigen::SparseMatrix<double> B_inv = qr_B.solve(V_tilde_transpose);
  cout << "B_inv: " << B_inv.rows() << " x " << B_inv.cols() << " Sum: " << B_inv.sum() << endl;

  // Psi: #V * #C by 16 (10) of \Psi_{ij}s.
  // To access \Psi_{ij}, look at row (i * m + j)
  Eigen::MatrixXd Psi(n * m, 16); // TODO: this could be 10 to reduce storage

  // TODO: FIXME: VECTORIZE THIS
  for (int i = 0; i < n; i++)
  {
    cout << "i = " << i << " / n = " << n << endl;
    for (int j = 0; j < m; j++)
    {
      Eigen::MatrixXd Psi_curr = Eigen::MatrixXd::Zero(4, 4);
      for (int k = 0; k < n; k++)
      {
        Psi_curr += B_inv.coeff(k, i) * W.coeff(k, j) * U.col(k) * U.col(k).transpose();
      }
      Eigen::VectorXd Psi_vector(Eigen::Map<Eigen::VectorXd>(Psi_curr.data(), Psi.cols()*Psi.rows()));
      Psi.row(i * m + j) = Psi_vector;
    }
  }
  cout << "Psi: " << Psi.rows() << " x " << Psi.cols() << " Sum: " << Psi.sum() << endl;

  // vector ps
  Eigen::MatrixXd P_vectors(n, 3);
  for (int i = 0; i < n; i++)
  {
    Eigen::Vector3d p_i = Eigen::Vector3d::Zero(3);
    // Eigen::seq not usable???
    for (int j = 0; j < m; j++)
    {
      Eigen::Vector3d p_i_curr(3);
      p_i_curr << Psi(i + j * n, 3), Psi(i + j * n, 7), Psi(i + j * n, 11);
      p_i += p_i_curr;
    }
    P_vectors.row(i) = p_i;
  }

  // Omega
  Omega.resize(n, 16);
  for (int i = 0; i < n; i++)
  {
    Eigen::Matrix3d p_matrix(4, 4);
    Eigen::Vector3d curr_p = P_vectors.row(i);
    p_matrix.block(0, 0, 3, 3) = curr_p * curr_p.transpose();
    p_matrix.block(3, 0, 3, 1) = curr_p.transpose();
    p_matrix.block(0, 3, 3, 3) = curr_p;
    p_matrix(3, 3) = 1;
    for (int j = 0; j < m; j++)
    {
      Eigen::MatrixXd Omega_curr(4, 4);
      Omega_curr = (1 - alpha) * Psi.block(i * m + j, 0, 1, 16).eval() + alpha * W_prime.coeff(i, j) * p_matrix;
      Eigen::VectorXd Omega_vector(Eigen::Map<Eigen::VectorXd>(Omega_curr.data(), Omega.cols()*Omega.rows()));
      Omega.row(i * m + j) = Omega_vector;
    }
  }


  cout << "END DDM Precomputation" << endl;
}

template <
  typename DerivedT,
  typename DerivedTAlloc,
  typename DerivedOmega,
  typename DerivedU>
IGL_INLINE void igl::direct_delta_mush_pose_evaluation(
  const std::vector<DerivedT, DerivedTAlloc> &T,
  const Eigen::MatrixBase<DerivedOmega> &Omega,
  Eigen::PlainObjectBase<DerivedU> &U)
{
  cout << "START DDM Pose Eval" << endl;
  cout << "END DDM Pose Eval" << endl;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
template void igl::direct_delta_mush<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double, Eigen::Transform<double, 3, 2, 0>, Eigen::aligned_allocator<Eigen::Transform<double, 3, 2, 0> >, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int> const&, std::__1::vector<Eigen::Transform<double, 3, 2, 0>, Eigen::aligned_allocator<Eigen::Transform<double, 3, 2, 0> > > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
template void igl::direct_delta_mush_precomputation<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int> const&, int, Eigen::Matrix<double, -1, -1, 0, -1, -1>::Scalar, Eigen::Matrix<double, -1, -1, 0, -1, -1>::Scalar, Eigen::Matrix<double, -1, -1, 0, -1, -1>::Scalar, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
template void igl::direct_delta_mush_pose_evaluation<Eigen::Transform<double, 3, 2, 0>, Eigen::aligned_allocator<Eigen::Transform<double, 3, 2, 0> >, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(std::__1::vector<Eigen::Transform<double, 3, 2, 0>, Eigen::aligned_allocator<Eigen::Transform<double, 3, 2, 0> > > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
#endif