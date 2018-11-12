// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2018 Alec Jacobson <alecjacobson@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#include "grad_intrinsic.h"
#include "grad.h"

template <typename Derivedl, typename DerivedF, typename Gtype>
IGL_INLINE void igl::grad_intrinsic(
  const Eigen::MatrixBase<Derivedl>&l,
  const Eigen::MatrixBase<DerivedF>&F,
  Eigen::SparseMatrix<Gtype> &G)
{
  assert(F.cols() ==3 && "Only triangles supported");
  // number of vertices
  const int n = F.maxCoeff()+1;
  // number of faces
  const int m = F.rows();
  typedef Eigen::Matrix<Gtype,Eigen::Dynamic,2> MatrixX2S;
  MatrixX2S V2 = MatrixX2S::Zero(3*m,2);
  //     1=[x,y]
  //     /\
  // l3 /   \ l2
  //   /      \
  //  /         \
  // 2-----------3
  //       l1
  //
  // x = (l2²-l1²-l3²)/(-2*l1)
  // y = sqrt(l3² - x²)
  //
  //
  // Place 3rd vertex at [l(:,1) 0]
  V2.block(2*m,0,m,1) = l.col(0);
  // Place second vertex at [0 0]
  // Place third vertex at [x y]
  V2.block(0,0,m,1) = 
    (l.col(1).cwiseAbs2()-l.col(0).cwiseAbs2()-l.col(2).cwiseAbs2()).array()/
    (-2.*l.col(0)).array();
  V2.block(0,1,m,1) = 
    (l.col(2).cwiseAbs2() - V2.block(0,0,m,1).cwiseAbs2()).array().sqrt();
  DerivedF F2(F.rows(),F.cols());
  std::vector<Eigen::Triplet<Gtype> > Pijv;
  Pijv.reserve(F.size());
  for(int f = 0;f<m;f++)
  {
    for(int c = 0;c<F.cols();c++)
    {
      F2(f,c) = f+c*m;
      Pijv.emplace_back(F2(f,c),F(f,c),1);
    }
  }
  Eigen::SparseMatrix<Gtype> P(m*3,n);
  P.setFromTriplets(Pijv.begin(),Pijv.end());
  Eigen::SparseMatrix<Gtype> G2;
  grad(V2,F2,G2);
  G = G2*P;
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template void igl::grad_intrinsic<Eigen::Matrix<double, -1, 3, 0, -1, 3>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, 3, 0, -1, 3> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&);
// generated by autoexplicit.sh
template void igl::grad_intrinsic<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&);
#endif