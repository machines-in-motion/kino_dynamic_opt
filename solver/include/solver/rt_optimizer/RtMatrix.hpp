/**
 * @file RtMatrix.hpp
 * @author Alexander Herzog
 * @author Brahayam Ponton (brahayam.ponton@tuebingen.mpg.de)
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-10-06
 */

#pragma once

#include <Eigen/Dense>

namespace rt_solver {

  // Class for creating a dynamic matrix with a pre-allocated maximum size
  template<int _MaxRows, int _MaxCols>
  struct RtMatrix
  {
    #ifdef RTEIG_DYN_ALLOCS
      typedef Eigen::MatrixXd d;
    #else
      static const int min_possible_elements_ = 2;
      typedef Eigen::Matrix<double, Eigen::Dynamic,
                            Eigen::Dynamic, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
                            (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> d;
      typedef Eigen::Matrix<float, Eigen::Dynamic,
                            Eigen::Dynamic, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
                            (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> f;
      typedef Eigen::Matrix<int, Eigen::Dynamic,
                            Eigen::Dynamic, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_,
                            (_MaxCols>=min_possible_elements_)?_MaxCols:min_possible_elements_> i;
    #endif
  };

  template<int _MaxRows, int _MaxCols>
  const int RtMatrix<_MaxRows, _MaxCols>::min_possible_elements_;

  // Class for creating a dynamic vector with a pre-allocated maximum size
  template<int _MaxRows>
  struct RtVector
  {
    #ifdef RTEIG_DYN_ALLOCS
      typedef Eigen::VectorXd d;
    #else
      static const int min_possible_elements_ = 2;
      typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> d;
      typedef Eigen::Matrix<float, Eigen::Dynamic, 1, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> f;
      typedef Eigen::Matrix<int, Eigen::Dynamic, 1, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> i;
      typedef Eigen::Matrix<bool, Eigen::Dynamic, 1, Eigen::AutoAlign,
                            (_MaxRows>=min_possible_elements_)?_MaxRows:min_possible_elements_, 1> b;
    #endif
  };

  template<int _MaxRows>
  const int RtVector<_MaxRows>::min_possible_elements_;

  // Class with utilities for Rt elements
  class RtMatrixUtils
  {
    private:
      RtMatrixUtils(){}
      virtual ~RtMatrixUtils(){}

    public:

      // creates a selector matrix based on entries of a selector vector which are more than specified threshold
      template<int MaxRows, int MaxCols>
      static inline void createSelectorMat(typename RtMatrix<MaxRows, MaxCols>::d& selector_mat,
                                           const typename RtVector<MaxRows>::d& row_selector, int num_valids, double threshold)
      {
        RtMatrixUtils::setZero(selector_mat, num_valids, row_selector.size());
        int row = 0;
        for (int col=0; col<row_selector.size(); col++)
          if (row_selector[col] > threshold)
            selector_mat(row++,col) = 1.0;
      }

      // pruning rows of matrix which are close to zero (squared norm row < threshold)
      template<int MaxRows, int MaxCols>
      static inline void removeZeroRows(typename RtMatrix<MaxRows, MaxCols>::d& mat, double threshold = remove_zero_thresh_)
      {
        if(mat.rows() == 0)
          return;

        typename RtVector<MaxRows>::d squared_norms;
        squared_norms = mat.rowwise().squaredNorm();
        const double normed_threash = squared_norms.size() * threshold * threshold;
        int num_valids = 0;
        for (int row_id=1; row_id<squared_norms.size(); row_id++)
          if(squared_norms[row_id] > normed_threash)
            ++num_valids;

        if(num_valids == mat.rows()) { return; }
        else if (num_valids == 0) { RtMatrixUtils::resize(mat, 0, mat.cols()); }
        else
        {
          typename RtMatrix<MaxRows, MaxCols>::d selector_mat;
          createSelectorMat(selector_mat, squared_norms, num_valids, normed_threash); // create selector matrix to prune zero rows
          mat = selector_mat * mat;
        }
      }

      /**!
       * Resize matrix by keeping data untouched without memory allocation.
       * Asserts that maximum size is not exceeded.
       * @param mat
       * @param rows
       * @param cols
       */
      template <typename Derived>
      static inline void conservativeResize(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(rows <= mat.MaxRowsAtCompileTime && cols <= mat.MaxColsAtCompileTime);
        #endif
        mat.derived().conservativeResize(rows, cols);
      }

      /**!
       * Resize matrix without memory allocation. Asserts that maximum size is not exceeded.
       * @param mat
       * @param rows
       * @param cols
       */
      template <typename Derived>
      static inline void resize(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(rows <= mat.MaxRowsAtCompileTime && cols <= mat.MaxColsAtCompileTime);
        #endif
        mat.derived().resize(rows, cols);
      }

      template <typename Derived>
      static inline void setZero(Eigen::MatrixBase<Derived>& mat)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(mat,1, 1);
        #else
          resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
        #endif
        mat.derived().setZero();
      }

      template <typename Derived>
      static inline void setZero(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
      {
        resize(mat, rows, cols);
        mat.derived().setZero();
      }

      template <typename Derived>
      static inline void setOnes(Eigen::MatrixBase<Derived>& mat)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(mat,1, 1);
        #else
          resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
        #endif
        mat.derived().setOnes();
      }

      template <typename Derived>
      static inline void setOnes(Eigen::MatrixBase<Derived>& mat, int rows, int cols)
      {
        resize(mat, rows, cols);
        mat.derived().setOnes();
      }

      template <typename Derived>
      static inline void setConstant(Eigen::MatrixBase<Derived>& mat, double scalar)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(mat,1, 1);
        #else
          resize(mat, mat.MaxRowsAtCompileTime, mat.MaxColsAtCompileTime);
        #endif
        mat.derived().setConstant(scalar);
      }

      template <typename Derived>
      static inline void setConstant(Eigen::MatrixBase<Derived>& mat, int rows, int cols, double scalar)
      {
        resize(mat, rows, cols);
        mat.derived().setConstant(scalar);
      }

      template <typename Derived>
      static inline void setIdentity(Eigen::MatrixBase<Derived>& mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(mat.MaxRowsAtCompileTime == mat.MaxColsAtCompileTime);
        #endif
        #ifdef RTEIG_DYN_ALLOCS
          resize(mat,1, 1);
        #else
          resize(mat, mat.MaxRowsAtCompileTime, mat.MaxRowsAtCompileTime);
        #endif
        mat.derived().setIdentity();
      }

      template <typename Derived>
      static inline void setIdentity(Eigen::MatrixBase<Derived>& mat, int rows)
      {
        resize(mat, rows, rows);
        mat.derived().setIdentity();
      }

      // Append a matrix to another one
      template <typename AffMatDerived, typename SubstMatDerived>
      static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(subst_mat.cols() == aff_mat.cols() && aff_mat.rows() + subst_mat.rows() <= aff_mat.MaxRowsAtCompileTime);
        #endif
        const int last_row = aff_mat.rows();
        if (last_row == 0) { aff_mat = subst_mat; }
        else {
          RtMatrixUtils::conservativeResize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols());
          aff_mat.block(last_row, 0, subst_mat.rows(), subst_mat.cols()) = subst_mat;
        }
      }

      /**!
       * Append a matrix at the end of another one starting from a certain column.
       * AffMat and SubstMat are not required to have the same number of columns.
       */
      template <typename AffMatDerived, typename SubstMatDerived>
      static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<SubstMatDerived>& subst_mat, int starting_column)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(subst_mat.cols() + starting_column <= aff_mat.cols() && aff_mat.rows() + subst_mat.rows() <= aff_mat.MaxRowsAtCompileTime);
        #endif
        const int last_row = aff_mat.rows();
        if (last_row == 0) { RtMatrixUtils::resize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols()); }
        else { RtMatrixUtils::conservativeResize(aff_mat, aff_mat.rows() + subst_mat.rows(), aff_mat.cols()); }
        aff_mat.block(last_row, 0, subst_mat.rows(), aff_mat.cols()).setZero();
        aff_mat.block(last_row, starting_column, subst_mat.rows(), subst_mat.cols()) = subst_mat;
      }

      template<typename sigmaPinvDer, typename svdMatDer>
      static void computeSigmaPinv(Eigen::MatrixBase<sigmaPinvDer>& sigma_pinv_vec, const Eigen::JacobiSVD<svdMatDer>& mat_svd);

      template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols, typename nullDer>
      static int computeNullspaceMap(
        const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat, Eigen::MatrixBase<nullDer>& null_map,
        const Eigen::JacobiSVD<typename RtMatrix<max_mat_rows, max_mat_cols>::d >& mat_svd, const double mat_cond_threas = mat_condition_thresh_);

      template<typename Mat>
      static inline double conditionNumber(const Mat& mat)
      {
        Eigen::JacobiSVD<Mat> mat_svd;
        mat_svd.compute(mat);
        return conditionNumberFromSvd(mat_svd);
      }

      template<typename Mat, int QRPreconditioner>
      static inline double conditionNumberFromSvd(const Eigen::JacobiSVD<Mat, QRPreconditioner>& mat_svd)
      { return mat_svd.singularValues()[0]/mat_svd.singularValues()[mat_svd.singularValues().size()-1]; }

      static const double remove_zero_thresh_;
      static const double mat_condition_thresh_;

  };

  template<typename sigmaPinvDer, typename svdMatDer>
  void RtMatrixUtils::computeSigmaPinv(Eigen::MatrixBase<sigmaPinvDer>& sigma_pinv_vec, const Eigen::JacobiSVD<svdMatDer>& mat_svd)
  {
    #ifndef RTEIG_NO_ASSERTS
      assert(sigma_pinv_vec.rows() == mat_svd.singularValues().size() && sigma_pinv_vec.cols() == 1 && "sigma_pinv_vec wrongly sized");
    #endif

    const double min_sing_val = mat_svd.singularValues()[0]/mat_condition_thresh_;
    int id;
    for (id=0; id<mat_svd.singularValues().size(); id++)
      if (mat_svd.singularValues()[id]>min_sing_val) { sigma_pinv_vec[id] = 1.0/mat_svd.singularValues()[id]; }
      else                                           { sigma_pinv_vec[id] = 0.0; break; }
    sigma_pinv_vec.segment(id+1, sigma_pinv_vec.size()-id);
  }

  template<int _mat_rows, int _mat_cols, int _mat_align, int max_mat_rows, int max_mat_cols, typename nullDer>
  int RtMatrixUtils::computeNullspaceMap(
    const Eigen::Matrix<double, _mat_rows, _mat_cols, _mat_align, max_mat_rows, max_mat_cols>& mat, Eigen::MatrixBase<nullDer>& null_map,
    const Eigen::JacobiSVD<typename RtMatrix<max_mat_rows, max_mat_cols>::d >& mat_svd, const double mat_cond_threas)
  {
    int dim_range = 0;

    const double first_sv = mat_svd.singularValues()[0];
    for(dim_range=0; dim_range<mat_svd.singularValues().size(); ++dim_range)
      if (mat_svd.singularValues()[dim_range] <= 0.0 || first_sv / mat_svd.singularValues()[dim_range] > mat_cond_threas)
        break;

    const int dim_null = mat.cols() - dim_range;
    RtMatrixUtils::resize(null_map, mat.cols(), dim_null);  //asserts that null_map has enough space

    if (null_map.cols() > 0)
      null_map = mat_svd.matrixV().rightCols(dim_null);

    #ifndef RTEIG_NO_ASSERTS
      assert((mat * null_map).norm() < 0.0001 && "Nullspace map is not correctly computed");
      typename RtMatrix<max_mat_cols, max_mat_cols>::d kernel_id;
      RtMatrixUtils::setIdentity(kernel_id, dim_null);
      assert((null_map.transpose() * null_map - kernel_id).norm() < 0.0001 && "Nullspace map is not correctly computed");
    #endif
    return dim_range;
  }


  class RtVectorUtils
  {
    private:
      RtVectorUtils(){}
      virtual ~RtVectorUtils(){}

    public:
      template <typename Derived>
      static inline void conservativeResize(Eigen::MatrixBase<Derived>& vec, int rows)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(rows <= vec.MaxSizeAtCompileTime);
        #endif
        vec.derived().conservativeResize(rows);
      }

      template <typename Derived>
      static inline void resize(Eigen::MatrixBase<Derived>& vec, int rows)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(rows <= vec.MaxSizeAtCompileTime);
        #endif
        vec.derived().resize(rows);
      }

      template <typename Derived>
      static inline void setZero(Eigen::MatrixBase<Derived>& vec)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(vec, 1);
        #else
          resize(vec, vec.MaxSizeAtCompileTime);
        #endif
        vec.derived().setZero();
      }

      template <typename Derived>
      static inline void setZero(Eigen::MatrixBase<Derived>& vec, int size)
      {
        resize(vec, size);
        vec.derived().setZero();
      }

      template <typename Derived>
      static inline void setOnes(Eigen::MatrixBase<Derived>& vec)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(vec, 1);
        #else
          resize(vec, vec.MaxSizeAtCompileTime);
        #endif
        vec.derived().setOnes();
      }

      template <typename Derived>
      static inline void setOnes(Eigen::MatrixBase<Derived>& vec, int size)
      {
        resize(vec, size);
        vec.derived().setOnes();
      }

      template <typename Derived>
      static inline void setConstant(Eigen::MatrixBase<Derived>& vec, double scalar)
      {
        #ifdef RTEIG_DYN_ALLOCS
          resize(vec, 1);
        #else
          resize(vec, vec.MaxSizeAtCompileTime);
        #endif
        vec.derived().setConstant(scalar);
      }

      template <typename Derived>
      static inline void setConstant(Eigen::MatrixBase<Derived>& vec, int size, double scalar)
      {
        resize(vec, size);
        vec.derived().setConstant(scalar);
      }

      // Append a vector to another one
      template <typename AffVecDerived, typename SubstVecDerived>
      static inline void append(Eigen::MatrixBase<AffVecDerived>& aff_vec, const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(aff_vec.rows() + subst_vec.rows() <= aff_vec.MaxSizeAtCompileTime);
        #endif
        const int last_row = aff_vec.rows();
        RtVectorUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_vec.rows());
        aff_vec.block(last_row, 0, subst_vec.rows(), 1) = subst_vec;
      }
  };

  class RtAffineUtils
  {
    private:
      RtAffineUtils(){}
      virtual ~ RtAffineUtils(){}

    public:

      // Append an affine mapping at the end of another one
      template <typename AffMatDerived, typename AffVecDerived, typename SubstMatDerived>
      static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat,
        Eigen::MatrixBase<AffVecDerived>& aff_vec, const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(aff_mat, aff_vec);
          assert(aff_vec.rows() + subst_mat.rows() <= aff_vec.MaxSizeAtCompileTime);
        #endif
        const int last_row = aff_vec.rows();
        RtVectorUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_mat.rows());
        RtMatrixUtils::append(aff_mat, subst_mat);
        aff_vec.block(last_row, 0, subst_mat.rows(),1).setZero();
      }

      template <typename AffMatDerived, typename AffVecDerived, typename SubstMatDerived, typename SubstVecDerived>
      static inline void append(Eigen::MatrixBase<AffMatDerived>& aff_mat, Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(aff_mat, aff_vec);
          assertMatchingDimensions(subst_mat, subst_vec);
        #endif

        const int last_row = aff_vec.rows();
        append(aff_mat, aff_vec, subst_mat);
        aff_vec.block(last_row, 0, subst_mat.rows(),1) = subst_vec;
      }

      /**!
       * Append an affine mapping at the end of another one starting from a certain column.
       * AffMat and SubstMat are not required to have the same number of columns.
       */
      template <typename AffMatDerived, typename AffVecDerived, typename SubstMatDerived>
      static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat, Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, int starting_column)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(aff_mat, aff_vec);
          assert(aff_vec.rows() + subst_mat.rows() <= aff_vec.MaxSizeAtCompileTime);
        #endif
        const int last_row = aff_vec.rows();
        RtVectorUtils::conservativeResize(aff_vec, aff_vec.rows() + subst_mat.rows());
        RtMatrixUtils::appendAtColumn(aff_mat, subst_mat, starting_column);
        aff_vec.block(last_row, 0, subst_mat.rows(),1).setZero();
      }

      template <typename AffMatDerived, typename AffVecDerived, typename SubstMatDerived, typename SubstVecDerived>
      static inline void appendAtColumn(Eigen::MatrixBase<AffMatDerived>& aff_mat, Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, const Eigen::MatrixBase<SubstVecDerived>& subst_vec, int starting_column)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(aff_mat, aff_vec);
          assertMatchingDimensions(subst_mat, subst_vec);
        #endif

        const int last_row = aff_vec.rows();
        appendAtColumn(aff_mat, aff_vec, subst_mat, starting_column);
        aff_vec.block(last_row, 0, subst_mat.rows(),1) = subst_vec;
      }

      template<int MaxRows, int MaxCols>
      static inline void removeZeroRows(typename RtMatrix<MaxRows, MaxCols>::d& mat,
        typename RtVector<MaxRows>::d& vec, double threshold = RtMatrixUtils::remove_zero_thresh_)
      {
        if (mat.rows() == 0) { return; }

        typename RtVector<MaxRows>::d squared_norms;
        squared_norms = mat.rowwise().squaredNorm();
        const double normed_thresh = std::sqrt(double(squared_norms.size())) * threshold * threshold;
        int num_valids = 0;
        for (int row_id=0; row_id<squared_norms.size(); row_id++)
          if(squared_norms[row_id] > normed_thresh)
            ++num_valids;

        if (num_valids == 0) {
          RtVectorUtils::resize(vec, 0);
          RtMatrixUtils::resize(mat, 0, mat.cols());
        } else {
          typename RtMatrix<MaxRows, MaxCols>::d selector_mat;
          RtMatrixUtils::createSelectorMat<MaxRows, MaxCols>(selector_mat, squared_norms, num_valids, normed_thresh);
          mat = selector_mat * mat;
          vec = selector_mat * vec;
        }
      }

      template <typename Mat, typename Vec>
      static inline void removeZeroRows(Eigen::EigenBase<Mat>& mat, Eigen::EigenBase<Vec>& vec)
      {
        const double length_thresh = 1.e-12;
        int first_empty_i=0;
        for (; first_empty_i < mat.rows(); ++first_empty_i)
          if (mat.derived().block(first_empty_i, 0, 1, mat.cols()).array().abs().sum() < length_thresh)
            break;

        // found an empty row, now find the next nonempty row
        int block_start_i = first_empty_i+1;
        while (true) {
          for (; block_start_i < mat.rows(); ++block_start_i)
            if (mat.derived().block(block_start_i, 0, 1, mat.cols()).array().abs().sum() > length_thresh)
              break;

          // if the rest of the matrix is zero, stop
          if (block_start_i >= mat.rows())
            break;

          // now find the end of the block
          int block_end_i=block_start_i+1;
          for (; block_end_i < mat.rows(); ++block_end_i)
            if (mat.derived().block(block_end_i, 0, 1, mat.cols()).array().abs().sum() < length_thresh)
              break;

          //now shift the block left
          vec.derived().block(first_empty_i, 0, block_end_i-block_start_i, 1) =
          vec.derived().block(block_start_i, 0, block_end_i-block_start_i,1);
          mat.derived().block(first_empty_i, 0, block_end_i-block_start_i, mat.cols()) =
          mat.derived().block(block_start_i, 0, block_end_i-block_start_i, mat.cols());

          first_empty_i += block_end_i-block_start_i;
          block_start_i = block_end_i;
        }
        RtVectorUtils::conservativeResize(vec.derived(), first_empty_i);
        RtMatrixUtils::conservativeResize(mat.derived(), first_empty_i, mat.cols());
      }

      /**!
       * Given aff_mat*x + aff_vec, substitute x = subst_mat * y + subst_vec into x.
       */
      template <typename AffMatDerived, typename AffVecDerived, typename SubstMatDerived, typename SubstVecDerived>
      static inline void substitute(Eigen::MatrixBase<AffMatDerived>& aff_mat, Eigen::MatrixBase<AffVecDerived>& aff_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(aff_mat, aff_vec);
          assertMatchingDimensions(subst_mat, subst_vec);
        #endif
        aff_vec += aff_mat*subst_vec;
        substitute(aff_mat, subst_mat);
      }

      template <typename AffMatDerived, typename SubstMatDerived>
      static inline void substitute(Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(aff_mat.rows()*subst_mat.cols() <= aff_mat.MaxSizeAtCompileTime);
        #endif
        aff_mat = aff_mat*subst_mat;
      }

      #ifndef RTEIG_NO_ASSERTS
        template <typename AffMatDerived, typename AffVecDerived>
        static inline void assertMatchingDimensions(const Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<AffVecDerived>& aff_vec)
        {
          assert(aff_mat.rows() == aff_vec.rows());
          assert(aff_vec.cols() == 1);
        }
      #endif
  };


  class RtQuadraticUtils
  {
    private:
      RtQuadraticUtils(){}
      virtual ~ RtQuadraticUtils(){}

    public:
      // CARE with const correctness: quad_mat is NOT const! const will be casted away!
      template <typename QuadMatDerived, typename AffMatDerived>
      static inline void addFromNorm(const Eigen::MatrixBase<QuadMatDerived>& quad_mat, const Eigen::MatrixBase<AffMatDerived>& aff_mat)
      {
        const_cast<Eigen::MatrixBase<QuadMatDerived>&>(quad_mat) += aff_mat.transpose() * aff_mat;
      }

      // CARE with const correctness: quad_mat is NOT const! const will be casted away!
      template <typename QuadMatDerived, typename QuadVecDerived, typename AffMatDerived, typename AffVecDerived>
      static inline void addFromNorm(const Eigen::MatrixBase<QuadMatDerived>& quad_mat, const Eigen::MatrixBase<QuadVecDerived>& quad_vec,
        const Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<AffVecDerived>& aff_vec)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(quad_mat, quad_vec);
          RtAffineUtils::assertMatchingDimensions(aff_mat, aff_vec);
        #endif
        addFromNorm(quad_mat, aff_mat);
        const_cast<Eigen::MatrixBase<QuadVecDerived>&>(quad_vec) += aff_mat.transpose() * aff_vec;
      }

      template <typename QuadMatDerived, typename AffMatDerived, typename WeightMatDerived>
      static inline void addFromWeightedNorm(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
        const Eigen::MatrixBase<AffMatDerived>& aff_mat, const Eigen::MatrixBase<WeightMatDerived>& weight_mat)
      {
        quad_mat += aff_mat.transpose() * weight_mat * aff_mat;
      }

      template <typename QuadMatDerived, typename QuadVecDerived, typename AffMatDerived, typename AffVecDerived, typename WeightMatDerived>
      static inline void addFromWeightedNorm(Eigen::MatrixBase<QuadMatDerived>& quad_mat,
        Eigen::MatrixBase<QuadVecDerived>& quad_vec, const Eigen::MatrixBase<AffMatDerived>& aff_mat,
        const Eigen::MatrixBase<AffVecDerived>& aff_vec, const Eigen::MatrixBase<WeightMatDerived>& weight_mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(quad_mat, quad_vec);
          RtAffineUtils::assertMatchingDimensions(aff_mat, aff_vec);
        #endif
        addFromWeightedNorm(quad_mat, aff_mat, weight_mat);
        quad_vec += aff_mat.transpose() * weight_mat * aff_vec;
      }

      template <typename QuadMatDerived, typename SubstMatDerived>
      static inline void substitute(Eigen::MatrixBase<QuadMatDerived>& quad_mat, const Eigen::MatrixBase<SubstMatDerived>& subst_mat)
      {
        #ifndef RTEIG_NO_ASSERTS
          assert(subst_mat.cols()*subst_mat.cols() <= quad_mat.MaxSizeAtCompileTime);
        #endif
        quad_mat = subst_mat.transpose() * quad_mat *subst_mat;
      }

      template <typename QuadMatDerived, typename QuadVecDerived, typename SubstMatDerived, typename SubstVecDerived>
      static inline void substitute(Eigen::MatrixBase<QuadMatDerived>& quad_mat, Eigen::MatrixBase<QuadVecDerived>& quad_vec,
        const Eigen::MatrixBase<SubstMatDerived>& subst_mat, const Eigen::MatrixBase<SubstVecDerived>& subst_vec)
      {
        #ifndef RTEIG_NO_ASSERTS
          assertMatchingDimensions(quad_mat, quad_vec);
          RtAffineUtils::assertMatchingDimensions(subst_mat, subst_vec);
          assert(subst_mat.cols() <= quad_vec.MaxSizeAtCompileTime);
        #endif
        quad_vec += quad_mat*subst_vec;
        quad_vec = subst_mat.transpose() * quad_vec;
        substitute(quad_mat, subst_mat);
      }

      #ifndef RTEIG_NO_ASSERTS
        template <typename QuadMatDerived, typename QuadVecDerived>
        static inline void assertMatchingDimensions(const Eigen::MatrixBase<QuadMatDerived>& quad_mat, const Eigen::MatrixBase<QuadVecDerived>& quad_vec)
        {
          assert(quad_mat.rows() == quad_mat.cols() && quad_mat.cols() == quad_vec.rows());
          assert(quad_vec.cols() == 1);
        }
      #endif

  };

}
