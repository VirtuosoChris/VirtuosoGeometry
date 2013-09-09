#ifndef VIRTUOSO_AABB_H_INCLUDED
#define VIRTUOSO_AABB_H_INCLUDED


#include <Eigen/Geometry>


namespace Virtuoso{
    namespace Geometry{

///bounding box in some dimension.  TYPE should be some floating point primitive type
///taking advantage of Eigen funtionality to reduce code duplication, but
///some operator and method overloads make this inheritance.  See the Eigen documentation for the full set of methods available to call on this class
template <class TYPE, std::size_t DIMS>
class AABB : public Eigen::AlignedBox<TYPE, DIMS>
{
public:

    typedef Eigen::Matrix<TYPE,DIMS,1> VectorType; ///vertex in DIMS dimensions.
    typedef Eigen::Matrix<TYPE,DIMS+1,1> VectorTypeH; ///augmented vertex in homogeneous coords

    AABB() {}

    AABB(const VectorType& minBounds, const VectorType& maxBounds);

    static std::size_t numCorners();

    Eigen::Matrix<TYPE, DIMS,1> getHalfVector()const;///vector from center to max corner

    operator bool()const;///returns true if this box has volume (is not null)

    bool operator==(const AABB& in)const;///use isApprox in eigen if you are not guaranteed that the bounds will line up exactly

    bool operator!=(const AABB& in)const;

    /***matrix multiply operators follow.  multiply each corner by the input matrix, and take the bounds of the resulting points.
    if the transformation is augmented (homogeneous space) perform perspective division.
    ***/

    AABB<TYPE, DIMS> operator*(const Eigen::Matrix<TYPE,DIMS,DIMS>& mat);

    AABB<TYPE, DIMS> operator*(const Eigen::Matrix<TYPE,DIMS+1,DIMS>& mat);

    AABB<TYPE, DIMS> operator*(const Eigen::Matrix<TYPE,DIMS+1,DIMS+1>& mat);

    AABB<TYPE, DIMS> operator*(const Eigen::Matrix<TYPE,DIMS,DIMS+1>& mat);

    AABB<TYPE, DIMS>& operator*=(const Eigen::Matrix<TYPE,DIMS,DIMS>& mat);

    AABB<TYPE, DIMS>& operator*=(const Eigen::Matrix<TYPE,DIMS+1,DIMS>& mat);

    AABB<TYPE, DIMS>& operator*=(const Eigen::Matrix<TYPE,DIMS+1,DIMS+1>& mat);

    AABB<TYPE, DIMS>& operator*=(const Eigen::Matrix<TYPE,DIMS,DIMS+1>& mat);


    AABB& operator*=(const TYPE& val); ///scalar multiply

};




template <typename TYPE, std::size_t DIMS>
AABB<TYPE,DIMS> operator*(const TYPE& scalar,const AABB<TYPE,DIMS> box);

template <typename TYPE, std::size_t DIMS>
AABB<TYPE,DIMS> operator*(const AABB<TYPE,DIMS> box,const TYPE& scalar);


///matrix multiply with a box.
template<typename TYPE,std::size_t MATROWS, std::size_t DIMS>
AABB<TYPE, DIMS> matrixMultiplyBox(const Eigen::Matrix<TYPE,MATROWS,DIMS>& mat, const AABB<TYPE,DIMS>& box);


///homogeneous matrix multiply with a box
template <typename TYPE, std::size_t MATROWS,std::size_t DIMS >
AABB<TYPE, DIMS> matrixMultiplyHomogeneous( const Eigen::Matrix<TYPE,MATROWS,DIMS+1>& mat, const  AABB<TYPE,DIMS>& box);


///print the min and max bounds if or indicate that the box is null to an output stream
template<typename TYPE, std::size_t DIMS>
std::ostream& operator<<(std::ostream& str,const AABB<TYPE,DIMS>& box);


/****some helpful typedefs for syntactic convenience**/

template <class TYPE>
using AABB3 = AABB<TYPE, 3>;

template <class TYPE>
using AABB2 = AABB<TYPE, 2>;

typedef AABB2<double> AABB2d;
typedef AABB3<double> AABB3d;

typedef AABB2<float> AABB2f;
typedef AABB3<float> AABB3f;


/***Left multiplication operators
note that these left multiplication operators are explicitly loaded for N-D aabb's because the template compiler can't figure it out otherwise
***/


template<typename TYPE>
AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,2,2>& mat, const AABB2<TYPE>& box);

template<typename TYPE>
AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,3,3>& mat, const AABB3<TYPE>& box);


template<typename TYPE>
AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,3,2>& mat, const AABB2<TYPE>& box);

template<typename TYPE>
AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,4,3>& mat, const AABB3<TYPE>& box);


//homogeneous versions
template<typename TYPE>
AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,2,3>& mat, const AABB2<TYPE>& box);

template<typename TYPE>
AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,3,4>& mat, const AABB3<TYPE>& box);

template<typename TYPE>
AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,3,3>& mat, const AABB2<TYPE>& box);

template<typename TYPE>
AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,4,4>& mat, const AABB3<TYPE>& box);


#include "AABB.tcc" //include inline definitions

    }
}


#endif // VIRTUOSO_AABB_H_INCLUDED
