

template <typename TYPE, std::size_t DIMS>
AABB<TYPE,DIMS>::AABB(const VectorType& minBounds, const VectorType& maxBounds)
    :
    Eigen::AlignedBox<TYPE,DIMS>(minBounds,maxBounds)
{
}



template <class TYPE, std::size_t DIMS>
AABB<TYPE,DIMS>& AABB<TYPE,DIMS>::operator*=(const TYPE& val)
{
    this->min() *= val;
    this->max() *= val;
    return *this;
}





template <class TYPE, std::size_t DIMS>
inline Eigen::Matrix<TYPE, DIMS,1>  AABB<TYPE,DIMS>::getHalfVector()const
{
    const TYPE sc = static_cast<TYPE>(.5);

    return  sc * this->diagonal();

}



template <class TYPE, std::size_t DIMS>
inline AABB<TYPE,DIMS>::operator bool()const
{

    return ! this->isNull();

}



template <class TYPE, std::size_t DIMS>
inline bool  AABB<TYPE,DIMS>::operator==(const AABB& in)const
{

    return (this->min() == in.min() )&& (this->max() == in.max());

}



template <class TYPE, std::size_t DIMS>
inline bool  AABB<TYPE,DIMS>::operator!=(const AABB& in)const
{
    return !((*this) == in);
}




template <class TYPE, std::size_t DIMS,class VERTEX_ITERATOR>
AABB<TYPE,DIMS> pointBounds(VERTEX_ITERATOR begin, VERTEX_ITERATOR end)
{

    typedef typename AABB<TYPE,DIMS>::VectorType VectorType;

    const TYPE& maxd = std::numeric_limits<TYPE>::max();

    VectorType minBoundsNew;

    for(std::size_t i = 0; i < DIMS; i++)
    {
        minBoundsNew[i] = maxd;
    }

    VectorType maxBoundsNew  = -minBoundsNew;

    for(VERTEX_ITERATOR it = begin; it != end; it++)
    {

        VectorType& vec = *it; //need to be able to get a vector out of this iterator type that points to a vertex in DIMS dimensions

        for(std::size_t d = 0; d < DIMS; d++)
        {
            minBoundsNew[d] = std::min(minBoundsNew[d], vec[d]);
            maxBoundsNew[d] = std::max(maxBoundsNew[d], vec[d]);
        }

    }

    return AABB<TYPE, DIMS> (minBoundsNew, maxBoundsNew);

}







template <typename TYPE, std::size_t DIMS>
inline AABB<TYPE,DIMS> operator*(const AABB<TYPE,DIMS> box,const TYPE& scalar){

    return AABB<TYPE,DIMS>(box.min() * scalar, box.max() * scalar);

}


template <typename TYPE, std::size_t DIMS>
inline AABB<TYPE,DIMS> operator*(const TYPE& scalar,const AABB<TYPE,DIMS> box){

    return box * scalar;

}




template<typename TYPE, std::size_t DIMS>
std::ostream& operator<<(std::ostream& str,const AABB<TYPE,DIMS>& box){

    if(box)
    str<<"AABB Min Bounds: \n"<<box.min()<<"\n\nAABB Max Bounds:\n"<<box.max()<<std::endl;

    else
    str<<"NULL Box"<<std::endl;

    return str;

}




template<typename TYPE,std::size_t MATROWS, std::size_t DIMS>
AABB<TYPE, DIMS> matrixMultiplyBox(const Eigen::Matrix<TYPE,MATROWS,DIMS>& mat, const AABB<TYPE,DIMS>& box)
{
//        AABB<TYPE,DIMS>& box = *this;

    typedef typename AABB<TYPE,DIMS>::VectorType VectorType;

    const std::size_t cornerCt = 1<<DIMS;

    VectorType boxCorners[2] = {box.min(), box.max()}; //min and max corners of the box

    VectorType transformedCorners[cornerCt];

    const TYPE& maxd = std::numeric_limits<TYPE>::max();

    VectorType minBoundsNew;

    for(std::size_t i = 0; i < DIMS; i++)
    {
        minBoundsNew[i] = maxd;
    }

    VectorType maxBoundsNew  = -minBoundsNew;

    //transform the corners
    for(std::size_t i = 0; i < cornerCt; i++)
    {

        VectorType tvec;

        //initialize the corners using binary iterations of the min and max
        //stored in boxCorners

        for(std::size_t d = 0; d < DIMS; d++)
        {

            tvec[d] = boxCorners[ (i>>d) & 1 ][d];

        }

        transformedCorners[i] = mat * tvec;

        //take the min and max since the transformed corners will no longer be axis aligned
        for(std::size_t d = 0; d < DIMS; d++)
        {

            minBoundsNew[d] = std::min(minBoundsNew[d], transformedCorners[i][d]);
            maxBoundsNew[d] = std::max(maxBoundsNew[d], transformedCorners[i][d]);

        }

    }

    return AABB<TYPE, DIMS>(minBoundsNew, maxBoundsNew);

}





///homogeneous multiply
template <typename TYPE, std::size_t MATROWS,std::size_t DIMS >
AABB<TYPE, DIMS> matrixMultiplyHomogeneous( const Eigen::Matrix<TYPE,MATROWS,DIMS+1>& mat, const  AABB<TYPE,DIMS>& box)
{
    typedef typename AABB<TYPE,DIMS>::VectorTypeH VectorTypeH;
    typedef typename AABB<TYPE,DIMS>::VectorType VectorType;

    const std::size_t cornerCt = 1<<DIMS;

    VectorType boxCorners[2] = {box.min(), box.max()}; //min and max corners of the box

    VectorTypeH transformedCorners[cornerCt];

    const TYPE& maxd = std::numeric_limits<TYPE>::max();

    VectorType minBoundsNew;

    for(std::size_t i = 0; i < DIMS; i++)
    {
        minBoundsNew[i] = maxd;
    }

    VectorType maxBoundsNew  = -minBoundsNew;

    //transform the corners
    for(std::size_t i = 0; i < cornerCt; i++)
    {

        VectorTypeH tvec;

        //initialize the corners using binary iterations of the min and max
        //stored in boxCorners

        for(std::size_t d = 0; d < DIMS; d++)
        {

            tvec[d] = boxCorners[ (i>>d) & 1 ][d];

        }

        tvec[DIMS] = 1.0;

        transformedCorners[i] = mat * tvec;

        transformedCorners[i] /= transformedCorners[i][DIMS];///divide by w to make the last coordinate always 1.0

        //take the min and max since the transformed corners will no longer be axis aligned
        for(std::size_t d = 0; d < DIMS; d++)
        {

            minBoundsNew[d] = std::min(minBoundsNew[d], transformedCorners[i][d]);
            maxBoundsNew[d] = std::max(maxBoundsNew[d], transformedCorners[i][d]);

        }

    }

    return AABB<TYPE, DIMS>(minBoundsNew, maxBoundsNew);
}


///////////////////////////////////////



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS> AABB<TYPE,DIMS>::operator*(const Eigen::Matrix<TYPE,DIMS,DIMS>& mat)
{
    return matrixMultiplyBox<TYPE,DIMS,DIMS>(mat,*this);
}



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS> AABB<TYPE,DIMS>::operator*(const Eigen::Matrix<TYPE,DIMS+1,DIMS>& mat)
{
    return matrixMultiplyBox<TYPE,DIMS+1,DIMS>(mat, *this);
}



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS> AABB<TYPE,DIMS>::operator*(const Eigen::Matrix<TYPE,DIMS+1,DIMS+1>& mat)
{
    return matrixMultiplyHomogeneous<TYPE,DIMS+1,DIMS>(mat, *this);
}



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS> AABB<TYPE,DIMS>::operator*(const Eigen::Matrix<TYPE,DIMS,DIMS+1>& mat)
{
    return matrixMultiplyHomogeneous<TYPE,DIMS,DIMS>(mat, *this);
}





template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS>& AABB<TYPE,DIMS>::operator*=(const Eigen::Matrix<TYPE,DIMS,DIMS>& mat)
{
       return ((*this) =  (*this)*mat);
}



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS>& AABB<TYPE,DIMS>::operator*=(const Eigen::Matrix<TYPE,DIMS+1,DIMS>& mat)
{
        return ((*this) =  (*this)*mat);
}


template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS>& AABB<TYPE,DIMS>::operator*=(const Eigen::Matrix<TYPE,DIMS+1,DIMS+1>& mat)
{
       return ((*this) =  (*this)*mat);
}



template<typename TYPE, std::size_t DIMS>
inline AABB<TYPE, DIMS>& AABB<TYPE,DIMS>::operator*=(const Eigen::Matrix<TYPE,DIMS,DIMS+1>& mat)
{
     return ((*this) =  (*this)*mat);
}


template <typename TYPE, std::size_t DIMS>
inline std::size_t AABB<TYPE,DIMS>::numCorners(){return 1<<(DIMS);}



template<typename TYPE>
inline AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,2,2>& mat, const AABB2<TYPE>& box){
    return mat * box;
}

template<typename TYPE>
inline AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,3,3>& mat, const AABB3<TYPE>& box){

    return mat * box;
}



template<typename TYPE>
inline AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,2,3>& mat, const AABB2<TYPE>& box)
{
return mat * box;

}

template<typename TYPE>
inline AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,3,4>& mat, const AABB3<TYPE>& box)
{

    return mat * box;
}


template<typename TYPE>
inline AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,3,3>& mat, const AABB2<TYPE>& box)
{
    return mat * box;

}


template<typename TYPE>
inline AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,4,4>& mat, const AABB3<TYPE>& box){
    return mat * box;
}




template<typename TYPE>
inline AABB2<TYPE> operator*(const Eigen::Matrix<TYPE,3,2>& mat, const AABB2<TYPE>& box){


    return mat * box;

}

template<typename TYPE>
inline AABB3<TYPE> operator*(const Eigen::Matrix<TYPE,4,3>& mat, const AABB3<TYPE>& box){
return mat * box;


}


