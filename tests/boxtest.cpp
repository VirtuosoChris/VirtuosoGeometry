#include <iostream>
#include "AABB.h"


using namespace std;
using namespace Virtuoso::Geometry;

int main()
{
    cout << "Virtuoso Axis Aligned Bounding Box Test Code" << endl;

    AABB2f box;

    std::cerr<<box<<std::endl;

    box = AABB2f(Eigen::Vector2f(0.0f,0.0f), Eigen::Vector2f(1.0f,1.0f));

    AABB2f eqbox(Eigen::Vector2f(0.0f,0.0f), Eigen::Vector2f(1.0f,1.0f));

    std::cerr<<box<<std::endl;

    std::cerr<<"testing box "<<eqbox<<" for equality\nEQUAL?: "<<(eqbox==box)<<
    "\nNOT EQUAL? "<<(box!=eqbox)<< std::endl;


    std::cerr<<"\nHalf vector of aabb is "<<box.getHalfVector()<<std::endl;

    std::cerr<<"\nCorners for 2d box is "<<box.numCorners()<<'\n'<<std::endl;


    AABB2f box2(Eigen::Vector2f(-1.0f,-1.0f),Eigen::Vector2f(1.0f,1.0f));

    std::cerr<<"Testing scalar multiply on box "<<box2<<std::endl;

    std::cerr<<"times 2 \n"<<box2*2.0f<<"\n\nTimes 3\n"<<3.0f*box2<<std::endl;

    std::cerr<<"Testing scalar assignment multiply by 1.5"<<std::endl;

    box2*= 1.5;

    std::cerr<<box2<<std::endl;



    std::cerr<<"\n\nTesting matrix multiplies "<<std::endl;

    Eigen::Matrix2f idm;
    idm<<1.5,0,0,1;

    std::cerr<<"multiplying by matrix"<<idm<<" \n"<<( box * idm)<<std::endl;


    AABB2f boxtmp = idm * box;


//    AABB2f lhsbox =(idm*box);
 //   std::cerr<<"Multiplying by matrix left "<<lhsbox<<std::endl;

    Eigen::Matrix2f idm2;
    idm2<<1.0,0,0,1.5;

    std::cerr<<"multiplying by matrix "<<idm2<<"\n"<<(box *= idm2)<<" \n";

    Eigen::Matrix3f idm3;
    idm3<<2.0,0.0,0.0,
          0.0,2.0,0.0,
          0.0,0.0,.5;

    std::cerr<<"testing homogenous matrix : "<<idm3<<"\non box\n"<<box<<std::endl;

    std::cerr<<"Result is "<<(box *= idm3);






    const unsigned int NUM_PTS = 3;
    Eigen::Vector2f pts[NUM_PTS] = { Eigen::Vector2f(-1,5),Eigen::Vector2f(4,0),Eigen::Vector2f(1,-3) };

    std::cerr<<"Testing model bounds : Our model has the points \n";

    for(unsigned int i = 0; i < NUM_PTS; i++)
    {

        std::cerr<<pts[i]<<'\n'<<std::endl;
    }



    AABB2f modelBounds = pointBounds<float, 2>(&pts[0], pts + NUM_PTS);

    std::cerr<<"Testing model bounds: Resulting AABB is "<<modelBounds<<std::endl;




    return 0;
}
