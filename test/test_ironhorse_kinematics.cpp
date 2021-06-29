

#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include "csvtools.h"

using namespace std;


#ifndef RBDL_BUILD_ADDON_URDFREADER
    #error "Error: RBDL addon urdfModel not enabled."
#endif

#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <rbdl/rbdl_utils.h>
#include "rbdl/Kinematics.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



/* Problem Constants */
int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);


    //problem specific constants
    int     nPts    = 100;
    double  t0      = 0;
    double  t1      = 3;


    //Integration settings
    double absTolVal   = 1e-10;
    double relTolVal   = 1e-6;

    VectorNd q, qd;

    Model* model  = NULL;
    model         = new Model();

    //3a. The  model is read in here, and turned into a series of 
    //    vectors and matricies in model which RBDL uses to evaluate 
    //    dynamics quantities
    if (!Addons::URDFReadFromFile ("../models/iron_horse/iron_horse_simple.urdf", 
                                       model, false)             ){        
        std::cerr     << "Error loading model ./model/pendulum.lua" 
                    << std::endl;
        abort();
    }

    q       = VectorNd::Zero (model->dof_count);
    qd      = VectorNd::Zero (model->dof_count);

    double t        = 0;             //time
    double ts       = 0;            //scaled time
    double dtsdt    = M_PI/(t1-t0);    //dertivative scaled time 
                                    //w.r.t. time

    printf("DoF: %i\n",model->dof_count);


    printf("Forward Dynamics \n");

    cout << "qsize is" << model->q_size << endl;

    for(int i=0;i<13;i++){
        cout << model->GetBodyName(i) << endl;
    }

    cout << model->GetBodyId("abduct_fl") << endl;
    cout << model->GetBodyId("LF_FOOT") << endl;
    
    std::cout << "Degree of freedom overview:" << std::endl;
    std::cout << Utils::GetModelDOFOverview(*model);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << Utils::GetModelHierarchy(*model);

    cout << Utils::GetNamedBodyOriginsOverview(*model);

    cout << model->mBodies.size() << endl;

    Vector3d leg_point(0,0,-0.372);
    q(0) = 0;//90*3.1415926/180;
    cout << CalcBodyToBaseCoordinates(*model, q, 3, leg_point) << endl;
    
    q(3) = 0;//90*3.1415926/180;
    cout << CalcBodyToBaseCoordinates(*model, q, 6, leg_point) << endl;

    q(6) = 0;//90*3.1415926/180;
    cout << CalcBodyToBaseCoordinates(*model, q, 9, leg_point) << endl;

    q(9) = 0;//90*3.1415926/180;
    cout << CalcBodyToBaseCoordinates(*model, q, 9, leg_point) << endl;

    MatrixNd G = MatrixNd(MatrixNd::Zero(3, 3));
    CalcPointJacobian(*model, q, 3, leg_point, G);
    cout << G << endl;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            cout << G(i,j) << endl;
        }
    }

    Vector3d qd_t(0.1,0,0);
    Vector3d v;
    v = G*qd_t;
    cout << v << endl;


    delete model;

    return 0;
        
}

