//SOLOMON-LOCASCIO ROUGH BIPED 2D LPD/FIL 3D REIMPLEMENTATION
//Ben Jackson 2017


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include <time.h>
#include <ode/ode.h>
#include <ode/threading_impl.h>
/*ODE_API */dThreadingImplementationID dThreadingAllocateSelfThreadedImplementation();
#include <drawstuff/drawstuff.h>
#include <math.h>
#include "texturepath.h"
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
/*
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
*/
#include <QVector>
#include <QFile>
#include <QTextStream>
#include "omp.h"

// for VC++, no precision loss complaints
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)
#endif
// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

//testing
#define ISTHREADED 0
#define HEADLESS 0
#define SPEEDMODE 0
#define ERP 0.8
#define CFM 0.005
#define SLIP 0.1
#define M 10000
#define MFMIN 250 //50
#define MFMAX 750 //200
#define SPEED 25 //500
#define PLAYBACK 0
//ga testing //pop 20 gen 50 for small tests
#define POPSIZE 40 //final pop size 150
#define GENS 100 //final gens 3000 FIL 500 LPD (needs to be at least 150 for MATLAB GRAPH)
#define ELITISM 1
#define ELITNUM 10 //number of elites in the population
#define FIXEDMUT 0

//body genotype
#define BODYGENES 4
#define BODYALLOWANCE 0.9 //starting with current total, 0.9 (0.3 bod, 0.29 shank. 0.31 thigh)

//2D/3D differences
#define ISFIL 0 //1 FIL, 0 LPD
#define THDINIT 1 //dont code init weights unless last resort
#define INITFORCE 0.25 //0.25
#define INITANG 0.8 //CHECK HIP POSITION IF THIS IS CHANGED

//walker
#define UPPERLENGTH 0.3155
#define UPPERRADIUS 0.06
#define LOWERLENGTH 0.2915
#define LOWERRADIUS 0.06
#define HIPLENGTH 0.166
#define HIPRADIUS 0.06
#define FOOTLENGTH 0.2
#define FOOTWIDTH 0.1
#define FOOTHEIGHT 0.014
#define BODLENGTH 0.3
#define RADIUS 0.05
#define GRAV -9.81 //-9.81
#define STARTZ  0.645 //0.646 is total height from ground of hip
#define TSTEP 0.005
#define ISFLYING 0 // HAVE TO ALLOW A FLIGHT PHASE
#define ISFALLING 1
#define ACTUATIONON 1
#define SECLIMIT 60

//maths
#define DEG2RAD 0.01745329251 //angle converter
#define PI 3.14159

//ga
#define NUMPLACES 2 //number of decimal places used for the diversity metric in comparing genotypes
#define TORQUECOSTLPD 3000;
#define TORQUECOSTFIL 1000;

//nn
#define HIPPGAINY 100
#define KNEEPGAIN 300
#define ANKLEPGAIN 5
#define HIPDGAINY 5
#define KNEEDGAIN 15
#define ANKLEDGAIN 0.25
#define TWDCONTCOUNT 6
#define TWDINPUTS 19


//populations of ODEs objects that absolutely have to be sodding static
//vectors allow for pointers to the array to be rearranged by fitness
static dsFunctions fn;
struct Controller
{
    int numinputs;
    int numextras;
    int actuatorused; 	//this controller is designed as stance knee

    QVector<double> input;
    QVector<double> extras;
    QVector<double>inweights;
    double desiredangle;
    double outputtorque;
    double prop;
    double deriv;
    double error;
    double eprev;
    bool leftmoreintravel = false;


    void init2D(int whichcont){
        numinputs = TWDINPUTS; //2D USING THE PD CONTROLLER
        numextras = 4;
        for(int i = 0; i < numinputs; i++){
            inweights.push_back(0);
        }
        for(int i = 0; i < numinputs; i++){
            input.push_back(0);
        }
        for(int i = 0; i < numextras; i++){
            extras.push_back(0);
        }
        actuatorused = whichcont;
        eprev = 0;
    }


    void tick2D(){

        //IF LPD, set ignored connection weights to 0
        if(ISFIL == 0){
            switch(actuatorused){
                case 0: //upper body
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(4,0);
                    inweights.replace(5,0);
                    inweights.replace(6,0);
                    inweights.replace(7,0);
                    inweights.replace(8,0);
                    inweights.replace(9,0);
                    inweights.replace(10,0);
                    inweights.replace(11,0);
                    inweights.replace(12,0);
                    inweights.replace(13,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                    break;

                case 4: //interleg
                    inweights.replace(0,0);
                    inweights.replace(1,0);
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(6,0);
                    inweights.replace(7,0);
                    inweights.replace(8,0);
                    inweights.replace(9,0);
                    inweights.replace(10,0);
                    inweights.replace(11,0);
                    inweights.replace(12,0);
                    inweights.replace(13,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                    break;

                case 6: //stance knee
                    inweights.replace(0,0);
                    inweights.replace(1,0);
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(4,0);
                    inweights.replace(5,0);
                    inweights.replace(8,0);
                    inweights.replace(9,0);
                    inweights.replace(10,0);
                    inweights.replace(11,0);
                    inweights.replace(12,0);
                    inweights.replace(13,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                    break;

                case 8: //swing knee
                    inweights.replace(0,0);
                    inweights.replace(1,0);
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(4,0);
                    inweights.replace(5,0);
                    inweights.replace(6,0);
                    inweights.replace(7,0);
                    inweights.replace(10,0);
                    inweights.replace(11,0);
                    inweights.replace(12,0);
                    inweights.replace(13,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                    break;

                case 10: //stance ankle
                    inweights.replace(0,0);
                    inweights.replace(1,0);
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(4,0);
                    inweights.replace(5,0);
                    inweights.replace(6,0);
                    inweights.replace(7,0);
                    inweights.replace(8,0);
                    inweights.replace(9,0);
                    inweights.replace(12,0);
                    inweights.replace(13,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                    break;

                case 12: //swing ankle
                    inweights.replace(0,0);
                    inweights.replace(1,0);
                    inweights.replace(2,0);
                    inweights.replace(3,0);
                    inweights.replace(4,0);
                    inweights.replace(5,0);
                    inweights.replace(6,0);
                    inweights.replace(7,0);
                    inweights.replace(8,0);
                    inweights.replace(9,0);
                    inweights.replace(10,0);
                    inweights.replace(11,0);
                    inweights.replace(14,0);
                    inweights.replace(15,0);
                    inweights.replace(16,0);
                   break;
            }
        }
        else{
            inweights.replace(15,0);
            inweights.replace(16,0);
            inweights.replace(18,0);
        }

        //normalise inputs to -1 +1
        double mininput = smallestval(input);
        double maxinput =  largestval(input);
        for(int i = 0; i < numinputs-2; i++){
            double transformed = 2*((input.at(i)- mininput)/(maxinput - mininput)) - 1;
            input.replace(i,transformed);
        }

        //compute desired angle from inputs and weights
        desiredangle = 0;
        for(int i = 0; i < numinputs; i++){
            //check weights (count the controllers for 1-6)
            //printf("weight sum %i : %f\n", i, (input[i]*inweights[i]));
            desiredangle += (input.at(i) * inweights.at(i));
        }
        //printf("--------------------------\n");

        //compute outputtorque from pd control on desired angle
        error = desiredangle - input.at(actuatorused);

        //proportional
        prop = error*input.at(15);
        //derivative of the error function* gain
        deriv = (error - eprev)*input.at(16);
        eprev = error;

        //output
        outputtorque = prop + deriv;


    }

    static double smallestval(QVector<double> array){
        double smallest = DBL_MAX;

        for (int i=0; i < array.size(); i++)
            if( array.at(i) < smallest ){
                 smallest = array.at(i) ;
            }

        return smallest;
    }

    static double largestval(QVector<double> array){
        double largest = -DBL_MAX;

        for (int i=0; i < array.size(); i++)
            if( array.at(i) > largest ){
                 largest = array.at(i) ;
            }

        return largest;
    }

};
struct Walker {
    int i;
    QVector<double> geno;
    QVector<double> bodgeno;
    dMass bodymass, hipmassl, hipmassr, upperleftmass, upperrightmass, lowerleftmass, lowerrightmass, leftfootmass, rightfootmass;
    dBodyID body[9];
    dJointID joint[9];
    dGeomID box[9];
    dReal torques[7];
    dReal angles[6];
    Controller filconts[7];
    dWorldID world;
    dSpaceID space;
    dGeomID ground;
    dJointGroupID contactgroup;
    dReal leftfootcollide;
    dReal rightfootcollide;
    dReal init3dang;
    dReal prevleftthigh;
    dReal prevrightthigh;
    dReal previnterleg;
    dReal prevbody;
    float fitness = 0;
    float torquecost = 0;
    bool isstanceleft;
    bool hasfallen = false;
    bool isfinished = false;
    int contcount;
    int name;
    bool iswinner = false;
    int age;
    dJointFeedback* tor1;
    dJointFeedback* tor2;
    dJointFeedback* tor3;
    dJointFeedback* tor4;
    dJointFeedback* tor5;
    dJointFeedback* tor6;
    double startz;


    Walker(int namae, QVector<double> genot, QVector<double> bodyt)
    {
          name = namae;
          geno = genot;
          bodgeno = bodyt;
          startz = bodgeno.at(1) + bodgeno.at(2) + FOOTHEIGHT - 0.025;
    }

    Walker(){}

    ~Walker()
    {
        dGeomDestroy (box[0]);
        dGeomDestroy (box[1]);
        dGeomDestroy (box[2]);
        dGeomDestroy (box[3]);
        dGeomDestroy (box[4]);
        dGeomDestroy (box[5]);
        dGeomDestroy (box[6]);
        dGeomDestroy (box[7]);
        dGeomDestroy (box[8]);
    }

    void printpos(){
        const dReal* printbod = dGeomGetPosition(box[0]);
        printf("BODPOS %f %f %f \n", printbod[0], printbod[1], printbod[2]);
    }

    void init(){

        age = 0;
        leftfootcollide = 0;
        rightfootcollide = 0;
        fitness = 0;
        world = dWorldCreate();
        space = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground = dCreatePlane(space,0,0,1,0);
        dWorldSetGravity (world,0,0,GRAV);

        dWorldSetERP(world, ERP);
        dWorldSetCFM(world, CFM);

        isstanceleft = false;
        init3dang = INITANG;

        if(ISFIL == 0){
            torquecost = 3000;
        }
        else{
            torquecost = 1000;
        }

        //initialise masses
        dMassSetParameters(&bodymass, 5.81, 0.00672,0.00244,0.226, 190, 194, 30.8, -0.237, 1.42, 2.20);
        dMassSetParameters(&hipmassl, 0.219, -0.00131, 0.515, -0.00661, 0.414, 0.128, 0.395, 0.0149, 0.0311, -0.00188);
        dMassSetParameters(&hipmassr, 0.219, 0.00131, -0.515, -0.00661, 0.414, 0.128, 0.395, 0.0149, -0.0311, 0.00188);
        dMassSetParameters(&upperleftmass, 1.94, -0.00167, 0.00371, -0.111, 10.4, 9.33, 1.65, 0.00286, 0.325, 0.0226);
        dMassSetParameters(&upperrightmass, 1.94, 0.00154, -0.00421, -0.111, 10.4, 9.33, 1.61, -0.00195, -0.127, 0.0231);
        dMassSetParameters(&lowerleftmass, 0.267, 0.00153, 0.00496, -0.105, 2.52, 2.45, 0.110, -0.000411, 0.0366, -0.0238);
        dMassSetParameters(&lowerrightmass, 0.267, 0.00153, 0.00496, -0.105, 2.52, 2.45, 0.110, -0.000411, 0.0366, -0.0238);
        dMassSetParameters(&leftfootmass, 0.164, 0.0331, 0.0000351, -0.00676,  0.0852, 0.481, 0.545, 0.000671, 0.0000249, -0.003328);
        dMassSetParameters(&rightfootmass, 0.164, 0.0331, 0.0000351, -0.00676, 0.0852, 0.481, 0.545, 0.000671, 0.0000249, -0.003328);
        {dMass* mass = &bodymass;       dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &hipmassl;       dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &hipmassr;       dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &upperleftmass;  dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &upperrightmass; dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &lowerleftmass;  dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &lowerrightmass; dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &leftfootmass;   dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}
        {dMass* mass = &rightfootmass;  dMassTranslate( mass, -mass->c[0], -mass->c[1], -mass->c[2] );}

        //controllers
        contcount = TWDCONTCOUNT;
        for(int i = 0; i < contcount; i++){
            torques[i] = 0;
            switch(i){
                  case 0: //upperbody
                        filconts[i].init2D(0);
                      break;
                  case 1: //interleg
                        filconts[i].init2D(4);
                      break;
                  case 2: //stance knee
                          filconts[i].init2D(6);
                      break;
                  case 3: //swing knee
                          filconts[i].init2D(8);
                          break;
                  case 4: //stance ankle
                            filconts[i].init2D(10);
                      break;
                  case 5: //swing ankle
                            filconts[i].init2D(12);
                      break;
            }
        }

        //translating genotype
        for(int i = 0; i < TWDCONTCOUNT; i++){
           for(int j = 0; j < TWDINPUTS; j++){
               int genum = j + TWDINPUTS*i;
               if(j == 18){
                   if(geno.at(genum) <= 0){
                       //printf("cont %i before it was %f\n", i, geno.at(genum));
                       geno.replace(genum,0.01);
                   }
               }
               filconts[i].inweights.replace(j,geno.at(genum));
           }
        }

        //hip block LEFT
        body[0] = dBodyCreate (world);
        dBodySetMass (body[0],&hipmassl);
        box[0] = dCreateBox (space,HIPRADIUS,HIPLENGTH/2,HIPRADIUS);
        dGeomSetBody (box[0],body[0]);
        dBodySetPosition (body[0],0,0.0415,startz);

        //hip block RIGHT
        body[8] = dBodyCreate (world);
        dBodySetMass (body[8],&hipmassr);
        box[8] = dCreateBox (space,HIPRADIUS,HIPLENGTH/2,HIPRADIUS);
        dGeomSetBody (box[8],body[8]);
        dBodySetPosition (body[8],0,-0.0415,startz);

        //body block
        body[1] = dBodyCreate (world);
        dBodySetMass (body[1],&bodymass);
        box[1] = dCreateBox (space,UPPERRADIUS,UPPERRADIUS,bodgeno.at(0));
        dGeomSetBody (box[1],body[1]);
        dBodySetPosition (body[1],0,0,startz + bodgeno.at(0)/2);

        //HIP LEFT/HIP RIGHT JOINT
        joint[0] = dJointCreateHinge(world,0);
        dJointAttach (joint[0],body[0],body[8]);
        dJointSetHingeAnchor (joint[0],0,0,startz);
        dJointSetHingeAxis (joint[0],1,0,0);
        dJointSetHingeParam (joint[0],dParamLoStop,0);
        dJointSetHingeParam (joint[0],dParamHiStop,0);

        //HIP LEFT/BODY JOINT
        joint[7] = dJointCreateHinge(world,0);
        dJointAttach (joint[7],body[1],body[8]);
        dJointSetHingeAnchor (joint[7],0,0,startz);
        dJointSetHingeAxis (joint[7],1,0,0);
        dJointSetHingeParam (joint[7],dParamLoStop,0);
        dJointSetHingeParam (joint[7],dParamHiStop,0);

        //HIP RIGHT/BODY JOINT
        joint[8] = dJointCreateHinge(world,0);
        dJointAttach (joint[8],body[1],body[0]);
        dJointSetHingeAnchor (joint[8],0,0,startz);
        dJointSetHingeAxis (joint[8],1,0,0);
        dJointSetHingeParam (joint[8],dParamLoStop,0);
        dJointSetHingeParam (joint[8],dParamHiStop,0);

        //right thigh
        body[2] = dBodyCreate (world);
        dBodySetMass (body[2],&upperrightmass);
        box[2] = dCreateBox (space,UPPERRADIUS,UPPERRADIUS,bodgeno.at(1));
        dGeomSetBody (box[2],body[2]);
        dBodySetPosition (body[2],0,-(HIPLENGTH/2),startz-(bodgeno.at(1)/2)-0.025);


        //left thigh
        body[3] = dBodyCreate (world);

        dBodySetMass (body[3],&upperleftmass);
        box[3] = dCreateBox (space,UPPERRADIUS,UPPERRADIUS,bodgeno.at(1));
        dGeomSetBody (box[3],body[3]);        if(THDINIT){
            //leg raise
            dBodySetPosition (body[3], ((bodgeno.at(1)/2)*sin(init3dang)), (HIPLENGTH/2),startz-0.025-((bodgeno.at(1)/2)*cos(init3dang))+0.01);
            //set thigh rotation
            dMatrix3 rot_ptr;
            dRFromAxisAndAngle (rot_ptr, 0, 1,0, -init3dang);
            dBodySetRotation(body[3],rot_ptr);
            dGeomSetRotation(box[3], rot_ptr);
        }else{
            dBodySetPosition (body[3],0,(HIPLENGTH/2),startz-(bodgeno.at(1)/2)-0.025);
        }


        const dReal* d1 = dGeomGetPosition(box[2]); //right thigh block position
        const dReal* d2 = dGeomGetPosition(box[3]); //left thigh block position

        //HIP/THIGH JOINTS
        joint[1] = dJointCreateHinge(world,0);
        dJointAttach (joint[1],body[2],body[8]);
        dJointSetHingeAnchor (joint[1],d1[0],d1[1],d1[2] + (bodgeno.at(1)/2));
        dJointSetHingeAxis (joint[1],0,1,0);
        dJointSetHingeParam (joint[1],dParamHiStop,PI/2);
        dJointSetHingeParam (joint[1],dParamLoStop,-PI/2);

        joint[2] = dJointCreateHinge(world,0);
        dJointAttach (joint[2],body[3],body[0]);
        if(THDINIT){
            //leg raise //check hippos if angle changes
            dJointSetHingeAnchor (joint[2],d2[0] - (bodgeno.at(1)/2)*sin(init3dang),d2[1], d2[2] + (bodgeno.at(1)/2)*cos(init3dang));
        }else{
            dJointSetHingeAnchor (joint[2],d2[0],d2[1],d2[2] + (bodgeno.at(1)/2));
        }
        dJointSetHingeAxis (joint[2],0,1,0);
        dJointSetHingeParam (joint[2],dParamHiStop,PI/2);
        dJointSetHingeParam (joint[2],dParamLoStop,-PI/2);

        //right shank
        body[4] = dBodyCreate (world);
        dBodySetMass (body[4],&lowerrightmass);
        box[4] = dCreateBox (space,LOWERRADIUS,LOWERRADIUS,bodgeno.at(2));
        dGeomSetBody (box[4],body[4]);
        dBodySetPosition (body[4],d1[0]+0.0011,d1[1]+0.0140,d1[2]-(bodgeno.at(1)/2)-(bodgeno.at(2)/2));

        //left shank
        body[5] = dBodyCreate (world);
        dReal hippos_ptr[3];
        dJointGetHingeAnchor(joint[2], hippos_ptr);
        dBodySetMass (body[5],&lowerleftmass);
        box[5] = dCreateBox (space,LOWERRADIUS,LOWERRADIUS,bodgeno.at(2));
        dGeomSetBody (box[5],body[5]);
        if(THDINIT){
            //leg raise
            dBodySetPosition (body[5], hippos_ptr[0] + (bodgeno.at(1)+(bodgeno.at(2)/2))*sin(init3dang) + 0.0011*cos(init3dang),hippos_ptr[1]-0.0140,hippos_ptr[2]-(bodgeno.at(1)+(bodgeno.at(2)/2))*cos(init3dang) + 0.0011*sin(init3dang));
            dBodyAddForce(body[5],0,0,INITFORCE);
            //set shank rotation
            dMatrix3 rot_ptr2;
            dRFromAxisAndAngle (rot_ptr2, 0, 1,0, -init3dang);
            dBodySetRotation(body[5],rot_ptr2);
            dGeomSetRotation(box[5], rot_ptr2);
        }else{
            dBodySetPosition (body[5],d2[0]+0.0011,d2[1]-0.0140,d2[2]-(bodgeno.at(1)/2)-(bodgeno.at(2)/2));
        }

        const dReal* e1 = dGeomGetPosition(box[4]); //right shank block position
        const dReal* e2 = dGeomGetPosition(box[5]); //left shank block position

        //KNEE Y JOINT
        joint[3] = dJointCreateHinge(world,0);
        dJointAttach (joint[3],body[2],body[4]);
        dJointSetHingeAnchor (joint[3],e1[0],e1[1],e1[2]+(bodgeno.at(2)/2));
        dJointSetHingeAxis (joint[3],0,1,0);
        dJointSetHingeParam (joint[3],dParamHiStop,0);
        dJointSetHingeParam (joint[3],dParamLoStop,-PI/2);
        joint[4] = dJointCreateHinge(world,0);
        dJointAttach (joint[4],body[3],body[5]);
        if(THDINIT){
            //leg raise
            dJointSetHingeAnchor(joint[4],hippos_ptr[0] + (bodgeno.at(1))*sin(init3dang) ,hippos_ptr[1]-0.0140,hippos_ptr[2]-(bodgeno.at(1))*cos(init3dang));
        }else{
            dJointSetHingeAnchor (joint[4],e2[0],e2[1],e2[2]+(bodgeno.at(2)/2));
        }
        dJointSetHingeAxis (joint[4],0,1,0);
        dJointSetHingeParam (joint[4],dParamHiStop,0);
        dJointSetHingeParam (joint[4],dParamLoStop,-PI/2);

        //right foot
        body[6] = dBodyCreate (world);
        dBodySetMass (body[6],&rightfootmass);
        box[6] = dCreateBox (space,FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT);
        dGeomSetBody (box[6],body[6]);
        dBodySetPosition (body[6],e1[0]+0.05+0.003,e1[1],e1[2]-(bodgeno.at(2)/2)-(FOOTHEIGHT/2));

        //left foot
        body[7] = dBodyCreate (world);
        dReal kneepos_ptr[3];
        dJointGetHingeAnchor(joint[4], kneepos_ptr);
        dBodySetMass (body[7],&leftfootmass);
        box[7] = dCreateBox (space,FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT);
        dGeomSetBody (box[7],body[7]);
        if(THDINIT){
            //leg raise
            dBodySetPosition (body[7], kneepos_ptr[0]+0.053*cos(init3dang) +(bodgeno.at(2))*sin(init3dang),kneepos_ptr[1],kneepos_ptr[2]+ 0.053*sin(init3dang) -(bodgeno.at(2))*cos(init3dang));
            //set angle rotation
            dMatrix3 rot_ptr3;
            dRFromAxisAndAngle (rot_ptr3, 0, 1,0, -init3dang);
            dBodySetRotation(body[7],rot_ptr3);
            dGeomSetRotation(box[7], rot_ptr3);
        }else{
            dBodySetPosition (body[7],e2[0]+0.05+0.003,e2[1],e2[2]-(bodgeno.at(2)/2)-(FOOTHEIGHT/2));
        }

        const dReal* f1 = dGeomGetPosition(box[6]); //right ankle block position
        const dReal* f2 = dGeomGetPosition(box[7]); //left ankle block position

        //ANKLE JOINTS X AND Y ORANGE IS 5 PINK IS 6
        joint[5] = dJointCreateHinge(world,0);
        dJointAttach (joint[5],body[4],body[6]);
        dJointSetHingeAnchor (joint[5],f1[0]-0.053,f1[1],f1[2]+(FOOTHEIGHT/2));
        dJointSetHingeAxis (joint[5],0,1,0);
        dJointSetHingeParam (joint[5],dParamHiStop,25*DEG2RAD);
        dJointSetHingeParam (joint[5],dParamLoStop,-25*DEG2RAD);
        joint[6] = dJointCreateHinge(world,0);
        dJointAttach (joint[6],body[5],body[7]);
        if(THDINIT){
            //leg raise
            dJointSetHingeAnchor(joint[6],kneepos_ptr[0]+0.0011+(bodgeno.at(2))*sin(init3dang),kneepos_ptr[1],kneepos_ptr[2]-(bodgeno.at(2))*cos(init3dang));
        }else{
            dJointSetHingeAnchor (joint[6],f2[0]-0.053,f2[1],f2[2]+(FOOTHEIGHT/2));
        }
            dJointSetHingeAxis (joint[6],0,1,0);
            dJointSetHingeParam (joint[6],dParamHiStop,25*DEG2RAD);
            dJointSetHingeParam (joint[6],dParamLoStop,-25*DEG2RAD);

        //HINGE PARAMS
        for(int i = 0; i < 9; i++){
            dJointSetHingeParam(joint[i], dParamStopERP, ERP);
            dJointSetHingeParam(joint[i], dParamStopCFM, CFM);
            dJointSetHingeParam(joint[i], dParamCFM, CFM);
        }

        //HINGE MONITORS
        tor1 = new dJointFeedback;
        dJointSetFeedback(joint[1],tor1);
        tor2 = new dJointFeedback;
        dJointSetFeedback(joint[2],tor2);
        tor3 = new dJointFeedback;
        dJointSetFeedback(joint[3],tor3);
        tor4 = new dJointFeedback;
        dJointSetFeedback(joint[4],tor4);
        tor5 = new dJointFeedback;
        dJointSetFeedback(joint[5],tor5);
        tor6 = new dJointFeedback;
        dJointSetFeedback(joint[6],tor6);

        //PREV ANGLES
        prevbody = 0;
        previnterleg =  init3dang;
        prevleftthigh = init3dang;
        prevrightthigh = 0;

        //checking hip positions
//        dReal hiprpos[3];
//        dReal hiplpos[3];
//        dJointGetHingeAnchor(joint[1], hiprpos);
//        dJointGetHingeAnchor(joint[2], hiplpos);
//        printf("hiprpos %f %f %f\n", hiprpos[0], hiprpos[1], hiprpos[2]);
//        printf("hip1pos %f %f %f\n", hiplpos[0], hiplpos[1], hiplpos[2]);


    }

    void draw()
    {

          dReal sides[3] = {HIPRADIUS,HIPLENGTH/2,HIPRADIUS};
          dReal sides15[3] = {HIPRADIUS,HIPRADIUS,(dReal)bodgeno.at(0)};
          dReal sides2[3] = {UPPERRADIUS,UPPERRADIUS,(dReal)bodgeno.at(1)};
          dReal sides25[3] = {LOWERRADIUS,LOWERRADIUS,(dReal)bodgeno.at(2)};
          dReal sides3[3] = {FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT};

          dsSetColor (0.6,0,0);
          dsDrawBox (dGeomGetPosition(box[1]),dGeomGetRotation(box[1]),sides15);
          dReal jointpos0[3];
          dJointGetHingeAnchor(joint[0],jointpos0);
          dsDrawSphere(jointpos0,dGeomGetRotation(box[1]),RADIUS);

          dsSetColor (0,0,1);
          dsDrawBox (dGeomGetPosition(box[0]),dGeomGetRotation(box[0]),sides);

          dsSetColor (1,1,1);
          dsDrawBox (dGeomGetPosition(box[8]),dGeomGetRotation(box[8]),sides);
          dsSetColor (0.6,0,0);

          if(!isstanceleft){
              dsSetColor(1,1,0);
          }
          //right thigh
          dsDrawBox (dGeomGetPosition(box[2]),dGeomGetRotation(box[2]),sides2);
          dsSetColor (0.6,0,0);

          if(isstanceleft){
              dsSetColor(1,1,0);
          }
          //left thigh
          dsDrawBox (dGeomGetPosition(box[3]),dGeomGetRotation(box[3]),sides2);
          dsSetColor (0.6,0,0);

          dReal jointpos1[3];
          dJointGetHingeAnchor(joint[1],jointpos1);
          dsDrawSphere(jointpos1, dGeomGetRotation(box[1]),RADIUS);
          dReal jointpos2[3];
          dJointGetHingeAnchor(joint[2],jointpos2);
          dsDrawSphere(jointpos2, dGeomGetRotation(box[2]),RADIUS);

          dsSetColor (0,0.6,0);
          dReal jointpos3[3];
          dJointGetHingeAnchor(joint[3],jointpos3);
          dsDrawSphere(jointpos3, dGeomGetRotation(box[3]),RADIUS);
          dReal jointpos4[3];
          dJointGetHingeAnchor(joint[4],jointpos4);
          dsDrawSphere(jointpos4, dGeomGetRotation(box[4]),RADIUS);

          if(!isstanceleft){
              dsSetColor(1,1,0);
          }
          dsDrawBox (dGeomGetPosition(box[4]),dGeomGetRotation(box[4]),sides25);
          dsSetColor (0,0.6,0);

          if(isstanceleft){
              dsSetColor(1,1,0);
          }
          dsDrawBox (dGeomGetPosition(box[5]),dGeomGetRotation(box[5]),sides25);
          dsSetColor (0,0.6,0);

          dsSetColor (1,0.5,0);
          if(rightfootcollide){
              dsSetColor(0,0,0.8);
          }
          dReal jointpos5[3];
          dJointGetHingeAnchor(joint[5],jointpos5);
          dsDrawSphere(jointpos5, dGeomGetRotation(box[5]),RADIUS);
          dsDrawBox (dGeomGetPosition(box[6]),dGeomGetRotation(box[6]),sides3);

          dsSetColor (1,0.4,0.6);
          if(leftfootcollide){
              dsSetColor(0,0,0.8);
          }
          dReal jointpos6[3];
          dJointGetHingeAnchor(joint[6],jointpos6);
          dsDrawSphere(jointpos6, dGeomGetRotation(box[6]),RADIUS);
          dsDrawBox (dGeomGetPosition(box[7]),dGeomGetRotation(box[7]),sides3);

    }

    void AlignToYPlane()
    {
        for(int i = 0; i < 8; i++){
            const dReal *rot = dBodyGetAngularVel( body[i] );
            const dReal *quat_ptr = dBodyGetQuaternion( body[i] );
            const dReal* vel_ptr= dBodyGetLinearVel(body[i]);
            dReal vel[3];
            vel[0] = vel_ptr[0];
            vel[1] = 0;
            vel[2] = vel_ptr[2];
            dReal quat[4], quat_len;
            quat[0] = quat_ptr[0];
            quat[1] = 0;
            quat[2] = quat_ptr[2];
            quat[3] = 0;
            quat_len = sqrt( quat[0] * quat[0] + quat[2] * quat[2] );
            quat[0] /= quat_len;
            quat[2] /= quat_len;
            dBodySetQuaternion( body[i], quat );
            dBodySetAngularVel( body[i], 0, rot[1], 0 );
            dBodySetLinearVel(body[i], vel[0], vel[1], vel[2]);
        }
    }

    void finalset(){
        const dReal* hippos = dBodyGetPosition(body[0]);
        fitness = hippos[0];
        printf("WINNER FITNESS: %f\n", this->fitness);
        printf("TORQUE REMAINING: %f\n", torquecost);
//        printf("GENOTYPE\n");
//        for(int i = 0; i < geno.length(); i++){
//            printf("%f ", geno.at(i));
//        }
//        printf("\n");
    }

    void LocalNearCallback (dGeomID o1, dGeomID o2){
          int i,n;
          const int N = 1000;
          dContact contact[N];

          // FILTER OUT EXTRA COLLISIONS THAT ARENT BETWEEN BODY AND GROUND
          dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
          if(b1 && b2 && dAreConnected(b1,b2)){
                return;
          }
          if(o1 != ground && o2 != ground){
              return;
          }
          if(o1 == o2){
            return;
          }

          int w1=-999;
          if (o1==ground) w1=42;
          else for (int x=0;x<=8;x++) if (o1==box[x]) w1=x;
//          if(w1 != 0){
//            return;
//          }

          // LIMITING NUMBER OF COLLISIONS PER FFET AND FLOOR
          int contactpointslimit = 100;
          n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
          if(n > contactpointslimit){
              n = contactpointslimit;
          }

          if (n > 0) {
            for (i=0; i<n; i++) {
              contact[i].surface.mode = dContactSlip1 | dContactSlip2 |dContactSoftERP | dContactSoftCFM | dContactApprox1;
              contact[i].surface.mu = 10;
              contact[i].surface.slip1 = SLIP;
              contact[i].surface.slip2 = SLIP;
              contact[i].surface.soft_erp = ERP;
              contact[i].surface.soft_cfm = CFM;
              dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
              dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
              if(o2 == ground){
                       if(dGeomGetBody(contact[i].geom.g1) == body[7]){
                           leftfootcollide = 1;
                       }
                       else if(dGeomGetBody(contact[i].geom.g1) == body[6]){
                           rightfootcollide = 1;
                       }
                       else if(dGeomGetBody(contact[i].geom.g1) == body[5]){
                            //do nothing for shanks
                       }
                       else if(dGeomGetBody(contact[i].geom.g1) == body[4]){
                            //do nothing for shanks
                       }
                       else{
                           if(ISFALLING){
                               printf("fall %d \n", w1);
                               hasfallen = true;
                           }
                       }
              }
            }
          }
     }

    static void nearCallback(void* data, dGeomID o1, dGeomID o2) {
        Walker* self = static_cast<Walker*>(data);
        self->LocalNearCallback(o1,o2);
    }

    void run(){
        //MOVING

        //STANCE LEG
        //in doublestance?
        if((leftfootcollide == 1) & (rightfootcollide == 1)){
            //caudal foot from motion vector (in 2d this is just (1,0,0))
            //choose furthest back and break ties (no initial weights to step forward yet)
            const dReal* leftfootpos = dGeomGetPosition(box[7]);
            const dReal* rightfootpos = dGeomGetPosition(box[6]);
            if( leftfootpos[0] <= rightfootpos[0]){
                     isstanceleft = true;
            }else{
                     isstanceleft = false;
            }
        }else{
            //choose activated foot
            if(leftfootcollide == 1){
                     isstanceleft = true;
            }
            else if(rightfootcollide == 1){
                     isstanceleft = false;
            }
            else{
                //stance remains the same, flying shouldnt be happening anyway so itll die
            }
        }

        dReal bodangle, t_st, il;

        //INPUT
        for(int i = 0; i < contcount; i++){

            //0 = upper body angle
            const dReal* bodyrot = dGeomGetRotation(box[1]);
            bodangle = 0;
            bodangle = 2* atan2(-bodyrot[12],sqrt(pow(bodyrot[13],2)+pow(bodyrot[14],2)));
            if(bodangle < 0){
                bodangle = (-PI) - bodangle;
            }else{
                bodangle = (PI) - bodangle;
            }
            //fix atan angle sign bug
            //upper body position
            const dReal* atanbodypos = dGeomGetPosition(box[1]);
            //upper body joint anchor
            dReal atanbodjoint[3];
            dJointGetHingeAnchor(joint[0], atanbodjoint);
            //if position is in front, equal to or behind
            if(atanbodypos[0] > atanbodjoint[0]){
                //in front, so angle should be positive
                if(bodangle < 0){
                    bodangle = - bodangle;
                }
            }else if(atanbodypos[0] < atanbodjoint[0]){
                //behind, so angle should be negative
                if(bodangle > 0){
                    bodangle = - bodangle;
                }
            }else{
                //equal so the angle is zero; do nothing
            }

            filconts[i].input.replace(0,bodangle);


            //1 = upper body derivative
            double ubd = (bodangle - prevbody) / TSTEP;
            filconts[i].input.replace(1,ubd);

            //2 = stance thigh angle (h_st - b)
            const dReal* lethrot = dGeomGetRotation(box[3]);
            const dReal* rithrot = dGeomGetRotation(box[2]);
            dReal h_st, lethangle, rithangle;
            t_st = 0;
            lethangle = 2* atan2(-lethrot[12],sqrt(pow(lethrot[13],2)+pow(lethrot[14],2)));
            if(lethangle < 0){
                lethangle = (-PI) - lethangle;
            }else{
                lethangle = (PI) - lethangle;
            }
            //fix atan angle sign bug
            //left thigh position
            const dReal* atanlthighpos = dGeomGetPosition(box[3]);
            //joint anchor
            dReal atanlknjoint[3];
            dJointGetHingeAnchor(joint[4], atanlknjoint);
            //if position is in front, equal to or behind
            if(atanlthighpos[0] > atanlknjoint[0]){
                //in front, so angle should be positive
                if(lethangle < 0){
                    lethangle = - lethangle;
                }
            }else if(atanlthighpos[0] < atanlknjoint[0]){
                //behind, so angle should be negative
                if(lethangle > 0){
                    lethangle = - lethangle;
                }
            }else{
                //equal so the angle is zero; do nothing
            }

            rithangle = 2* atan2(-rithrot[12],sqrt(pow(rithrot[13],2)+pow(rithrot[14],2)));
            if(rithangle < 0){
                rithangle = (-PI) - rithangle;
            }else{
                rithangle = (PI) - rithangle;
            }
            //fix atan angle sign bug
            //right thigh position
            const dReal* atanrthighpos = dGeomGetPosition(box[2]);
            //joint anchor
            dReal atanrknjoint[3];
            dJointGetHingeAnchor(joint[3], atanrknjoint);
            //if position is in front, equal to or behind
            if(atanrthighpos[0] > atanrknjoint[0]){
                //in front, so angle should be positive
                if(rithangle < 0){
                    rithangle = - rithangle;
                }
            }else if(atanrthighpos[0] < atanrknjoint[0]){
                //behind, so angle should be negative
                if(rithangle > 0){
                    rithangle = - rithangle;
                }
            }else{
                //equal so the angle is zero; do nothing
            }

            //h_st
            if(isstanceleft){
                h_st = lethangle - bodangle;
            }else{
                h_st = rithangle - bodangle;
            }
            //t_st
            t_st = h_st + bodangle;
            filconts[i].input.replace(2,t_st);

            //3 = stance thigh angle derivative
            double stad;
            if(isstanceleft){
                    stad = (t_st - prevleftthigh) / TSTEP;
                    filconts[i].input.replace(3,stad);
            }else{
                    stad = (t_st - prevrightthigh) / TSTEP;
                    filconts[i].input.replace(3,stad);
            }

            //4 = interleg angle
            //swing hip angle h_sw (angle between negative body vec and swing thigh)
            dReal h_sw;
            il = 0;
            //h_sw
            if(isstanceleft){
                h_sw = rithangle - bodangle;
            }else{
                h_sw = lethangle - bodangle;
            }
            //interleg is hsw + hst
            il = h_sw - h_st;
            //accounting for the atan fix
            il = -il;
            il = fabs(il);
            filconts[i].input.replace(4,il);

            //5 = interleg derivative
            //add velocities if in oppositee direction, minus if same for il change
            double ildev = (il - previnterleg)/ TSTEP;
            filconts[i].input.replace(5,ildev);


            //6 = stance knee angle
            dReal leknangle = dJointGetHingeAngle(joint[4]);
            dReal riknangle = dJointGetHingeAngle(joint[3]);
            dReal k_st;
            //k_st
            if(isstanceleft){
                k_st = leknangle;
            }else{
                k_st = riknangle;
            }
            filconts[i].input.replace(6,k_st);

            //7 = stance knee angle derivative
            if(isstanceleft){
                    filconts[i].input.replace(7,dJointGetHingeAngleRate(joint[4]));
            }else{
                    filconts[i].input.replace(7,dJointGetHingeAngleRate(joint[3]));
            }

            //8 = swing knee angle
            dReal k_sw;
            if(isstanceleft){
                k_sw = riknangle;
            }else{
                k_sw = leknangle;
            }
            filconts[i].input.replace(8,k_sw);

            //9 = swing knee derivative
            if(isstanceleft){
                    filconts[i].input.replace(9,dJointGetHingeAngleRate(joint[3]));
            }else{
                    filconts[i].input.replace(9,dJointGetHingeAngleRate(joint[4]));
            }

            //10 = stance ankle angle
            dReal leanangle = dJointGetHingeAngle(joint[6]);
            dReal rianangle = dJointGetHingeAngle(joint[5]);
            dReal a_st;
            //a_st
            if(isstanceleft){
                a_st = leanangle;
            }else{
                a_st = rianangle;
            }
            a_st = (2*PI) - a_st;
            filconts[i].input.replace(10,a_st);

            //11 = stance ankle derivative
            if(isstanceleft){
                    filconts[i].input.replace(11,dJointGetHingeAngleRate(joint[6]));
            }else{
                    filconts[i].input.replace(11,dJointGetHingeAngleRate(joint[5]));
            }

            //12 = swing ankle angle
            dReal a_sw;
            if(isstanceleft){
                a_sw = rianangle;
            }else{
                a_sw = leanangle;
            }
            filconts[i].input.replace(12,a_sw);

            //13 = swing ankle derivative
            if(isstanceleft){
                    filconts[i].input.replace(13,dJointGetHingeAngleRate(joint[5]));
            }else{
                    filconts[i].input.replace(13,dJointGetHingeAngleRate(joint[6]));
            }

            //14 = double_stance
            //are both feet touching the ground?
            if((leftfootcollide == 1) & (rightfootcollide == 1)){
                  filconts[i].input.replace(14,1);
            }else{
                  filconts[i].input.replace(14,0);
            }

            //15 = P GAIN
            if((filconts[i].actuatorused == 0) | (filconts[i].actuatorused == 4)){
                filconts[i].input.replace(15,HIPPGAINY);
            }else if((filconts[i].actuatorused == 8) | (filconts[i].actuatorused == 6)){
                filconts[i].input.replace(15,KNEEPGAIN);
            }else if((filconts[i].actuatorused == 12) |(filconts[i].actuatorused == 10)){
                filconts[i].input.replace(15,ANKLEPGAIN);
            }
            //16 = D GAIN
            if((filconts[i].actuatorused == 0) | (filconts[i].actuatorused == 4)){
                filconts[i].input.replace(16,HIPDGAINY);
            }else if((filconts[i].actuatorused == 8) | (filconts[i].actuatorused == 6)){
                filconts[i].input.replace(16,KNEEDGAIN);
            }else if((filconts[i].actuatorused == 12) | (filconts[i].actuatorused == 10)){
                filconts[i].input.replace(16,ANKLEDGAIN);
            }
            //17 = bias
            filconts[i].input.replace(17,1); //the weight is the bias and adjusted accordingly

            //18 = learning/mutation rate
            filconts[i].input.replace(18,1); //the weight is used as the mutation rate and adjusted accordingly

            //EXTRAS UNUSED BY SYSTEM
            //E0 = foot contact A
            //is the left foot touching the ground
//            filconts[i].extras.replace(0,leftfootcollide);

//            //E1 = foot contact B
//            //is the right foot touching the ground
//            filconts[i].extras.replace(1,rightfootcollide);

//            //E2 = swing thigh angle
//            //t_sw
//            dReal t_sw;
//            t_sw = h_sw + bodangle;
//            filconts[i].extras.replace(2,t_sw);

//            //E3 = swing thigh derivative
//            double swad;
//            if(isstanceleft){
//                    swad =  (t_sw - prevrightthigh) / TSTEP;
//                    filconts[i].extras.replace(3,swad);
//                    prevrightthigh = t_sw;
//            }else{
//                    swad = (t_sw - prevleftthigh) / TSTEP;
//                    filconts[i].extras.replace(3,swad);
//                    prevleftthigh = t_sw;
//            }

            //3D INPUTS LATER
        }

        //setting previous angles
        prevbody = bodangle;
        if(isstanceleft){
                prevleftthigh = t_st;
        }else{
                prevrightthigh = t_st;
        }
        previnterleg = il;


        //CHECK INPUTS
//        if(age == 0){
//            printf("INPUTS\n");
//            for(int i = 0; i < TWDCONTCOUNT; i++){
//                for(int j = 0; j < TWDINPUTS; j++){
//                        printf("%f ", filconts[i].input.at(j));
//                }
//                printf("\n");
//            }
//        }



        //CONTROL (WITHOUT PD)
        for(int i = 0; i < contcount; i++){
            filconts[i].tick2D();
            angles[i] = filconts[i].desiredangle;
        }

        //check angles
//        if(age == 0){
//            for(int i = 0; i < contcount; i++){
//                printf("angle %i : %f\n", i, angles[i]);
//            }
//            printf("---------------\n");
//        }


        dReal m = M; //or 200 or 50
        dReal mfmin = MFMIN; //tweak this? started at 0.4
        dReal mfmax = MFMAX; //tweak this? started at 10
        dReal s = SPEED; //started at 6
        dReal mf[contcount];


        for(int i = 0; i < contcount; i++){
            mf[i] = m*fabs(angles[i]);
            if(mf[i] < mfmin){
                mf[i] = mfmin;
            }
            if(mf[i] > mfmax){
                mf[i] = mfmax;
            }
        }

        //check mfs
//        if(age == 0){
//            for(int i = 0; i < contcount; i++){
//                printf("mf %i : %f\n", i, mf[i]);
//            }
//            printf("---------------\n");
//        }


        //ACTUATION
        if(ACTUATIONON){
            if(isstanceleft){
                //0 body
                //1 interleg - equal and opposite on either hip
                dJointSetHingeParam( joint[1],  dParamFMax,  (mf[0] + mf[1])*0.5);
                dJointSetHingeParam( joint[1] , dParamVel,  s*(angles[0] - angles[1]*0.5));
                dJointSetHingeParam( joint[2],  dParamFMax,  (mf[0] + mf[1])*0.5);
                dJointSetHingeParam( joint[2] , dParamVel,  s*-(angles[0] - angles[1]*0.5));
                //2 stance knee
                dJointSetHingeParam( joint[4],  dParamFMax,  mf[2]);
                dJointSetHingeParam( joint[4] , dParamVel,  s*angles[2]);
                //3 swing knee
                dJointSetHingeParam( joint[3],  dParamFMax,  mf[3]);
                dJointSetHingeParam( joint[3] , dParamVel,  s*angles[3]);
                //4 stance ankle
                dJointSetHingeParam( joint[6],  dParamFMax,  mf[4]);
                dJointSetHingeParam( joint[6] , dParamVel,  s*angles[4]);
//                //5 swing ankle
                dJointSetHingeParam( joint[5],  dParamFMax,  mf[5]);
                dJointSetHingeParam( joint[5] , dParamVel,  s*angles[5]);
            }else{
                //0 body
                //1 interleg - equal and opposite on either hip
                dJointSetHingeParam( joint[2],  dParamFMax,  (mf[0] + mf[1])*0.5);
                dJointSetHingeParam( joint[2] , dParamVel,  s*(angles[0] - angles[1]*0.5));
                dJointSetHingeParam( joint[1],  dParamFMax,  (mf[0] + mf[1])*0.5);
                dJointSetHingeParam( joint[1] , dParamVel,  s*-(angles[0] - angles[1]*0.5));
                //2 stance knee
                dJointSetHingeParam( joint[3],  dParamFMax,  mf[2]);
                dJointSetHingeParam( joint[3] , dParamVel,  s*angles[2]);
                //3 swing knee
                dJointSetHingeParam( joint[4],  dParamFMax,  mf[3]);
                dJointSetHingeParam( joint[4] , dParamVel,  s*angles[3]);
                //4 stance ankle
                dJointSetHingeParam( joint[5],  dParamFMax,  mf[4]);
                dJointSetHingeParam( joint[5] , dParamVel,  s*angles[4]);
//                //5 swing ankle
                dJointSetHingeParam( joint[6],  dParamFMax,  mf[5]);
                dJointSetHingeParam( joint[6] , dParamVel,  s*angles[5]);

            }
        }
        //3D ACTUATION LATER

        //no foot collision by default
        leftfootcollide = 0;
        rightfootcollide = 0;
        //stepping


        dSpaceCollide (space,this,&nearCallback);
        dWorldStep (world,TSTEP);
        dJointGroupEmpty (contactgroup);

        //torque cost
        dReal totaltorque = 0;

        tor1 = dJointGetFeedback(joint[1]);
        tor2 = dJointGetFeedback(joint[2]);
        tor3 = dJointGetFeedback(joint[3]);
        tor4 = dJointGetFeedback(joint[4]);
        tor5 = dJointGetFeedback(joint[5]);
        tor6 = dJointGetFeedback(joint[6]);

        for(int i = 0; i < 3; i++){
            totaltorque += TSTEP*fabs(tor1->t1[i]);
            totaltorque += TSTEP*fabs(tor1->t2[i]);
            totaltorque += TSTEP*fabs(tor2->t1[i]);
            totaltorque += TSTEP*fabs(tor2->t2[i]);
            totaltorque += TSTEP*fabs(tor3->t1[i]);
            totaltorque += TSTEP*fabs(tor3->t2[i]);
            totaltorque += TSTEP*fabs(tor4->t1[i]);
            totaltorque += TSTEP*fabs(tor4->t2[i]);
            totaltorque += TSTEP*fabs(tor5->t1[i]);
            totaltorque += TSTEP*fabs(tor5->t2[i]);
            totaltorque += TSTEP*fabs(tor6->t1[i]);
            totaltorque += TSTEP*fabs(tor6->t2[i]);
        }

        if(totaltorque > torquecost){
                printf("WALKER %i EXCEEDED TORQUE\n", name);
                if(iswinner){
                    finalset();
                }
                if(!iswinner){
                    kill();
                }
                isfinished = true;

        }

        //CHECK TORQUES
//        printf("torque %f cost %f\n", totaltorque, torquecost);
        torquecost -= totaltorque;



        //no flying
        if(ISFLYING){
            if(!leftfootcollide && !rightfootcollide){
                hasfallen = true;
            }
        }

        //falling
        if(hasfallen & (!isfinished)){
                printf("WALKER %i FELL OR FLEW\n", name);
                if(iswinner){
                    finalset();
                }
                if(!iswinner){
                   kill();
                }
                isfinished = true;
        }

        //timing out
        age++;
        //if its been there longer than a minute
        if((age*TSTEP > SECLIMIT) & (!isfinished)){
            printf("WALKER %i TIMED OUT\n", name);
            if(iswinner){
                finalset();
            }
            if(!iswinner){
               kill();
            }
            isfinished = true;
        }
    }

    void kill(){
        dJointGroupDestroy (contactgroup);
        dSpaceDestroy (space);
        dWorldDestroy (world);
    }
};
static QVector<Walker*> walkers;
static QVector<double> fitnesses, bigfitnesses, mutations;
static Walker* winner;

static void start()
    {
      dAllocateODEDataForThread(dAllocateMaskAll);
      static float xyz[3] = {0,-1.5,0.8000f};
      static float hpr[3] = {90,0,0.0000f};
      dsSetViewpoint (xyz,hpr);
    }

static void command (int cmd)
    {
      switch (cmd) {
          case 'w': case 'W':
          //dJointAddHingeTorque(winner->joint[1], 1000);
            break;
          case 's': case 'S':
          break;
          case 'a': case 'A':
            break;
          case 'd': case 'D':
            break;
        }
    }

void step( Walker* wal){
    wal->AlignToYPlane();
    wal->run();
}

void simulateWalk(Walker* wal){
    //print system time
//    clock_t time;
//    time = clock();

    //INIT PHYSX
    if(!wal->iswinner){
        wal->init();
    }

    //WHILE NOT FINISHED, RUN
    while(!wal->isfinished){
        step(wal);
    }


    //SET FITNESS
    const dReal* hippos = dBodyGetPosition(wal->body[0]);
    wal->fitness = hippos[0];


    //print realtime ratio
//    time = clock() - time;
//    float timeinsecs = (float)time/CLOCKS_PER_SEC;
//    float realt = (wal->age*TSTEP) / timeinsecs;
//    printf("ratio of real time = %f\n", realt);
}

static void simLoop (int pause)
{
    if(!pause){
        //winner run
        if(!winner->isfinished){
          step(winner);
        }
    }
    for(int j = 0; j < POPSIZE; j++){
        winner->draw();
    }
}

void popSim(){

    if(ISTHREADED){
        //THREADING
        int threadnum = POPSIZE/10;

        omp_set_num_threads(threadnum);

        #pragma omp parallel
        {
            dAllocateODEDataForThread(dAllocateMaskAll);
            dThreadingImplementationID threading = dThreadingAllocateSelfThreadedImplementation();
            int ID = omp_get_thread_num();
            for(int i = 0; i < 10; i++){
                int walkerid = i+(ID*10);
                dWorldSetStepThreadingImplementation(walkers.at(walkerid)->world, dThreadingImplementationGetFunctions(threading), threading);
                simulateWalk(walkers.at(walkerid));
                dWorldSetStepThreadingImplementation(walkers.at(walkerid)->world, NULL, NULL);
            }
            dThreadingFreeImplementation(threading);
        }


    }else{
    //NO THREADING
      for(int j = 0; j < POPSIZE; j++){
          simulateWalk(walkers.at(j));
      }
    }

}

QVector<Walker*> fitsort(QVector<Walker*> mems){
    for (int count = 0; count < mems.size(); count++){
        for(int comp = 0; comp < mems.size()-1; comp++){
            if(mems.at(comp)->fitness < mems.at(comp+1)->fitness){
                Walker* temp;
                temp = mems.at(comp);
                mems.replace(comp,mems.at(comp + 1));
                mems.replace(comp + 1,temp);
            }
        }
    }
    return mems;
}

double smallestval(QVector<double> array){
    double smallest = DBL_MAX;

    for (int i=0; i < array.size(); i++)
        if( array.at(i) < smallest ){
             smallest = array.at(i) ;
        }

    return smallest;
}

double largestval(QVector<double> array){
    double largest = -DBL_MAX;

    for (int i=0; i < array.size(); i++)
        if( array.at(i) > largest ){
             largest = array.at(i) ;
        }

    return largest;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void geneticAlg(){
        //GENETIC ALGORITHM
        //initialise population of members
        QVector<Walker*> lastbest50;
        double propselect,toppercent,p1,p2;
        QVector<QVector<double>> elite;

        //SIMULATE GEN 0
        popSim();
        //sort by fitness
        walkers = fitsort(walkers);
        //add initial fittest to metric
        fitnesses.push_back(walkers.at(0)->fitness);
        //take raw fitnesses
        for(int n = 0; n < POPSIZE; n++){
            bigfitnesses.push_back(walkers.at(n)->fitness);
        }
        //track mutation rates
        for(int m = 0; m < TWDCONTCOUNT; m++){
            mutations.push_back(walkers.at(0)->filconts[m].inweights.at(18));
        }
        printf("0-------------------------\n");

        //GENERATION LOOP
        for(int k = 0; k < GENS; k++){

            //if in the last fifty generations, add the top 20% to championsleague
            if((k > GENS - 50) && (k != 0)){
                for(int i = 0; i < (POPSIZE*0.2); i++){
                    lastbest50.push_back(walkers.at(i));
                }
            }

            //save best for elitism
            if(ELITISM){
                for(int i = 0; i < ELITNUM; i++){
                    elite.push_back(walkers.at(i)->geno);
                }
            }

            //MUTATION AND CROSSOVER
            for(int i = ELITNUM; i < POPSIZE; i++){
                QVector<double> replacement;
                QVector<double> bodreplacement;
                replacement.clear();
                bodreplacement.clear();
                for(int j = 0; j < (TWDCONTCOUNT*TWDINPUTS); j++){
                    replacement.push_back(0);
                }
                for(int j = 0; j < BODYGENES; j++){
                    bodreplacement.push_back(0);
                }
                // 25% mutation, 25% crossover, 50% both
                propselect = rand() % 100;
                //24,25,49 | 4 5 69
                if(propselect > 24){
                    //discrete crossover the top 20%
                    toppercent = POPSIZE*0.2;
                    p1 = rand() % (int)toppercent;
                    p2 = rand() % (int)toppercent;
                    //parents must be different agents
                    if(p1 == p2){
                         if(p2 > 0){
                            p2  -=1;
                         }else{
                             p2 +=1;
                         }
                    }

                    //pick genes at random to flip
                    double randogene[TWDCONTCOUNT*TWDINPUTS];
                    for(int j = 0; j < (TWDCONTCOUNT*TWDINPUTS); j++){
                        randogene[j] = (double)(rand()%2);
                        if(randogene[j] == 0){
                            replacement.replace(j,walkers.at(p1)->geno.at(j));
                        }else{
                            replacement.replace(j,walkers.at(p2)->geno.at(j));
                        }
                    }

                    //body flip (due to proportional allowance, body is swapped and mutated as a single gene)
                    double randobod = (double)(rand()%2);
                    if(randobod == 0){
                        bodreplacement.replace(0,walkers.at(p1)->bodgeno.at(0));
                        bodreplacement.replace(1,walkers.at(p1)->bodgeno.at(1));
                        bodreplacement.replace(2,walkers.at(p1)->bodgeno.at(2));
                    }else{
                        bodreplacement.replace(0,walkers.at(p2)->bodgeno.at(0));
                        bodreplacement.replace(1,walkers.at(p2)->bodgeno.at(1));
                        bodreplacement.replace(2,walkers.at(p2)->bodgeno.at(2));
                    }

                }

                if(propselect < 25 || propselect  > 49){
                //randomly mutate the 80% offspring using normally distributed vector with 0.005 StandDev
                    toppercent = POPSIZE*0.2;
                    p1 = rand() % (int)toppercent;
                    std::random_device rd;
                    std::normal_distribution<double> ndLPD(0,0.05);
                    std::normal_distribution<double> ndFIL(0,0.005);
                    //mutation vector over all weights
                    double mutationvector[TWDINPUTS*TWDCONTCOUNT];
                    for(int j = 0; j < (TWDINPUTS*TWDCONTCOUNT); j++){
                              if(FIXEDMUT){
                                  mutationvector[j] = ndFIL(rd);
                              }
                              else{
                                  mutationvector[j] = ndLPD(rd) * walkers.at(i)->filconts[(int)floor(j/TWDINPUTS)].inweights.at(18); //learnable mutation rate for LPD
                              }
                              if(propselect < 25){
                                    replacement.replace(j,walkers.at(p1)->geno.at(j) + mutationvector[j]);
                              }
                              if(propselect > 49){
                                    replacement.replace(j,replacement.at(j) + mutationvector[j]);
                              }
                    }

                    //mutating the body but total mutation across 3 parts is zero
                    double mutbod1, mutbod2, mutbod3;

                    mutbod1 = fRand(-0.05, 0.05);
                    mutbod2 = fRand(0, -mutbod1);
                    mutbod3 = -(mutbod1 + mutbod2);

                    //making sure none of the values result in a negative length
                    if(propselect < 25){
                        while((walkers.at(p1)->bodgeno.at(0) + mutbod1 <= 0) || (walkers.at(p1)->bodgeno.at(1) + mutbod2 <= 0) || (walkers.at(p1)->bodgeno.at(2) + mutbod3 <= 0)){
                            mutbod1 = fRand(-0.05, 0.05);
                            mutbod2 = fRand(0, -mutbod1);
                            mutbod3 = -(mutbod1 + mutbod2);
                        }
                          bodreplacement.replace(0,walkers.at(p1)->bodgeno.at(0) + mutbod1);
                          bodreplacement.replace(1,walkers.at(p1)->bodgeno.at(1) + mutbod2);
                          bodreplacement.replace(2,walkers.at(p1)->bodgeno.at(2) + mutbod3);
                    }
                    if(propselect > 49){
                        while((bodreplacement.at(0) + mutbod1 <= 0) || (bodreplacement.at(1) + mutbod2 <= 0) || (bodreplacement.at(2) + mutbod3 <= 0)){
                            mutbod1 = fRand(-0.05, 0.05);
                            mutbod2 = fRand(0, -mutbod1);
                            mutbod3 = -(mutbod1 + mutbod2);
                        }
                          bodreplacement.replace(0,bodreplacement.at(0) + mutbod1);
                          bodreplacement.replace(1,bodreplacement.at(1) + mutbod2);
                          bodreplacement.replace(2,bodreplacement.at(2) + mutbod3);
                    }

                }
              // replace with replacement
              walkers.replace(i,new Walker(i, replacement, bodreplacement));
            }

            //put best back in population if elitism
            if(ELITISM){
                for(int i = 0; i < ELITNUM; i++){
                    walkers.replace(i,new Walker(0, walkers.at(i)->geno, walkers.at(i)->bodgeno));
                }
            }

            //SIMULATE
            popSim();
            //sort by fitness
            walkers = fitsort(walkers);
            //add highest fitness to metric
            fitnesses.push_back(walkers.at(0)->fitness);
            //take raw fitnesses
            for(int n = 0; n < POPSIZE; n++){
                bigfitnesses.push_back(walkers.at(n)->fitness);
            }
            //track mutation rates
            for(int m = 0; m < TWDCONTCOUNT; m++){
                mutations.push_back(walkers.at(0)->filconts[m].inweights.at(18));
            }

            printf("%i--------------------------\n", k+1);
        }

        //add top 20% to last bit of champions league
        for(int i = 0; i < (POPSIZE*0.2); i++){
            lastbest50.push_back(walkers.at(i));
        }


        //COMPARE ELITES OF FINAL 50 GENS FOR THE WINNER
        QVector<double> finalfits;
        finalfits.clear();
        for(int i = 0; i < lastbest50.size(); i++){
            finalfits.push_back(lastbest50.at(i)->fitness);
            printf("CHAMPIONS LEAGUE %i, FITNESS %f\n", i, lastbest50.at(i)->fitness);
        }

        //WORK OUT WINNER
        double largestfitness = largestval(finalfits);
        for(int i = 0; i < finalfits.size(); i++){
            if(finalfits.at(i) == largestfitness){
                winner = lastbest50.at(i);
            }
        }
        printf("WINNER IS %i WITH %f\n", winner->name, winner->fitness);
//        printf("WINNER GENOTYPE\n");
//        for(int i = 0; i < winner->geno.length(); i++){
//            printf("%f ", winner->geno.at(i));
//        }
//        printf("\n");
//        printf("--------------------------\n");

}

void exportCSV(){
    QString filename = "fitness.csv";
    QFile thefile(filename);
    if(!thefile.open(QFile::WriteOnly|QIODevice::Text))
    return;
    QTextStream out(&thefile);
    out << "generation FIL =" << ISFIL << ",fittest\n";
    for(int i = 0; i < fitnesses.length(); i++){
        out << i << "," << fitnesses.at(i) << "\n";
    }

    QString filename2 = "mutation.csv";
    QFile thefile2(filename2);
    if(!thefile2.open(QFile::WriteOnly|QIODevice::Text))
    return;
    QTextStream out2(&thefile2);
    out2 << "generation FIL = " << ISFIL << ",mutation r1, mutation r2, mutation r3, mutation r4, mutation r5, mutation r6 \n";
    for(int i = 0; i < (mutations.length()/TWDCONTCOUNT); i++){
        out2 << i;
        for(int j = 0; j < TWDCONTCOUNT; j++){
            out2 << "," << mutations.at((i*TWDCONTCOUNT)+j);
        }
        out2 << "\n";
    }

    QString filename3 = "bigfitness.csv";
    QFile thefile3(filename3);
    if(!thefile3.open(QFile::WriteOnly|QIODevice::Text))
    return;
    QTextStream out3(&thefile3);
    for(int i = 0; i < (bigfitnesses.length()/POPSIZE); i++){
        out3 << i;
        for(int j = 0; j < POPSIZE; j++){
            out3 << "," << bigfitnesses.at((i*POPSIZE)+j);
        }
        out3 << "\n";
    }
    out3 << ISFIL;

}

void writeWinner(){
    std::ofstream file;
    file.open("winner_geno.csv");
    for(int i = 0; i < winner->geno.length(); i++){
        char buf[50];
        snprintf(buf, 50, "%a", winner->geno.at(i));
        file << buf << "\n";
    }
    file.close();

    std::ofstream bodfile;
    bodfile.open("winner_bodgeno.csv");
    for(int i = 0; i < winner->bodgeno.length(); i++){
        char bodbuf[50];
        snprintf(bodbuf, 50, "%a", winner->bodgeno.at(i));
        bodfile << bodbuf << "\n";
    }
    bodfile.close();
}

QVector<double> loadlastWinner(){
    QVector<double> loaded;
    std::string line2;
    std::ifstream myfile("winner_geno.csv");
    if (myfile.is_open())
    {
      while ( std::getline (myfile,line2) )
      {
        loaded.push_back(stod(line2));
      }
      myfile.close();
    }
    else std::cout << "Unable to open file";

    return loaded;
}

QVector<double> loadlastWinnerBod(){
    QVector<double> loaded;
    std::string line2;
    std::ifstream myfile("winner_bodgeno.csv");
    if (myfile.is_open())
    {
      while ( std::getline (myfile,line2) )
      {
        loaded.push_back(stod(line2));
      }
      myfile.close();
    }
    else std::cout << "Unable to open file";

    return loaded;
}

int main (int argc, char *argv[])
{
    //seed rng with system time
    srand(time(NULL));

    //START ODE
    dInitODE2(0);

    //biasing mutation rates together
    double mutcentre;
    mutcentre = (double)((rand() % 11)-5)/100;
    mutcentre = fabs(mutcentre);
    std::random_device mutrd;
    std::normal_distribution<double> mutnd(mutcentre,0.01);

    //init walkers
    for(int i = 0; i < POPSIZE; i++){
        QVector<double> initgeno;
        for(int j = 0; j < TWDCONTCOUNT*TWDINPUTS; j++){
            double weightnum = j % TWDINPUTS;
            if(weightnum == 18){
                initgeno.push_back(mutnd(mutrd));
            }else{
                initgeno.push_back((double)((rand() % 11)-5)/100);
            }
        }
        //BODY GENO
        QVector<double> initbod;
        double bodgen1 = fRand(0, BODYALLOWANCE);
        double bodgen2 = fRand(0, BODYALLOWANCE - bodgen1);
        double bodgen3 = BODYALLOWANCE - (bodgen1 + bodgen2);
        initbod.push_back(bodgen1);
        initbod.push_back(bodgen2);
        initbod.push_back(bodgen3);
        initbod.push_back(mutnd(mutrd));

        //array of walkers
        walkers.push_back(new Walker(i, initgeno, initbod));
    }

    //biasing mutation rates together
    double winmutcentre;
    winmutcentre = (double)((rand() % 11)-5)/100;
    winmutcentre = fabs(winmutcentre);
    std::random_device winmutrd;
    std::normal_distribution<double> winmutnd(winmutcentre,0.01);

    //initialise winner walker object
    QVector<double> wingeno;
    for(int j = 0; j < TWDCONTCOUNT*TWDINPUTS; j++){
        double winweightnum = j % TWDINPUTS;
        if(winweightnum == 18){
            wingeno.push_back(winmutnd(winmutrd));
        }else{
            wingeno.push_back((double)((rand() % 11)-5)/100);

        }
    }
    //BODY GENO
    QVector<double> winbod;
    double winbodgen1 = fRand(0, BODYALLOWANCE);
    double winbodgen2 = fRand(0, BODYALLOWANCE - winbodgen1);
    double winbodgen3 = BODYALLOWANCE - (winbodgen1 + winbodgen2);
    winbod.push_back(winbodgen1);
    winbod.push_back(winbodgen2);
    winbod.push_back(winbodgen3);
    winbod.push_back(mutnd(mutrd));

    if(PLAYBACK){
    }else{
        //WALKER GA
        geneticAlg();
        writeWinner();
        //EXPORT metric arrays to CSV
        exportCSV();
    }

    //load winner
    wingeno = loadlastWinner();
    winbod = loadlastWinnerBod();

    //resim and draw winning walker
    winner = new Walker(0, wingeno, winbod);
    winner->iswinner = true;
    winner->init();
    if(HEADLESS){
       //simulate walker old fashioned way
        simulateWalk(winner);
    }else{
        // setup pointers to drawstuff callback functions
        fn.version = DS_VERSION;
        fn.start = &start;
        fn.step = &simLoop;
        fn.command = &command;
        fn.stop = 0;
        fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
        dsSimulationLoop (argc,argv,500,500,&fn);
    }


    //CLOSE ODE
    dCloseODE();


    return 0;
}