//SOLOMON-LOCASCIO ROUGH BIPED 3D REIMPLEMENTATION
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
#include "drawstuff/drawstuff.h"
#include <math.h>
#include "texturepath.h"
#include "omp.h"
#include <vector>

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

// ga parameters
#define POPSIZE 5 //final pop size 150
#define GENS 10 //final gens 500
#define THREED 1 //3d or 2d
#define LOADER 1 //load in a csv or new run
#define DISPLAY 1 //visualise winner
#define ELITISM 1 //elitism toggle
#define ELITNUM POPSIZE * 0.15 //number of elites in the population
#define PARENTPERCENT 0.2 //percent considered as parents for mutation and crossover
#define CENTBIAS 1 //biasing initial mutation rates with a normal distribution toggle
#define SECLIMIT 120 //walker lifetime (s)

//torque total and reduction
#define TORQUEREMOVER 2.2 
#define TORQUETOTAL 3000; 

//fail conditions toggles (0 is off)
#define ISFALLING 1
#define ISFLYING 0 
#define FALLHEIGHT 0.1 //height off the ground for both feet for flight phase 

//allow actuation
#define ACTUATIONON 1

//initial angle of raised leg
#define INITANG 0.8 

//maths + engine parameters
#define DEG2RAD 0.01745329251 //angle converter
#define PI 3.14159
#define GRAV -9.81 //-9.81
#define TSTEP 0.005
#define ERP 0.8
#define CFM 0.005
#define SLIP 0.1
#define M 10000
#define MFMIN 250 
#define MFMAX 750
#define SPEED 25

//joint parameters
#define HIPERP 0.8
#define HIPCFM 0.005
#define KNEEERP 0.8
#define KNEECFM 0.005
#define ANKLEERP 0.8
#define ANKLECFM 0.005

//body parameters
#define UPPERLENGTH 0.3155
#define LOWERLENGTH 0.2915
#define HIPLENGTH 0.083
#define FOOTLENGTH 0.2
#define FOOTWIDTH 0.1
#define FOOTHEIGHT 0.014
#define BODLENGTH 0.3
#define RADIUS 0.04

//mass
#define BODMAS 5.81
#define HIPMAS 0.219
#define UPMAS 1.94 
#define LOMAS 0.26 
#define FOMAS 0.164 
//scaling parameters to apply appropriate density in dmasssetboxtotal using above mass values
#define BODSFX 149.8708566269
#define BODSFY 131.5209583241
#define BODSFZ 63.6079588218
#define HIPSFX 43.2022957279
#define HIPSFY 47.3952600578
#define HIPSFZ 50.1709406696
#define UPSFX 33.4833394484
#define UPSFY 72.510219685 
#define UPSFZ 23.6700779671
#define LOSFX 23.702273157 
#define LOSFY 50.2801142365 
#define LOSFZ 35.8291380885
#define FTSFX 29.3340724223
#define FTSFY 23.363511263 
#define FTSFZ 46.335563509


//initial height
#define STARTZ  (UPPERLENGTH + LOWERLENGTH + FOOTHEIGHT + RADIUS/2) //0.646 is total height from ground of hip

//nn parameters
#define CONTCOUNT 7 //controllers
#define INPUTS 19 //inputs
#define TOTAL CONTCOUNT * INPUTS
#define BODYTOTAL 9 //number of bodies

//turning parameters
#define TURNMODE 1 
#define WANTPOSX 8 //point x
#define WANTPOSY 4 //point y
#define MAXFIT ceil(2 * sqrt((WANTPOSX * WANTPOSX) + (WANTPOSY * WANTPOSY))) //distance measure

//EXTRAS
/////////////////////////////////////////////////

//scaled noise to sensors, actions and fitness, unused
#define NOISEON 0
#define ALLSCALE 1
#define SENSESIG 0
#define ACTSIG 0
#define FITSIG 0

/////////////////////////////////

using namespace std;
//drawstuff function object
static dsFunctions fn;
//random device for normal distributions
std::random_device nrd;
//generations value declaration
double gens;

//controller object
struct Controller
{

    int numinputs;
    int actuatorused; 	//integer to designate which controller it is

    std::vector<double> input;
    std::vector<double>inweights; //weights
    double desiredangle;

	//initialise controller  with empty values and assign number
    void initcon(int whichcont){
        numinputs = INPUTS; 
        for(int i = 0; i < numinputs; i++){
            inweights.push_back(0);
        }
        for(int i = 0; i < numinputs; i++){
            input.push_back(0);
        }
        actuatorused = whichcont;
    }


    void tick(){
        //add noise to sensors
		/*
        if(NOISEON && (SENSESIG > 0)){
            std::normal_distribution<double> ndsens(0,SENSESIG*ALLSCALE);
            for(int i = 0; i < (int)input.size(); i++ ){
                input.at(i) += ndsens(nrd);
            }

        }
		*/

        //set weights of controllers to 0 to remove inputs and force simple configuration
        switch(actuatorused){
            case 0: //upper body
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 4: //interleg
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 6: //stance knee
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 8: //swing knee
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 10: //stance ankle
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 12: //swing ankle
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(14) = 0;
                inweights.at(17) = 0;
                inweights.at(18) = 0;
                break;

            case 33: //interhip
                inweights.at(0) = 0;
                inweights.at(1) = 0;
                inweights.at(2) = 0;
                inweights.at(3) = 0;
                inweights.at(4) = 0;
                inweights.at(5) = 0;
                inweights.at(6) = 0;
                inweights.at(7) = 0;
                inweights.at(8) = 0;
                inweights.at(9) = 0;
                inweights.at(10) = 0;
                inweights.at(11) = 0;
                inweights.at(12) = 0;
                inweights.at(13) = 0;
                inweights.at(14) = 0;
               break;
        }

        //normalise angle/derivative inputs to -1 +1
        double mininput = smallestval(input);
        double maxinput =  largestval(input);
        for(int i = 0; i < numinputs-2; i++){
            double transformed = 2*((input.at(i)- mininput)/(maxinput - mininput)) - 1;
            input.at(i) = transformed;
        }

        //compute desired angle from inputs and weights
        desiredangle = 0;
        for(int i = 0; i < numinputs; i++){
            desiredangle += (input.at(i) * inweights.at(i));
        }

    }

//utility method for smallest value in array
    static double smallestval(std::vector<double> array){
        double smallest = DBL_MAX;
        int arraysize = array.size();
        for (int i=0; i < arraysize; i++)
            if( array.at(i) < smallest ){
                 smallest = array.at(i) ;
            }

        return smallest;
    }

//utility method for largest value in array
    static double largestval(std::vector<double> array){
        double largest = -DBL_MAX;
        int arraysize = array.size();
        for (int i=0; i < arraysize; i++)
            if( array.at(i) > largest ){
                 largest = array.at(i) ;
            }

        return largest;
    }

};

//walker object
struct Walker {

	//genotypes (plus unused body genotype)
    std::vector<double> geno;
    std::vector<double> bodgeno;
	
	//path tracking
    std::vector<double> trajectoryx;
    std::vector<double> trajectoryy;
	
	//body parameters
    dMass bodymass, hipmassl, hipmassr, upperleftmass, upperrightmass, lowerleftmass, lowerrightmass, leftfootmass, rightfootmass;
    dBodyID body[9];
    dJointID joint[9];
    dGeomID box[9];
    dReal torques[7];
    dReal angles[7];
    Controller filconts[7];
	int contcount;
	dReal initang; //leg raise angle
	double startz;
	
	//ode world for each walker
    dWorldID world;
    dSpaceID space;
    dGeomID ground;
    dJointGroupID contactgroup;
	
	//ground contact booleans and derivatives
	bool hasfallen = false;
    bool isfinished = false;
    dReal leftfootcollide;
    dReal rightfootcollide;
    dReal prevleftthigh;
    dReal prevrightthigh;
    dReal previnterleg;
    dReal prevbody;
    dReal previnterhip;
	
	//default values for parameters
    float fitness = 0;
    float torquecost = 0;
    bool isstanceleft;
    bool iswinner = false;
    int name; //unique identifier int
    int age; //age in timesteps
	
	//feedback measures for torque values
    dJointFeedback* tor1;
    dJointFeedback* tor2;
    dJointFeedback* tor3;
    dJointFeedback* tor4;
    dJointFeedback* tor5;
    dJointFeedback* tor6;
	
	//centres of mass are attatched as blocks to bodies
    dBodyID extrabod[9];


	//constructor
    Walker(int namae, std::vector<double> genot)
    {
          name = namae;
          geno = genot;
          startz = STARTZ;
		  //setting the bodgeno to default values (these wont change)
          bodgeno.push_back(UPPERLENGTH);
          bodgeno.push_back(LOWERLENGTH);
          bodgeno.push_back(0);
          bodgeno.push_back(UPMAS);
          bodgeno.push_back(LOMAS);
          bodgeno.push_back(0);
          bodgeno.push_back(-0.111);
          bodgeno.push_back(-0.105);
          bodgeno.push_back(0);
    }

    Walker(){}

    ~Walker()
    {
		//destroying ode objects
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

	//utility method to get position
    void printpos(){
        const dReal* printbod = dGeomGetPosition(box[0]);
        printf("BODPOS %f %f %f \n", printbod[0], printbod[1], printbod[2]);
    }

	//initialise walker
    void init(){

        age = 0;
        leftfootcollide = 0;
        rightfootcollide = 0;
        fitness = 0;
		isstanceleft = false;
        initang = INITANG;
        torquecost = TORQUETOTAL;
		
        world = dWorldCreate();
        space = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground = dCreatePlane(space,0,0,1,0);
        dWorldSetGravity (world,0,0,GRAV);
        dWorldSetERP(world, ERP);
        dWorldSetCFM(world, CFM);
		

        //initialise masses (the masses on the thigh and shanks are attatched as separate blocks for an centre of mass movement genotype extension)
        dMassSetBoxTotal(&bodymass,BODMAS,RADIUS*BODSFX,RADIUS*BODSFY,BODLENGTH*BODSFZ);
        dMassSetBoxTotal(&hipmassl,HIPMAS,RADIUS*HIPSFX,HIPSFY*HIPLENGTH,RADIUS*HIPSFZ);
        dMassSetBoxTotal(&hipmassr,HIPMAS,RADIUS*HIPSFX,HIPSFY*HIPLENGTH,RADIUS*HIPSFZ);
        dMassSetBoxTotal(&upperleftmass, 0.01,RADIUS*UPSFX,RADIUS*UPSFY,UPPERLENGTH*UPSFZ);
        dMassSetBoxTotal(&upperrightmass,0.01,RADIUS*UPSFX,RADIUS*UPSFY,UPPERLENGTH*UPSFZ);
        dMassSetBoxTotal(&lowerleftmass,0.01,RADIUS*LOSFX,RADIUS*LOSFY,LOWERLENGTH*LOSFZ);
        dMassSetBoxTotal(&lowerrightmass,0.01,RADIUS*LOSFX,RADIUS*LOSFY,LOWERLENGTH*LOSFZ);
        dMassSetBoxTotal(&leftfootmass,FOMAS,FOOTLENGTH*FTSFX,FOOTWIDTH*FTSFY,FOOTHEIGHT*FTSFZ);
        dMassSetBoxTotal(&rightfootmass,FOMAS,FOOTLENGTH*FTSFX,FOOTWIDTH*FTSFY,FOOTHEIGHT*FTSFZ);

        //initialise controllers
        contcount = CONTCOUNT;
        for(int i = 0; i < contcount; i++){
            torques[i] = 0;
            switch(i){
                  case 0: //upperbody
                        filconts[i].initcon(0);
                      break;
                  case 1: //interleg
                        filconts[i].initcon(4);
                      break;
                  case 2: //stance knee
                          filconts[i].initcon(6);
                      break;
                  case 3: //swing knee
                          filconts[i].initcon(8);
                          break;
                  case 4: //stance ankle
                            filconts[i].initcon(10);
                      break;
                  case 5: //swing ankle
                            filconts[i].initcon(12);
                      break;
                case 6: //interhip
                      filconts[i].initcon(33);
                break;

            }
        }

        //applying genotype to controllers
        for(int i = 0; i < CONTCOUNT; i++){
           for(int j = 0; j < INPUTS; j++){
               int genum = j + INPUTS*i;
               if(j == 16){
                   if(geno.at(genum) <= 0){
                       //mutation rate cannot be 0
                       geno.at(genum) = 0.01;
                   }
               }
               filconts[i].inweights.at(j) = geno.at(genum);
           }
        }

        //hip block LEFT
        body[0] = dBodyCreate (world);
        dBodySetMass (body[0],&hipmassl);
        box[0] = dCreateBox (space,RADIUS,HIPLENGTH,RADIUS);
        dGeomSetBody (box[0],body[0]);
        dBodySetPosition (body[0],0, 0.0415,startz);

        //hip block RIGHT
        body[8] = dBodyCreate (world);
        dBodySetMass (body[8],&hipmassr);
        box[8] = dCreateBox (space,RADIUS,HIPLENGTH,RADIUS);
        dGeomSetBody (box[8],body[8]);
        dBodySetPosition (body[8],0,-0.0415,startz);

        //body block
        body[1] = dBodyCreate (world);
        dBodySetMass (body[1],&bodymass);
        box[1] = dCreateBox (space,RADIUS,RADIUS,BODLENGTH);
        dGeomSetBody (box[1],body[1]);
        dBodySetPosition (body[1],0,0,startz + BODLENGTH/2);

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
        box[2] = dCreateBox (space,RADIUS,RADIUS,bodgeno.at(0));
        dGeomSetBody (box[2],body[2]);
        dBodySetPosition (body[2],0,-(HIPLENGTH),startz-(bodgeno.at(0)/2)-0.025);

            //RTHIGH CENTRE OF MASS BLOCK
            extrabod[0] = dBodyCreate(world);
            dMass extrathighrmass;
            dMassSetBox(&extrathighrmass, 1, 0.001,0.001,0.001);
            dMassAdjust(&extrathighrmass, bodgeno.at(3));
            dBodySetMass(extrabod[0], &extrathighrmass);
            const dReal* thighrextrapos = dBodyGetPosition(body[2]);
            dBodySetPosition(extrabod[0],thighrextrapos[0],thighrextrapos[1],thighrextrapos[2]+bodgeno.at(6));
            dJointID thighrjoin = dJointCreateHinge(world,0);
            dJointAttach(thighrjoin,body[2],extrabod[0]);
            dJointSetHingeParam(thighrjoin, dParamHiStop, 0);
            dJointSetHingeParam(thighrjoin, dParamLoStop, 0);

        //left thigh
        body[3] = dBodyCreate (world);
        dBodySetMass (body[3],&upperleftmass);
        box[3] = dCreateBox (space,RADIUS,RADIUS,bodgeno.at(0));
        dGeomSetBody (box[3],body[3]);
        //leg raise
        dBodySetPosition (body[3], ((bodgeno.at(0)/2)*sin(initang)), (HIPLENGTH),startz-0.025-((bodgeno.at(0)/2)*cos(initang)));
        //set thigh rotation
        dMatrix3 rot_ptr;
        dRFromAxisAndAngle (rot_ptr, 0, 1,0, -initang);
        dBodySetRotation(body[3],rot_ptr);
        dGeomSetRotation(box[3], rot_ptr);

            //LTHIGH CENTRE OF MASS BLOCK
            extrabod[1] = dBodyCreate(world);
            dMass extrathighlmass;
            dMassSetBox(&extrathighlmass, 1, 0.001,0.001,0.001);
            dMassAdjust(&extrathighlmass, bodgeno.at(3));
            dBodySetMass(extrabod[1], &extrathighlmass);
            dMatrix3 extrarotptr1;
            dRFromAxisAndAngle (extrarotptr1, 0, 1,0, -initang);
            dBodySetRotation(extrabod[1],extrarotptr1);
            const dReal* thighlextrapos = dBodyGetPosition(body[3]);
            dBodySetPosition(extrabod[1],thighlextrapos[0]-(bodgeno.at(6))*sin(initang),thighlextrapos[1],thighlextrapos[2]+(bodgeno.at(6))*cos(initang));
            dJointID thighljoin = dJointCreateHinge(world,0);
            dJointAttach(thighljoin,body[3],extrabod[1]);
            dJointSetHingeParam(thighljoin, dParamHiStop, 0);
            dJointSetHingeParam(thighljoin, dParamLoStop, 0);

        const dReal* d1 = dGeomGetPosition(box[2]); //right thigh block position
        const dReal* d2 = dGeomGetPosition(box[3]); //left thigh block position

        //HIP/THIGH JOINTS
        joint[1] = dJointCreateUniversal(world,0);
        dJointAttach (joint[1],body[2],body[8]);
        dJointSetUniversalAnchor (joint[1],d1[0],d1[1],d1[2] + (bodgeno.at(0)/2));
        dJointSetUniversalAxis2 (joint[1],0,1,0);
        dJointSetUniversalParam (joint[1],dParamHiStop2,PI/2); 
        dJointSetUniversalParam (joint[1],dParamLoStop2,-PI/2);
        dJointSetUniversalAxis1 (joint[1],1,0,0);
        if(THREED){
            dJointSetUniversalParam (joint[1],dParamHiStop, 0);
            dJointSetUniversalParam (joint[1],dParamLoStop, -10*DEG2RAD); 
        }else{
            dJointSetUniversalParam (joint[1],dParamHiStop, 0);
            dJointSetUniversalParam (joint[1],dParamLoStop, 0);
        }

        joint[2] = dJointCreateUniversal(world,0);
        dJointAttach (joint[2],body[3],body[0]);
        //leg raise 
        dJointSetUniversalAnchor (joint[2],d2[0] - (bodgeno.at(0)/2)*sin(initang),d2[1], d2[2] + (bodgeno.at(0)/2)*cos(initang));
        dJointSetUniversalAxis2 (joint[2],0,1,0);
        dJointSetUniversalParam (joint[2],dParamHiStop2,PI/2); 
        dJointSetUniversalParam (joint[2],dParamLoStop2,-PI/2); 
        dJointSetUniversalAxis1 (joint[2],1,0,0);
        if(THREED){
            dJointSetUniversalParam (joint[2],dParamHiStop, 10*DEG2RAD);
            dJointSetUniversalParam (joint[2],dParamLoStop, 0);
        }else{
            dJointSetUniversalParam (joint[2],dParamHiStop, 0);
            dJointSetUniversalParam (joint[2],dParamLoStop, 0);
        }


        //right shank
        body[4] = dBodyCreate (world);
        dBodySetMass (body[4],&lowerrightmass);
        box[4] = dCreateBox (space,RADIUS,RADIUS,bodgeno.at(1));
        dGeomSetBody (box[4],body[4]);
        dBodySetPosition (body[4],d1[0]+0.0011,d1[1]+0.0140,d1[2]-(bodgeno.at(0)/2)-(bodgeno.at(1)/2));

            //RSHANK CENTRE OF MASS BLOCK
            extrabod[2] = dBodyCreate(world);
            dMass extrarshankmass;
            dMassSetBox(&extrarshankmass, 1, 0.001,0.001,0.001);
            dMassAdjust(&extrarshankmass, bodgeno.at(4));
            dBodySetMass(extrabod[2], &extrarshankmass);
            const dReal* rshankextrapos = dBodyGetPosition(body[4]);
            dBodySetPosition(extrabod[2],rshankextrapos[0],rshankextrapos[1],rshankextrapos[2]+bodgeno.at(7));
            dJointID rshankjoin = dJointCreateHinge(world,0);
            dJointAttach(rshankjoin,body[4],extrabod[2]);
            dJointSetHingeParam(rshankjoin, dParamHiStop, 0);
            dJointSetHingeParam(rshankjoin, dParamLoStop, 0);

        //left shank
        body[5] = dBodyCreate (world);
        dReal hippos_ptr[3];
        dJointGetUniversalAnchor(joint[2], hippos_ptr);
        dBodySetMass (body[5],&lowerleftmass);
        box[5] = dCreateBox (space,RADIUS,RADIUS,bodgeno.at(1));
        dGeomSetBody (box[5],body[5]);
        //leg raise
        dBodySetPosition (body[5], hippos_ptr[0] + (bodgeno.at(0)+(bodgeno.at(1)/2))*sin(initang) + 0.0011*cos(initang),hippos_ptr[1]-0.0140,hippos_ptr[2]-(bodgeno.at(0)+(bodgeno.at(1)/2))*cos(initang) + 0.0011*sin(initang));
        //set shank rotation
        dMatrix3 rot_ptr2;
        dRFromAxisAndAngle (rot_ptr2, 0, 1,0, -initang);
        dBodySetRotation(body[5],rot_ptr2);
        dGeomSetRotation(box[5], rot_ptr2);

            //LSHANK CENTRE OF MASS BLOCK
            extrabod[3] = dBodyCreate(world);
            dMass extralshankmass;
            dMassSetBox(&extralshankmass, 1, 0.001,0.001,0.001);
            dMassAdjust(&extralshankmass, bodgeno.at(4));
            dBodySetMass(extrabod[3], &extralshankmass);
            dMatrix3 extrarotptr2;
            dRFromAxisAndAngle (extrarotptr2, 0, 1,0, -initang);
            dBodySetRotation(extrabod[3],extrarotptr2);
            const dReal* lshankextrapos = dBodyGetPosition(body[5]);
            dBodySetPosition(extrabod[3],lshankextrapos[0]-(bodgeno.at(7))*sin(initang),lshankextrapos[1],lshankextrapos[2]+bodgeno.at(7)*cos(initang));
            dJointID lshankextrajoin = dJointCreateHinge(world,0);
            dJointAttach(lshankextrajoin,body[5],extrabod[3]);
            dJointSetHingeParam(lshankextrajoin, dParamHiStop, 0);
            dJointSetHingeParam(lshankextrajoin, dParamLoStop, 0);

        const dReal* e1 = dGeomGetPosition(box[4]); //right shank block position

        //KNEE Y JOINTS
        joint[3] = dJointCreateHinge(world,0);
        dJointAttach (joint[3],body[2],body[4]);
        dJointSetHingeAnchor (joint[3],e1[0],e1[1],e1[2]+(bodgeno.at(1)/2));
        dJointSetHingeAxis (joint[3],0,1,0);
        dJointSetHingeParam (joint[3],dParamHiStop,0);
        dJointSetHingeParam (joint[3],dParamLoStop,-PI/2); //-PI/2
		
        joint[4] = dJointCreateHinge(world,0);
        dJointAttach (joint[4],body[3],body[5]);
        //leg raise
        dJointSetHingeAnchor(joint[4],hippos_ptr[0] + (bodgeno.at(0))*sin(initang) ,hippos_ptr[1]-0.0140,hippos_ptr[2]-(bodgeno.at(0))*cos(initang));
        dJointSetHingeAxis (joint[4],0,1,0);
        dJointSetHingeParam (joint[4],dParamHiStop,0);
        dJointSetHingeParam (joint[4],dParamLoStop,-PI/2); //-PI/2

        //right foot
        body[6] = dBodyCreate (world);
        dBodySetMass (body[6],&rightfootmass);
        box[6] = dCreateBox (space,FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT);
        dGeomSetBody (box[6],body[6]);
        dBodySetPosition (body[6],e1[0]+0.053,e1[1],e1[2]-(bodgeno.at(1)/2)-(FOOTHEIGHT/2));

        //left foot
        body[7] = dBodyCreate (world);
        dReal kneepos_ptr[3];
        dJointGetHingeAnchor(joint[4], kneepos_ptr);
        dBodySetMass (body[7],&leftfootmass);
        box[7] = dCreateBox (space,FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT);
        dGeomSetBody (box[7],body[7]);
        //leg raise
        dBodySetPosition (body[7], kneepos_ptr[0]+0.053*cos(initang)+(bodgeno.at(1)+FOOTHEIGHT/2)*sin(initang),kneepos_ptr[1],kneepos_ptr[2]+ 0.053*sin(initang)-(bodgeno.at(1)+FOOTHEIGHT/2)*cos(initang));
        //set angle rotation
        dMatrix3 rot_ptr3;
        dRFromAxisAndAngle (rot_ptr3, 0, 1,0, -initang);
        dBodySetRotation(body[7],rot_ptr3);
        dGeomSetRotation(box[7], rot_ptr3);

        const dReal* f1 = dGeomGetPosition(box[6]); //right ankle block position

        //ANKLE JOINTS 
        joint[5] = dJointCreateUniversal(world,0);
        dJointAttach (joint[5],body[4],body[6]);
        dJointSetUniversalAnchor (joint[5],f1[0]-0.053,f1[1],f1[2]+(FOOTHEIGHT/2));
        dJointSetUniversalAxis2 (joint[5],0,1,0);
        dJointSetUniversalParam (joint[5],dParamHiStop2,25*DEG2RAD);
        dJointSetUniversalParam (joint[5],dParamLoStop2,-25*DEG2RAD);
        dJointSetUniversalAxis1 (joint[5],1,0,0);
        if(THREED){
            dJointSetUniversalParam (joint[5],dParamHiStop,30*DEG2RAD);
            dJointSetUniversalParam (joint[5],dParamLoStop,-30*DEG2RAD);
        }else{
            dJointSetUniversalParam (joint[5],dParamHiStop,0);
            dJointSetUniversalParam (joint[5],dParamLoStop,0);
        }

        joint[6] = dJointCreateUniversal(world,0);
        dJointAttach (joint[6],body[5],body[7]);
        //leg raise
        dJointSetUniversalAnchor(joint[6],kneepos_ptr[0]+0.0011+(bodgeno.at(1))*sin(initang),kneepos_ptr[1],kneepos_ptr[2]-(bodgeno.at(1))*cos(initang));
        dJointSetUniversalAxis2 (joint[6],0,1,0);
        dJointSetUniversalParam (joint[6],dParamHiStop2,25*DEG2RAD);
        dJointSetUniversalParam (joint[6],dParamLoStop2,-25*DEG2RAD);
        dJointSetUniversalAxis1 (joint[6],1,0,0);
        if(THREED){
            dJointSetUniversalParam (joint[6],dParamHiStop,30*DEG2RAD);
            dJointSetUniversalParam (joint[6],dParamLoStop,-30*DEG2RAD);
        }else{
            dJointSetUniversalParam (joint[6],dParamHiStop,0);
            dJointSetUniversalParam (joint[6],dParamLoStop,0);
        }

        //JOINT PARAMS
        dJointSetHingeParam(joint[0], dParamStopERP, ERP);
        dJointSetHingeParam(joint[0], dParamStopCFM, CFM);
        dJointSetHingeParam(joint[0], dParamCFM, CFM);

        for(int i = 1; i < 3; i++){
            dJointSetUniversalParam(joint[i], dParamStopERP1, HIPERP);
            dJointSetUniversalParam(joint[i], dParamStopCFM1, HIPCFM);
            dJointSetUniversalParam(joint[i], dParamCFM1, HIPCFM);
            dJointSetUniversalParam(joint[i], dParamStopERP2, HIPERP);
            dJointSetUniversalParam(joint[i], dParamStopCFM2, HIPCFM);
            dJointSetUniversalParam(joint[i], dParamCFM2, HIPCFM);
        }

        for(int i = 3; i < 5; i++){
            dJointSetHingeParam(joint[i], dParamStopERP, KNEEERP);
            dJointSetHingeParam(joint[i], dParamStopCFM, KNEECFM);
            dJointSetHingeParam(joint[i], dParamCFM, KNEECFM);
        }

        for(int i = 5; i < 7; i++){
            dJointSetUniversalParam(joint[i], dParamStopERP1, ANKLEERP);
            dJointSetUniversalParam(joint[i], dParamStopCFM1, ANKLECFM);
            dJointSetUniversalParam(joint[i], dParamCFM1, ANKLECFM);

            dJointSetUniversalParam(joint[i], dParamStopERP2, ANKLEERP);
            dJointSetUniversalParam(joint[i], dParamStopCFM2, ANKLECFM);
            dJointSetUniversalParam(joint[i], dParamCFM2, ANKLECFM);
        }

        for(int i = 7; i < 9; i++){
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

        //PREV ANGLES INITIAL SETTING
        prevbody = 0;
        previnterleg =  initang;
        prevleftthigh = initang;
        prevrightthigh = 0;
        previnterhip = 0;

    }

	//draw walker
    void draw()
    {
		  //boxes
          dReal sides[3] = {RADIUS,HIPLENGTH,RADIUS};
          dReal sides15[3] = {RADIUS,RADIUS,BODLENGTH};
          dReal sides2[3] = {RADIUS,RADIUS,(dReal)bodgeno.at(0)};
          dReal sides25[3] = {RADIUS,RADIUS,(dReal)bodgeno.at(1)};
          dReal sides3[3] = {FOOTLENGTH,FOOTWIDTH,FOOTHEIGHT};
          dReal masscube[3] = {RADIUS*1.4, RADIUS*1.4, RADIUS*1.4};

		  //upperbody
          dsSetColor (0.6,0,0);
          dsDrawBox (dGeomGetPosition(box[1]),dGeomGetRotation(box[1]),sides15);
          dReal jointpos0[3];
          dJointGetHingeAnchor(joint[0],jointpos0);
          dsDrawSphere(jointpos0,dGeomGetRotation(box[1]),RADIUS);

		  //hips
          dsSetColor (0,0,1);
          dsDrawBox (dGeomGetPosition(box[0]),dGeomGetRotation(box[0]),sides);
          dsSetColor (1,1,1);
          dsDrawBox (dGeomGetPosition(box[8]),dGeomGetRotation(box[8]),sides);
          dsSetColor (0.6,0,0);

          if(!isstanceleft){
              dsSetColor(1,1,0);
          }
          //right thigh (change colour if stance leg)
          dsDrawBox (dGeomGetPosition(box[2]),dGeomGetRotation(box[2]),sides2);
          dsSetColor (0.6,0,0);

          if(isstanceleft){
              dsSetColor(1,1,0);
          }
          //left thigh (change colour if stance leg)
          dsDrawBox (dGeomGetPosition(box[3]),dGeomGetRotation(box[3]),sides2);
          dsSetColor (0.6,0,0);

		  //draw balls at the joints
          dReal jointpos1[3];
          dJointGetUniversalAnchor(joint[1],jointpos1);
          dsDrawSphere(jointpos1, dGeomGetRotation(box[1]),RADIUS);
          dReal jointpos2[3];
          dJointGetUniversalAnchor(joint[2],jointpos2);
          dsDrawSphere(jointpos2, dGeomGetRotation(box[2]),RADIUS);
          dsSetColor (0,0.6,0);
          dReal jointpos3[3];
          dJointGetHingeAnchor(joint[3],jointpos3);
          dsDrawSphere(jointpos3, dGeomGetRotation(box[3]),RADIUS);
          dReal jointpos4[3];
          dJointGetHingeAnchor(joint[4],jointpos4);
          dsDrawSphere(jointpos4, dGeomGetRotation(box[4]),RADIUS);

		  //right shank (change colour if stance leg)
          if(!isstanceleft){
              dsSetColor(1,1,0);
          }
          dsDrawBox (dGeomGetPosition(box[4]),dGeomGetRotation(box[4]),sides25);
          dsSetColor (0,0.6,0);

		  //left shank (change colour if stance leg)
          if(isstanceleft){
              dsSetColor(1,1,0);
          }
          dsDrawBox (dGeomGetPosition(box[5]),dGeomGetRotation(box[5]),sides25);
          dsSetColor (0,0.6,0);

		  //right foot(change colour if touching ground)
          dsSetColor (1,0.5,0);
          if(rightfootcollide){
              dsSetColor(0,0,0.8);
          }
          dReal jointpos5[3];
          dJointGetUniversalAnchor(joint[5],jointpos5);
          dsDrawSphere(jointpos5, dGeomGetRotation(box[5]),RADIUS);
          dsDrawBox (dGeomGetPosition(box[6]),dGeomGetRotation(box[6]),sides3);

		  //left foot(change colour if touching ground)
          dsSetColor (1,0.4,0.6);
          if(leftfootcollide){
              dsSetColor(0,0,0.8);
          }
          dReal jointpos6[3];
          dJointGetUniversalAnchor(joint[6],jointpos6);
          dsDrawSphere(jointpos6, dGeomGetRotation(box[6]),RADIUS);
          dsDrawBox (dGeomGetPosition(box[7]),dGeomGetRotation(box[7]),sides3);

          //DRAW MASS CUBES
              dsSetColor (1-(bodgeno.at(3)/2.2),1-(bodgeno.at(3)/2.2),1-(bodgeno.at(3)/2.2));
              dsDrawBox(dBodyGetPosition(extrabod[0]),dBodyGetRotation(extrabod[0]), masscube);
              dsDrawBox(dBodyGetPosition(extrabod[1]),dBodyGetRotation(extrabod[1]), masscube);
              dsSetColor(1-(bodgeno.at(4)/2.2),1-(bodgeno.at(4)/2.2),1-(bodgeno.at(4)/2.2));
              dsDrawBox(dBodyGetPosition(extrabod[2]),dBodyGetRotation(extrabod[2]), masscube);
              dsDrawBox(dBodyGetPosition(extrabod[3]),dBodyGetRotation(extrabod[3]), masscube);

		  //draw turning point
          if(TURNMODE){
              dReal pointpos[3];
              pointpos[0] = WANTPOSX;
              pointpos[1] = WANTPOSY;
              pointpos[2] = 0;
              dsSetColor (1,0,0);
              dsDrawSphere(pointpos, dBodyGetRotation(body[0]), 0.25);
          }
    }

	//cancel 3d forces on bodies if constraining to 2D
    void AlignToYPlane()
    {
        for(int i = 0; i < 9; i++){
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

        for(int i = 0; i < 4; i++){
            const dReal *rot = dBodyGetAngularVel( extrabod[i] );
            const dReal *quat_ptr = dBodyGetQuaternion( extrabod[i] );
            const dReal* vel_ptr= dBodyGetLinearVel(extrabod[i]);
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
            dBodySetQuaternion( extrabod[i], quat );
            dBodySetAngularVel( extrabod[i], 0, rot[1], 0 );
            dBodySetLinearVel(extrabod[i], vel[0], vel[1], vel[2]);
        }

    }

	//setting final fitness values 
    void finalset(){
        const dReal* hippos = dBodyGetPosition(body[0]);
        double legspan = 2*(UPPERLENGTH + LOWERLENGTH);
		//fitness is steps, distance over span
        fitness = (hippos[0]/legspan);
		
		//alternate fitness for turning
        if(TURNMODE){
            double distfitness = sqrt(pow((WANTPOSX - hippos[0]),2) + pow((WANTPOSY - hippos[1]),2));
            if(distfitness < 2){
                fitness = (MAXFIT - distfitness) + 50;
            }
            else{
                fitness = (MAXFIT - distfitness);
            }
        }

		/* //noise to fitness
        if(NOISEON && (FITSIG > 0)){
            std::normal_distribution<double> ndfits(0,FITSIG*ALLSCALE);
            fitness += ndfits(nrd);
        }
		*/
		
        printf("WINNER FITNESS: %f\n", this->fitness);        
        printf("TORQUE REMAINING: %f\n", torquecost);
//        printf("GENOTYPE\n");
//        for(int i = 0; i < geno.length(); i++){
//            printf("%f ", geno.at(i));
//        }
//        printf("\n");
        printf("AGE %f\n", age*TSTEP);
    }

//handling collisions in the walkers ode world
    void LocalNearCallback (dGeomID o1, dGeomID o2){
          int i,n;
          const int N = 1000;
          dContact contact[N];

          // FILTER OUT EXTRA COLLISIONS THAT ARENT BETWEEN A BODY AND THE GROUND
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

		  //IDENTIFYING COLLIDING BODIES
          int w1=-999;
          if (o1==ground) w1=42;
          else for (int x=0;x<=8;x++) if (o1==box[x]) w1=x;


          // LIMITING NUMBER OF COLLISIONS PER FFET AND FLOOR TO PREVENT CRASHING
          int contactpointslimit = 100;
          n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
          if(n > contactpointslimit){
              n = contactpointslimit;
          }

		  //SETTING COLLISION PARAMETERS AND CREATING CONTACT JOINT
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
			  //COLLISION EVENTS
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
        //run the walker for one timestep

        //STANCE LEG
        //in doublestance?
        if((leftfootcollide == 1) & (rightfootcollide == 1)){
            if(THREED){
                //caudal (furthest back in direction of travel) ankle

                //get vector for leg xy (ankle position - hip position)
                dReal lhippos[3], rhippos[3], lanklepos[3], ranklepos[3];
                dJointGetUniversalAnchor(joint[1], rhippos);
                dJointGetUniversalAnchor(joint[2], lhippos);
                dJointGetUniversalAnchor(joint[5], ranklepos);
                dJointGetUniversalAnchor(joint[6], lanklepos);

                dReal legvecl[3], legvecr[3];
                legvecl[0] = lanklepos[0]-lhippos[0];
                legvecl[1] = lanklepos[1]-lhippos[1];
                legvecl[2] = lanklepos[2]-lhippos[2];
                legvecr[0] = ranklepos[0]-rhippos[0];
                legvecr[1] = ranklepos[1]-rhippos[1];
                legvecr[2] = ranklepos[2]-rhippos[2];

                //rotation transpose
                const dReal* bearing = dBodyGetRotation(body[1]);
                dReal tmatrix[3][3];
                tmatrix[0][0]= bearing[0];
                tmatrix[0][1]= bearing[4];
                tmatrix[0][2]= bearing[8];
                tmatrix[1][0]= bearing[1];
                tmatrix[1][1]= bearing[5];
                tmatrix[1][2]= bearing[9];
                tmatrix[2][0]= bearing[2];
                tmatrix[2][1]= bearing[6];
                tmatrix[2][2]= bearing[10];

                //vector multiplication row by column
                dReal multvecl[3], multvecr[3];
                multvecl[0] = (tmatrix[0][0]*legvecl[0]) + (tmatrix[0][1]*legvecl[1]) + (tmatrix[0][2]*legvecl[2]);
                multvecl[1] = (tmatrix[1][0]*legvecl[0]) + (tmatrix[1][1]*legvecl[1]) + (tmatrix[1][2]*legvecl[2]);
                multvecl[2] = (tmatrix[2][0]*legvecl[0]) + (tmatrix[2][1]*legvecl[1]) + (tmatrix[2][2]*legvecl[2]);

                multvecr[0] = (tmatrix[0][0]*legvecr[0]) + (tmatrix[1][0]*legvecr[1]) + (tmatrix[2][0]*legvecr[2]);
                multvecr[1] = (tmatrix[1][0]*legvecr[0]) + (tmatrix[1][1]*legvecr[1]) + (tmatrix[2][1]*legvecr[2]);
                multvecr[2] = (tmatrix[2][0]*legvecr[0]) + (tmatrix[2][2]*legvecr[1]) + (tmatrix[2][2]*legvecr[2]);


                //compare x coordinates
                const dReal* bodcaudpos = dBodyGetPosition(body[1]);

                dReal leftfootx = bodcaudpos[0] + multvecl[0];
                dReal rightfootx = bodcaudpos[0] + multvecr[0];

                if(leftfootx < rightfootx){
                    isstanceleft = true;
                }else{
                    isstanceleft = false;
                }
            }else{
                //caudal foot from motion vector (in 2d this is just (1,0,0))
                //choose furthest back and break ties 
                const dReal* leftfootpos = dGeomGetPosition(box[7]);
                const dReal* rightfootpos = dGeomGetPosition(box[6]);
                if( leftfootpos[0] <= rightfootpos[0]){
                         isstanceleft = true;
                }else{
                         isstanceleft = false;
                }
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
                //flying, remains the same as before
            }
        }

        //INPUTS based on diagram in paper
		dReal bodangle, t_st, il, ih;
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
            filconts[i].input.at(0) = bodangle;


            //1 = upper body derivative
            double ubd = (bodangle - prevbody) / TSTEP;
            filconts[i].input.at(1) = ubd;

            //2 = stance thigh angle (angle between stance thigh and vertical)
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

            //h_st (angle betwwn stance thigh and upper body, seen in solomon et al)
            if(isstanceleft){
                h_st = lethangle - bodangle;
            }else{
                h_st = rithangle - bodangle;
            }
			//stance thigh angle is h_st + upper body
            t_st = h_st + bodangle;
            filconts[i].input.at(2) = t_st;
			

            //3 = stance thigh angle derivative
            double stad;
            if(isstanceleft){
                    stad = (t_st - prevleftthigh) / TSTEP;
                    filconts[i].input.at(3) = stad;
            }else{
                    stad = (t_st - prevrightthigh) / TSTEP;
                    filconts[i].input.at(3) = stad;
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
            //interleg is hsw - hst
            il = h_sw - h_st;
            //accounting for the atan fix
            il = -il;
            il = fabs(il);
            filconts[i].input.at(4) = il;

            //5 = interleg derivative
            double ildev = (il - previnterleg)/ TSTEP;
            filconts[i].input.at(5) = ildev;

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
            filconts[i].input.at(6) = k_st;

            //7 = stance knee angle derivative
            if(isstanceleft){
                    filconts[i].input.at(7) = dJointGetHingeAngleRate(joint[4]);
            }else{
                    filconts[i].input.at(7) = dJointGetHingeAngleRate(joint[3]);
            }

            //8 = swing knee angle
            dReal k_sw;
            if(isstanceleft){
                k_sw = riknangle;
            }else{
                k_sw = leknangle;
            }
            filconts[i].input.at(8) = k_sw;

            //9 = swing knee derivative
            if(isstanceleft){
                    filconts[i].input.at(9) = dJointGetHingeAngleRate(joint[3]);
            }else{
                    filconts[i].input.at(9) = dJointGetHingeAngleRate(joint[4]);
            }

            //10 = stance ankle angle
            dReal leanangle = dJointGetUniversalAngle2(joint[6]);
            dReal rianangle = dJointGetUniversalAngle2(joint[5]);
            dReal a_st;
            //a_st
            if(isstanceleft){
                a_st = leanangle;
            }else{
                a_st = rianangle;
            }
            a_st = (2*PI) - a_st;
            filconts[i].input.at(10) = a_st;

            //11 = stance ankle derivative
            if(isstanceleft){
                    filconts[i].input.at(11) = dJointGetUniversalAngle2Rate(joint[6]);
            }else{
                    filconts[i].input.at(11) = dJointGetUniversalAngle2Rate(joint[5]);
            }

            //12 = swing ankle angle
            dReal a_sw;
            if(isstanceleft){
                a_sw = rianangle;
            }else{
                a_sw = leanangle;
            }
            filconts[i].input.at(12) = a_sw;

            //13 = swing ankle derivative
            if(isstanceleft){
                    filconts[i].input.at(13) = dJointGetUniversalAngle2Rate(joint[5]);
            }else{
                    filconts[i].input.at(13) = dJointGetUniversalAngle2Rate(joint[6]);
            }

            //14 = double_stance
            //are both feet touching the ground?
            if((leftfootcollide == 1) & (rightfootcollide == 1)){
                  filconts[i].input.at(14) = 1;
            }else{
                  filconts[i].input.at(14) = 0;
            }

            //15 = bias
            filconts[i].input.at(15) = 1; //the weight is the bias and adjusted accordingly

            //16 = learning/mutation rate
            filconts[i].input.at(16) = 1; //the weight is used as the mutation rate and adjusted accordingly

            //3D INPUTS LATER
            //17 = hip il angle

            const dReal* hiprrot = dGeomGetRotation(box[2]);
            const dReal* hiplrot = dGeomGetRotation(box[3]);
            dReal hiplangle, hiprangle;

            hiplangle = atan2(hiprrot[9],hiprrot[10]);
            hiprangle = atan2(hiplrot[9],hiplrot[10]);

            //get average of two hip angles 
            ih = (abs(hiplangle) + abs(hiprangle))/2;
            filconts[i].input.at(17) = ih;

            //18 = hip il derivative
            double ihdev = (ih - previnterhip)/ TSTEP;
            filconts[i].input.at(18) = ihdev;

        }

        //setting previous angles
        prevbody = bodangle;
        if(isstanceleft){
                prevleftthigh = t_st;
        }else{
                prevrightthigh = t_st;
        }
        previnterleg = il;
        previnterhip = ih;

        //process controllers
        for(int i = 0; i < contcount; i++){
            filconts[i].tick();
            angles[i] = filconts[i].desiredangle;
        }

		//enforcing max force on ODE joints for stability
        dReal m = M; 
        dReal mfmin = MFMIN; 
        dReal mfmax = MFMAX; 
        dReal s = SPEED; 
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

		//noise to actions
		/*
        if(NOISEON && (ACTSIG > 0)){
            std::normal_distribution<double> ndacts(0,ACTSIG*ALLSCALE);
            for(int i; i < 7; i++)
            angles[i] = angles[i] + ndacts(nrd);
        }
		*/

        //ACTUATION
        if(ACTUATIONON){
            if(isstanceleft){ //which is the stance leg
                //0 body
                //1 interleg - equal and opposite on either hip
                dJointSetUniversalParam( joint[1],  dParamFMax2,  (mf[0] + mf[1])*0.5);
                dJointSetUniversalParam( joint[1] , dParamVel2,  s*(angles[0] - angles[1]*0.5));
                dJointSetUniversalParam( joint[2],  dParamFMax2,  (mf[0] + mf[1])*0.5);
                dJointSetUniversalParam( joint[2] , dParamVel2,  s*-(angles[0] - angles[1]*0.5));
                //2 stance knee
                dJointSetHingeParam( joint[4],  dParamFMax,  mf[2]);
                dJointSetHingeParam( joint[4] , dParamVel,  s*angles[2]);
                //3 swing knee
                dJointSetHingeParam( joint[3],  dParamFMax,  mf[3]);
                dJointSetHingeParam( joint[3] , dParamVel,  s*angles[3]);
                //4 stance ankle
                dJointSetUniversalParam( joint[6],  dParamFMax2,  mf[4]);
                dJointSetUniversalParam( joint[6] , dParamVel2,  s*angles[4]);
//                //5 swing ankle
                dJointSetUniversalParam( joint[5],  dParamFMax2,  mf[5]);
                dJointSetUniversalParam( joint[5] , dParamVel2,  s*angles[5]);
                if(THREED){
                    //6 interhip - lateral spacing
                    dJointSetUniversalParam( joint[1],  dParamFMax,  mf[6]*0.5);
                    dJointSetUniversalParam( joint[1] , dParamVel,  s*(angles[6]*0.5));
                    dJointSetUniversalParam( joint[2],  dParamFMax,  mf[6]*0.5);
                    dJointSetUniversalParam( joint[2] , dParamVel,  s*-(angles[6]*0.5));
                }
            }else{
                //0 body
                //1 interleg - equal and opposite on either hip
                dJointSetUniversalParam( joint[2],  dParamFMax2,  (mf[0] + mf[1])*0.5);
                dJointSetUniversalParam( joint[2] , dParamVel2,  s*(angles[0] - angles[1]*0.5));
                dJointSetUniversalParam( joint[1],  dParamFMax2,  (mf[0] + mf[1])*0.5);
                dJointSetUniversalParam( joint[1] , dParamVel2,  s*-(angles[0] - angles[1]*0.5));
                //2 stance knee
                dJointSetHingeParam( joint[3],  dParamFMax,  mf[2]);
                dJointSetHingeParam( joint[3] , dParamVel,  s*angles[2]);
                //3 swing knee
                dJointSetHingeParam( joint[4],  dParamFMax,  mf[3]);
                dJointSetHingeParam( joint[4] , dParamVel,  s*angles[3]);
                //4 stance ankle
                dJointSetUniversalParam( joint[5],  dParamFMax2,  mf[4]);
                dJointSetUniversalParam( joint[5] , dParamVel2,  s*angles[4]);
//                //5 swing ankle
                dJointSetUniversalParam( joint[6],  dParamFMax2,  mf[5]);
                dJointSetUniversalParam( joint[6] , dParamVel2,  s*angles[5]);
                if(THREED){
                    //6 interhip - lateral spacing
                    dJointSetUniversalParam( joint[2],  dParamFMax,  mf[6]*0.5);
                    dJointSetUniversalParam( joint[2] , dParamVel,  s*(angles[6]*0.5));
                    dJointSetUniversalParam( joint[1],  dParamFMax,  mf[6]*0.5);
                    dJointSetUniversalParam( joint[1] , dParamVel,  s*-(angles[6]*0.5));
                }

            }
        }

        //reset foot collision
        leftfootcollide = 0;
        rightfootcollide = 0;
        
		//world stepping
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
		//addiing all torque to total
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

		//if time alive in seconds is before removal time, compare torque
        if((age * TSTEP) < TORQUEREMOVER){
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

			//subtract from total
            torquecost -= totaltorque;
        }

        //flight phase
        if(ISFLYING){
            const dReal* footrpos = dBodyGetPosition(body[6]);
            const dReal* footlpos = dBodyGetPosition(body[7]);
            if(!leftfootcollide && !rightfootcollide && (footlpos[2] > FALLHEIGHT) && (footrpos[2] > FALLHEIGHT)){
                hasfallen = true;
            }
        }

        //falling boolean from collision
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

        //incrementing age
        age++;

        //recording position
        const dReal* hippos = dBodyGetPosition(body[0]);
        if(age % (int)(1/(2*TSTEP)) == 0){
            trajectoryx.push_back(hippos[0]);
            trajectoryy.push_back(hippos[1]);
        }


        //if its been there longer than a minute time it out
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
		//destroy ode objects
        dJointGroupDestroy (contactgroup);
        dSpaceDestroy (space);
        dWorldDestroy (world);
    }
};

//declaring the population, winner, and arrays for results recording
static std::vector<Walker*> walkers;
static std::vector<double> fitnesses, bigfitnesses, torquesleft;
static Walker* winner;
static std::vector<double> winvector, winbodvector;
static std::string fileseed;

//ode start method, sets camera 
static void start()
    {
      dAllocateODEDataForThread(dAllocateMaskAll);
      static float xyz[3] = {0,-2,STARTZ}; //-40
      static float hpr[3] = {90,0,0.0000f};
      dsSetViewpoint (xyz,hpr);
    }

//boolean for pausing simulation
static bool pauser = false;
static void command (int cmd)
    {
    switch (cmd) {
        case 'w': case 'W': //testing keypress commands go here
        dBodyAddForce(winner->body[1],0,0,1000);
          break;
      }
    }

//if walker is 2D constrain it before running
void step( Walker* wal){
    if(THREED){

    }else{
       wal->AlignToYPlane();
    }
    wal->run();
}

//simulate winner without visualisation
void simulateWalk(Walker* wal){

    //initialise
    if(!wal->iswinner){
        wal->init();
    }

    //step through
    while(!wal->isfinished){
        step(wal);
    }


    //fitness
    const dReal* hippos = dBodyGetPosition(wal->body[0]);
    double legspan = 2*(UPPERLENGTH + LOWERLENGTH);
    wal->fitness = (hippos[0]/legspan);

	//turning fitness
    if(TURNMODE){
        double distfitness = sqrt(pow((WANTPOSX - hippos[0]),2) + pow((WANTPOSY - hippos[1]),2));
        if(distfitness < 2){
            wal->fitness = (MAXFIT - distfitness) + 50;
        }
        else{
            wal->fitness = (MAXFIT - distfitness);
        }
    }

}

//simulate winner with visualisation in ODE
static void simLoop (int pause)
{   
    static double timeAheadOfPhysics = 0.0;
    if(pauser) //pausing boolean
        timeAheadOfPhysics = 0.0;
    else {
        timeAheadOfPhysics += dsElapsedTime();
        while (timeAheadOfPhysics >= TSTEP) {
          //winner run
          if(!winner->isfinished){
            step(winner);
          }
          timeAheadOfPhysics -= TSTEP;
        }
    }
	//draw
    winner->draw();

}

//simulate walkers inside the GA
void popSim(){

      for(int j = 0; j < POPSIZE; j++){
          simulateWalk(walkers.at(j));
      }

}

//bubble sort an array of walkers by fitness
std::vector<Walker*> fitsort(std::vector<Walker*> mems){
    int memssize = mems.size();
    for (int count = 0; count < memssize; count++){
        for(int comp = 0; comp < memssize-1; comp++){
            if(mems.at(comp)->fitness < mems.at(comp+1)->fitness){
                Walker* temp;
                temp = mems.at(comp);
                mems.at(comp) = mems.at(comp + 1);
                mems.at(comp + 1) = temp;
            }
        }
    }
    return mems;
}

//utility method for smallest value in an array
double smallestval(std::vector<double> array){
    double smallest = DBL_MAX;
    int arraysize = array.size();
    for (int i=0; i < arraysize; i++)
        if( array.at(i) < smallest ){
             smallest = array.at(i) ;
        }

    return smallest;
}

//utility method for largest value in an array
double largestval(std::vector<double> array){
    double largest = -DBL_MAX;
    int arraysize = array.size();
    for (int i=0; i < arraysize; i++)
        if( array.at(i) > largest ){
             largest = array.at(i) ;
        }

    return largest;
}

//utility method for random value in range
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


//GA
void geneticAlg(){
		//ga parameters
        double propselect,toppercent,p1,p2;
        gens = 0;
		
        //GENERATION LOOP
        for(int k = 0; k < GENS; k++){

            gens++;

            printf("%i--------------------------\n", k+1);
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
					
            //torque remaining for each member
            torquesleft.push_back(walkers.at(0)->torquecost);

            //MUTATION AND CROSSOVER		
			//number of elites in population
            int elitnumber = ELITNUM;
			//for the non elites
            for(int i = elitnumber; i < POPSIZE; i++){
				//replacement array for new genotype
                std::vector<double> replacement;
                replacement.clear();
                for(int j = 0; j < (CONTCOUNT*INPUTS); j++){
                    replacement.push_back(0);
                }

                // select 25% mutation, 25% crossover, 50% both
				// 1-25 OR 50-100 mutation, 25-100 crossover, therefore 50-100 both 
                propselect = rand() % 100;

                if(propselect > 24){ //crossover
                    //discrete crossover the top 20%
                    toppercent = POPSIZE*PARENTPERCENT;
					//seclect parents
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
                    double randogene[CONTCOUNT*INPUTS];

                    //choose genes for replacement from random parents
                    for(int j = 0; j < (CONTCOUNT*INPUTS); j++){
                        randogene[j] = (double)(rand()%2);
                        if(randogene[j] == 0){
                            replacement.at(j) = walkers.at(p1)->geno.at(j);
                        }else{
                            replacement.at(j) = walkers.at(p2)->geno.at(j);
                        }
                    }
                }

                if(propselect < 25 || propselect  > 49){ //mutation
					//randomly select parent
					toppercent = POPSIZE*PARENTPERCENT;
                    p1 = rand() % (int)toppercent;
					
					//randomly mutate the parent using normally distributed vector with 0.005 StandDev
                    std::random_device rd;
                    std::normal_distribution<double> ndLPD(0,0.05);

                    //mutation vector over all weights
                    double mutationvector[INPUTS*CONTCOUNT];
                    for(int j = 0; j < (INPUTS*CONTCOUNT); j++){
							  //mutation added on is random vector multiplied by mutation rate
                              mutationvector[j] = ndLPD(rd) * walkers.at(i)->filconts[(int)floor(j/INPUTS)].inweights.at(16); //learnable mutation rate for LPD
                              //if only mutation add vector
							  if(propselect < 25){
                                    replacement.at(j) = walkers.at(p1)->geno.at(j) + mutationvector[j];
                              }
							  //if both add mutation to crossover replacement
                              if(propselect > 49){
                                    replacement.at(j) = replacement.at(j) + mutationvector[j];
                              }
                    }
                }
              
			  // replace previous walker (bodgeno unchanged)
              walkers.at(i) = new Walker(i, replacement);


            }

                //put elites back in popultation
                for(int i = 0; i < elitnumber; i++){
                   walkers.at(i) = new Walker(0, walkers.at(i)->geno);
                }
        }


        printf("---------------------------\n");


        //take winner as final best member of last population
        winner = walkers.at(0);
        printf("WINNER IS %i WITH %f\n", winner->name, winner->fitness);
        winvector.clear();
        winbodvector.clear();
        winvector = winner->geno;
        winbodvector = winner->bodgeno;

}

//WRITING RESULTS TO FILE
void writeFilesSetSeed(){

	//generate file seed from system clock
    clock_t seedtime;
    seedtime = clock();
    fileseed  = std::to_string(seedtime);

	//main results file, contains fitness, and torque remaining of best at each generation
    std::ofstream fitfile;
    std::string filename = fileseed;
    filename += "_fitness.csv";
    fitfile.open(filename);
    fitfile << "generation," << "fittest," << "torque left\n";
    int fitnessessize = fitnesses.size();
    for(int i = 0; i < fitnessessize; i++){
        char buf1[50];
        snprintf(buf1, 50, "%f", (double)i);
        char buf2[50];
        snprintf(buf2, 50, "%f", fitnesses.at(i));
        char buf3[50];
        snprintf(buf3, 50, "%f", torquesleft.at(i));
        fitfile << buf1 << "," << buf2 << "," << buf3 << "\n";
    }
    fitfile.close();

	//bulk fitnessess file, fitness of every agent in the population at every generation
    std::ofstream bigfile;
    std::string filename3 = fileseed;
    filename3 += "_bigfitness.csv";
    bigfile.open(filename3);
    int bigfitnessessize = bigfitnesses.size();
    for(int i = 0; i < (bigfitnessessize/POPSIZE); i++){
        char buf1[50];
        snprintf(buf1, 50, "%f", (double)i);
        bigfile << buf1;
        for(int j = 0; j < POPSIZE; j++){
            char buf2[50];
            snprintf(buf2, 50, "%f", bigfitnesses.at((i*POPSIZE)+j));
            bigfile << "," << buf2;
        }
        bigfile << "\n";
    }
    bigfile.close();

	//winner genotype file, written in hexadecimal to avoid rounding inconsistencies on reload
    std::ofstream file;
    std::string filename4 = fileseed;
    filename4 += "_winner_geno";
    filename4 += ".csv";
    file.open(filename4);
    int genosize = winner->geno.size();
    for(int i = 0; i < genosize; i++){
        char buf[50];
        snprintf(buf, 50, "%a", winner->geno.at(i));
        file << buf << "\n";
    }
    file.close();

	//winner trajectory file
    std::ofstream trajfile;
    std::string trajfilename  = fileseed;
    trajfilename += "_trajectory";
    trajfilename += ".csv";
    trajfile.open(trajfilename);
    int trajsize = winner->trajectoryx.size();
    char tbuf3[50];
    snprintf(tbuf3, 50, "%i", WANTPOSX);
    trajfile << tbuf3 << ",";
    char tbuf4[50];
    snprintf(tbuf4, 50, "%i", WANTPOSY);
    trajfile << tbuf4 << "\n";
    for(int i = 0; i < trajsize; i++){
        char tbuf[50];
        snprintf(tbuf, 50, "%f", winner->trajectoryx.at(i));
        trajfile << tbuf << ",";
        char tbuf2[50];
        snprintf(tbuf2, 50, "%f", winner->trajectoryy.at(i));
        trajfile << tbuf2 << "\n";
    }
    trajfile.close();

	//print seed
    std::cout << "SEED FOR THIS RUN: " << fileseed << "\n";
}

//load genotype as a vector
std::vector<double> loadGenofromRunSeed(){
    std::vector<double> loaded;
    std::string line2;
    std::string filename2;
    std::string seedstring;
    std::cout << "ENTER GENO SEED\n";
    std::cin >> seedstring;
    filename2 += seedstring;
    filename2 += "_winner_geno.csv";
    std::ifstream myfile(filename2);
    if (myfile.is_open())
    {
      while ( std::getline (myfile,line2) )
      {
        loaded.push_back(stod(line2));
      }
      myfile.close();
    }
    else std::cout << "Unable to open file\n";

    return loaded;

}

//main method
int main (int argc, char *argv[])
{

    //seed rng with system time
    srand(time(NULL));

    //START ODE
    dInitODE2(0);

    //biasing initial mutation rates together around random centre
    double mutcentre;
    mutcentre = (double)((rand() % 11)-5)/100;
    mutcentre = fabs(mutcentre);
    std::random_device mutrd;
    std::normal_distribution<double> mutnd(mutcentre,0.01);


    //init walkers
    for(int i = 0; i < POPSIZE; i++){
		
	//initialising walker genotype
        std::vector<double> initgeno;


        for(int j = 0; j < CONTCOUNT*INPUTS; j++){
            double weightnum = j % INPUTS;
            if(weightnum == 16){
				//if its a mutation rate input use random centre or just a random value
                if(CENTBIAS){
                    initgeno.push_back(mutnd(mutrd));
                }else{
                    initgeno.push_back((double)((rand() % 11)-5)/100);
                }
            }else{
				//other inputs randomised between -0.05 and 0.05
                initgeno.push_back((double)((rand() % 11)-5)/100);
            }
        }

        //add to array of walkers
        walkers.push_back(new Walker(i, initgeno));
    }

    if(LOADER){
    }else{
        //run ga and save files if not loading
        geneticAlg();
        writeFilesSetSeed();
    }

    //load winner if loading
    std::vector<double> wingeno;
    std::vector<double> winbod;
    if(!LOADER){
        wingeno = winvector;
    }else{
        wingeno = loadGenofromRunSeed();
    }

    //resim and draw winning walker
    winner = new Walker(0, wingeno);
    winner->iswinner = true;
    winner->init();
    if(!DISPLAY){
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
		//visualise walker
        dsSimulationLoop (argc,argv,500,500,&fn);
    }


    //CLOSE ODE
    dCloseODE();


    return 0;
}