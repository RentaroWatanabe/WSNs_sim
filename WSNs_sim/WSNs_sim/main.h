//
//  main.h
//  WSNs_sim
//
//  Created by Rentaro Watanabe on 2016/08/30.
//  Copyright © 2016年 Rentaro Watanabe. All rights reserved.
//


#ifndef main_h
#define main_h


//CPP header
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <algorithm>


//C header


//ORIGINAL header
#include "Node.h"
//#include "Base.h"


// using std namespace for CPP
using namespace std;



/*************************************************
 Experimental Parameters
 *************************************************/
#define N 200        // Number of Nodes
#define ROUND 10000     // Number of Total Rounds
#define MAX_E 2.0   // Maximum Energy of nodes
#define AREA_W 100.0  // Area Width
#define AREA_D 100.0  // Area Depth
int R_C;   // Communication Range (Radius)
#define R_S 10.0   // Sensing Range (Radius)
#define L 4000     // Bit-long of Message
#define BS 1    // Number of Base Station
#define TRGPTN 222  // Trigger occuring patter
                    //111 means UNIFORM
                    //222 means FIXED
int ALG;    // 123:Random alg 3:Shortest Path alg
                    // 2:Improved Random alg
			// 0:Possible Shortest Path alg
			// 1:More Improved Random alg
#define ALG_NUM 4
#define FNC 001    // Probability Functioin
                        // 001:i, 002:log(i+1), 003:sqrt(i), 004:tmp


// Set Coefficient Number
// Dont Set 0 !!
double CO_BEGIN = 1.0;
double CO_END = 3.0;
double CO_INTERVAL = 0.10;
double CO = CO_BEGIN;  

#define RUN 100    // Experimental Running Times

double DL_BEGIN = 0.01;
double DL_END = 0.8;
double DL_INTERVAL = 0.01;
double DL = DL_BEGIN;  // Deadline Rounds of Single Running


int INTENCIVE;  // Number of Rounds Messages Generate at Fixed Location (where TRGPTN = 222)
int NO;		// Input Running Number from "parameter.txt"

// Parameters of Energy Consumption Model
#define E_ELEC 5.0e-8
#define EPS_FS 1.0e-11
#define EPS_MP 1.3e-15


// Debugging Variable
#define DbgMode 0    //
	// 2 means outputting Log File
#define ViewPath 0


// Open File of Parameters
ifstream infile("parameters.txt");
// Open Output File
ofstream outfile("result.txt");

// Input Variable about using status
// 1 means Use Straged Status;
// 0 means Reset and Generate Status;
bool RESERVING_STATUS;



/******** End of Experimental Parameters *******/



// Global Variables
array <pair<double, double>, N+BS> Location;     // Positions of Nodes
int NN[N+BS][N+BS];      // Conneciton of Nodes
bool Dead[N+BS];  // 1 means Energy of the node is Empty
Node node[N+BS];
//Base base[BS];
int CountR;
int TotalMsg;   // Number of Total Messages
int LostTrg;    // Number of Lost Messages
int MaxDst;     // Max Depth of Graph
double P[N+BS-1][3];
	//P[i][0]:probability of sending upper layer
	//P[i][1]:probability of sending same layer
	//P[i][2]:probability of sending downer layer

unsigned int Seed;  // For Debug
bool Term;  // Termination Flag
int DeadCounta;
int Rpt = 0;
//#define Sim_Type_Num (((ALG_NUM - 2) + (((int)((CO_END - CO_BEGIN)*CO_INTERVAL) + 1)*2))*((int)((DL_END - DL_BEGIN)*DL_INTERVAL) + 1))
#define Sim_Type_Num 3520
int OP_R[Sim_Type_Num]; // Round Count
int OP_TMSG[Sim_Type_Num];	// Total MSG Count
int OP_LMSG[Sim_Type_Num];	// Lost MSG Count
int OP_FWD[Sim_Type_Num];	// Fowarding Count
int OP_PS[Sim_Type_Num];	// Routing Pass Count
// pair<double, double> Trg_Point;
int Fixed_Sender;
int Trg_Count;  // Times Messages Occured at Same Location
int Suc_Fwd;	// Number of Messages Fowarded to BS
int Pass_Long;	// Routing Pass Longth
int Pass_Long_tmp; // tmp variable for counting
vector<int> Reserved_Sender{-1};
int rs; //index of Reserved_Sender
int sim; //simulated running number(distingish CO or DL)
		// 0 ~ (Sim_Type_Num-1)

// for outputing terminated time
char buff[] = "";
time_t now;
struct tm *pnow;

#endif /* main_h */
