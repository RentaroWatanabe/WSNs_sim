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
#define N 200       // Number of Nodes
#define ROUND 10000     // Number of Total Rounds
#define MAX_E 2.0   // Maximum Energy of nodes
#define AREA_W 100.0  // Area Width
#define AREA_D 100.0  // Area Depth
#define R_C 20.0   // Communication Range (Radius)
#define R_S 10.0   // Sensing Range (Radius)
#define L 4000     // Bit-long of Message
#define BS 1    // Number of Base Station
#define TRGPTN 222  // Trigger occuring patter
                    //111 means UNIFORM
                    //222 means FIXED
int ALG;    // 123:Random alg 456:Shortest Path alg
                    // 789:Improved Random alg
int FNC;    // Probability Functioin
                        // 001:i, 002:log(i+1), 003:sqrt(i), 004:tmp
double CO;  // Set Coefficient Number
                  // Dont Set 0 !!
#define RUN 200    // Experimental Running Times
#define DL 0.2  // Deadline Rounds of Single Running
int INTENCIVE;  // Number of Rounds Messages Generate at Fixed Location (where TRGPTN = 222)
int NO;		// Input Running Number from "parameter.txt"

// Parameters of Energy Consumption Model
#define E_ELEC 5.0e-8
#define EPS_FS 1.0e-11
#define EPS_MP 1.3e-15


// Input Variable
//int ALG;
//int FNC;


// Debugging Variable
#define DbgMode 0    //
	// 2 means outputting Log File
#define ViewPath 0


// Open File of Parameters
ifstream infile("./parameters.txt");
// Open Output File
ofstream outfile("./result.txt");



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
int Rpt;
int OP_R, OP_TMSG, OP_LMSG, OP_FWD;
// pair<double, double> Trg_Point;
int Fixed_Sender;
int Trg_Count;  // Times Messages Occured at Same Location
int Suc_Fwd;	// Number of Messages Fowarded to BS




#endif /* main_h */
