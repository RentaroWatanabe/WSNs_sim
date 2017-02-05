//
//  main.cpp
//  WSNs_sim
//
//  Created by Rentaro Watanabe on 2016/08/30.
//  Copyright © 2016年 Rentaro Watanabe. All rights reserved.
//


#include "main.h"


//
// node[0~(BS-1)] : Base Station
// node[BS~(N+BS-1)] : Sensor Node
//


void InitNode(int i){
	node[i].id = i;

	// Set Locationdfgdg


	if (i < BS){
		node[i].location.first = AREA_W / 2.0;
		node[i].location.second = AREA_D / 2.0;
		Location[i] = node[i].location;
		node[i].isbs = 1;
		node[i].dst = 0;
	} 
	else{
		if (TOPOLOGY == 0){ //Random Topology Mode
			node[i].location.first = fmod(rand(), AREA_W * 100) / 100;
			node[i].location.second = fmod(rand(), AREA_D * 100) / 100;
		}
		else if (TOPOLOGY == 1){    //Grid Topology Mode
			// TODO
			double w_iv = (double)(int)(AREA_W / sqrtN + 0.5);
			double d_iv = (double)(int)(AREA_D / sqrtN + 0.5);
			node[i].location.first = ((i - BS) % sqrtN) * w_iv;
			node[i].location.second = ((i - BS) / sqrtN) * d_iv;
		}
		Location[i] = node[i].location;
		node[i].isbs = 0;
		node[i].resE = MAX_E;
		node[i].dst = -1;
	}
}



void InitVar(){
	MaxDst = 0;
	Fixed_Sender = -1;
	Trg_Count = 1;
	sim = 0;
	//Reserved_Sender.erase(Reserved_Sender.begin(), Reserved_Sender.end());
	Reserved_Sender.clear();
	//vector<int> (Reserved_Sender).swap(Reserved_Sender);
	for (int i = 0; i<N + BS; i++){
		node[i].ResetVar();
		Dead[i] = 0;

		for (int j = 0; j<(N + BS); j++){
			NN[i][j] = -1;
		}
	}
}



void Reset(){
	Fixed_Sender = -1;
	Trg_Count = 1;
	sim = 0;
	DeadCounta = 0;
	//Reserved_Sender.erase(Reserved_Sender.begin(), Reserved_Sender.end());
	Reserved_Sender.clear();
	//vector<int> (Reserved_Sender).swap(Reserved_Sender);

	for (int i = 0; i< N + BS; i++){
		node[i].resE = MAX_E;
		Dead[i] = 0;
	}
}



bool IsNeighbor(int ida, int idb){  // return whether given 2 nodes are neigbor each other
	pair<double, double> loca = node[ida].location;
	pair<double, double> locb = node[idb].location;
	if (pow((loca.first - locb.first), 2.0) + pow((loca.second - locb.second), 2.0) <= pow(R_C, 2.0))
		return true;
	else return false;
}




double GetRandom(double min, double max){   // Get Random Number of Specified Range
	double p = min + (rand()*(max - min) / (RAND_MAX));
	if (p>max || p<min)
		cout << "Coution." << endl;
	return p;
}




double Cutback(double p){
	if (0.0 <= p)
		if (p <= 1.0)
			return p;
		else return 1.0;
	else return 0.0;
}









void CreateGraph(){     // Set NN  -1:Self Node 0:DISconnected 1:Connected

	if (TOPOLOGY == 1){
		for (int i = BS; i<(N + BS); i++){
			for (int bs = 0; bs < BS; bs++){   // calc BS's neigbor
				NN[bs][i] = IsNeighbor(bs, i);
				NN[i][bs] = NN[bs][i];
				if (NN[bs][i] == 1){
					node[bs].neighbor.push_back(i);
					node[i].neighbor.push_back(bs);
				}
			}

			for (int r = i - sqrtN * R_C; r <= i + sqrtN * R_C; r += sqrtN){
				for (int c = r - R_C; c <= r + R_C; c++){
					if (c >= BS  && c < (N + BS) && i != c){
						if (NN[i][c] < 0){     // Not Calculated
							NN[i][c] = IsNeighbor(i, c);   // Input 1 or 0
							NN[c][i] = NN[i][c];
							if (NN[i][c] == 1){
								node[i].neighbor.push_back(c);
								node[c].neighbor.push_back(i);
							}
						}
					}
				}
			}
		}
	}
}





void SetDst(int staying, int pop){
	vector<int> to_visit;

	to_visit.insert(to_visit.end(), node[staying].neighbor.begin(), node[staying].neighbor.end());
	pop++;
	while (!to_visit.empty()){
		staying = to_visit.back();
		if ((node[staying].dst < 0) || (node[staying].dst > pop)){
			node[staying].dst = pop;
			SetDst(staying, pop);
		}
		to_visit.pop_back();
	}
}



void SetMaxDst(){
	if (TOPOLOGY == 1){
		MaxDst = node[BS].dst;
	}
	else {
		for (int i = 0; i < N + BS; i++){
			if (node[i].dst > MaxDst)
				MaxDst = node[i].dst;
		}
	}
}



void SetProbablity(){
	if (ALG == 3 || ALG == 2 || ALG == 0 || ALG == 1
		){    // Random alg
		if (FNC == 001){  // Expected Value : CO*i
			for (int i = BS + 1; i <= MaxDst; i++){
				P[i][0] = GetRandom(1 / CO, (CO + 1) / (2 * CO));
				P[i][1] = 1 + 1 / CO - 2 * P[i][0];
				P[i][2] = P[i][0] - 1 / CO;
			}
		}
		else
			if (FNC == 002){  // Expected Value : CO*log(i+1)
				for (int i = BS + 1; i <= MaxDst; i++){
					int dst_i = node[i].dst;
					P[i][0] = GetRandom(0.0, max(0.0, (CO*log(dst_i + 1) - CO*log(dst_i) - 1) / (CO*log(dst_i + 2) - CO*log(dst_i))));
					P[i][1] = (CO*P[i][0] * log(dst_i) - CO*P[dst_i][0] * log(dst_i + 2) - CO*log(dst_i) + CO*log(dst_i + 1) - 1) / (CO*log(dst_i + 1) - CO*log(dst_i));
					P[i][2] = 1 - P[i][0] - P[i][1];
				}
			}
			else if (FNC == 003){  // Expected Value : tekito
				for (int i = BS + 1; i <= MaxDst; i++){
					P[i][0] = 0.9;
					P[i][1] = 0.05;
					P[i][2] = 0.05;
				}
			}
			else if (FNC == 004){
				for (int i = BS + 1; i <= MaxDst; i++){
					P[i][0] = 1.0;
					P[i][1] = 0.0;
					P[i][2] = 0.0;
				}
			}
	}

}



int GetNextDst(int sender){
	int tmp_dst = node[sender].dst;

	if (tmp_dst < 0)
		return -1;
	else if (tmp_dst == 1)
		return 0;
	else if (node[sender].dst != 1){
		vector<int>::iterator i;
		vector<int> can_u, can_s, can_l;
		bool exist_up = 0;
		bool exist_same = 0;
		bool exist_low = 0;
		double p_up = 0;
		double p_same = 0;
		double p_low = 0;


		for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++){
			if (node[*i].dst == (node[sender].dst - 1) && !Dead[*i])
				can_u.push_back(*i);
			else if (node[*i].dst == (node[sender].dst) && !Dead[*i])
				can_s.push_back(*i);
			else if (node[*i].dst == (node[sender].dst + 1) && !Dead[*i])
				can_l.push_back(*i);
		}

		if (!can_u.empty())
			exist_up = 1;
		if (!can_s.empty())
			exist_same = 1;
		if (!can_l.empty())
			exist_low = 1;

		if (!exist_up && !exist_same && !exist_low)
			return -1;

		if (ALG == 3 || ALG == 0){
			p_low = 0;
			p_same = 0;
			p_up = 1 * exist_up;
		}

		else if (ALG == 2 || ALG == 1){
			p_low = P[tmp_dst][2] * (1.0 / ((P[tmp_dst][0] * exist_up) + (P[tmp_dst][1] * exist_same) + (P[tmp_dst][2] * exist_low))) * exist_low;
			p_same = P[tmp_dst][1] * (1.0 / ((P[tmp_dst][0] * exist_up) + (P[tmp_dst][1] * exist_same) + (P[tmp_dst][2] * exist_low))) * exist_same;
			p_up = (1 - p_low - p_same) * exist_up;
		}


		//when upper layer nodes are all dead, alg 0 selects other node
		if (ALG == 0 &&
			!exist_up &&
			(exist_same || exist_low)){
			if (exist_same)
				return can_s[rand() % can_s.size()];
			else return can_l[rand() % can_l.size()];
		}


		double tmp_p = (double)rand() / RAND_MAX;


		//Receiver select part for 1
		if (ALG == 1){
			double maxE = 0;
			int tmp_receiver = -1;

			if (tmp_p < p_up){
				for (i = can_u.begin(); i != can_u.end(); i++){
					if (node[*i].resE > maxE)
						tmp_receiver = *i;
				}
			}
			else
				if (tmp_p < p_up + p_same){
					for (i = can_s.begin(); i != can_s.end(); i++){
						if (node[*i].resE > maxE)
							tmp_receiver = *i;
					}
				}

				else if (tmp_p < p_up + p_same + p_low){
					for (i = can_l.begin(); i != can_l.end(); i++){
						if (node[*i].resE > maxE)
							tmp_receiver = *i;
					}
				}
				else return -1;
				return tmp_receiver;
		}


		//Receiver select part for 3,0,2
		if (tmp_p < p_up){
			return can_u[rand() % can_u.size()];
		}
		else
			if (tmp_p < p_up + p_same){
				return can_s[rand() % can_s.size()];
			}

			else if (tmp_p < p_up + p_same + p_low){
				return can_l[rand() % can_l.size()];
			}
			else return -1;
	}
	else return -1;
}





void EnergyConsume(int sender, int receiver){
	// Calc Energy Consumption
	// Judge existece

	if (sender != 0){
		double sqrd;
		if (receiver == -1)
			sqrd = pow(R_C, 2.0);
		else
			// Sender Side
			sqrd = pow((node[sender].location.first - node[receiver].location.first), 2.0) + pow((node[sender].location.second - node[receiver].location.second), 2.0);
		double csm = (L * E_ELEC) + (L * EPS_FS * pow(sqrd, 2.0));
		node[sender].resE -= csm;

		//debug
		if (NO == monNO && ALG == monALG)
			logfile << sender << "(s)\t-" << csm << "\t> " << node[sender].resE << "\t\t(" << NO << "," << ALG << "," << CountR << ")" << endl;

		if (node[sender].resE < 0){
			Dead[sender] = 1;
			if (NO == monNO && ALG == monALG)
				logfile << sender << " is dead." << endl;
		}
	}
	if (receiver > 0){
		// Receiver Side
		node[receiver].resE -= L * E_ELEC;

		//debug
		if (NO == monNO && ALG == monALG)
			logfile << receiver << "(r)\t-" << L * E_ELEC << "\t> " << node[receiver].resE << "\t\t(" << NO << "," << ALG << "," << CountR << ")" << endl;


		if (node[receiver].resE < 0){
			Dead[receiver] = 1;
			if (NO == monNO && ALG == monALG)
				logfile << receiver << " is dead." << endl;
		}
	}

}





void ForwardMsg(int sender){
	int receiver = sender;

	if (sender == -1){
		LostTrg++;
		if (ViewPath == 1)
			cout << " L" << endl;
		if (DbgMode == 2){
			outfile << endl;
			outfile << "Message Lost. [" << CountR << "]" << endl;
		}
		if (monNO == -1){
			logfile << "LOST in " << Fixed_Sender << endl;
		}
	}
	else
		if ((sender != 0) && (Dead[sender]))  // Dead Node
		{
			LostTrg++;
			if (ViewPath == 1)
				cout << " L" << endl;
			if (DbgMode == 2){
				outfile << endl;
				outfile << "Message Lost. [" << CountR << "]" << endl;
			}
			if (monNO == -1){
				logfile << "LOST in " << Fixed_Sender << endl;
			}

		}
		else if (node[sender].dst < 0){     // Disconnected Node
			LostTrg++;
			if (ViewPath == 1)
				cout << " L" << endl;
			if (DbgMode == 2){
				outfile << endl;
				outfile << "Message Lost. [" << CountR << "]" << endl;
			}
			if (monNO == -1){
				logfile << "LOST in " << Fixed_Sender << endl;
			}

			//EnergyConsume(sender, -1);
		}
		else {   //Connected Node
			Pass_Long_tmp = 0;
			while (receiver != 0){
				sender = receiver;
				receiver = GetNextDst(sender);

				if (receiver == -1){    // All the Neighbors Are Already Dead
					LostTrg++;
					//EnergyConsume(sender, receiver);
					if (ViewPath == 1)
						cout << " L" << endl;
					if (DbgMode == 2){
						outfile << endl;
						outfile << "Message Lost. [" << CountR << "]" << endl;
					}
					if (monNO == -1){
						logfile << "LOST in " << Fixed_Sender << endl;
					}

					return;
				}
				if (ViewPath == 1)
					cout << " .";
				if (DbgMode == 2){
					outfile << " " << receiver;
				}
				Pass_Long_tmp++;
				TotalMsg++;
				EnergyConsume(sender, receiver);
				if (Dead[receiver] || Dead[sender]){    // Dead Node sent or received
					LostTrg++;
					if (ViewPath == 1)
						cout << " L" << endl;
					if (DbgMode == 2){
						outfile << endl;
						outfile << "Message Lost. [" << CountR << "]" << endl;
					}
					if (monNO == -1){
						logfile << "LOST in " << Fixed_Sender << endl;
					}

					return;
				}
			}
			Suc_Fwd++;
			Pass_Long += Pass_Long_tmp;
			Pass_Long_tmp = 0;
			if (receiver == 0 && ViewPath == 1)
				cout << " B" << endl;
			if (DbgMode == 2){
				outfile << endl;
				outfile << "BS received message. [" << CountR << "]" << endl;
			}
			if (monNO == -1){
				logfile << "SUCCESSED in " << Fixed_Sender << endl;
			}

		}
}



//return whether the neighbors of BS are still exist at least one
bool ConnectToBS(int bs){
	vector<int>::iterator i;
	for (i = node[bs].neighbor.begin(); i != node[bs].neighbor.end(); i++){
		if (!Dead[*i])
			return true;
	}
	return false;
}



void CountDeadNode(){
	int dead_tmp = 0;
	for (int i = 0; i < N + BS; i++){
		if (Dead[i]) dead_tmp++;
	}
	DeadCounta = dead_tmp;
}



void OutPutNS(int r){
	double TotalE = 0;
	for (int i = BS; i < N + BS; i++)
		TotalE += node[i].resE;

	if (ALG == 0){
		if (OP_NS_0.size() < r){
			tuple<int, double, int> empty;
			get<0>(empty) = 0;
			get<1>(empty) = 0.0;
            get<2>(empty) = 0;

			OP_NS_0.push_back(empty);
		}
		get<0>(OP_NS_0[r - 1]) += (N - DeadCounta);
		get<1>(OP_NS_0[r - 1]) += TotalE;
        get<2>(OP_NS_0[r - 1])++;

	}
	else if (ALG == 1){
		if (OP_NS_1.size() < r){
            tuple<int, double, int> empty;
            get<0>(empty) = 0;
            get<1>(empty) = 0.0;
            get<2>(empty) = 0;
            
            OP_NS_1.push_back(empty);
        }
        get<0>(OP_NS_1[r - 1]) += (N - DeadCounta);
        get<1>(OP_NS_1[r - 1]) += TotalE;
        get<2>(OP_NS_1[r - 1])++;
    }
	else if (ALG == 2){
		if (OP_NS_2.size() < r){
            tuple<int, double, int> empty;
            get<0>(empty) = 0;
            get<1>(empty) = 0.0;
            get<2>(empty) = 0;
            
            OP_NS_2.push_back(empty);
        }
        get<0>(OP_NS_2[r - 1]) += (N - DeadCounta);
        get<1>(OP_NS_2[r - 1]) += TotalE;
        get<2>(OP_NS_2[r - 1])++;
    }
	else{
		if (OP_NS_3.size() < r){
			tuple<int, double, int> empty;
			get<0>(empty) = 0;
			get<1>(empty) = 0.0;
			get<2>(empty) = 0;

			OP_NS_3.push_back(empty);
		}
		get<0>(OP_NS_3[r - 1]) += (N - DeadCounta);
		get<1>(OP_NS_3[r - 1]) += TotalE;
		get<2>(OP_NS_3[r - 1])++;
	}
}


int main() {
	Seed = (unsigned int)time(0);
	//Seed = 1485332344;
	srand(Seed);

	if (infile.fail()) {
		cerr << "Infile do not exist.\n";
		exit(0);
	}

	logfile << "Seed:" << Seed << endl << endl;

	if (DbgMode == 2)
		outfile << "Seed : " << Seed << endl;

	if (DbgMode == 1)
		cout << "Simutation Start." << endl;

	while (infile >> NO >> R_C >> INTENCIVE)
	{

		/*if (DbgMode == 2){
		outfile << "************************************" << endl;
		outfile << "Sequence Num :" << NO << endl;
		outfile << "#node : " << N << endl;
		outfile << "ALG : " << ALG << endl;
		outfile << "FNC : " << FNC << endl;
		outfile << "Coefficient : " << CO << endl;
		outfile << "Intencive : " << INTENCIVE << endl;
		outfile << "************************************" << endl;
		outfile << endl;
		}*/


		InitVar();
		if (DbgMode == 1)
			cout << "Variable Initiation Completed." << endl;

		for (int i = 0; i < (N + BS); i++)
			InitNode(i);
		if (DbgMode == 1)
			cout << "Node Initiation Completed." << endl;

		CreateGraph();
		if (DbgMode == 1)
			cout << "New Graph Created." << endl;
		SetDst(0, 0);
		SetMaxDst();
		if (DbgMode == 1)
			cout << "Depth of Each Node Caliculated." << endl;

		ALG = 0;
		while (Rpt < RUN){
			//do {
			while (DL_END + DL_INTERVAL - DL > 0.000001){
				do {

					////// DEBUG SPACE ///////////////


					/////////////////////

					if (ALG == 0 && DL == DL_BEGIN){
						if (DbgMode == 1)
							cout << Rpt << "th Running" << endl;

						if (DbgMode == 2)
							outfile << "***** " << Rpt << "(/100)th Running *****" << endl;

						Reset();   // Reset variables
						if (DbgMode == 1)
							cout << "Reset Variable Completed." << endl;
					}
					else {
						for (int i = 0; i < (N + BS); i++){
							node[i].resE = MAX_E;
							Dead[i] = false;
						}
						Fixed_Sender = -1;
						Trg_Count = 1;
						DeadCounta = 0;
					}

					rs = -1;
					Term = false;
					CountR = 0;

					SetProbablity();
					if (DbgMode == 1)
						cout << "Forwarding Probabilities Have Been Set." << endl;

					//if (DbgMode == 2){	//output dependent relationship
					//	outfile << endl;
					//	outfile << " --- Dependent Relationship --- " << endl;
					//		for (int i = 0; i < N + BS; i++){
					//			outfile << i << "(" << node[i].dst <<") :";
					//			for (auto itr = node[i].neighbor.begin(); itr != node[i].neighbor.end(); itr++) {
					//				outfile << " " << *itr;
					//			}
					//			outfile << endl;
					//		}
					//	outfile << " ------------------------------ " << endl;
					//	outfile << endl;
					//}



					while (!Term){     // Main Loop
						pair<double, double> trg_location;
						
						// DEBUG PROCESS
						//for (int i = 0; i <= BS; i++)
						//	if (!ConnectToBS(i))
						//		cout << "alert!!!!" << endl;

						if (TRGPTN == 111){   // AppMSG Occurs Uniformly
							trg_location.first = fmod(rand(), AREA_W);
							trg_location.second = fmod(rand(), AREA_D);

							//            else
							//                if (TRGPTN == 222){ // Messages are Generated at Fixed Point
							//                    trg_location.first = Trg_Point.first;
							//                    trg_location.second = Trg_Point.second;
							//                }


							int can_sender = -1;
							double min_dist = pow(R_S, 2.0);

							// Search App MSG Sender
							for (int i = 0; i<(N + BS); i++){

								if (node[i].location.first >(trg_location.first - R_S)
									&& node[i].location.first < (trg_location.first + R_S)
									&& node[i].location.second >(trg_location.second - R_S)
									&& node[i].location.second < (trg_location.second + R_S)){
									double tmp_dist = pow((node[i].location.first - trg_location.first), 2.0) + pow((node[i].location.second - trg_location.second), 2.0);
									if (min_dist >= tmp_dist){
										min_dist = tmp_dist;
										can_sender = i;
									}
								}
							}


							if (can_sender != -1)
								// Begin MSG Forwarding
								ForwardMsg(can_sender);

						}
						else
							if (TRGPTN == 222){

								CountR++;
								if (Fixed_Sender == -1 || // initiated status
									// Dead[Fixed_Sender] ||
									Trg_Count > INTENCIVE){ // Change Fixed Sender
									Trg_Count = 1;
									rs++;
									if ((rs + 1) > (int)Reserved_Sender.size()){
										Reserved_Sender.push_back((rand() % N) + BS);
									}
									Fixed_Sender = Reserved_Sender[rs];
									if (DbgMode == 2){
										outfile << endl;
										outfile << "New trigger occured at (" << Fixed_Sender << ")." << endl;
									}
								}
								if (DbgMode == 2)
									outfile << Fixed_Sender;
								ForwardMsg(Fixed_Sender);
								Trg_Count++;
							}

						CountDeadNode();
						if (monNS == 1 && CountR % step == 0)	//monitoring Network Status Mode
							OutPutNS(CountR / step);

						if (
							//CountR > 100 &&
							//                (double)DeadCounta/(double)(N+BS) > 0.7
							(double)Suc_Fwd / ((double)Suc_Fwd + (double)LostTrg) < DL
							){
							Term = true;
							if (NO == monNO && ALG == monALG){
								logfile << "Dead : " << DeadCounta << endl;
								double totalR = 0;
								for (int i = BS; i < N + BS; i++){
									totalR += node[i].resE;
								}
								logfile << "Total Residual Energy : " << totalR << endl;
							}

							if (monNO == -1){
								logfile << endl << endl << endl << endl << endl;
								logfile << "---------------------" << endl;
								logfile << endl << endl << endl << endl << endl;
							}
						}
						if (DbgMode == 1){
							cout << "Dead Rate(%) = " << (double)DeadCounta / (double)N * 100 << endl;
							if (Term)
								cout << "Running Terminated." << endl;
							else
								cout << "Running Continued." << endl;
						}
					}

					OP_R[sim] += CountR - 1;
					OP_TMSG[sim] += TotalMsg;
					OP_LMSG[sim] += LostTrg;
					OP_FWD[sim] += Suc_Fwd;
					OP_PS[sim] += Pass_Long;

					CountR = 0;
					TotalMsg = 0;
					LostTrg = 0;
					Suc_Fwd = 0;
					Pass_Long = 0;

					sim++;
					if (sim == Sim_Type_Num)
						//if (ALG = ALG_NUM - 1)
					{
						if (ALG == 3)
							Rpt++;
						else{
							cout << endl << "ALERT!!" << endl;
							outfile << endl << "ALERT!!" << endl;
						}
					}

					CO += CO_INTERVAL;
				} while ((ALG == 1 || ALG == 2) && (CO_END + CO_INTERVAL - CO > 0.000001));
				CO = CO_BEGIN;

				/*} while ((ALG == 1 || ALG == 2) && (DL <= DL_END));*/

				if (ALG == ALG_NUM - 1)
					DL += DL_INTERVAL;
				ALG = (ALG + 1) % ALG_NUM;
			}
			DL = DL_BEGIN;

		}   // End Main Loop

		//cout << "-----" << "\n";
		//cout << "Result N." << Rpt << endl;
		//cout << "Round : " << CountR << "\n";
		//cout << "Forwarding Msg : " << TotalMsg << "\n";
		//cout << "Lost Msg : " << LostTrg << "\n";
		//cout << "\n";


		sim = 0;
		ALG = 0;
		now = time(NULL);
		tm *pnow = localtime(&now);
		sprintf(buff, "%04d/%02d/%02d %02d:%02d:%02d", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday,
			pnow->tm_hour, pnow->tm_min, pnow->tm_sec);

		for (double tmp_dl = DL_BEGIN; DL_END - tmp_dl > -0.001; tmp_dl += DL_INTERVAL){

			//ALG == 0
			cout << endl;
			cout << "----- Total Result -----" << endl;
			cout << "Algorithm ID : " << ALG << endl;
			cout << "Average Round : " << (double)OP_R[sim] / RUN << endl;
			cout << "Average Forwarding Msg : " << (double)OP_TMSG[sim] / RUN << endl;
			cout << "Average Lost Msg : " << (double)OP_LMSG[sim] / RUN << endl;
			cout << "Terminated Time : " << buff << endl;
			cout << endl;

			outfile << NO << " " << tmp_dl << " " << ALG << " NA " << (double)OP_R[sim] / RUN
				<< " " << (double)OP_TMSG[sim] / RUN << " " << (double)OP_LMSG[sim] / RUN
				<< " " << (double)OP_FWD[sim] / RUN << " " << (double)OP_PS[sim] / OP_FWD[sim] << endl;

			OP_R[sim] = 0;
			OP_TMSG[sim] = 0;
			OP_LMSG[sim] = 0;
			OP_FWD[sim] = 0;
			OP_PS[sim] = 0;
			sim++;
			ALG = (ALG + 1) % ALG_NUM;

			//ALG == 1
			for (double tmp_co = CO_BEGIN; CO_END - tmp_co > -0.001; tmp_co += CO_INTERVAL){
				cout << endl;
				cout << "----- Total Result -----" << endl;
				cout << "Algorithm ID : " << ALG << endl;
				cout << "Average Round : " << (double)OP_R[sim] / RUN << endl;
				cout << "Average Forwarding Msg : " << (double)OP_TMSG[sim] / RUN << endl;
				cout << "Average Lost Msg : " << (double)OP_LMSG[sim] / RUN << endl;
				cout << "Terminated Time : " << buff << endl;
				cout << endl;

				outfile << NO << " " << tmp_dl << " " << ALG << " " << tmp_co << " " << (double)OP_R[sim] / RUN
					<< " " << (double)OP_TMSG[sim] / RUN << " " << (double)OP_LMSG[sim] / RUN
					<< " " << (double)OP_FWD[sim] / RUN << " " << (double)OP_PS[sim] / OP_FWD[sim] << endl;
				OP_R[sim] = 0;
				OP_TMSG[sim] = 0;
				OP_LMSG[sim] = 0;
				OP_FWD[sim] = 0;
				OP_PS[sim] = 0;
				sim++;
			}
			ALG = (ALG + 1) % ALG_NUM;

			//ALG == 2
			for (double tmp_co = CO_BEGIN; CO_END - tmp_co > -0.001; tmp_co += CO_INTERVAL){
				cout << endl;
				cout << "----- Total Result -----" << endl;
				cout << "Algorithm ID : " << ALG << endl;
				cout << "Average Round : " << (double)OP_R[sim] / RUN << endl;
				cout << "Average Forwarding Msg : " << (double)OP_TMSG[sim] / RUN << endl;
				cout << "Average Lost Msg : " << (double)OP_LMSG[sim] / RUN << endl;
				cout << "Terminated Time : " << buff << endl;
				cout << endl;

				outfile << NO << " " << tmp_dl << " " << ALG << " " << tmp_co << " " << (double)OP_R[sim] / RUN
					<< " " << (double)OP_TMSG[sim] / RUN << " " << (double)OP_LMSG[sim] / RUN
					<< " " << (double)OP_FWD[sim] / RUN << " " << (double)OP_PS[sim] / OP_FWD[sim] << endl;
				OP_R[sim] = 0;
				OP_TMSG[sim] = 0;
				OP_LMSG[sim] = 0;
				OP_FWD[sim] = 0;
				OP_PS[sim] = 0;
				sim++;
			}
			ALG = (ALG + 1) % ALG_NUM;

			//ALG == 3
			cout << endl;
			cout << "----- Total Result -----" << endl;
			cout << "Algorithm ID : " << ALG << endl;
			cout << "Average Round : " << (double)OP_R[sim] / RUN << endl;
			cout << "Average Forwarding Msg : " << (double)OP_TMSG[sim] / RUN << endl;
			cout << "Average Lost Msg : " << (double)OP_LMSG[sim] / RUN << endl;
			cout << "Terminated Time : " << buff << endl;
			cout << endl;

			outfile << NO << " " << tmp_dl << " " << ALG << " NA " << (double)OP_R[sim] / RUN
				<< " " << (double)OP_TMSG[sim] / RUN << " " << (double)OP_LMSG[sim] / RUN
				<< " " << (double)OP_FWD[sim] / RUN << " " << (double)OP_PS[sim] / OP_FWD[sim] << endl;

			OP_R[sim] = 0;
			OP_TMSG[sim] = 0;
			OP_LMSG[sim] = 0;
			OP_FWD[sim] = 0;
			OP_PS[sim] = 0;
			sim++;
			ALG = (ALG + 1) % ALG_NUM;

		}

		if (sim != Sim_Type_Num){
			outfile << endl;
			outfile << "ALERT!!!" << endl;
			outfile << endl;
		}

		Rpt = 0;
		sim = 0;
		ALG = 0;

	} // End Repeat
	outfile << endl;
	outfile << Seed << endl;


	if (monNS == 1){	//output monitored network status data
		int index = 0;
		while (OP_NS_0.size() > index){
			fileNS << "0 " << (index + 1) * step << " " << (double)get<0>(OP_NS_0[index]) / get<2>(OP_NS_0[index]) << " " << get<1>(OP_NS_0[index]) / get<2>(OP_NS_0[index]) << endl;
			index++;
		}
		index = 0;
		while (OP_NS_1.size() > index){
            fileNS << "1 " << (index + 1) * step << " " << (double)get<0>(OP_NS_1[index]) / get<2>(OP_NS_1[index]) << " " << get<1>(OP_NS_1[index]) / get<2>(OP_NS_1[index]) << endl;
            index++;
        }
		index = 0;
		while (OP_NS_2.size() > index){
            fileNS << "2 " << (index + 1) * step << " " << (double)get<0>(OP_NS_2[index]) / get<2>(OP_NS_2[index]) << " " << get<1>(OP_NS_2[index]) / get<2>(OP_NS_2[index]) << endl;
            index++;
        }
		index = 0;
		while (OP_NS_3.size() > index){
            fileNS << "3 " << (index + 1) * step << " " << (double)get<0>(OP_NS_3[index]) / get<2>(OP_NS_3[index]) << " " << get<1>(OP_NS_3[index]) / get<2>(OP_NS_3[index]) << endl;
            index++;
        }
	}


	return 0;

}

