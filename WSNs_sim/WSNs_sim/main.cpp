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

	// Set Location
	

	if (i < BS){
		node[i].location.first = AREA_W/2.0;
		node[i].location.second = AREA_D/2.0;
		Location[i] = node[i].location;
		node[i].isbs = 1;
		node[i].dst = 0;
	}
	else{
		node[i].location.first = fmod(rand(), AREA_W * 100) / 100;
		node[i].location.second = fmod(rand(), AREA_D * 100) / 100;
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
	for (int i = 0; i<N + BS; i++){
		node[i].ResetVar();
		Dead[i] = 0;

		for (int j = 0; j<(N + BS); j++){
			NN[i][j] = -1;
		}
	}
}



bool IsNeighbor(int ida, int idb){  // return whether given 2 nodes are neigbor each other
	pair<int, int> loca = node[ida].location;
	pair<int, int> locb = node[idb].location;
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

	for (int i = 0; i<(N + BS); i++){
		for (int j = 0; j<(N + BS); j++){
			if (i == j);
			else if (NN[i][j] < 0){     // Not Calculated
				NN[i][j] = IsNeighbor(i, j);   // Input 1 or 0
				NN[j][i] = NN[i][j];
				if (NN[i][j] == 1){
					node[i].neighbor.push_back(j);
					node[j].neighbor.push_back(i);
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
	for (int i = 0; i < N + BS; i++){
		if (node[i].dst > MaxDst)
			MaxDst = node[i].dst;
	}
}



void SetProbablity(){
	if (ALG == 3 || ALG == 2 || ALG == 0 || ALG == 1
		){    // Random alg
		if (FNC == 001){  // Expected Value : CO*i
			for (int i = BS + 1; i<=MaxDst; i++){
				P[i][0] = GetRandom(1 / CO, (CO + 1) / (2 * CO));
				P[i][1] = 1 + 1 / CO - 2 * P[i][0];
				P[i][2] = P[i][0] - 1 / CO;
			}
		}
		else
			if (FNC == 002){  // Expected Value : CO*log(i+1)
				for (int i = BS + 1; i<=MaxDst; i++){
					int dst_i = node[i].dst;
					P[i][0] = GetRandom(0.0, max(0.0, (CO*log(dst_i + 1) - CO*log(dst_i) - 1) / (CO*log(dst_i + 2) - CO*log(dst_i))));
					P[i][1] = (CO*P[i][0] * log(dst_i) - CO*P[dst_i][0] * log(dst_i + 2) - CO*log(dst_i) + CO*log(dst_i + 1) - 1) / (CO*log(dst_i + 1) - CO*log(dst_i));
					P[i][2] = 1 - P[i][0] - P[i][1];
				}
			}
			else if (FNC == 003){  // Expected Value : tekito
				for (int i = BS + 1; i<=MaxDst; i++){
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


//int GetNextDst(int sender){
//	if (ALG == 123){
//		if (node[sender].dst != 1){
//			int tmp_dst = node[sender].dst;
//			vector<int> can_u, can_s, can_l;
//			vector<int>::iterator i;
//
//
//			for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++){
//				if (node[*i].dst == (node[sender].dst - 1) && !Dead[*i])
//					can_u.push_back(*i);
//				else if (node[*i].dst == (node[sender].dst) && !Dead[*i])
//					can_s.push_back(*i);
//				else if (node[*i].dst == (node[sender].dst + 1) && !Dead[*i])
//					can_l.push_back(*i);
//			}
//
//			if (!can_u.empty() && P[tmp_dst][0]>0){
//				if (!can_s.empty() && P[tmp_dst][1]>0){
//					if (!can_l.empty() && P[tmp_dst][2]>0){
//						double tmp_p = (double)rand() / RAND_MAX;
//
//						if (tmp_p < P[tmp_dst][0])
//							return can_u[rand() % can_u.size()];
//						else
//							if (tmp_p < P[tmp_dst][0] + P[tmp_dst][1])
//								return can_s[rand() % can_s.size()];
//							else
//								return can_l[rand() % can_l.size()];
//					}
//					else {
//						double tmp_p = (double)rand() / RAND_MAX * (P[tmp_dst][0] + P[tmp_dst][1]);
//
//						if (tmp_p < P[tmp_dst][0])
//							return can_u[rand() % can_u.size()];
//						else
//							if (tmp_p < P[tmp_dst][0] + P[tmp_dst][1])
//								return can_s[rand() % can_s.size()];
//							else
//								return -1;  // Something Wrong!
//					}
//				}
//				else if (!can_l.empty() && P[tmp_dst][2]>0){
//					double tmp_p = (double)rand() / RAND_MAX * (P[tmp_dst][0] + P[tmp_dst][2]);
//
//					if (tmp_p < P[tmp_dst][0])
//						return can_u[rand() % can_u.size()];
//					else
//						if (tmp_p < P[tmp_dst][0] + P[tmp_dst][2])
//							return can_l[rand() % can_l.size()];
//						else
//							return -1;  // Something Wrong!
//				}
//				else
//					return can_u[rand() % can_u.size()];
//			}
//			else
//				if (!can_s.empty() && P[tmp_dst][1]>0){
//					if (!can_l.empty() && P[tmp_dst][2]>0){
//
//						double tmp_p = (double)rand() / RAND_MAX * (P[tmp_dst][1] + P[tmp_dst][2]);
//
//						if (tmp_p < P[tmp_dst][1])
//							return can_s[rand() % can_s.size()];
//						else
//							if (tmp_p < P[tmp_dst][1] + P[tmp_dst][2])
//								return can_l[rand() % can_l.size()];
//							else
//								return -1;  // Something Wrong!
//					}
//					else
//						return can_s[rand() % can_s.size()];
//				}
//				else if (!can_l.empty() && P[tmp_dst][2]>0){
//					return can_l[rand() % can_l.size()];
//
//				}
//				else
//					return -1;  // No Neighbors Alive
//		}
//		else return 0;
//	}
//	else
//		if (ALG == 3){     // Shortest Path alg
//			vector<int> can_dst;
//			vector<int>::iterator i;
//
//			for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++){
//				if (node[*i].dst == (node[sender].dst - 1)
//					&& !Dead[*i])
//					can_dst.push_back(*i);
//			}
//			if (!can_dst.empty())   // All Candidate Nodes are Dead
//				return can_dst[rand() % can_dst.size()];
//			else return -1;
//		}
//		else
//			if (ALG == 2){    // Probability Computation will be done at First
//				if (node[sender].dst != 1){
//					int tmp_dst = node[sender].dst;
//					vector<int> can_dst;
//					vector<int>::iterator i;
//
//					double tmp_p = (double)rand() / RAND_MAX;
//
//					if (tmp_p < P[tmp_dst][0]){
//						for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++) {
//							if (node[*i].dst == (node[sender].dst - 1) && !Dead[*i])
//								can_dst.push_back(*i);
//						}
//					}
//					else
//						if (tmp_p < P[tmp_dst][0] + P[tmp_dst][1]){
//							for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++) {
//								if (node[*i].dst == (node[sender].dst) && !Dead[*i])
//									can_dst.push_back(*i);
//							}
//						}
//
//						else {
//							for (i = node[sender].neighbor.begin(); i != node[sender].neighbor.end(); i++){
//								if (node[*i].dst == (node[sender].dst + 1) && !Dead[*i])
//									can_dst.push_back(*i);
//							}
//						}
//
//						if (!can_dst.empty())
//							return can_dst[rand() % can_dst.size()];
//						else return -1;
//
//				}
//				else return 0;
//			}
//
//}


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

		/*if (ALG == 3){
			p_low = 0;
			p_same = 0;
			p_up = 1 * exist_up;
			}*/

		//else if (ALG == 2){
		p_low = P[tmp_dst][2] * (1.0 / ((P[tmp_dst][0] * exist_up) + (P[tmp_dst][1] * exist_same) + (P[tmp_dst][2] * exist_low))) * exist_low;
		p_same = P[tmp_dst][1] * (1.0 / ((P[tmp_dst][0] * exist_up) + (P[tmp_dst][1] * exist_same) + (P[tmp_dst][2] * exist_low))) * exist_same;
		p_up = (1 - p_low - p_same) * exist_up;
		//}


		//when upper layer nodes are all dead, alg 0 selects other node
		if (ALG == 0 &&
			!exist_up &&
			(exist_same || exist_low)){
			if (exist_same)
				return can_s[rand() % can_s.size()];
			else return can_l[rand() % can_l.size()];
		}


		double tmp_p = (double)rand() / RAND_MAX;


		//TODO : Receiver select part for 1
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
					for (i = can_l .begin(); i != can_l.end(); i++){
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

			else if (tmp_p < p_up + p_same+ p_low){
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
		if (node[sender].resE < 0)
			Dead[sender] = 1;
	}
	if (receiver > 0){
		// Receiver Side
		node[receiver].resE -= L * E_ELEC;
		if (node[receiver].resE < 0)
			Dead[receiver] = 1;
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
		}
		else if (node[sender].dst < 0){     // Disconnected Node
			LostTrg++;
			if (ViewPath == 1)
				cout << " L" << endl;
			if (DbgMode == 2){
				outfile << endl;
				outfile << "Message Lost. [" << CountR << "]" << endl;
			}
			//EnergyConsume(sender, -1);
		}
		else {   //Connected Node
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
					return;
				}
				if (ViewPath == 1)
					cout << " .";
				if (DbgMode == 2){
					outfile << " " << receiver;
				}
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
					return;
				}
			}
			Suc_Fwd++;
			if (receiver == 0 && ViewPath == 1)
				cout << " B" << endl;
			if (DbgMode == 2){
				outfile << endl;
				outfile << "BS received message. [" << CountR << "]" << endl;
			}
		}
}







int main() {
	Seed = (unsigned int)time(0);
	//Seed = 1477423634;
	srand(Seed);

    if(infile.fail()) {
        cerr << "Infile do not exist.\n";
        exit(0);
    }


	if (DbgMode == 2)
		outfile << "Seed : " << Seed << endl;

	if (DbgMode == 1)
		cout << "Simutation Start." << endl;

	while (infile >> NO >> R_C >> DL >> CO >> INTENCIVE)
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

		ALG = 0;
		while (Rpt < RUN){

			if (ALG == 0){
				if (DbgMode == 1)
					cout << Rpt << "th Running" << endl;

				if (DbgMode == 2)
					outfile << "***** " << Rpt << "(/100)th Running *****" << endl;

				InitVar();   // Initiation variables
				if (DbgMode == 1)
					cout << "Variable Initiation Completed." << endl;


				// INITIATION of Nodes and Base Station and CREATION Graph
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
			}
			else {
				for (int i = 0; i < (N + BS); i++){
					node[i].resE = MAX_E;
					Dead[i] = false;
				}
				Fixed_Sender = -1;
				Trg_Count = 1;
			}

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
						if (Fixed_Sender == -1 || Dead[Fixed_Sender] || Trg_Count > INTENCIVE){ // Change Fixed Sender

							vector<int> can_sender;
							vector<int>::iterator i;

							for (int i = BS; i < BS + N; i++){
								if (!Dead[i]) can_sender.push_back(i);
							}
							if (!can_sender.empty())
								Fixed_Sender = can_sender[rand() % can_sender.size()];
							else Fixed_Sender = -1;
							Trg_Count = 1;

							if (DbgMode == 2){
								outfile << endl;
								outfile << "New trigger occured at (" << Fixed_Sender <<")." << endl;
							}
						}
						if (DbgMode == 2)
							outfile << Fixed_Sender;
						ForwardMsg(Fixed_Sender);
						Trg_Count++;
					}


				DeadCounta = 0;
				for (int i = 0; i < N + BS; i++){
					if (Dead[i]) DeadCounta++;
				}

				if (
					//CountR > 100 &&
					//                (double)DeadCounta/(double)(N+BS) > 0.7
					(double)Suc_Fwd / (LostTrg*100 + Suc_Fwd) < DL
					)
					Term = true;

				if (DbgMode == 1){
					cout << "Dead Rate(%) = " << (double)DeadCounta / (double)N * 100 << endl;
					if (Term)
						cout << "Running Terminated." << endl;
					else
						cout << "Running Continued." << endl;
				}
			}

			OP_R[ALG] += CountR - 1;
			OP_TMSG[ALG] += TotalMsg;
			OP_LMSG[ALG] += LostTrg;
			OP_FWD[ALG] += Suc_Fwd;

			CountR = 0;
			TotalMsg = 0;
			LostTrg = 0;
			Suc_Fwd = 0;

			if (ALG == ALG_NUM -1)
				Rpt++;
			ALG = (ALG + 1) % ALG_NUM;

		}   // End Main Loop

		//cout << "-----" << "\n";
		//cout << "Result N." << Rpt << endl;
		//cout << "Round : " << CountR << "\n";
		//cout << "Forwarding Msg : " << TotalMsg << "\n";
		//cout << "Lost Msg : " << LostTrg << "\n";
		//cout << "\n";


		for (int i = 0; i < ALG_NUM; i++){
			cout << endl;
			cout << "----- Total Result -----" << endl;
			cout << "Algorithm ID : " << i << endl;
			cout << "Average Round : " << (double)OP_R[i] / RUN << endl;
			cout << "Average Forwarding Msg : " << (double)OP_TMSG[i] / RUN << endl;
			cout << "Average Lost Msg : " << (double)OP_LMSG[i] / RUN << endl;
			cout << endl;

			outfile << NO << " " << (double)OP_R[i] / RUN << " " << (double)OP_TMSG[i] / RUN << endl;

			OP_R[i] = 0;
			OP_TMSG[i] = 0;
			OP_LMSG[i] = 0;
		}

		Rpt = 0;

	} // End Repeat
	outfile << endl;
	outfile << Seed << endl;

	return 0;

}

