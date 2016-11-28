//
//  Node.hpp
//  WSNs_sim
//
//  Created by Rentaro Watanabe on 2016/08/30.
//  Copyright © 2016年 Rentaro Watanabe. All rights reserved.
//

#ifndef Node_h
#define Node_h

#include <vector>
#include <array>

using namespace std;

class Node {
public:
    int id;
    pair<double,double> location; // Location of This Node((0~AREA_W-1),(0~AREA_D))
    vector<int> neighbor;
    double resE;    // Residual Energy of This Node
    int dst;
    bool isbs;  // 1 Means Base Station
    
    
    Node();
    ~Node();
    
    void ResetVar();
    
};

#endif /* Node_hpp */
