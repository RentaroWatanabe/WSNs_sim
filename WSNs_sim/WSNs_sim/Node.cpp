//
//  Node.cpp
//  WSNs_sim
//
//  Created by Rentaro Watanabe on 2016/08/30.
//  Copyright © 2016年 Rentaro Watanabe. All rights reserved.
//

#include "Node.h"

Node::Node(){}
Node::~Node(){}

void Node::ResetVar(){
    location.first = -1.0;
    location.second = -1.0;
    neighbor.clear();
    resE = 2.0;
    dst = -1;
    isbs = 0;
}

