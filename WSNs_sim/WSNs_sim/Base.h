//
//  Base.h
//  WSNs_sim
//
//  Created by Rentaro Watanabe on 2016/09/02.
//  Copyright © 2016年 Rentaro Watanabe. All rights reserved.
//

#ifndef Base_h
#define Base_h

#include <vector>
#include <array>

using namespace std;

class Base{
public:
    int id;
    pair<int,int> location; // Location of This BS((0~AREA_W-1),(0~AREA_D))
    vector<int> neighbor;
    
    Base();
    ~Base();
    
};


#endif /* Base_h */
