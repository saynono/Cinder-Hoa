//
//  UrgController.h
//
//  Created by say nono on 10.11.16
//  Copyright (c) 2016 nono. All rights reserved.
//

#pragma once


#include "cinder/app/App.h"
#include "cinder/utilities.h"
#include "cinder/Log.h"

#include "Hoa.hpp"


#include <boost/signals2.hpp>


using namespace ci;
using namespace ci::app;
using namespace std;


namespace nono {
    
    class HoaWrapper {

public:
      
    struct URG_Properties{
        long distanceMin;
        long distanceMax;
    };

    struct URG_Point{
        double degrees;
        double radians;
        long distance;
    };
        
    struct URG_Frame{
        URG_Properties properties;
        long timestamp;
        vector<URG_Point> points;
    };
        
        
    using URG_FrameRef = shared_ptr<URG_Frame>;
    
	
    HoaWrapper();
    ~HoaWrapper();
    
    void setup();
    void kill();
    
    boost::signals2::signal<void(void)> onData;

    
private:
        
    
};

} // namespace nono



