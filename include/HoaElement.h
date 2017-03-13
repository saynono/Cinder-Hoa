//
//  HoaElement.h
//  CinderProject
//
//  Created by say nono on 12.11.16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#pragma once


#include "cinder/app/App.h"

using namespace ci;
using namespace ci::app;
using namespace std;


namespace nono {
    namespace audio {
        

typedef std::shared_ptr<class HoaElement>		HoaElementRef;
        
class HoaElement{

public:
    HoaElement();
    void setPosition( vec3 pos );
    const vec3& getPosition();
    bool hasChanged();
    
private:
    
    vec3 mPos;
    bool bHasChanged;
	
};

} } // namespaces nono::audio
