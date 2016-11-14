//
//  HoaElement.cpp
//  CinderProject
//
//  Created by say nono on 12.11.16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#include "HoaElement.h"



namespace nono {
    namespace audio {
        
        
HoaElement::HoaElement(): bHasChanged( false ){            
}
        
void HoaElement::setPosition( vec3 pos ){
    mPos = pos;
    bHasChanged = true;
}
        
const vec3& HoaElement::getPosition(){
    return mPos;
}

bool HoaElement::hasChanged(){
    bool b = bHasChanged;
    bHasChanged = false;
    return b;
}


} } // namespaces nono::audio