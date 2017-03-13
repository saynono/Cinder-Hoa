//
//  HoaNode.h
//  CinderProject
//
//  Created by say nono on 12.11.16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#pragma once


#include "cinder/audio/Node.h"
#include "cinder/audio/Param.h"
#include "cinder/audio/audio.h"
#include "cinder/audio/Context.h"
#include "cinder/audio/Buffer.h"

#include "Hoa.hpp"

#include "HoaElement.h"

using namespace std;
using namespace cinder::audio;

namespace nono {
    namespace audio {
        
        
struct HoaSource {
    NodeRef             mInput;
    HoaElementRef       mHoaElement;
    int                 mHoaId;
    bool                bEnabled;
};

struct HoaOutput {
    NodeRef             mOutput;
    HoaElementRef       mHoaElement;
    int                 mHoaId;
    bool                bEnabled;
};
        
typedef std::shared_ptr<struct HoaSource>		HoaSourceRef;
typedef std::shared_ptr<struct HoaOutput>		HoaOutputRef;
        
typedef std::shared_ptr<class HoaNode>          HoaNodeRef;
typedef std::shared_ptr<class HoaNodeBinaural>	HoaNodeBinauralRef;
typedef std::shared_ptr<class HoaNodeMulti>		HoaNodeMultiRef;

        
class HoaNode : public cinder::audio::ChannelRouterNode {

public:
    
    enum class Mode { BINAURAL, DC, MULTI };
	
    virtual ~HoaNode() {};

    void addInputRoute( const NodeRef &input, int inputChannelId=0 );
    void setInputChannel( const NodeRef &input, int inputChannelId );
    void connectInput( const NodeRef &input ) override;
    void disconnectInput( const NodeRef &input ) override;
    void disconnectAllInputs( ) override;
    
    HoaSourceRef getHoaInput( const NodeRef &input );
    std::vector<HoaSourceRef> getHoaInputs();
    
    void updatePositions();
    
//protected:
  
    HoaNode( Mode mode, int channelsIn, int channelsOut, const Format &format = Format() );

    void uninitialize()				override;
    
    virtual void process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut );
    void sumInputs() override;
    
    
    Mode                                        mMode;

    std::vector<HoaSourceRef>                   mSources;
    
    size_t                                      mChannelsIn;
    size_t                                      mChannelsOut;
    int                                         mHoaOrder;
    
    float*                                      mHarmonicsBuffer;
    hoa::Encoder<hoa::Hoa2d, float>*            mHoaEncoder;
    hoa::Decoder<hoa::Hoa2d, float>*            mHoaDecoder;
    hoa::Optim<hoa::Hoa2d, float> *             mHoaOptim;
    hoa::PolarLines<hoa::Hoa2d, float>          mHoaLines;
    float*                                      mHoaLineBuffer;

    BufferDynamic                               mHoaInternalBuffer;
    
};
       

// ------------------------------------------------------------------------------------------
//      HOA MULTI NODE
// ------------------------------------------------------------------------------------------

        
class HoaNodeMulti : public HoaNode {
            
public:
            
    HoaNodeMulti( int channelsIn, int channelsOut, const Format &format = Format() ) : HoaNode( Mode::MULTI, channelsIn, channelsOut, format ) {};
    virtual ~HoaNodeMulti() {};
    
    HoaOutputRef getHoaOutput( int id );
    std::list<HoaOutputRef> getHoaOutputs();
    
    void updateOutputPositions();

    
protected:
            
    void initialize()				override;
    void process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut ) override;
            
private:
            
    hoa::Encoder<hoa::Hoa2d, float>::Multi*         mHoaEncoderMulti;
    hoa::Decoder<hoa::Hoa2d, float>::Irregular*     mHoaDecoderIrregular;
    
    std::list<HoaOutputRef>                         mOutputs;

};


// ------------------------------------------------------------------------------------------
//      HOA BINAURAL NODE
// ------------------------------------------------------------------------------------------

class HoaNodeBinaural : public HoaNode {
    
public:

    HoaNodeBinaural( int channelsIn, const Format &format = Format() ) : HoaNode( Mode::BINAURAL, channelsIn, 2, format ) {};
    virtual ~HoaNodeBinaural() {};
    
protected:
    
    void initialize()				override;
    void process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut ) override;
    
private:
    
    hoa::Encoder<hoa::Hoa2d, float>::DC*        mHoaEncoderDC;
    hoa::Encoder<hoa::Hoa2d, float>::Multi*     mHoaEncoderMulti;
    hoa::Decoder<hoa::Hoa2d, float>::Binaural*  mHoaDecoderBinaural;
    
    float ** harmonicMatrix;
    float ** outputMatrix;
    
};

const HoaNodeRef& operator>>( const NodeRef &input, const ChannelRouterNode::RouteConnector &route );

    } // namespace audio
} // namespace nono
