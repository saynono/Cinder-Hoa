//
//  HoaNode.cpp
//  CinderProject
//
//  Created by say nono on 12.11.16.
//  Copyright (c) 2016 __MyCompanyName__. All rights reserved.
//

#include "HoaNode.h"

#include "cinder/Rand.h"
#include "cinder/CinderMath.h"


using namespace cinder::audio;
using namespace hoa;

namespace nono {
    namespace audio {
        

HoaNode::HoaNode( Mode mode, int channelsIn, int channelsOut, const Format &format )
        : ChannelRouterNode( format ), mChannelsIn(channelsIn), mChannelsOut(channelsOut), mMode( mode ), mHoaLines(channelsIn) {
}
        
        
void HoaNode::uninitialize(){
    delete mHoaEncoder;
    delete mHoaDecoder;
    delete mHoaOptim;
    delete [] mHarmonicsBuffer;
}

        
HoaSourceRef HoaNode::getHoaInput( const NodeRef &input ){
    for( auto it = mSources.begin(); it != mSources.end(); ++it ) {
        if( (*it)->mInput == input ) {
//            mSources.erase( it );
            return (*it);
        }
    }
    return nullptr;
}

std::vector<HoaSourceRef> HoaNode::getHoaInputs(){
    return mSources;
}

void HoaNode::addInputRoute( const NodeRef &input, int inputChannelId ){
    
    int index = getInputs().size();
    ChannelRouterNode::addInputRoute( input, inputChannelId, index, 1);

    HoaSourceRef source = std::make_shared<HoaSource>();
    source->mInput = input;
    source->mHoaElement = std::make_shared<HoaElement>();
    source->mHoaId = index;
    source->bEnabled = true;
    
    vec3 pos = source->mHoaElement->getPosition();
//    pos = normalize(cinder::Rand::randVec3());
    pos = (cinder::Rand::randVec3());
//    pos -= vec3(.5,.5,.5);
//    pos *= vec3(2.0f);
    source->mHoaElement->setPosition(pos);
    mHoaLines.setRadiusDirect(index, hoa::Math<float>::radius(pos.x,pos.y) );
    mHoaLines.setAzimuthDirect(index, hoa::Math<float>::azimuth(pos.x,pos.y) );
    
    /*
    
    inline void setSourcePositionDirect(int index, ofVec3f position){
        _line.setRadiusDirect(index, Math<T>::radius(position.x - _ambisonicCenter.x,
                                                     (ofGetHeight() - position.y) - _ambisonicCenter.y) * 1.0/_ambisonicRadius);
        
        _line.setAzimuthDirect(index, Math<T>::azimuth(position.x - _ambisonicCenter.x,
                                                       (ofGetHeight() - position.y) - _ambisonicCenter.y));
    }
    hoaCoord->setSourcePositionDirect( i, pos );

    */
    
    lock_guard<mutex> lock( getContext()->getMutex() );
    mSources.push_back(source);
}
        
void HoaNode::setInputChannel( const NodeRef &input, int inputChannelId ){
    lock_guard<mutex> lock( getContext()->getMutex() );
    for( auto it = mRoutes.begin(); it != mRoutes.end(); ++it ) {
        if( it->mInput == input ) {
            it->mInputChannelIndex = inputChannelId;
            return;
        }
    }
}

        
void HoaNode::connectInput( const NodeRef &input ){
    ChannelRouterNode::connectInput(input);
    console() << "HOA Connect with " << input->getName() << std::endl;
}

void HoaNode::disconnectInput( const NodeRef &input ){
    ChannelRouterNode::disconnectInput( input );
    lock_guard<mutex> lock( getContext()->getMutex() );
            
    for( auto it = mSources.begin(); it != mSources.end(); ++it ) {
        if( (*it)->mInput == input ) {
            mSources.erase( it );
            return;
        }
    }
}
        
void HoaNode::disconnectAllInputs() {
    ChannelRouterNode::disconnectAllInputs();
    lock_guard<mutex> lock( getContext()->getMutex() );
    mSources.clear();
}
        
void HoaNode::sumInputs(){

    ChannelRouterNode::sumInputs();
    
    cinder::audio::Buffer *internalBuffer = getInternalBuffer();
    
    dsp::mixBuffers( internalBuffer, &mHoaInternalBuffer );
    if( isEnabled() )
        process( &mHoaInternalBuffer, internalBuffer );

}

void HoaNode::process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut ) {
    
//    int index;
//    const size_t numFrames = bufferIn->getNumFrames();
//    int numSources = mChannelsIn;
//    int numOutputs = mChannelsOut;
//    
//    console() << "numFrames : " << numFrames << std::endl;
//    bufferOut->zero();
    
//    float * inputBuffer = new float[numSources];
//    float * outputBuffer = new float[numOutputs];
//    
//    for( size_t i = 0; i < numFrames; i++ ) {
//        
//        // CALCULATE SMOOTHED VALUES
//        mHoaLines.process(mHoaLineBuffer);
//        
//        // SET CURRENT POSITION FOR EACH PARTICLE
////        for( auto it = mSources.begin(); it != mSources.end(); ++it ) {
//        for( auto s: mSources ){
//            index = s->mHoaId;
//            inputBuffer[index] = bufferIn->getChannel(index)[i];
////            const vector<float>& rawSamples = sounds[j].audio.getRawSamples();
////            inputBuffer[j] = rawSamples[sounds[j].samplePos++];//*2.0f;
//            
//            //GET CALCULATED RADIUS AND AZIMUTH AND PASS THEM TO THE ENCODER
////            mHoaEncoderMulti->setRadius( index, mHoaLineBuffer[index]);
////            mHoaEncoderMulti->setAzimuth( index, mHoaLineBuffer[index+numSources]);
//        
//            mHoaEncoder->setRadius( mHoaLineBuffer[index] );
//            mHoaEncoder->setAzimuth( mHoaLineBuffer[index+1] );
////            hoaEncoder.process(&input, harmonicsBuffer);
////            hoaOptim.process(harmonicsBuffer, harmonicsBuffer);
//        }
//        
//
//        // CREATE THE SPHERICAL HARMONICS
//        mHoaEncoder->process( inputBuffer, mHarmonicsBuffer);
//        
//        // PROCESS THE HARMONICS WITH OPTIM
//        mHoaOptim->process( mHarmonicsBuffer, mHarmonicsBuffer);
//        
///*        // DECODE THE HARMONICS; AUDIO TREATEMENTS ARE POSSIBLE IN BETWEEN THESE STEPS
//        mHoaDecoder->process(mHarmonicsBuffer, outputBuffer);
//        
//        for( int j=0;j<mChannelsOut;j++ ){
//            bufferOut->getChannel(j)[i] = outputBuffer[i];
////            mHoaDecoder->process(mHarmonicsBuffer, buffer->getData()+i*numChannels);
//        }
//        
////        mHoaDecoder->process( mHarmonicsBuffer, buffer->getChannel(i) );
//        
//    }
//  */
//        
//        for (int j = 0; j<3*2+1; j++) {
//            harmonicMatrix[j][i] = mHarmonicsBuffer[j];
//        }
//    }
//    
//    mHoaDecoder->processBlock(const_cast<const float **>(harmonicMatrix), outputMatrix);
////
//    float* ch1 = bufferOut->getChannel(0);
//    float* ch2 = bufferOut->getChannel(1);
//    for (int i = 0; i<numFrames; ++i) {
//        ch1[i] = outputMatrix[0][i];
//        ch2[i] = outputMatrix[1][i];
//    }
  
}

        
void HoaNode::updatePositions(){
    for( auto s: mSources ){
        int index = s->mHoaId;
        vec3 pos = s->mHoaElement->getPosition();
        float radius = hoa::Math<float>::radius(pos.x,pos.y);
        float azimuth = hoa::Math<float>::azimuth(pos.x,pos.y);
        mHoaLines.setRadius(index, radius );
        mHoaLines.setAzimuth(index, azimuth );
    }
    mHoaLines.process(mHoaLineBuffer);
}
        
        
        
// ------------------------------------------------------------------------------------------
//
//      HOA BINAURAL NODE
//
// ------------------------------------------------------------------------------------------
        
void HoaNodeBinaural::initialize(){
            
    cinder::Rand::randomize ();
            
    auto ctx = cinder::audio::Context::master();
            
    setChannelMode( ChannelMode::SPECIFIED );
    setNumChannels( max(mChannelsIn,mChannelsOut) );
    
    mHoaLineBuffer = new float[mChannelsIn*2];
    memset(mHoaLineBuffer, 0, sizeof(float)*mChannelsIn*2);
    
    for(int i = 0; i<mChannelsIn;i++){
        mHoaLines.setRadiusDirect(i,10000);
        mHoaLines.setAzimuthDirect(i,0);
    }
    mHoaLines.setRamp( .25 * ctx->getSampleRate() );
    
    // SETUP HOA
    mHoaOrder = 3;
    int order = mHoaOrder;
    //CREATE THE SPHERICAL HARMONICS BUFFER, IT MUST HAVE ORDER*2+1 VALUES FOR 2 DIMENSIONS
    mHarmonicsBuffer = new float[order*2+1];
            
    // THE ENCODER CALCULATES THE SPHERICAL HARMONICS
//    mHoaEncoder = mHoaEncoderDC = new Encoder<Hoa2d, float>::DC( order );
    
    // THE ENCODERMULTI ALLOWS TO DISTRIBUTE MULTIPLE SOURCES IN SPACE
    mHoaEncoder = mHoaEncoderMulti = new Encoder<Hoa2d, float>::Multi( order, mChannelsIn );

    
            /* THE DECODER TRANSLATES THE HARMONICS INTO AUDIO SIGNALS FOR OUTPUT.
             THE NUMBER OF MINIMUM OUPUT CHANNELS FOR REGULAR MODE = ORDER*2+1
             SMALLER VALUES MAY BE USED, BUT THE RESULTING SOUND WON'T BE AS EXPECTED
             FOR SMALL DIFFERENCES ( 5 OR 6 INSTEAD OF 7 SPEAKERS) IRREGULAR MODE MAY BE USED */
            //    hoaDecoder = new Decoder<Hoa2d, float>::Regular(order, nOutputs);
            //    mHoaDecoder = new Decoder<Hoa2d, float>::Irregular( order, mChannelsOut );
    
    // BINAURAL MODE SET FOR USE WITH HEADPHONES
    mHoaDecoder = mHoaDecoderBinaural = new Decoder<Hoa2d, float>::Binaural( order );
    
    // This will prevent cracking sounds in the output. Need to find out what it actually does.
    mHoaDecoderBinaural->setCropSize(256);

    /* RENDERING IS COMPUTED IN RELATION TO THE SPEAKER'S ANGLES
        THEY MAYBE SET WITH THE FUNCTION hoaDecoder->setPlanewaveAzimuth(const ulong index,
        const float azimuth); */
    
    mHoaDecoder->computeRendering( ctx->getFramesPerBlock() );
    
    
    /*THE OPTIM ALLOWS TO ACOUNT FOR DISPLACEMENTS IN IDEAL SPEAKER POSITION
    "Basic" WORKS AS A BYPASS.
    "InPhase" AND "MaxRe" SHOULD BE USED IF THE AMBSIONICS CIRCLE/SPHERE IS NOT PERFECT */
    //     hoaOptim = new Optim<Hoa2d, float>::Basic(order);
    mHoaOptim = new Optim<Hoa2d, float>::InPhase(order);
            
            
    mHoaInternalBuffer.setSize( ctx->getFramesPerBlock(), mChannelsIn );
            
            
            
    harmonicMatrix = new float * [order * 2+1];
            
    for (int i = 0; i< order*2+1;++i) harmonicMatrix[i] = new float[ctx->getFramesPerBlock()];
            
    outputMatrix = new float * [2];
    outputMatrix[0] = new float[ctx->getFramesPerBlock()];
    outputMatrix[1] = new float[ctx->getFramesPerBlock()];
            
}
        

void HoaNodeBinaural::process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut ) {
    
    const size_t numFrames = bufferIn->getNumFrames();
    int numSources = mChannelsIn;
    
    float** chIns = new float*[numSources];
    for( int j=0;j<numSources;j++ ){
        chIns[j] = bufferIn->getChannel(j);
    }
    
    float* input = new float[numSources];
    memset(input,0,numSources*sizeof(float));
    
    for( size_t i = 0; i < numFrames; i++ ) {
                
        // CALCULATE SMOOTHED VALUES
        mHoaLines.process(mHoaLineBuffer);
        
        
        // SET SOUND AND POSITIONS
        for( int j=0;j<numSources;j++ ){
            input[j] = chIns[j][i];
            mHoaEncoderMulti->setRadius( j, mHoaLineBuffer[j] );
            mHoaEncoderMulti->setAzimuth( j, mHoaLineBuffer[j+numSources] );
        }

        // CREATE THE SPHERICAL HARMONICS
        mHoaEncoderMulti->process( input, mHarmonicsBuffer);
                
        // PROCESS THE HARMONICS WITH OPTIM
        mHoaOptim->process( mHarmonicsBuffer, mHarmonicsBuffer);
        
        for (int j = 0; j<mHoaOrder*2+1; j++) {
            harmonicMatrix[j][i] = mHarmonicsBuffer[j];
        }
    }
    
    // DECODE THE HARMONICS; AUDIO TREATEMENTS ARE POSSIBLE IN BETWEEN THESE STEPS
    mHoaDecoderBinaural->processBlock(const_cast<const float **>(harmonicMatrix), outputMatrix);
            //
    bufferOut->zero();
    
    float* ch1 = bufferOut->getChannel(0);
    float* ch2 = bufferOut->getChannel(1);
    for (int i = 0; i<numFrames; ++i) {
        ch1[i] = outputMatrix[0][i];
        ch2[i] = outputMatrix[1][i];
//        ch1[i] = chIn1[i];
//        ch2[i] = chIn1[i];
    }
    
}
        
        
        
        
        
        
// ------------------------------------------------------------------------------------------
//
//      HOA MULTI NODE
//
// ------------------------------------------------------------------------------------------
        
void HoaNodeMulti::initialize(){
            
    cinder::Rand::randomize ();
            
    auto ctx = cinder::audio::Context::master();
            
    setChannelMode( ChannelMode::SPECIFIED );
    setNumChannels( max(mChannelsIn,mChannelsOut) );
            
    mHoaLineBuffer = new float[mChannelsIn*2];
    memset(mHoaLineBuffer, 0, sizeof(float)*mChannelsIn*2);
            
    for(int i = 0; i<mChannelsIn;i++){
        mHoaLines.setRadiusDirect(i,10000);
        mHoaLines.setAzimuthDirect(i,0);
    }
    mHoaLines.setRamp( .25 * ctx->getSampleRate() );
            
    // SETUP HOA
    mHoaOrder = 3;
    int order = mHoaOrder;
    //CREATE THE SPHERICAL HARMONICS BUFFER, IT MUST HAVE ORDER*2+1 VALUES FOR 2 DIMENSIONS
    mHarmonicsBuffer = new float[order*2+1];
            
    // THE ENCODER CALCULATES THE SPHERICAL HARMONICS
    //    mHoaEncoder = mHoaEncoderDC = new Encoder<Hoa2d, float>::DC( order );
            
    // THE ENCODERMULTI ALLOWS TO DISTRIBUTE MULTIPLE SOURCES IN SPACE
    mHoaEncoder = mHoaEncoderMulti = new Encoder<Hoa2d, float>::Multi( order, mChannelsIn );
            
            
    /* THE DECODER TRANSLATES THE HARMONICS INTO AUDIO SIGNALS FOR OUTPUT.
        THE NUMBER OF MINIMUM OUPUT CHANNELS FOR REGULAR MODE = ORDER*2+1
        SMALLER VALUES MAY BE USED, BUT THE RESULTING SOUND WON'T BE AS EXPECTED
        FOR SMALL DIFFERENCES ( 5 OR 6 INSTEAD OF 7 SPEAKERS) IRREGULAR MODE MAY BE USED */
    mHoaDecoder = mHoaDecoderIrregular = new Decoder<Hoa2d, float>::Irregular( order, mChannelsOut );
  
//    for( int i=0;i<mChannelsOut;i++ ){
//      mHoaEncoderMulti->setRadius(i, (float).5);
//      console() << (i+1) << " ===> " << mHoaEncoderMulti->getRadius(i) << std::endl;
//    }
  
//    // BINAURAL MODE SET FOR USE WITH HEADPHONES
//    mHoaDecoder = mHoaDecoderBinaural = new Decoder<Hoa2d, float>::Binaural( order );
//            
//    // This will prevent cracking sounds in the output. Need to find out what it actually does.
//    mHoaDecoderBinaural->setCropSize(256);
    
    /* RENDERING IS COMPUTED IN RELATION TO THE SPEAKER'S ANGLES
        THEY MAYBE SET WITH THE FUNCTION hoaDecoder->setPlanewaveAzimuth(const ulong index,
        const float azimuth); */
            
    mHoaDecoder->computeRendering( ctx->getFramesPerBlock() );
            
            
    /*THE OPTIM ALLOWS TO ACOUNT FOR DISPLACEMENTS IN IDEAL SPEAKER POSITION
        "Basic" WORKS AS A BYPASS.
        "InPhase" AND "MaxRe" SHOULD BE USED IF THE AMBSIONICS CIRCLE/SPHERE IS NOT PERFECT */
    //     hoaOptim = new Optim<Hoa2d, float>::Basic(order);
    mHoaOptim = new Optim<Hoa2d, float>::InPhase(order);
            
            
    mHoaInternalBuffer.setSize( ctx->getFramesPerBlock(), mChannelsIn );
            
    int num = mHoaDecoder->getNumberOfPlanewaves();
    for( int i=0;i<num;i++ ){
        // Rotate it initially so 0/360 degree is top
        mHoaDecoder->setPlanewaveAzimuth(i, mHoaDecoder->getPlanewaveAzimuth(i)-HOA_PI );
        mHoaDecoder->getPlanewaveHeight(i);
        mHoaDecoder->getPlanewaveOrdinate(i);        
        float angle = -mHoaDecoder->getPlanewaveAzimuth(i);
        HoaOutputRef output = std::make_shared<HoaOutput>();
//        source->mInput = input;
        output->mHoaElement = std::make_shared<HoaElement>();
        output->mHoaId = i;
        output->mHoaElement->setPosition(vec3( (float)sin(angle), (float)cos(angle), 0.0f ));
        output->bEnabled = true;
        mOutputs.push_back(output);        
    }
    
    // Re-Render the matrix after rotating it by PI
    mHoaDecoder->computeRendering( ctx->getFramesPerBlock() );


}
        
        
void HoaNodeMulti::process( cinder::audio::Buffer *bufferIn, cinder::audio::Buffer *bufferOut ) {
            
    const size_t numFrames = bufferIn->getNumFrames();
    const size_t numSources = mChannelsIn;
    const size_t numOutputs = mChannelsOut;//min(bufferOut->getNumChannels(),  mChannelsOut);
            
//        console() << "HoaNodeMulti numFrames : " << numFrames << " Sources : " << numSources << "/" << bufferIn->getNumChannels() << "    Outputs: " << numOutputs <<  "/" << bufferOut->getNumChannels() << std::endl;
  
    //    float * inputBuffer = new float[numSources];
    //    float * outputBuffer = new float[numOutputs];
    //    float* chIn1 = bufferIn->getChannel(0);
  
    float** chIns = new float*[numSources];
    for( int j=0;j<numSources;j++ ){
        chIns[j] = bufferIn->getChannel(j);
    }
    float* input = new float[numSources];
    memset(input,0,numSources*sizeof(float));
    bool* inActives = new bool[mSources.size()];
    for( auto source: mSources ){
        inActives[source->mHoaId] = source->bEnabled;
    }
    
    
    float** chOuts = new float*[numOutputs];
    bool* outActives = new bool[numOutputs];
    for( int j=0;j<numOutputs;j++ ){
        chOuts[j] = bufferOut->getChannel(j);
    }
    for( auto os : mOutputs ){
        outActives[os->mHoaId] = os->bEnabled;
    }
    

    float* outBuffer = new float[numOutputs];
    
    for( size_t i = 0; i < numFrames; i++ ) {
                
        // CALCULATE SMOOTHED VALUES
        mHoaLines.process(mHoaLineBuffer);
                
        for( int j=0;j<numSources;j++ ){
            input[j] = chIns[j][i]*inActives[j];
            mHoaEncoderMulti->setRadius( j, mHoaLineBuffer[j] );
            mHoaEncoderMulti->setAzimuth( j, mHoaLineBuffer[j+numSources] );
        }
                
        // CREATE THE SPHERICAL HARMONICS
        mHoaEncoderMulti->process( input, mHarmonicsBuffer);
        
        // PROCESS THE HARMONICS WITH OPTIM
        mHoaOptim->process( mHarmonicsBuffer, mHarmonicsBuffer);
                

        // DECODE THE HARMONICS; AUDIO TREATEMENTS ARE POSSIBLE IN BETWEEN THESE STEPS
        mHoaDecoder->process(mHarmonicsBuffer, outBuffer);

    
        for( int j=0;j<numOutputs;j++ ){
            chOuts[j][i] = outBuffer[j]*outActives[j];
        }
        
    }
 
}
        
        
void HoaNodeMulti::updateOutputPositions(){
    for( auto os : mOutputs ){
        vec3 pos = os->mHoaElement->getPosition();
//        float radius = hoa::Math<float>::radius(pos.x,pos.y);
        float azimuth = hoa::Math<float>::azimuth(pos.x,pos.y);
        mHoaDecoder->setPlanewaveAzimuth(os->mHoaId, azimuth);
        
        int i = os->mHoaId;
//        console() << i << " mHoaDecoder->getPlanewaveHeight: " << mHoaDecoder->getPlanewaveHeight(i) << std::endl;
//        console() << i << " mHoaDecoder->getPlanewaveOrdinate: " << mHoaDecoder->getPlanewaveOrdinate(i) << std::endl;
//        console() << i << " mHoaDecoder->getPlanewaveElevation: " << mHoaDecoder->getPlanewaveElevation(i) << std::endl;
//        console() << i << " mHoaDecoder->getPlanewaveAbscissa: " << mHoaDecoder->getPlanewaveAbscissa(i) << std::endl;
//        console() << endl;
    }
    
    auto ctx = cinder::audio::Context::master();
    mHoaDecoder->computeRendering( ctx->getFramesPerBlock() );
    
}
        
        
        
HoaOutputRef HoaNodeMulti::getHoaOutput( int id ){
    for( auto o: mOutputs ){
        if( o->mHoaId == id ) return o;
    }
    return nullptr;
}

std::list<HoaOutputRef> HoaNodeMulti::getHoaOutputs(){
    return mOutputs;
}

        
        
} } // namespaces nono::audio