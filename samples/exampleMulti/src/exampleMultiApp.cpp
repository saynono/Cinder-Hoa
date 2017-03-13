#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/audio/audio.h"

#include "HoaNode.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace nono::audio;

class exampleMultiApp : public App {
public:
    
    void setup() override;
    void setupInputs();
    void setupAudioDevice();
    void update() override;
    void draw() override;
    void mouseDrag( MouseEvent event ) override;
    void mouseMove( MouseEvent event ) override;
    
    nono::audio::HoaNodeMultiRef    mHoaNode;
    
    audio::GainNodeRef				mGain;
    audio::ChannelRouterNodeRef     mChannelRouterRef;
    
    audio::InputDeviceNodeRef		p4;
    vector<audio::InputNodeRef>     mPlayers;
    
    nono::audio::HoaSourceRef       mHoaSourceHover;
    nono::audio::HoaSourceRef       mHoaSourceSelected;
    
    nono::audio::HoaOutputRef       mHoaOutputHover;
    nono::audio::HoaOutputRef       mHoaOutputSelected;
    
};

void exampleMultiApp::setup()
{
    
    setupAudioDevice();
    
    auto ctx = audio::Context::master();
    
    setupInputs();
    
    int numAudioSources = mPlayers.size();
    int numAudioOutputs = 2;
    
    mHoaNode = ctx->makeNode( new HoaNodeMulti( numAudioSources, numAudioOutputs) );
    mHoaNode->enable();
    
    // add a Gain to reduce the volume
    mGain = ctx->makeNode( new audio::GainNode( 0.25f ) );
    
    // connect and enable the Context
    mHoaNode >> mGain >> ctx->getOutput();
    
    
    console() << " =========================== ADDING ROUTES =======================" << std::endl;
    for( int i=0;i<mPlayers.size();i++ ){
        mHoaNode->addInputRoute(mPlayers[i]);
    }
    
    
    ctx->enable();
}

void exampleMultiApp::setupInputs(){
    
    auto ctx = audio::Context::master();
    
    string file = "../../../../../samples/data/sound/DrainMagic.ogg";
    audio::SourceFileRef sourceFile = audio::load( loadAsset(file), ctx->getSampleRate() );
    audio::BufferRef buffer = sourceFile->loadBuffer();
    audio::BufferPlayerNodeRef p1 = ctx->makeNode( new audio::BufferPlayerNode( buffer ) );
    p1->setLoopEnabled();
    p1->start();
    p1->setName("Player1");
    
    
    audio::GenNodeRef p2 = ctx->makeNode( new audio::GenOscNode( audio::WaveformType::SINE, 440 ) );
    p2->setName("OSC Sine 0");
    p2->enable();
    
    audio::GenNodeRef p3 = ctx->makeNode( new audio::GenOscNode( audio::WaveformType::SQUARE, 220 ) );
    p3->setName("OSC Square 220");
    p3->enable();
    
    audio::InputDeviceNodeRef p4 = ctx->createInputDeviceNode();
    p4->enable();
    p4->setName("Microphone");
    
//    mPlayers.push_back( p1 );
    mPlayers.push_back( p2 );
//    mPlayers.push_back( p3 );
//    mPlayers.push_back( p4 );
    
}

void exampleMultiApp::setupAudioDevice(){
    
    // debug print all devices to console
    console() << audio::Device::printDevicesToString() << endl;
    
    audio::DeviceRef deviceWithMaxOutputs;
    
    for( const auto &dev : audio::Device::getDevices() ) {
        if( ! deviceWithMaxOutputs || deviceWithMaxOutputs->getNumOutputChannels() < dev->getNumOutputChannels() )
            deviceWithMaxOutputs = dev;
    }
    
    getWindow()->setTitle( "Cinder HOA Test. Output[" + deviceWithMaxOutputs->getName() +"]" );
    
    
    auto ctx = audio::master();
    audio::OutputDeviceNodeRef multichannelOutputDeviceNode = ctx->createOutputDeviceNode( deviceWithMaxOutputs, audio::Node::Format().channels( deviceWithMaxOutputs->getNumOutputChannels() ) );
    ctx->setOutput( multichannelOutputDeviceNode );
    
    
}

void exampleMultiApp::update()
{
}

void exampleMultiApp::draw()
{
    gl::clear( Color( 0, 0, 0 ) );
    gl::color( Color( 1,1,1 ) );
    
    gl::pushMatrices();
    gl::translate( getWindowWidth()/2, getWindowHeight()/2 );
    float scale = getWindowWidth()/2;
    
    auto outputs = mHoaNode->getHoaOutputs();
    for( const auto& os: outputs ){
        vec3 pos = os->mHoaElement->getPosition();
        vec2 pos2D(pos.x*scale,pos.y*scale);
        if( mHoaOutputHover == os ){
            gl::color(1, 0, 0);
        }else{
            gl::color(1, 1, 0);
        }
        gl::drawSolidCircle(pos2D, 10);
    }
    
    
    auto sources = mHoaNode->getHoaInputs();
    for( const auto& s: sources ){
        vec3 pos = s->mHoaElement->getPosition();
        vec2 pos2D(pos.x*scale,pos.y*scale);
        if( mHoaSourceHover == s ){
            gl::color(1, 0, 0);
        }else{
            gl::color(1, 1, 1);
        }
        gl::drawSolidCircle(pos2D, 10);
    }
    gl::popMatrices();
}

void exampleMultiApp::mouseDrag( MouseEvent e ){
    
    vec2 mPos = vec2((float)e.getPos().x,(float)e.getPos().y) - vec2(getWindowWidth()/2, getWindowHeight()/2);
    float scale = getWindowWidth()/2;
    
    if( mHoaOutputHover != nullptr ){
        vec3 pos( mPos.x/scale, mPos.y/scale, 0 );
        mHoaOutputHover->mHoaElement->setPosition(pos);
        mHoaNode->updateOutputPositions();
    }
    
    if( mHoaOutputHover == nullptr && mHoaSourceHover != nullptr ){
        vec3 pos( mPos.x/scale, mPos.y/scale, 0 );
        mHoaSourceHover->mHoaElement->setPosition(pos);
        mHoaNode->updatePositions();
    }
    
}

void exampleMultiApp::mouseMove( MouseEvent e ){
    
    if( !mHoaNode ) return;
    vec2 mPos = vec2((float)e.getPos().x,(float)e.getPos().y) - vec2(getWindowWidth()/2, getWindowHeight()/2);
    float scale = getWindowWidth()/2;
    
    auto outputs = mHoaNode->getHoaOutputs();
    mHoaOutputHover = nullptr;
    for( auto os: outputs ){
        vec3 pos = os->mHoaElement->getPosition();
        vec2 pos2D( pos.x*scale, pos.y*scale );
        if( length(pos2D-mPos) < 10 ){
            mHoaOutputHover = os;
        }
    }
    
    
    if( mHoaOutputHover == nullptr ){
        auto sources = mHoaNode->getHoaInputs();
        mHoaSourceHover = nullptr;
        for( auto s: sources ){
            vec3 pos = s->mHoaElement->getPosition();
            vec2 pos2D( pos.x*scale, pos.y*scale );
            if( length(pos2D-mPos) < 10 ){
                mHoaSourceHover = s;
            }
        }
    }
}

CINDER_APP( exampleMultiApp, RendererGl (RendererGl::Options().stencil().msaa (16)),
           [&] (App::Settings * settings)
{
    settings->setWindowSize (400, 400);
    settings->setFrameRate (60.0f);
    settings->setTitle ("Cinder HOA Wrapper Multi");
    settings->setHighDensityDisplayEnabled();
})