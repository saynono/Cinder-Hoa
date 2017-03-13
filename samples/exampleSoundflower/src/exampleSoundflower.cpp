#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/Serial.h"

#include "cinder/audio/audio.h"
#include "cinder/GeomIo.h"
#include "cinder/audio/ChannelRouterNode.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"

#include "HoaNode.h"

using namespace ci;
using namespace ci::app;
using namespace std;

using namespace nono::audio;

class CinderProjectApp : public App {
  public:

	void setup() override;
    void setupInputs();
    void setupAudioDevice();
	void update() override;
	void draw() override;
    void mouseDrag( MouseEvent event ) override;
    void mouseMove( MouseEvent event ) override;
    void mouseUp( MouseEvent event ) override;
    
    void keyDown( KeyEvent event ) override;
	
    nono::audio::HoaNodeMultiRef    mHoaNode;
    
    audio::GainNodeRef				mGain;
    audio::ChannelRouterNodeRef     mChannelRouterRef;
    float                           mDisplayScale;
    
    audio::InputDeviceNodeRef		p4;
    vector<audio::NodeRef>          mPlayers;
    Rand                            mRandom;
    audio::DeviceRef                mInputDevice;
    audio::DeviceRef                mOutputDevice;
    
    
    nono::audio::HoaSourceRef       mHoaSourceHover;
    nono::audio::HoaSourceRef       mHoaSourceSelected;

    nono::audio::HoaOutputRef       mHoaOutputHover;
    nono::audio::HoaOutputRef       mHoaOutputSelected;
    
    

};

void CinderProjectApp::setup()
{
    
    setupAudioDevice();

    auto ctx = audio::Context::master();
    
    setupInputs();
    
    mDisplayScale = .25;
    
    int numAudioSources = mPlayers.size();
    int numAudioOutputs = 6;
    
    mHoaNode = ctx->makeNode( new HoaNodeMulti( numAudioSources, numAudioOutputs) );
    mHoaNode->enable();
    
    // add a Gain to reduce the volume
    mGain = ctx->makeNode( new audio::GainNode( 0.75f ) );
    
    // connect and enable the Context
    mHoaNode >> mGain >> ctx->getOutput();
    
    
    console() << " =========================== ADDING ROUTES =======================" << std::endl;
    console() << " ===> " << mInputDevice->getNumInputChannels() << std::endl;
    
    int cnt = 0;
    for( auto player: mPlayers ){
        mHoaNode->addInputRoute(player,cnt++);
        
    }
//    mHoaNode->addInputRoute(mPlayers[1],0);
//    mHoaNode->addInputRoute(mPlayers[2],1);
//    mHoaNode->addInputRoute(mPlayers[3],2);
//    mHoaNode->addInputRoute(mPlayers[4],3);
    
    
    ctx->enable();
}

void CinderProjectApp::setupInputs(){
    
    auto ctx = audio::Context::master();
    
    
    // this part is where you add the Soundflower Channels as audio inputs.
    
    auto format = ci::audio::Node::Format().channels( mInputDevice->getNumInputChannels() );
    format.setChannelMode(audio::Node::ChannelMode::MATCHES_INPUT);
    audio::InputDeviceNodeRef input = ctx->createInputDeviceNode(mInputDevice,format);
    input->enable();

    
    for( int i=0;i<4;i++){
        auto format = ci::audio::Node::Format().channels( mInputDevice->getNumInputChannels() );
        format.setChannelMode(audio::Node::ChannelMode::MATCHES_INPUT);
        audio::GainNodeRef gain = ctx->makeNode( new audio::GainNode(format) );
        input >> gain;
        gain->setName("Channel "+ toString(i+1));
        mPlayers.push_back( gain );
    }

}

void CinderProjectApp::setupAudioDevice(){
    
    // debug print all devices to console
    console() << audio::Device::printDevicesToString() << endl;
    
    audio::DeviceRef deviceWithMaxOutputs;
    
    
    for( const auto &dev : audio::Device::getDevices() ) {
        if( ! deviceWithMaxOutputs || deviceWithMaxOutputs->getNumOutputChannels() < dev->getNumOutputChannels() )
            deviceWithMaxOutputs = dev;
    }
    
    // This will tell the cinder to use 64ch Soundflower as input
    mInputDevice = audio::Device::findDeviceByName("Soundflower (64ch)");
    
    // I am using a Scarlet Sound card...
    audio::DeviceRef outputDevice = audio::Device::findDeviceByName("Scarlett 18i20 USB");
    
    // If the system can't find this device it will switch to either the Built-in output or the default option.
    if( outputDevice == nullptr ){
        outputDevice = audio::Device::findDeviceByName("Built-in Output");
    }
    if( outputDevice == nullptr ){
        outputDevice = audio::Device::getDefaultOutput();
    }

    getWindow()->setTitle( "Cinder HOA Soundflower Example. Output[" + outputDevice->getName() +"]" );


    auto ctx = audio::master();
    auto format = audio::Node::Format().channels( outputDevice->getNumOutputChannels());
    audio::OutputDeviceNodeRef multichannelOutputDeviceNode = ctx->createOutputDeviceNode( outputDevice, format );
    ctx->setOutput( multichannelOutputDeviceNode );
    mOutputDevice = outputDevice;
    
}

void CinderProjectApp::update()
{
}

void CinderProjectApp::draw()
{	
    gl::clear( Color( 0, 0, 0 ) );
    gl::color( Color( 1,1,1 ) );
    
    gl::pushMatrices();
    gl::translate( getWindowWidth()/2, getWindowHeight()/2 );
    float scale = getWindowWidth()*mDisplayScale;
    
    auto outputs = mHoaNode->getHoaOutputs();
    for( const auto& os: outputs ){
        vec3 pos = os->mHoaElement->getPosition();
        vec2 pos2D(pos.x*scale,pos.y*scale);
        if( mHoaOutputHover == os ){
            gl::color(1, 0, 0);
        }else{
            gl::color(1, 1, 0);
        }
        if( os->bEnabled ){
            gl::drawSolidCircle(pos2D, 10);
        }else{
            gl::drawStrokedCircle(pos2D, 10);
        }
        gl::color(1, 1, 1);
        gl::drawString( toString(os->mHoaId+1), pos2D+vec2(20,0));
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
        if( s->bEnabled ){
            gl::drawSolidCircle(pos2D, 10);
        }else{
            gl::drawStrokedCircle(pos2D, 10);
        }
        gl::color(1, 1, 1);
        gl::drawString( s->mInput->getName(), pos2D+vec2(20,0));
   }
    gl::popMatrices();
}

void CinderProjectApp::mouseUp( MouseEvent e ){
    
    // enabling or disabling sound sources
    if( e.isRight() || e.isControlDown() ){
        if( mHoaOutputHover != nullptr ){
            mHoaOutputHover->bEnabled = !mHoaOutputHover->bEnabled;
        }else if ( mHoaSourceHover != nullptr ){
            mHoaSourceHover->bEnabled = !mHoaSourceHover->bEnabled;
        }
    }
}

void CinderProjectApp::mouseDrag( MouseEvent e ){

    // dragging sound sources around...
    
    vec2 mPos = vec2((float)e.getPos().x,(float)e.getPos().y) - vec2(getWindowWidth()/2, getWindowHeight()/2);
    float scale = getWindowWidth()*mDisplayScale;
    
    if( mHoaOutputHover != nullptr ){
        
        vec3 pos( mPos.x/scale, mPos.y/scale, 0 );
        pos /= length(pos);
        mHoaOutputHover->mHoaElement->setPosition(pos);
        mHoaNode->updateOutputPositions();
    }
    
    if( mHoaOutputHover == nullptr && mHoaSourceHover != nullptr ){
        vec3 pos( mPos.x/scale, mPos.y/scale, 0 );
        mHoaSourceHover->mHoaElement->setPosition(pos);
        mHoaNode->updatePositions();
    }
    
}

void CinderProjectApp::mouseMove( MouseEvent e ){

    if( !mHoaNode ) return;
    vec2 mPos = vec2((float)e.getPos().x,(float)e.getPos().y) - vec2(getWindowWidth()/2, getWindowHeight()/2);
    float scale = getWindowWidth()*mDisplayScale;
    
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

void CinderProjectApp::keyDown( KeyEvent event ){
    int val = event.getChar() - '0';
//    if( val > 0 && val < 7 ){
//        
//    }
//    if( event.getChar() == ' ' && mHoaOutputHover != nullptr ){
//        mHoaNode->getHoaOutput( mHoaOutputHover->mHoaId )->bEnabled = !mHoaNode->getHoaOutput( mHoaOutputHover->mHoaId )->bEnabled;
//    }
}

CINDER_APP( CinderProjectApp, RendererGl (RendererGl::Options().stencil().msaa (16)),
           [&] (App::Settings * settings)
{
    settings->setWindowSize (800, 800);
    settings->setFrameRate (60.0f);
    settings->setTitle ("Cinder HOA Wrapper Basic");
    settings->setHighDensityDisplayEnabled();
})
