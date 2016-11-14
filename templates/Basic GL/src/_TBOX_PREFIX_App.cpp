#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/audio/audio.h"

#include "HoaNode.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class _TBOX_PREFIX_App : public App {
  public:

	void setup();
	void update();
	void draw();
	
  nono::audio::HoaNodeRef               mHoaNode;

};

void _TBOX_PREFIX_App::setup()
{


}

void _TBOX_PREFIX_App::update()
{	
}

void _TBOX_PREFIX_App::draw()
{	
    gl::clear( Color( 0, 0, 0 ) );
    gl::color( Color( 1,1,1 ) );
         
}

CINDER_APP( _TBOX_PREFIX_App, RendererGl (RendererGl::Options().stencil().msaa (16)),
           [&] (App::Settings * settings)
{
    settings->setWindowSize (800, 800);
    settings->setFrameRate (60.0f);
    settings->setTitle ("Cinder HOA Node Example");
    settings->setHighDensityDisplayEnabled();
})
