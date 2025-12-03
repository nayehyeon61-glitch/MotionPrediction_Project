#include "stdafx.h"

//#include "../../MainLib/WrapperLua/ScriptWin.h" // if you want to detach PhysicsLib dependencies
#include "../../PhysicsLib/ScriptBaseWin.h"
#ifdef NO_GUI
#include "../../MainLib/console/dummies.h"
#endif
#include "stdafx.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/MovableText.h"
#include "../../MainLib/OgreFltk/Loader.h"
//#include "../../MainLib/OgreFltk/Joystick.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
//#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
#include "../../BaseLib/utility/checkPoints.h"
#include "../../BaseLib/motion/MotionRetarget.h"
#include "../../BaseLib/math/hyperMatrixN.h"
class MotionPanel;
class FrameSensor;
class PhysicsWin : public ScriptBaseWin
{
public:
	PhysicsWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~PhysicsWin (void);



	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// PLDPrimSkin::DrawCallback
	virtual void draw(const Motion& mot, int iframe);


	// FrameSensor::Trigger 
	virtual void Triggered(FrameSensor* pTimer);


	virtual void initLuaEnvironment();

	void show();
	void hide();
	void firstInit();

	virtual int work(TString const& workname, lunaStack& L);
};



//enum { WIDTH=1275-(1000-640), HEIGHT=700, RENDERER_WIDTH=640, RENDERER_HEIGHT=260, RENDERER=0};// dump video2 (half view)
//enum { WIDTH=1275-(1000-640), HEIGHT=700, RENDERER_WIDTH=640, RENDERER_HEIGHT=300, RENDERER=0};// classification 1024*757
//enum { WIDTH=1024, HEIGHT=768, RENDERER_WIDTH=800, RENDERER_HEIGHT=600, RENDERER=0};	// demo
//enum { WIDTH=632, HEIGHT=453, RENDERER_WIDTH=500, RENDERER_HEIGHT=275, RENDERER=1};// classification demo
//enum { WIDTH=1275-(1000-640), HEIGHT=1024, RENDERER_WIDTH=640, RENDERER_HEIGHT=500, RENDERER=0};// classification

namespace RE
{
	extern Globals* g_pGlobals;
}
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
class MainWinNoPython : public Fl_Window
{
public:
	MainWinNoPython(int w, int h, int rw, int rh, OgreRenderer * pOgreRenderer, const char* title=NULL) 
	: Fl_Window(w, h, title)		
	{

#ifndef NO_GUI
		m_tile=new Fl_Tile (0,0,w,h);
#endif
		int RENDERER_WIDTH=rw;
		int RENDERER_HEIGHT=rh;
		int WIDTH=w;
		int HEIGHT=h;
		
		m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);
		m_Renderer=new FltkRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
		m_Renderer->end();
		
		int ww, hh;

		ww=WIDTH-RENDERER_WIDTH;
		hh=RENDERER_HEIGHT;

		m_pRightWin=new PhysicsWin(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer);

#ifndef NO_GUI
		m_tile->end();
#endif
		end();		

#ifndef NO_GUI
		resizable(this);
#endif
	}
	~MainWinNoPython()
	{		
	}

#ifndef NO_GUI
	Fl_Tile* m_tile;
#endif
	FltkRenderer* m_Renderer;
	PhysicsWin* m_pRightWin;
	MotionPanel* m_motionPanel;
};


static OgreRenderer* g_pRenderer=NULL;
static MainWinNoPython* g_pMainWin=NULL;
extern ConfigTable config;
static void createMainWin()
{

	if(g_pRenderer) { printf("mainwin already created\n"); return;}
	srand((unsigned)time( NULL ));

	auto* _renderer=new OgreRenderer();

	try {
		int rw=config.GetInt("renderer_width");
		int rh=config.GetInt("renderer_height");
		int w=rw+config.GetInt("right_panel_width");
		int h=rh+config.GetInt("right_panel_width");
		//netlab::gptest();
		//netlab::gptest2();

#ifndef NO_GUI
		if (!Fl::visual(FL_DOUBLE|FL_INDEX))
			printf("Xdbe not supported, faking double buffer with pixmaps.\n"); 
		Fl::scheme("plastic");
#endif
		g_pRenderer=_renderer;

		OgreRenderer& renderer=*g_pRenderer;
		{
			g_pMainWin=new MainWinNoPython(w,h,rw,rh,&renderer, "KickBoxer");
			MainWinNoPython& win=*g_pMainWin;

		}
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e ) 
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}	
#endif
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::runtime_error& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}
}

void showMainWin()
{
	MainWinNoPython& win=*g_pMainWin;
	win.show();

	win.m_Renderer->firstInit(&win);

	{
		LUAwrapper L;
		Register_baselib(L.L);
		Register_mainlib(L.L);

		L.dofile("../Resource/scripts/loadBG_default.lua");
	}
}

void startMainLoop()
{
	try
	{
		MainWinNoPython& win=*g_pMainWin;
		win.m_Renderer->loop(win);	
	}
#ifndef NO_OGRE
	catch( Ogre::Exception& e ) 
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}	
#endif
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::runtime_error& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}
}
PhysicsWin* getPhysicsWin()
{
	return g_pMainWin->m_pRightWin;
}
extern int snippetMain(int, const char*const*);
void earlyInitPhysX();
int main(int argc, const char* argv[])
{ 
	earlyInitPhysX();
#ifdef RENDER_SNIPPET
	printf("a\n");
	snippetMain(argc, argv);
	printf("b\n");
#endif

	std::string scriptFile="../../taesooLib/Samples/scripts/RigidBodyWin/GUI_tools/WRLviewer.lua";
	std::string option;
    if (argc==2)
        scriptFile=argv[1];
    else if (argc==3)
	{
        option=argv[1];
        scriptFile=argv[2];
	}
	std::cout<<argc <<" "<<scriptFile<< " "<<option<<std::endl;
	createMainWin();
	showMainWin();

	TString opt=option.c_str();
	if(opt.subString(0,10)=="--dostring")
	{
		getPhysicsWin()->__loadEmptyScript();
		printf("dostring %s\n", opt.subString(11).ptr());
		getPhysicsWin()->dostring(opt.subString(11));
		getPhysicsWin()->dofile(scriptFile.c_str());
		getPhysicsWin()->dostring("ctor()");
	}
	else
	{
		printf("loading %s\n", scriptFile.c_str());
		getPhysicsWin()->__loadScript(scriptFile.c_str());
	}

	startMainLoop();

	//system("leaks nopython_d"); for mac osx debugging
	return 0;
}


	

//////////////////////////////////////////////////////////////////////////////
// extend

PhysicsWin::PhysicsWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
//:FlLayout(x,y,w,h),m_motionPanel(mp),mRenderer(renderer)
:ScriptBaseWin(x,y,w,h,mp, renderer,  "WRLviewer.lua",  "../Samples/scripts/RigidBodyWin/")
{

	//removeWidgets(-2);
	//setUniformGuidelines(10);
	//create("Button", "X", "X", 9);
	updateLayout();

#ifndef NO_OGRE
	renderer.setHandler(this);
#endif
	renderer.ogreRenderer().addFrameMoveObject(this);
}

PhysicsWin::~PhysicsWin(void)
{
}

#include <fstream>
void readFile(TString & a, const char* fn)
{
	std::ifstream inFile;
	inFile.open(fn);
	if(inFile)
	{
		char buf[1000];
		while(!inFile.eof())
		{
			inFile.getline(buf,1000,'\n');
#ifdef _DEBUG
			// import mainlibÀ» import mainlib_debug as mainlibÀ¸?? Ä¡È¯???Ö¾????Ñ´?.

			TString temp=buf;
			if(temp.find("import mainlib")!=-1)
				sprintf(buf, "import mainlib_debug as mainlib");

#endif
			a.add("%s\n", buf);
		}
		inFile.close();
	}
}


void PhysicsWin::show()
{
	FlLayout::show();
#ifndef NO_GUI
	m_renderer->setHandler(this);
#endif
}

void PhysicsWin::hide()
{
	FlLayout::hide();
}
void PhysicsWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	ScriptBaseWin::onCallback(w, pWidget, userData);
}


// PLDPrimSkin::DrawCallback
void PhysicsWin::draw(const Motion& mot, int iframe)
{
}


// FrameSensor::Trigger 
void PhysicsWin::Triggered(FrameSensor* pTimer)
{
}

bool OIS_event_ctrl();
bool OIS_event_shift();
bool OIS_event_alt();


void Register_classificationLib_bind(lua_State*L);
void Register_QP(lua_State*L);
void PhysicsWin::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_QP(L);
}
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
int PhysicsWin::work(TString const& workname, lunaStack& L)
{
	if (workname=="hasPython")
	{
		L<<false;
		return 1;
	}

	else return ScriptBaseWin::work(workname, L);
}
