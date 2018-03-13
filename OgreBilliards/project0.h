#ifndef BASICAPP_H
#define BASICAPP_H

// Ogre Includes ------------------------------------------
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreEntity.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>
#include <OgreLogManager.h>
#include "OgreManualObject.h"
// OIS Includes --------------------------------------------
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>
// CEGUI Includes ------------------------------------------
#include <CEGUI/CEGUI.h>
#include <CEGUI/RendererModules/Ogre/Renderer.h>
// Bullet Includes -----------------------------------------
#include "btBulletDynamicsCommon.h"
#include "btHeightfieldTerrainShape.h"
// Camera Includes
#include <SdkCameraMan.h>
// Standard Includes
#include <vector>
#include <string>

#define BIT(x) (1<<(x)) 
enum collisionTypes{
	COL_NOTHING = 0,
	COL_TABLE = BIT(0),
	COL_BUMPER = BIT(1),
	COL_POCKET = BIT(2),
	COL_CUE = BIT(3),
	COL_BALL = BIT(4)
};

struct CollisionObject{
	int id;
	Ogre::SceneNode* node;
	btRigidBody* body;
	bool setdelete;
	CollisionObject(btRigidBody* b, Ogre::SceneNode* n, int i) : body(b), node(n), id(i) { setdelete = false; timer = 0; color = "white"; }
	std::vector<int> indexes;
	float timer;
	std::string color;
};

class BasicApp
	: public Ogre::WindowEventListener,
	public Ogre::FrameListener,
	public OIS::KeyListener,
	public OIS::MouseListener
{
public:
	BasicApp();
	~BasicApp();

	void go();

	float collisionTimer;
	int yellowscore;
	int redscore;
	bool redTurn;
	bool yellowTurn;
	int yellowcount;
	int redcount;
	bool canShoot;
	bool started;
	bool onTable;
	bool firstShot;
	bool gameOver;

private:
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& fe);
	bool frameStarted(const Ogre::FrameEvent &evt);

	Ogre::Vector3 size;

// Input System -------------------------------------------------------------------------
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
	virtual bool mouseMoved(const OIS::MouseEvent& me);
	virtual bool mousePressed(const OIS::MouseEvent& me, OIS::MouseButtonID id);
	virtual bool mouseReleased(const OIS::MouseEvent& me, OIS::MouseButtonID id);
	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

	// OIS
	OIS::Mouse* mMouse;
	OIS::Keyboard* mKeyboard;
	OIS::InputManager* mInputMgr;

// Setup ----------------------------------------------------------------------
	bool setup();
	bool configure();
	void chooseSceneManager();
	void createCamera();
	void createScene();
	void destroyScene();
	void createFrameListener();
	void createViewports();
	void setupResources();
	void createResourceListener();
	void loadResources();

	bool mShutdown;
	Ogre::Root* mRoot;
	Ogre::Camera* mCamera;
	Ogre::SceneManager* mSceneMgr;
	Ogre::RenderWindow* mWindow;
	Ogre::String mResourcesCfg;
	Ogre::String mPluginsCfg;
	OgreBites::SdkCameraMan* mCameraMan;
	Ogre::SceneNode* ghostNode;
	Ogre::Plane plane;

// CEGUI ------------------------------------------------------------------------
	void setupCEGUI();

	CEGUI::OgreRenderer* mRenderer;
	CEGUI::Window* mHUD;
	CEGUI::Slider* speedValue;
	CEGUI::Window* turn;
	CEGUI::Window* yellow;
	CEGUI::Window* red;
	CEGUI::Window* instructions;
	CEGUI::PushButton* startButton;

// Bullet ---------------------------------------------------------------------------------
	
	void createBulletSim(void);
	void CreateBumper(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name);
	void CreatePocket(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name);
	void CheckCollisions();
	void destroyObject(CollisionObject * ptrToOgreObject);
	void destroyCollided();
	void sweepCollisions();
	void rackBalls();
	void createBall(btVector3 &Position, btScalar Mass, const btVector3 &scale, Ogre::String name);
	void Shoot(float Speed);

	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	std::map<std::string, btRigidBody *> physicsAccessors;
	CollisionObject* cueBall;
	std::vector<CollisionObject*> sphereBodies;
	std::vector<CollisionObject*> tableBodies;
	bool ballType;

protected:
	bool quit(const CEGUI::EventArgs &e);
	bool start(const CEGUI::EventArgs &e);

	int BallCollidesWith;
	int CueCollidesWith;
	int BumperCollidesWith;
	int TableCollidesWith;
	int PocketCollidesWith;
};

#endif