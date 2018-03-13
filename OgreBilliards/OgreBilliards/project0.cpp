#include "project0.h"
#include "DynamicLines.h"

class MyMotionState : public btMotionState {
public:
	MyMotionState(const btTransform &initialpos, Ogre::SceneNode *node) {
		mVisibleobj = node;
		mPos1 = initialpos;
	}
	virtual ~MyMotionState() {    }
	void setNode(Ogre::SceneNode *node) {
		mVisibleobj = node;
	}
	virtual void getWorldTransform(btTransform &worldTrans) const {
		worldTrans = mPos1;
	}
	virtual void setWorldTransform(const btTransform &worldTrans) {
		if (NULL == mVisibleobj) return; // silently return before we set a node
		btQuaternion rot = worldTrans.getRotation();
		mVisibleobj->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		btVector3 pos = worldTrans.getOrigin();
		// TODO **** XXX need to fix this up such that it renders properly since this doesnt know the scale of the node
		// also the getCube function returns a cube that isnt centered on Z
		mVisibleobj->setPosition(pos.x(), pos.y() + 5, pos.z() - 5);
	}
protected:
	Ogre::SceneNode *mVisibleobj;
	btTransform mPos1;
};

BasicApp::BasicApp()
	: mShutdown(false),
	mRoot(0),
	mCamera(0),
	mSceneMgr(0),
	mWindow(0),
	mResourcesCfg(Ogre::StringUtil::BLANK),
	mPluginsCfg(Ogre::StringUtil::BLANK),
	mCameraMan(0),
	mRenderer(0),
	ghostNode(0),
	plane(Ogre::Vector3::UNIT_Y, 0),
	mMouse(0),
	mKeyboard(0),
	mInputMgr(0),
	mHUD(0),
	size(Ogre::Vector3::ZERO),
	ballType(true),
	collisionTimer(0),
	turn(0),
	yellow(0),
	red(0),
	yellowscore(0),
	redscore(0),
	yellowTurn(true),
	redTurn(false),
	yellowcount(7),
	redcount(7),
	canShoot(false),
	instructions(0),
	startButton(0),
	started(false),
	onTable(false),
	firstShot(true),
	gameOver(false)
{
	BallCollidesWith = COL_CUE | COL_BUMPER | COL_TABLE | COL_POCKET | COL_BALL;
	CueCollidesWith = COL_BALL | COL_TABLE | COL_BUMPER | COL_POCKET;
	BumperCollidesWith = COL_BALL | COL_TABLE | COL_CUE;
	TableCollidesWith = COL_BUMPER | COL_POCKET | COL_BALL | COL_CUE;
	PocketCollidesWith = COL_CUE | COL_TABLE | COL_BALL;
}

BasicApp::~BasicApp()
{
	// cleanup bulletdyanmics

	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	delete dynamicsWorld;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collisionConfiguration;
	if (mCameraMan) delete mCameraMan;

	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);

	delete mRoot;
}

void BasicApp::go()
{
#ifdef _DEBUG
	mResourcesCfg = "resources_d.cfg";
	mPluginsCfg = "plugins_d.cfg";
#else
	mResourcesCfg = "resources.cfg";
	mPluginsCfg = "plugins.cfg";
#endif

	if (!setup())
		return;

	mRoot->startRendering();

	destroyScene();
}

bool BasicApp::frameRenderingQueued(const Ogre::FrameEvent& fe)
{
	if (mKeyboard->isKeyDown(OIS::KC_ESCAPE))
		mShutdown = true;

	if (mShutdown)
		return false;

	if (mWindow->isClosed())
		return false;

	mKeyboard->capture();
	mMouse->capture();

	mCameraMan->frameRenderingQueued(fe);

	CEGUI::System::getSingleton().injectTimePulse(fe.timeSinceLastFrame);

	Ogre::SceneNode *lnode = dynamic_cast<Ogre::SceneNode*>(mSceneMgr->getRootSceneNode()->getChild("lines"));
	DynamicLines *lines = dynamic_cast<DynamicLines*>(lnode->getAttachedObject(0));
	if (cueBall->node != NULL && !cueBall->setdelete)
	{
		lines->setPoint(0, cueBall->node->getPosition());
		lines->setPoint(1, ghostNode->getPosition());
		lines->update();
		if (cueBall->body->isActive())
		{
			canShoot = false;
		}
		else
		{
			canShoot = true;
			lines->setVisible(true);
			mSceneMgr->getEntity("ghost")->setVisible(true);
		}
	}

	return true;
}

bool BasicApp::keyPressed(const OIS::KeyEvent& ke)
{
	mCameraMan->injectKeyDown(ke);

	return true;
}

bool BasicApp::keyReleased(const OIS::KeyEvent& ke)
{
	mCameraMan->injectKeyUp(ke);

	return true;
}

bool BasicApp::mouseMoved(const OIS::MouseEvent& me)
{
	CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
	context.injectMouseMove(me.state.X.rel, me.state.Y.rel);

	Ogre::Viewport* vp = mSceneMgr->getCurrentViewport();

	// get window height and width
	Ogre::Real x = me.state.X.abs / Ogre::Real(vp->getActualWidth());
	Ogre::Real y = me.state.Y.abs / Ogre::Real(vp->getActualHeight());

	// set up the ray
	Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(x, y);

	// check if the ray intersects our plane
	// intersects() will return whether it intersects or not (the bool value) and
	// what distance (the Real value) along the ray the intersection is
	std::pair<bool, Ogre::Real> result = mouseRay.intersects(plane);

	if (result.first) {
		// if the ray intersect the plane, we have a distance value
		// telling us how far from the ray origin the intersection occurred.
		// the last thing we need is the point of the intersection.
		// Ray provides us getPoint() function which returns a point
		// along the ray, supplying it with a distance value.

		// get the point where the intersection is
		Ogre::Vector3 point = mouseRay.getPoint(result.second);

		// position our ninja to that point
		if (point.x >= -49 && point.x <= 49 && point.z >= -25 && point.z <= 25 && started)
		{
			// canShoot = true;
			onTable = true;
			ghostNode->setPosition(point);
		}
		else
		{
			// canShoot = false;
			onTable = false;
		}
	}

	return true;
}

CEGUI::MouseButton convertButton(OIS::MouseButtonID id)
{
	switch (id)
	{
	case OIS::MB_Left:
		return CEGUI::LeftButton;
	case OIS::MB_Right:
		return CEGUI::RightButton;
	case OIS::MB_Middle:
		return CEGUI::MiddleButton;
	default:
		return CEGUI::LeftButton;
	}
}

bool BasicApp::mousePressed(const OIS::MouseEvent& me, OIS::MouseButtonID id)
{
	CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
	context.injectMouseButtonDown(convertButton(id));

	if (cueBall->node != NULL && !cueBall->setdelete && canShoot && started && onTable && !gameOver)
	{
		float Speed = speedValue->getCurrentValue();
		Shoot(Speed);

		if (yellowTurn)
		{
			yellowTurn = false;
			redTurn = true;
			turn->setText("It's Red's Turn");
		}
		else
		{
			redTurn = false;
			yellowTurn = true;
			turn->setText("It's Yellow's Turn");
		}
	}

	return true;
}

bool BasicApp::mouseReleased(const OIS::MouseEvent& me, OIS::MouseButtonID id)
{
	CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
	context.injectMouseButtonUp(convertButton(id));

	return true;
}

void BasicApp::windowResized(Ogre::RenderWindow* rw)
{
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);

	const OIS::MouseState& ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}

void BasicApp::windowClosed(Ogre::RenderWindow* rw)
{
	if (rw == mWindow)
	{
		if (mInputMgr)
		{
			mInputMgr->destroyInputObject(mMouse);
			mInputMgr->destroyInputObject(mKeyboard);

			OIS::InputManager::destroyInputSystem(mInputMgr);
			mInputMgr = 0;
		}
	}
}

bool BasicApp::setup()
{
	mRoot = new Ogre::Root(mPluginsCfg);

	setupResources();

	if (!configure())
		return false;

	chooseSceneManager();
	createCamera();
	createViewports();

	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

	createResourceListener();
	loadResources();
	setupCEGUI();
	createScene();
	createFrameListener();

	return true;
}

bool BasicApp::frameStarted(const Ogre::FrameEvent &evt)
{
	dynamicsWorld->stepSimulation(evt.timeSinceLastFrame);
	collisionTimer += evt.timeSinceLastFrame;

	if (collisionTimer > float(1 / 60))
	{
		CheckCollisions();
		collisionTimer = 0;
	}
	destroyCollided();
	return true;
}

bool BasicApp::configure()
{
	if (!(mRoot->restoreConfig() || mRoot->showConfigDialog()))
	{
		return false;
	}

	mWindow = mRoot->initialise(true, "Ogre Billiards");

	return true;
}

void BasicApp::chooseSceneManager()
{
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);
}

void BasicApp::createCamera()
{
	mCamera = mSceneMgr->createCamera("PlayerCam");
	Ogre::SceneNode* camNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	camNode->attachObject(mCamera);

	mCamera->setNearClipDistance(.1);

	mCameraMan = new OgreBites::SdkCameraMan(mCamera);
	mCameraMan->setStyle(OgreBites::CS_MANUAL);

	camNode->setPosition(Ogre::Vector3(0, 100, 0));
	camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD, Ogre::Vector3::NEGATIVE_UNIT_Z);
}

void BasicApp::createBulletSim(void) {
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new   btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -980, 0));

	// Setup Bullet objects -----------------------------------------------------------------------------------------------------------------------

	// Create ground

	Ogre::MeshPtr planePtr = Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane, 50, 100, 2, 2, true, 1, 5, 5, Ogre::Vector3::UNIT_Z);

	Ogre::Entity *groundEntity = mSceneMgr->createEntity("ground");
	Ogre::SceneNode *groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("groundNode");
	groundNode->rotate(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y));
	groundNode->attachObject(groundEntity);
	groundEntity->setMaterialName("Project/Table");

	//create the plane entity to the physics engine, and attach it to the node

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	btScalar groundMass(0.); //the mass is 0, because the ground is immovable (static)
	btVector3 localGroundInertia(0, 0, 0);

	btCollisionShape *groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
	btDefaultMotionState *groundMotionState = new btDefaultMotionState(groundTransform);

	groundShape->calculateLocalInertia(groundMass, localGroundInertia);

	btRigidBody::btRigidBodyConstructionInfo groundRBInfo(groundMass, groundMotionState, groundShape, localGroundInertia);
	groundRBInfo.m_friction = 10.f;
	groundRBInfo.m_restitution = 0.f;
	groundRBInfo.m_linearDamping = 0.f;
	groundRBInfo.m_angularDamping = 0.f;
	btRigidBody *groundBody = new btRigidBody(groundRBInfo);

	tableBodies.push_back(new CollisionObject(groundBody, groundNode, 0));
	groundBody->setUserPointer(tableBodies[tableBodies.size() - 1]);

	//add the body to the dynamics world
	dynamicsWorld->addRigidBody(groundBody, COL_TABLE, TableCollidesWith);

	// create bumpers
	CreateBumper(btVector3(0, 0, -19), 10000.f, btVector3(.95, .02, 0.01), "BackWall");
	CreateBumper(btVector3(0, 0, 28), 10000.f, btVector3(.95, .02, 0.01), "FrontWall");
	CreateBumper(btVector3(-48, 0, 4), 10000.f, btVector3(0.01, .02, .45), "LeftWall");
	CreateBumper(btVector3(48, 0, 4), 10000.f, btVector3(0.01, .02, .45), "RightWall");

	// create pockets
	CreatePocket(btVector3(0, 0, -19), 1.f, btVector3(.015, .015, 0.015), "Side1");
	CreatePocket(btVector3(0, 0, 28), 1.f, btVector3(.015, .015, 0.015), "Side2");
	CreatePocket(btVector3(-48, 0, -18), 1.f, btVector3(.015, .015, 0.015), "Corner1");
	CreatePocket(btVector3(48, 0, -18), 1.f, btVector3(.015, .015, 0.015), "Corner2");
	CreatePocket(btVector3(-48, 0, 27), 1.f, btVector3(.015, .015, 0.015), "Corner3");
	CreatePocket(btVector3(48, 0, 27), 1.f, btVector3(.015, .015, 0.015), "Corner4");
}

void BasicApp::CreatePocket(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name)
{
	Ogre::Entity *ballEntity;
	Ogre::SceneNode *ballNode;

	// empty ogre vectors for the cubes size and position 
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::Vector3 size = Ogre::Vector3::ZERO;

	// Convert the bullet physics vector to the ogre vector 
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();

	// create the enity for shape ----------------------------------------------------------------------------------------------------------
	ballEntity = mSceneMgr->createEntity(name, "sphere.mesh"); // name = entity name, "cube.mes = mesh name to be based on
	ballEntity->setCastShadows(true);

	ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	ballNode->attachObject(ballEntity);

	// scale the shape based on parameters
	ballNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));

	// setup bounding box --------------------------------------------------------------------------------------------------------------------
	Ogre::AxisAlignedBox boundingB = ballEntity->getBoundingBox();
	
	// set bounding box scale to match node
	boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	// make the shape size slightly less than the bounding box
	size = boundingB.getSize()*.9;

	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(Position);

	MyMotionState *MotionState = new MyMotionState(Transform, ballNode); // initial position, node to get state from

	// setup rigid body ------------------------------------------------------------------------------------------------------------------------
	btCollisionShape *Shape = new btSphereShape(size.x*0.5); // parameter is vector3 for size of the box
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(Mass, LocalInertia);
	btRigidBody::btRigidBodyConstructionInfo bodyInfo(Mass, MotionState, Shape, LocalInertia);
	bodyInfo.m_friction = 0.f;
	bodyInfo.m_restitution = 0.f;
	btRigidBody *RigidBody = new btRigidBody(bodyInfo);
	RigidBody->setFlags(2);

	// Store a pointer to the Ogre Node so we can update it later
	sphereBodies.push_back(new CollisionObject(RigidBody, ballNode, 3));
	RigidBody->setUserPointer(sphereBodies[sphereBodies.size() - 1]);

	// Add it to the physics world
	dynamicsWorld->addRigidBody(RigidBody, COL_POCKET, PocketCollidesWith);
}

void BasicApp::CreateBumper(const btVector3 &Position, btScalar Mass, const btVector3 &scale, char * name){
	// empty ogre vectors for the cubes size and position 
	Ogre::Vector3 size = Ogre::Vector3::ZERO;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;
	Ogre::SceneNode *boxNode;
	Ogre::Entity *boxentity;
	// Convert the bullet physics vector to the ogre vector 
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();

	boxentity = mSceneMgr->createEntity(name, "cube.mesh");
	boxentity->setMaterialName("Examples/Rockwall");
	boxentity->setCastShadows(true);

	boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	boxNode->attachObject(boxentity);
	boxNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));

	Ogre::AxisAlignedBox boundingB = boxentity->getBoundingBox();
	
	boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
	size = boundingB.getSize();
	
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(btVector3(Position.getX(), Position.getY()+(size.y*.5), Position.getZ()));

	MyMotionState *MotionState = new MyMotionState(Transform, boxNode);
	//Give the rigid body half the size 
	// of our cube and tell it to create a BoxShape (cube) 
	btVector3 HalfExtents(size.x*0.5f, size.y*0.5f, size.z*0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(Mass, LocalInertia);

	btRigidBody::btRigidBodyConstructionInfo bodyInfo(Mass, MotionState, Shape, LocalInertia);
	// bodyInfo.m_rollingFriction = 0.05f;
	bodyInfo.m_friction = 0.1f;
	bodyInfo.m_restitution = 0.15f;
	bodyInfo.m_linearDamping = 0.f;
	bodyInfo.m_angularDamping = 0.f;

	btRigidBody *RigidBody = new btRigidBody(bodyInfo);
	RigidBody->setFlags(2);
	tableBodies.push_back(new CollisionObject(RigidBody, boxNode, 0));
	RigidBody->setUserPointer(tableBodies[tableBodies.size() - 1]);
	
	// Add it to the physics world 
	dynamicsWorld->addRigidBody(RigidBody, COL_BUMPER, BumperCollidesWith);
}

void BasicApp::createScene() 
{
	bool infiniteClip =
		mRoot->getRenderSystem()->getCapabilities()->hasCapability(
		Ogre::RSC_INFINITE_FAR_PLANE);

	if (infiniteClip)
		mCamera->setFarClipDistance(0);
	else
		mCamera->setFarClipDistance(50000);

	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);

	Ogre::Vector3 lightDir(0.55, 0.3, 0.75);
	lightDir.normalise();
	Ogre::Light* light = mSceneMgr->createLight("SceneLight");
	light->setType(Ogre::Light::LT_DIRECTIONAL);
	light->setDirection(lightDir);
	light->setDiffuseColour(Ogre::ColourValue(0.4, 0.4, 0.4));
	light->setSpecularColour(Ogre::ColourValue(0.2, 0.2, 0.2));

	// Setup Physics Engine --------------------------------------------------------------------------------------------------------------
	createBulletSim();
	rackBalls();

	ghostNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("ghostNode");
	Ogre::Entity* ghostBall = mSceneMgr->createEntity("ghost", "sphere.mesh");
	ghostNode->setScale(.01, .01, .01);
	ghostBall->setCastShadows(false);
	ghostBall->setMaterialName("Custom/BrushedMetal");
	ghostNode->attachObject(ghostBall);

	// Setup Line Renderer ---------------------------------------------------------------------------------------------------------------
	Ogre::Vector3 ghostPosition = ghostNode->getPosition();
	Ogre::Vector3 cuePosition = cueBall->node->getPosition();

	DynamicLines *lines = new DynamicLines(Ogre::RenderOperation::OT_LINE_LIST);
	lines->addPoint(cuePosition.x, 1, cuePosition.z);
	lines->addPoint(ghostPosition.x, 1, ghostPosition.z);
	lines->update();
	Ogre::SceneNode *linesNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("lines");
	linesNode->attachObject(lines);
}

void BasicApp::Shoot(float Speed)
{
	Ogre::Vector3 ogreDirection = Ogre::Vector3((ghostNode->getPosition()) - (cueBall->node->getPosition())); // change to mouse position - cue position
	btVector3 Direction = btVector3(ogreDirection.x, ogreDirection.y, ogreDirection.z);
	
	btVector3 nonConstVelocity;
	nonConstVelocity = Direction;
	nonConstVelocity.normalize();
	btVector3 rel = nonConstVelocity;
	
	if (firstShot)
	{
		float breakSpeed = Speed*4;
		nonConstVelocity *= breakSpeed;
	}
	else
	{
		nonConstVelocity *= Speed;
	}	
	
	cueBall->body->activate(true);
	cueBall->body->applyCentralImpulse(nonConstVelocity);

	Ogre::SceneNode *lnode = dynamic_cast<Ogre::SceneNode*>(mSceneMgr->getRootSceneNode()->getChild("lines"));
	DynamicLines *lines = dynamic_cast<DynamicLines*>(lnode->getAttachedObject(0));
	lines->setVisible(false);

	mSceneMgr->getEntity("ghost")->setVisible(false);
}

bool BasicApp::quit(const CEGUI::EventArgs &e)
{
	mShutdown = true;
	return true;
}

bool BasicApp::start(const CEGUI::EventArgs &e)
{
	started = true;
	startButton->setVisible(false);
	instructions->setVisible(false);
	return true;
}

void BasicApp::destroyScene()
{
}

void BasicApp::createFrameListener()
{
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

	mInputMgr = OIS::InputManager::createInputSystem(pl);

	mKeyboard = static_cast<OIS::Keyboard*>(
		mInputMgr->createInputObject(OIS::OISKeyboard, true));
	mMouse = static_cast<OIS::Mouse*>(
		mInputMgr->createInputObject(OIS::OISMouse, true));

	mKeyboard->setEventCallback(this);
	mMouse->setEventCallback(this);

	windowResized(mWindow);
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mRoot->addFrameListener(this);

	Ogre::LogManager::getSingletonPtr()->logMessage("Finished");
}

void BasicApp::createViewports()
{
	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

	mCamera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) /
		Ogre::Real(vp->getActualHeight()));
}

void BasicApp::setupResources()
{
	Ogre::ConfigFile cf;
	cf.load(mResourcesCfg);

	Ogre::String secName, typeName, archName;
	Ogre::ConfigFile::SectionIterator secIt = cf.getSectionIterator();

	while (secIt.hasMoreElements())
	{
		secName = secIt.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap* settings = secIt.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator setIt;

		for (setIt = settings->begin(); setIt != settings->end(); ++setIt)
		{
			typeName = setIt->first;
			archName = setIt->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
	}
}

void BasicApp::createResourceListener()
{
}

void BasicApp::loadResources()
{
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void BasicApp::setupCEGUI()
{
	mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();

	// Initialize all CEGUI resources
	CEGUI::ImageManager::setImagesetDefaultResourceGroup("Imagesets");
	CEGUI::Font::setDefaultResourceGroup("Fonts");
	CEGUI::Scheme::setDefaultResourceGroup("Schemes");
	CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
	CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
	CEGUI::SchemeManager::getSingleton().createFromFile("TaharezLook.scheme");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().setDefaultImage("TaharezLook/MouseArrow");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().setPosition(CEGUI::Vector2f(0, 0));

	// create the window manager
	CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();

	// load the layout and render it to the screen
	mHUD = wmgr.loadLayoutFromFile("billiards.layout");
	CEGUI::System::getSingleton().getDefaultGUIContext().setRootWindow(mHUD);

	speedValue = static_cast<CEGUI::Slider*>(mHUD->getChildRecursive("Slider"));
	speedValue->setMaxValue(30.0f);
	speedValue->setClickStep(5.0f);
	speedValue->setCurrentValue(30.0f);

	turn = static_cast<CEGUI::Window*>(mHUD->getChildRecursive("StaticText"));
	yellow = static_cast<CEGUI::Window*>(mHUD->getChildRecursive("Titlebar"));
	red = static_cast<CEGUI::Window*>(mHUD->getChildRecursive("Titlebar2"));
	instructions = static_cast<CEGUI::Window*>(mHUD->getChildRecursive("Instructions"));
	startButton = static_cast<CEGUI::PushButton*>(mHUD->getChildRecursive("StartButton"));

	turn->setText("It's Yellow's Turn");
	yellow->setText("Yellow Score: 0");
	red->setText("Red Score: 0");

	// subscribe the quit function to the QuitButton window
	mHUD->getChildRecursive("QuitButton")->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&BasicApp::quit, this));
	startButton->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&BasicApp::start, this));
}

void BasicApp::rackBalls()
{
	Ogre::String shapeName;
	btVector3 shapeScale = btVector3(0.01, 0.01, 0.01);
	Ogre::Vector3 pos = Ogre::Vector3(25, 0, 5);
	btVector3 targetPos = btVector3(pos.x, pos.y + (size.x), pos.z);
	int count = 0;
	int numRows = 5;
	bool left = true;
	btVector3 cueTarget = btVector3(-33, 0, 5);

	for (int i = 1; i <= numRows; i++) 
	{
		targetPos.setX(pos.x + ((size.x) * (i - 1)));

		int rowBalls = i;
		int shift = i;
		if (i % 2 != 0) // check if there will be an odd or even number of balls in the current row
		{
			count++;
			shapeName = Ogre::StringConverter::toString(count);
			createBall(targetPos, .25f, shapeScale, shapeName);
			rowBalls--;
		}
		while (rowBalls > 0)
		{
			if (left) // adding a ball to the left of the triangle
			{
				targetPos.setZ(pos.z - ((size.x * .5) * (shift - 1))); // start at the outter left edge
				count++;
				shapeName = Ogre::StringConverter::toString(count);
				createBall(targetPos, .25f, shapeScale, shapeName);
				left = !left; // toggle sides
				rowBalls--;
			}
			else // adding a ball to the right of the triangle
			{
				targetPos.setZ(pos.z + ((size.x * .5) * (shift - 1))); // move to the outter right edge
				count++;
				shapeName = Ogre::StringConverter::toString(count);
				createBall(targetPos, .25f, shapeScale, shapeName);
				left = !left; // toggle sides
				rowBalls--;
				shift--; // move in to fill in the row
			}
		}
	}

	createBall(cueTarget, 1.f, btVector3(.01, .01, .01), "cue");
}

void BasicApp::createBall(btVector3 &Position, btScalar Mass, const btVector3 &scale, Ogre::String name)
{
	Ogre::Entity *ballEntity;

	// empty ogre vectors for the cubes size and position 
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;

	// Convert the bullet physics vector to the ogre vector 
	pos.x = Position.getX();
	pos.y = Position.getY();
	pos.z = Position.getZ();

	// create the enity for shape ----------------------------------------------------------------------------------------------------------
	ballEntity = mSceneMgr->createEntity(name, "sphere.mesh"); // name = entity name, "cube.mes = mesh name to be based on
	ballEntity->setCastShadows(true);

	// set node to attach shape -------------------------------------------------------------------------------------------------------------
	if (name == "cue")
	{
		Ogre::SceneNode *cueNode;
		ballEntity->setMaterialName("Custom/BrushedMetal");
		cueNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

		cueNode->attachObject(ballEntity);

		// scale the shape based on parameters
		cueNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));

		// setup bounding box --------------------------------------------------------------------------------------------------------------------
		Ogre::AxisAlignedBox boundingB = ballEntity->getBoundingBox();

		// set bounding box scale to match node
		boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
		// make the shape size slightly less than the bounding box
		size = boundingB.getSize()*.95;

		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(Position);

		MyMotionState *MotionState = new MyMotionState(Transform, cueNode); // initial position, node to get state from

		// setup rigid body ------------------------------------------------------------------------------------------------------------------------
		btCollisionShape *Shape = new btSphereShape(size.x*0.5); // parameter is vector3 for size of the box
		btVector3 LocalInertia;
		Shape->calculateLocalInertia(Mass, LocalInertia);
		btRigidBody::btRigidBodyConstructionInfo bodyInfo(Mass, MotionState, Shape, LocalInertia);
		bodyInfo.m_rollingFriction = 5.f;
		bodyInfo.m_friction = 1.f;
		bodyInfo.m_restitution = 0.15f;
		// bodyInfo.m_linearDamping = .45f;
		// bodyInfo.m_angularDamping = .5f;
		bodyInfo.m_linearSleepingThreshold = 3.f;
		bodyInfo.m_angularSleepingThreshold = 3.f;
		btRigidBody *RigidBody = new btRigidBody(bodyInfo);

		// store the cue
		cueBall = new CollisionObject(RigidBody, cueNode, 1);

		RigidBody->setUserPointer(cueBall);
	
		// Add it to the physics world
		dynamicsWorld->addRigidBody(RigidBody, COL_CUE, CueCollidesWith);
	}
	else
	{
		ballType = !ballType;
		Ogre::SceneNode *ballNode;
		if (name == "4")
			ballEntity->setMaterialName("Custom/Blue");
		else if (name == "15")
			ballEntity->setMaterialName("Custom/Red");
		else if (ballType)
			ballEntity->setMaterialName("Custom/Red");
		else
			ballEntity->setMaterialName("Custom/Yellow");

		ballNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
		ballNode->attachObject(ballEntity);

		// scale the shape based on parameters
		ballNode->scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));

		// setup bounding box --------------------------------------------------------------------------------------------------------------------
		Ogre::AxisAlignedBox boundingB = ballEntity->getBoundingBox();

		// set bounding box scale to match node
		boundingB.scale(Ogre::Vector3(scale.getX(), scale.getY(), scale.getZ()));
		// make the shape size slightly less than the bounding box
		size = boundingB.getSize()*.95;

		btTransform Transform;
		Transform.setIdentity();
		Transform.setOrigin(Position);

		MyMotionState *MotionState = new MyMotionState(Transform, ballNode); // initial position, node to get state from

		// setup rigid body ------------------------------------------------------------------------------------------------------------------------
		btCollisionShape *Shape = new btSphereShape(size.x*0.5); // parameter is vector3 for size of the box
		btVector3 LocalInertia;
		Shape->calculateLocalInertia(Mass, LocalInertia);
		btRigidBody::btRigidBodyConstructionInfo bodyInfo(Mass, MotionState, Shape, LocalInertia);
		bodyInfo.m_rollingFriction = 5.f;
		bodyInfo.m_friction = 1.f;
		bodyInfo.m_restitution = 0.15f;
		// bodyInfo.m_linearDamping = .45f;
		// bodyInfo.m_angularDamping = .5f;
		bodyInfo.m_linearSleepingThreshold = 3.f;
		bodyInfo.m_angularSleepingThreshold = 3.f;
		btRigidBody *RigidBody = new btRigidBody(bodyInfo);

		// Store a pointer to the Ogre Node so we can update it later
		sphereBodies.push_back(new CollisionObject(RigidBody, ballNode, 2));
		RigidBody->setUserPointer(sphereBodies[sphereBodies.size() - 1]);

		// Add it to the physics world
		dynamicsWorld->addRigidBody(RigidBody, COL_BALL, BallCollidesWith);
	}
}

void BasicApp::CheckCollisions(){
	int TotalManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();

	for (int i = 0; i < TotalManifolds; i++){
		btPersistentManifold* Manifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		//We're not only going to record if one of the objects was a Box, but actually get the pointer to the box in case want to manipulate it further
		btCollisionObject *BoxObject0(0), *BoxObject1(0);

		//getBody0 and getBody1 access the actual collision objects.
		BoxObject0 = const_cast<btCollisionObject*>(Manifold->getBody0());
		BoxObject1 = const_cast<btCollisionObject*>(Manifold->getBody1());

		CollisionObject* object1 = (CollisionObject*)BoxObject0->getUserPointer();
		CollisionObject* object2 = (CollisionObject*)BoxObject1->getUserPointer();

		//Ball hits pocket
		if ((object1->id == 2) && (object2->id == 3)){
			object1->setdelete = true;
		}
		// Pocket hits ball
		else if ((object1->id == 3) && (object2->id == 2)){
			object2->setdelete = true;

		}
		// cue hits pocket
		else if ((object1->id == 1) && (object2->id == 3)){
			// scratch
			object1->setdelete = true;

		}
		// pocket hits cue
		else if ((object1->id == 3) && (object2->id == 1)){
			// scratch
			object2->setdelete = true;
		}
	}
}

void BasicApp::destroyCollided()
{
	if (cueBall->setdelete == true)
	{
		firstShot = false;
		destroyObject(cueBall);

		btVector3 cueTarget = btVector3(-33, 0, 5);
		createBall(cueTarget, .25f, btVector3(.01, .01, .01), "cue");
	}
	for (int i = sphereBodies.size() - 1; i >= 0; --i)
	{
		if (sphereBodies[i]->setdelete == true)
		{
			Ogre::Entity* temp = static_cast<Ogre::Entity*>(sphereBodies[i]->node->getAttachedObject(0));
			std::string material = temp->getSubEntity(0)->getMaterialName();
			if (material == "Custom/Yellow")
			{
				// add score to yellow
				yellowscore += 1;
				yellowcount--;
				std::string score = std::to_string(yellowscore);
				yellow->setText("Yellow Score: " + score);

				std::swap(sphereBodies[i], sphereBodies.back());
				destroyObject(sphereBodies.back());
				sphereBodies.pop_back();
				sphereBodies.shrink_to_fit();
			}
			else if (material == "Custom/Red")
			{
				// add score to red
				redscore += 1;
				redcount--;
				std::string score = std::to_string(redscore);
				red->setText("Red Score: " + score);

				std::swap(sphereBodies[i], sphereBodies.back());
				destroyObject(sphereBodies.back());
				sphereBodies.pop_back();
				sphereBodies.shrink_to_fit();
			}
			else if (material == "Custom/Blue")
			{
				if (yellowTurn && redcount==0)//red hit it in
				{
					std::swap(sphereBodies[i], sphereBodies.back());
					destroyObject(sphereBodies.back());
					sphereBodies.pop_back();
					sphereBodies.shrink_to_fit();
					turn->setText("Game Over Red Wins!");
					gameOver = true;
				}
				else if (redTurn && yellowcount == 0)//yellow hit it in
				{
					std::swap(sphereBodies[i], sphereBodies.back());
					destroyObject(sphereBodies.back());
					sphereBodies.pop_back();
					sphereBodies.shrink_to_fit();
					turn->setText("Game Over Yellow Wins!");
					gameOver = true;
				}
				else //not all players balls are in yet
				{
					std::swap(sphereBodies[i], sphereBodies.back());
					destroyObject(sphereBodies.back());
					sphereBodies.pop_back();
					sphereBodies.shrink_to_fit();
					createBall(btVector3(0,0,0), .25f, btVector3(.01, .01, .01), "4");
				}
			}
		}
	}
}

void BasicApp::destroyObject(CollisionObject * ptrToOgreObject)
{

	// delete the ogre aspect of the object 

	// detach the entity from the parent sceneNode, destroy the entity, destroy the sceneNode, and set the sceneNode to NULL 
	Ogre::Entity* ent = ((Ogre::Entity*)ptrToOgreObject->node->getAttachedObject(0));
	ptrToOgreObject->node->getAttachedObject(0)->detachFromParent();

	mSceneMgr->destroyEntity(ent);
	mSceneMgr->destroySceneNode(ptrToOgreObject->node);
	ptrToOgreObject->node = NULL;

	// delete the bullet aspect of the object, ours should always have motion state 
	if (ptrToOgreObject->body && ptrToOgreObject->body->getMotionState())
		delete ptrToOgreObject->body->getMotionState();

	delete ptrToOgreObject->body->getCollisionShape();

	dynamicsWorld->removeCollisionObject(ptrToOgreObject->body);
	ptrToOgreObject->body = NULL;

	delete ptrToOgreObject;
	ptrToOgreObject = NULL;
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
	INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
	int main(int argc, char *argv[])
#endif
	{
		BasicApp app;

		try
		{
			app.go();
		}
		catch (Ogre::Exception& e)
		{
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
			MessageBox(
				NULL,
				e.getFullDescription().c_str(),
				"An exception has occured!",
				MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
			std::cerr << "An exception has occured: " <<
				e.getFullDescription().c_str() << std::endl;
#endif
		}

		return 0;
	}

#ifdef __cplusplus
}
#endif