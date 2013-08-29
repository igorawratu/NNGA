#include "graphicsengine.h"

GraphicsEngine::GraphicsEngine(SimulationContainer* _simulation) : mSimulation(_simulation){

    //debug and release plugins use different binaries thus different plugin config files need to be loaded
#ifdef _DEBUG
    mPlugins = "plugins_d.cfg";
#else
    mPlugins = "plugins.cfg";
#endif

    //create new ogre root
    mRoot = new Ogre::Root(mPlugins);

    //create window with its own input system
    mWindowManager = new WindowManager(mRoot);

    //adds this class as a frame listener
    mRoot->addFrameListener(this);

    //after this occurs, can then load resources in resource manager
    mResourceManager = new ResourceManager;

}

GraphicsEngine::~GraphicsEngine(){
    delete mResourceManager;
    mResourceManager = 0;
    delete mRoot;
    mRoot = 0;
}

void GraphicsEngine::renderSimulation(){
    if(!mSimulation->isInitialised()){
        cerr << "Error: Simulation not initialised, will proceed to initialise the simulation" << endl;
        if(!mSimulation->initialise(mResourceManager)){
            cerr << "Error: unable to initialise the simulation" << endl;
            return;
        }
    }

    mUpdateInterval = 1.f/mSimulation->getCyclesPerSecond();
    mTimer = mUpdateInterval;

    mSceneManager = mRoot->createSceneManager("DefaultSceneManager");
    
    Ogre::Camera* camera;
    Ogre::Viewport* viewport;

    camera = mSceneManager->createCamera("Camera");
    camera->setNearClipDistance(1);

    Ogre::SceneNode* camNode = mSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode");
    camNode->setPosition(Ogre::Vector3(0, 150, 0));
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);
    camNode->attachObject(camera);
    camNode->setFixedYawAxis(true);

    mWindowManager->addViewport(viewport, camera);
    mWindowManager->setCameraNode(camNode);

    mSceneManager->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
    Ogre::Light* light = mSceneManager->createLight("MainLight");
    light->setPosition(50, 50, 50);

    //setup scene manager
    for(map<string, ObjectInfo>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second.get<0>();
        string resourceName = iter->second.get<1>();
        vector3 scale = iter->second.get<2>();

        Ogre::Entity* entity = mSceneManager->createEntity(entityName, resourceName);
        Ogre::SceneNode* node = mSceneManager->getRootSceneNode()->createChildSceneNode(entityName);
        node->attachObject(entity);
        
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();

        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
        node->setScale(Ogre::Vector3(scale.x, scale.y, scale.z));
        
    }

    mRoot->startRendering();
}

bool GraphicsEngine::frameRenderingQueued(const Ogre::FrameEvent& event){
    if(mWindowManager->isWindowClosed())
        return false;
    
    mWindowManager->getInputManager()->capture();
    
    if(mWindowManager->getInputManager()->isKeyDown(OIS::KC_ESCAPE))
        return false;

    mWindowManager->getInputManager()->updateCamera(event.timeSinceLastFrame);

    mTimer -= event.timeSinceLastFrame;

    if(mTimer <= 0){
        while(mTimer <= 0){
            mSimulation->iterate();
            mTimer += mUpdateInterval;
        }
    }

    //sync
    for(map<string, ObjectInfo>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second.get<0>();

        Ogre::SceneNode* node = mSceneManager->getSceneNode(entityName);
        
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();
        
        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
    }

    if(mWindowManager->getInputManager()->isMousebuttonDown(OIS::MB_Left)){
        Ogre::SceneNode* cam = mSceneManager->getSceneNode("CameraNode");
        cout << cam->getPosition().x << " " << cam->getPosition().y << " " << cam->getPosition().z << endl;
    }

    return true;
}