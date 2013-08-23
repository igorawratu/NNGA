#include "graphicsengine.h"

GraphicsEngine::GraphicsEngine(Simulation* _simulation) : mSimulation(_simulation){

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

    mSceneManager = mRoot->createSceneManager("DefaultSceneManager");
    
    Ogre::Camera* camera;
    Ogre::Viewport* viewport;

    camera = mSceneManager->createCamera("Camera");
    camera->setNearClipDistance(1);

    Ogre::SceneNode* camNode = mSceneManager->getRootSceneNode()->createChildSceneNode("CameraNode");
    camNode->setPosition(Ogre::Vector3(100, 100, 100));
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);
    camNode->attachObject(camera);

    mWindowManager->addViewport(viewport, camera);

    mSceneManager->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
    Ogre::Light* light = mSceneManager->createLight("MainLight");
    light->setPosition(100, 100, 100);

    //setup scene manager
    for(map<string, pair<btRigidBody*, string>>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second.first;
        string resourceName = iter->second.second;

        Ogre::Entity* entity = mSceneManager->createEntity(entityName, resourceName);
        Ogre::SceneNode* node = mSceneManager->getRootSceneNode()->createChildSceneNode(entityName);
        node->attachObject(entity);
        
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();

        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
    }
 
    mRoot->startRendering();
}

bool GraphicsEngine::frameRenderingQueued(const Ogre::FrameEvent& event){
    if(mWindowManager->isWindowClosed())
        return false;
    
    mWindowManager->getInputManager()->capture();
    
    if(mWindowManager->getInputManager()->getLastKey() == OIS::KC_ESCAPE)
        return false;


    updateCamera(event);

    //put this on a timer
    mSimulation->iterate();

    //sync
    for(map<string, pair<btRigidBody*, string>>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second.first;

        Ogre::SceneNode* node = mSceneManager->getSceneNode(entityName);
        
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();
        
        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
    }
            

    return true;
}

void GraphicsEngine::updateCamera(const Ogre::FrameEvent& event){
    Ogre::SceneNode* camNode = mSceneManager->getSceneNode("CameraNode");
    
    Ogre::Vector3 translate(0, 0, 0);
    
    float move = 125;
    float rotate = 0.13;

    switch(mWindowManager->getInputManager()->getLastKey()){
        case OIS::KC_A:
            translate.x = -move;
            break;
        case OIS::KC_S:
            translate.z = move;
            break;
        case OIS::KC_D:
            translate.x = move;
            break;
        case OIS::KC_W:
            translate.z = -move;
            break;
        default:
            break;
    }

    camNode->translate(translate * event.timeSinceLastFrame, Ogre::Node::TS_LOCAL);

}