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

string GraphicsEngine::createLineObject(int _index){
    string objectName = "GELine" + boost::lexical_cast<string>(_index);

    Ogre::ManualObject* lineobj = mSceneManager->createManualObject(objectName);
    lineobj->setDynamic(true);

    lineobj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    lineobj->position(0, 0, 0);
    lineobj->position(0, 0, 0);
    lineobj->end();

    return objectName;
}

string GraphicsEngine::attachNewLine(Line _line){
    string name = mLinePool.back();
    mLinePool.pop_back();

    Ogre::ManualObject* lineobj = mSceneManager->getManualObject(name);
    
    lineobj->beginUpdate(0);
    lineobj->position(_line.p1.x, _line.p1.y, _line.p1.z);
    lineobj->position(_line.p2.x, _line.p2.y, _line.p2.z);
    lineobj->end();

    mSceneManager->getRootSceneNode()->attachObject(lineobj);

    return name;
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
    camNode->setPosition(Ogre::Vector3(0, 0, 10));
    camNode->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);
    camNode->attachObject(camera);
    camNode->setFixedYawAxis(true);

    mWindowManager->addViewport(viewport, camera);
    mWindowManager->setCameraNode(camNode);

    mSceneManager->setAmbientLight(Ogre::ColourValue(0.2, 0.2, 0.2));
    Ogre::Light* light = mSceneManager->createLight("MainLight");
    light->setPosition(50, 50, 50);

    //setup scene manager
    for(map<string, Agent*>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second->getRigidBody();
        string resourceName = iter->second->getModelName();
        vector3 scale = iter->second->getScale();

        Ogre::Entity* entity = mSceneManager->createEntity(entityName, resourceName);
        Ogre::SceneNode* node = mSceneManager->getRootSceneNode()->createChildSceneNode(entityName);
        node->attachObject(entity);
        
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();

        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));
        node->setScale(Ogre::Vector3(scale.x, scale.y, scale.z));
        
    }

    //setup initial line pool
    mLinePool.push_back(createLineObject(0));

    mRoot->startRendering();
}

void GraphicsEngine::destroyAllAttachedMovableObjects(Ogre::SceneNode* _sceneNode)
{
    Ogre::SceneNode::ObjectIterator iter = _sceneNode->getAttachedObjectIterator();

   while(iter.hasMoreElements()){
       Ogre::MovableObject* obj = static_cast<Ogre::MovableObject*>(iter.getNext());
      _sceneNode->getCreator()->destroyMovableObject(obj);
   }

   Ogre::SceneNode::ChildNodeIterator childiter = _sceneNode->getChildIterator();

   while (childiter.hasMoreElements()){
       Ogre::SceneNode* child = static_cast<Ogre::SceneNode*>(childiter.getNext());
       destroyAllAttachedMovableObjects(child);
   }
}

bool GraphicsEngine::frameRenderingQueued(const Ogre::FrameEvent& event){
    if(mWindowManager->isWindowClosed())
        return false;

    mWindowManager->getInputManager()->capture();
    
    if(mWindowManager->getInputManager()->isKeyDown(OIS::KC_ESCAPE))
        return false;

    if(mWindowManager->getInputManager()->isKeyDown(OIS::KC_R))
        mSimulation->resetSimulation();

    mWindowManager->getInputManager()->updateCamera(event.timeSinceLastFrame);

    mTimer -= event.timeSinceLastFrame;

    if(mTimer <= 0){
        while(mTimer <= 0){
            mSimulation->iterate();
            mTimer += mUpdateInterval;
        }
    }

    
    //line rendering logic

    //check if there are lines needed to be rendered
    vector<Line> simulationLines = mSimulation->getLines();
    if(simulationLines.size() > 0 || mLinesInUse.size() > 0){
        //check for lines to remove
        vector<string> removableLines;
        vector<Line> newLines;
        //get lines to remove
        for(uint k = 0; k < mLinesInUse.size(); ++k){
            bool inSimLines = false;

            for(uint i = 0; i < simulationLines.size(); ++i){
                if(mLinesInUse[k].second == simulationLines[i]){
                    inSimLines = true;
                    break;
                }
            }

            if(!inSimLines)
                removableLines.push_back(mLinesInUse[k].first);
        }
        //remove lines from scene graph and return to free pool
        for(uint k = 0; k < removableLines.size(); ++k){
            //remove from scene graph
            mSceneManager->getRootSceneNode()->detachObject(mSceneManager->getManualObject(removableLines[k]));

            //remove from in use list
            uint erasepos;
            for(uint i = 0; i < mLinesInUse.size(); ++i){
                if(mLinesInUse[i].first == removableLines[k]){
                    erasepos = i;
                    break;
                }
            }
            mLinesInUse.erase(mLinesInUse.begin() + erasepos);

            //return to pool
            mLinePool.push_back(removableLines[k]);
        }
        
        
        //get list of new lines to be rendered
        vector<Line> newlines;
        for(uint k = 0; k < simulationLines.size(); ++k){
            bool found = false;
            for(uint i = 0; i < mLinesInUse.size(); ++i){
                if(mLinesInUse[i].second == simulationLines[k]){
                    found = true;
                    break;
                }
            }
            if(!found)
                newlines.push_back(simulationLines[k]);
        }

        //check if pool needs to be expanded, expand by doubling size each time
        int expandsize = mLinePool.size() + mLinesInUse.size();
        while(mLinePool.size() < newlines.size()){
            for(uint k = 0; k < expandsize; ++k)
                mLinePool.push_back(createLineObject(expandsize + k));

            expandsize = mLinePool.size() + mLinesInUse.size();
        }

        //attach new lines
        for(uint k = 0; k < newlines.size(); ++k)
            mLinesInUse.push_back(make_pair(attachNewLine(newlines[k]), newlines[k]));

    }

    //remove specified scene nodes
    vector<string> toRemove = mSimulation->getRemoveList();
    for(uint k = 0; k < toRemove.size(); ++k){
        Ogre::SceneNode* node = mSceneManager->getSceneNode(toRemove[k]);
        node->detachObject(mSceneManager->getEntity(toRemove[k]));
    }

    //sync
    for(map<string, Agent*>::const_iterator iter = mSimulation->getSimulationState().begin(); iter != mSimulation->getSimulationState().end(); iter++){
        string entityName = iter->first;
        btRigidBody* entityBody = iter->second->getRigidBody();

        Ogre::SceneNode* node = mSceneManager->getSceneNode(entityName);
        
        if(node->numAttachedObjects() == 0)
            node->attachObject(mSceneManager->getEntity(entityName));
            
        btQuaternion rot = entityBody->getWorldTransform().getRotation();
        btVector3 pos = entityBody->getWorldTransform().getOrigin();
        
        node->setPosition(Ogre::Vector3(pos.getX(), pos.getY(), pos.getZ()));
        node->setOrientation(Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()));

        //animation logic

        //agent has animations
        if(iter->second->getAnimationName() != ""){
            Ogre::Entity* ent = dynamic_cast<Ogre::Entity*>(node->getAttachedObject(entityName));
            //check if a new animation needs to be played
            if(mLastAnimationPlayed.find(entityName) == mLastAnimationPlayed.end()){
                mLastAnimationPlayed[entityName] = iter->second->getAnimationName();
                ent->getAnimationState(iter->second->getAnimationName())->setEnabled(true);
                ent->getAnimationState(iter->second->getAnimationName())->setLoop(iter->second->getAnimationLoop());
                ent->getAnimationState(iter->second->getAnimationName())->setTimePosition(0);
            }
            else if(mLastAnimationPlayed[entityName] != iter->second->getAnimationName()){
                ent->getAnimationState(mLastAnimationPlayed[entityName])->setEnabled(false);
                ent->getAnimationState(mLastAnimationPlayed[entityName])->setWeight(0);

                mLastAnimationPlayed[entityName] = iter->second->getAnimationName();
                ent->getAnimationState(iter->second->getAnimationName())->setEnabled(true);
                ent->getAnimationState(iter->second->getAnimationName())->setLoop(iter->second->getAnimationLoop());
                ent->getAnimationState(iter->second->getAnimationName())->setTimePosition(0);
                ent->getAnimationState(iter->second->getAnimationName())->setWeight(1);
            }

            //step animation
            ent->getAnimationState(mLastAnimationPlayed[entityName])->addTime(event.timeSinceLastFrame);
        }
    }
    if(mWindowManager->getInputManager()->isMousebuttonDown(OIS::MB_Left)){
        Ogre::SceneNode* cam = mSceneManager->getSceneNode("CameraNode");
        cout << cam->getPosition().x << " " << cam->getPosition().y << " " << cam->getPosition().z << endl;
    }

    return true;
}