#ifndef GRAPHICSENGINE_H
#define GRAPHICSENGINE_H

#include "simulationcontainer.h"
#include "windowmanager.h"
#include "inputmanager.h"
#include "resourcemanager.h"
#include "common.h"

#include <OgreRoot.h>
#include <OgreConfigFile.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

#include <btBulletDynamicsCommon.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>

using namespace std;

class GraphicsEngine : public Ogre::FrameListener
{
public:
    GraphicsEngine(SimulationContainer* _simulation);
    ~GraphicsEngine();

    void renderSimulation();
    void setSimulation(SimulationContainer* _simulation){
        mSimulation = _simulation;
    }

    virtual bool frameRenderingQueued(const Ogre::FrameEvent& event);

    ResourceManager* getResourceManager(){
        return mResourceManager;
    }

private:
    void destroyAllAttachedMovableObjects(Ogre::SceneNode* _sceneNode);
    string createLineObject(int _index);
    string attachNewLine(Line _line);

private:
    Ogre::Root* mRoot;
    Ogre::String mPlugins;
    SimulationContainer* mSimulation;
    WindowManager* mWindowManager;
    Ogre::SceneManager* mSceneManager;
    ResourceManager* mResourceManager;
    double mUpdateInterval, mTimer;

    vector<pair<string, Line>> mLinesInUse;
    vector<string> mLinePool;
    map<string, string> mLastAnimationPlayed;

private:
    GraphicsEngine(){}
};

#endif