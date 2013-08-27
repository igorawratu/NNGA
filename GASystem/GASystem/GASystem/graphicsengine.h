#ifndef GRAPHICSENGINE_H
#define GRAPHICSENGINE_H

#include "simulationcontainer.h"
#include "windowmanager.h"
#include "inputmanager.h"
#include "resourcemanager.h"

#include <OgreRoot.h>
#include <OgreConfigFile.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreEntity.h>

#include <btBulletDynamicsCommon.h>

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

private:
    Ogre::Root* mRoot;
    Ogre::String mPlugins;
    SimulationContainer* mSimulation;
    WindowManager* mWindowManager;
    Ogre::SceneManager* mSceneManager;
    ResourceManager* mResourceManager;
    double mUpdateInterval, mTimer;

private:
    GraphicsEngine(){}
};

#endif