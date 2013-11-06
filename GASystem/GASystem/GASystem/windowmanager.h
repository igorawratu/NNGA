#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <iostream>
#include "inputmanager.h"
#include <OgreWindowEventUtilities.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include "common.h"

using namespace std;

class WindowManager : public Ogre::WindowEventListener
{
public:
    WindowManager(Ogre::Root* _root);

    ~WindowManager();

    void addViewport(Ogre::Viewport*& _viewport, Ogre::Camera*& _camera);

    virtual void windowResized(Ogre::RenderWindow* _rw);

    virtual void windowClosed(Ogre::RenderWindow* _rw);

    void setCameraNode(Ogre::SceneNode* _camNode);

    bool isWindowClosed();

    InputManager* getInputManager();

private:
    Ogre::RenderWindow* mWindow;
    Ogre::Root* mRoot;
    InputManager* mInputManager;
    string mWindowName;

private:
    WindowManager();
};

#endif