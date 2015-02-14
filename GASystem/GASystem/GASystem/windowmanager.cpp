#include "windowmanager.h"

WindowManager::WindowManager(Ogre::Root* _root){
    mRoot = _root;

    //load config
    if(!(mRoot->restoreConfig() || mRoot->showConfigDialog())){
        cerr << "Error: unable to set Engine config" << endl;
        return;
    }

    mWindow = mRoot->initialise(true, "Simulation Render");
    
    //get handler name and send to input manager
    size_t windowHnd = 0;
    ostringstream sstream;
    mWindow->getCustomAttribute("WINDOW", &windowHnd);
    sstream << windowHnd;
    mInputManager = new InputManager(sstream.str());
    
    //update input manager with window size
    windowResized(mWindow);

    //add this as a window listener
    Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);
}

WindowManager::~WindowManager(){
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
}

void WindowManager::addViewport(Ogre::Viewport*& _viewport, Ogre::Camera*& _camera){
    _viewport = mWindow->addViewport(_camera);
    _viewport->setBackgroundColour(Ogre::ColourValue(1, 1, 1));
    _camera->setAspectRatio(Ogre::Real(_viewport->getActualWidth()) / Ogre::Real(_viewport->getActualHeight()));
}

void WindowManager::windowResized(Ogre::RenderWindow* _rw){
    uint w, h, d;
    int l, t;

    _rw->getMetrics(w, h, d, l, t);
    mInputManager->updateWindowDim(w, h);
}

void WindowManager::windowClosed(Ogre::RenderWindow* _rw){
    if(_rw == mWindow){
        if(mInputManager){
            delete mInputManager;
            mInputManager = 0;
        }
    }
}

void WindowManager::setCameraNode(Ogre::SceneNode* _camNode){
    mInputManager->setCameraNode(_camNode);
}

bool WindowManager::isWindowClosed(){
    return mWindow->isClosed();
}

InputManager* WindowManager::getInputManager(){
    return mInputManager;
}

WindowManager::WindowManager(){}