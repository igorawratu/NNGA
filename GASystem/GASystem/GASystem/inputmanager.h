#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>
#include <string>
#include <iostream>
#include <OgreRoot.h>

const float MOVE = 125;
const float ROTATE = 0.13;

using namespace std;

class InputManager : public OIS::KeyListener, public OIS::MouseListener
{
public:
    InputManager(string hndName);

    ~InputManager();

    void setCameraNode(Ogre::SceneNode* _camNode);

    virtual bool keyPressed(const OIS::KeyEvent &_kEvent);
    virtual bool keyReleased(const OIS::KeyEvent &_kEvent);

    virtual bool mouseMoved(const OIS::MouseEvent &_mEvent);
    virtual bool mousePressed(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id);
    virtual bool mouseReleased(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id);

    bool isKeyDown(OIS::KeyCode _key);

    bool isMousebuttonDown(OIS::MouseButtonID _id);
    void updateWindowDim(int _width, int _height);

    void capture();

    void updateCamera(Ogre::Real _timeElapsed);

    void lastMouseState(int& _xRel, int& _yRel, int& _zRel, int& _xAbs, int& _yAbs, int& _zAbs);

private:
    void cameraTransDown(const OIS::KeyEvent &_kEvent);

    void cameraTransUp(const OIS::KeyEvent &_kEvent);

    void updateCameraRot(const OIS::MouseEvent &_mEvent);

    InputManager();

private:
    OIS::InputManager* mInputManager;
    OIS::Mouse* mMouse;
    OIS::Keyboard* mKeyboard;
    
    map<OIS::KeyCode, bool> mKeyPressedList;
    map<OIS::MouseButtonID, bool> mMousePressedList;
    int mMouseXRel, mMouseYRel, mMouseZRel, mMouseXAbs, mMouseYAbs, mMouseZAbs;
    
    Ogre::SceneNode* mCameraNode;
    Ogre::Vector3 mCamTrans;
};

#endif