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
    InputManager(string hndName){
        //attach input system to window
        OIS::ParamList parameters;
        parameters.insert(make_pair(string("WINDOW"), hndName));
        mInputManager = OIS::InputManager::createInputSystem(parameters);

        //create keyboard and mouse
        cout << mInputManager->getNumberOfDevices(OIS::OISKeyboard) << endl;
        cout << mInputManager->getNumberOfDevices(OIS::OISMouse) << endl;
        mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
        mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));

        //set this class as a listener for the keyboard and mouse just created
        mKeyboard->setEventCallback(this);
        mMouse->setEventCallback(this);

        mMouseXRel = mMouseYRel = mMouseZRel = mMouseXAbs = mMouseYAbs = mMouseZAbs = 0;

        mCameraNode = 0;
        mCamTrans = Ogre::Vector3::ZERO;
    }

    ~InputManager(){
        if(mInputManager){
            mInputManager->destroyInputObject(mMouse);
            mInputManager->destroyInputObject(mKeyboard);

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }

    void setCameraNode(Ogre::SceneNode* _camNode){
        mCameraNode = _camNode;
    }

    virtual bool keyPressed(const OIS::KeyEvent &_kEvent){
        cameraTransDown(_kEvent);

        mKeyPressedList[_kEvent.key] = true;

        return true;
    }
    virtual bool keyReleased(const OIS::KeyEvent &_kEvent){
        cameraTransUp(_kEvent);

        mKeyPressedList[_kEvent.key] = false;

        return true;
    }

    virtual bool mouseMoved(const OIS::MouseEvent &_mEvent){
        mMouseXRel = _mEvent.state.X.rel;
        mMouseYRel = _mEvent.state.Y.rel;
        mMouseZRel = _mEvent.state.Z.rel;

        mMouseXAbs = _mEvent.state.X.abs;
        mMouseYAbs = _mEvent.state.Y.abs;
        mMouseZAbs = _mEvent.state.Z.abs;

        updateCameraRot(_mEvent);

        return true;
    }

    virtual bool mousePressed(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
        mMousePressedList[_id] = true;

        return true;
    }

    virtual bool mouseReleased(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
        mMousePressedList[_id] = false;

        return true;
    }

    bool isKeyDown(OIS::KeyCode _key){
        if(mKeyPressedList.find(_key) == mKeyPressedList.end())
            return false;
        else return mKeyPressedList[_key];
    }

    bool isMousebuttonDown(OIS::MouseButtonID _id){
        if(mMousePressedList.find(_id) == mMousePressedList.end())
            return false;
        else return mMousePressedList[_id];
    }

    void updateWindowDim(int _width, int _height){
        const OIS::MouseState &state = mMouse->getMouseState();
        state.width = _width;
        state.height = _height;
    }

    void capture(){
        mMouse->capture();
        mKeyboard->capture();
    }

    void updateCamera(Ogre::Real _timeElapsed){
        if(mCameraNode){
            mCameraNode->translate(mCamTrans * _timeElapsed, Ogre::Node::TS_LOCAL);
            if(isKeyDown(OIS::KC_Q))
                mCameraNode->roll(Ogre::Degree(ROTATE), Ogre::Node::TS_LOCAL);
            if(isKeyDown(OIS::KC_E))
                mCameraNode->roll(Ogre::Degree(-ROTATE), Ogre::Node::TS_LOCAL);
        }
    }

    void lastMouseState(int& _xRel, int& _yRel, int& _zRel, int& _xAbs, int& _yAbs, int& _zAbs){
        _xRel = mMouseXRel;
        _yRel = mMouseYRel;
        _zRel = mMouseZRel;

        _xAbs = mMouseXAbs;
        _yRel = mMouseYAbs;
        _zRel = mMouseZAbs;
    }

private:
    void cameraTransDown(const OIS::KeyEvent &_kEvent){
        switch(_kEvent.key){
            case OIS::KC_A:
                mCamTrans.x = -MOVE;
                break;
            case OIS::KC_S:
                mCamTrans.z = MOVE;
                break;
            case OIS::KC_D:
                mCamTrans.x = MOVE;
                break;
            case OIS::KC_W:
                mCamTrans.z = -MOVE;
                break;
            default:
                break;
        }
    }

    void cameraTransUp(const OIS::KeyEvent &_kEvent){
        switch(_kEvent.key){
            case OIS::KC_A:
                mCamTrans.x = 0;
                break;
            case OIS::KC_S:
                mCamTrans.z = 0;
                break;
            case OIS::KC_D:
                mCamTrans.x = 0;
                break;
            case OIS::KC_W:
                mCamTrans.z = 0;
                break;
            default:
                break;
        }
    }

    void updateCameraRot(const OIS::MouseEvent &_mEvent){
        if(mCameraNode){
            mCameraNode->yaw(Ogre::Degree(ROTATE * -_mEvent.state.X.rel), Ogre::Node::TS_LOCAL);
            mCameraNode->pitch(Ogre::Degree(ROTATE * -_mEvent.state.Y.rel), Ogre::Node::TS_LOCAL);
        }
    }

    InputManager(){}

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