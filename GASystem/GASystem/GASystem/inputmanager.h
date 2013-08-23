#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>
#include <string>
#include <iostream>

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
    }

    ~InputManager(){
        if(mInputManager){
            mInputManager->destroyInputObject(mMouse);
            mInputManager->destroyInputObject(mKeyboard);

            OIS::InputManager::destroyInputSystem(mInputManager);
            mInputManager = 0;
        }
    }

    virtual bool keyPressed(const OIS::KeyEvent &_kEvent){
        mLastKey = _kEvent.key;

        return true;
    }
    virtual bool keyReleased(const OIS::KeyEvent &_kEvent){
        mLastKey = OIS::KC_UNASSIGNED;

        return true;
    }

    virtual bool mouseMoved(const OIS::MouseEvent &_mEvent){
        mMouseXRel = _mEvent.state.X.rel;
        mMouseYRel = _mEvent.state.Y.rel;
        mMouseZRel = _mEvent.state.Z.rel;

        mMouseXAbs = _mEvent.state.X.abs;
        mMouseYAbs = _mEvent.state.Y.abs;
        mMouseZAbs = _mEvent.state.Z.abs;

        return true;
    }

    //only keeps hold of the last key and mouse event, needs some sort of map or queue in order to keep track of multiple state updates(currently just overriding)
    virtual bool mousePressed(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
        mLastMouseButton = _id;

        return true;
    }

    virtual bool mouseReleased(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
        mLastMouseButton = OIS::MB_Button7;

        return true;
    }

    OIS::KeyCode getLastKey(){
        return mLastKey;
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

    void getMouseRelative(float& _x, float& _y, float& _z){
        _x = mMouseXRel;
        _y = mMouseYRel;
        _z = mMouseZRel;
    }

private:
    InputManager(){}

private:
    OIS::InputManager* mInputManager;
    OIS::Mouse* mMouse;
    OIS::Keyboard* mKeyboard;
    OIS::KeyCode mLastKey;
    OIS::MouseButtonID mLastMouseButton;
    int mMouseXRel, mMouseYRel, mMouseZRel;
    int mMouseXAbs, mMouseYAbs, mMouseZAbs;
};

#endif