#include "inputmanager.h"

InputManager::InputManager(string hndName){
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

InputManager::~InputManager(){
    if(mInputManager){
        mInputManager->destroyInputObject(mMouse);
        mInputManager->destroyInputObject(mKeyboard);

        OIS::InputManager::destroyInputSystem(mInputManager);
        mInputManager = 0;
    }
}

void InputManager::setCameraNode(Ogre::SceneNode* _camNode){
    mCameraNode = _camNode;
}

bool InputManager::keyPressed(const OIS::KeyEvent &_kEvent){
    cameraTransDown(_kEvent);

    mKeyPressedList[_kEvent.key] = true;

    return true;
}

bool InputManager::keyReleased(const OIS::KeyEvent &_kEvent){
    cameraTransUp(_kEvent);

    mKeyPressedList[_kEvent.key] = false;

    return true;
}

bool InputManager::mouseMoved(const OIS::MouseEvent &_mEvent){
    mMouseXRel = _mEvent.state.X.rel;
    mMouseYRel = _mEvent.state.Y.rel;
    mMouseZRel = _mEvent.state.Z.rel;

    mMouseXAbs = _mEvent.state.X.abs;
    mMouseYAbs = _mEvent.state.Y.abs;
    mMouseZAbs = _mEvent.state.Z.abs;

    updateCameraRot(_mEvent);

    return true;
}

bool InputManager::mousePressed(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
    mMousePressedList[_id] = true;

    return true;
}

bool InputManager::mouseReleased(const OIS::MouseEvent &_mEvent, OIS::MouseButtonID _id){
    mMousePressedList[_id] = false;

    return true;
}

bool InputManager::isKeyDown(OIS::KeyCode _key){
    if(mKeyPressedList.find(_key) == mKeyPressedList.end())
        return false;
    else return mKeyPressedList[_key];
}

bool InputManager::isMousebuttonDown(OIS::MouseButtonID _id){
    if(mMousePressedList.find(_id) == mMousePressedList.end())
        return false;
    else return mMousePressedList[_id];
}

void InputManager::updateWindowDim(int _width, int _height){
    const OIS::MouseState &state = mMouse->getMouseState();
    state.width = _width;
    state.height = _height;
}

void InputManager::capture(){
    mMouse->capture();
    mKeyboard->capture();
}

void InputManager::updateCamera(Ogre::Real _timeElapsed){
    if(mCameraNode){
        mCameraNode->translate(mCamTrans * _timeElapsed, Ogre::Node::TS_LOCAL);
        if(isKeyDown(OIS::KC_Q))
            mCameraNode->roll(Ogre::Degree(ROTATE), Ogre::Node::TS_LOCAL);
        if(isKeyDown(OIS::KC_E))
            mCameraNode->roll(Ogre::Degree(-ROTATE), Ogre::Node::TS_LOCAL);
    }
}

void InputManager::lastMouseState(int& _xRel, int& _yRel, int& _zRel, int& _xAbs, int& _yAbs, int& _zAbs){
    _xRel = mMouseXRel;
    _yRel = mMouseYRel;
    _zRel = mMouseZRel;

    _xAbs = mMouseXAbs;
    _yRel = mMouseYAbs;
    _zRel = mMouseZAbs;
}

void InputManager::cameraTransDown(const OIS::KeyEvent &_kEvent){
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

void InputManager::cameraTransUp(const OIS::KeyEvent &_kEvent){
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

void InputManager::updateCameraRot(const OIS::MouseEvent &_mEvent){
    if(mCameraNode){
        mCameraNode->yaw(Ogre::Degree(ROTATE * -_mEvent.state.X.rel), Ogre::Node::TS_LOCAL);
        mCameraNode->pitch(Ogre::Degree(ROTATE * -_mEvent.state.Y.rel), Ogre::Node::TS_LOCAL);
    }
}

InputManager::InputManager(){}