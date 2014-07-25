#include "mouseescapesimulation.h"

MouseEscapeSimulation::MouseEscapeSimulation(double _rangefinderRadius, uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed)
: Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(MouseEscapeSimulation::tickCallBack, this, true);
    mCollisions = mRangefinderVals = 0;
    mSeed = _seed;
    mRangefinderRadius = _rangefinderRadius;

    for(uint k = 0; k < 35; ++k)
        mMouseAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    for(uint k = 35; k < 40; ++k)
        mRobotAgents.push_back("Agent" + boost::lexical_cast<string>(k));

    mVelocityAcc = 0;
}

MouseEscapeSimulation::~MouseEscapeSimulation(){

}

void MouseEscapeSimulation::iterate(){
    if(mCycleCounter > mNumCycles)
        return;

    mObjectsToRemove.clear();

    if(mCycleCounter % mCyclesPerDecision == 0){
        mRaysShot.clear();
        for(uint k = 0; k < mMouseAgents.size(); ++k)
            applyUpdateRules(mMouseAgents[k], 0);
        for(uint k = 0; k < mRobotAgents.size(); ++k)
            applyUpdateRules(mRobotAgents[k], 1);

        //remove dead agents from simulation
        for(uint k = 0; k < mObjectsToRemove.size(); ++k){
            if(mWorldEntities.find(mObjectsToRemove[k]) != mWorldEntities.end()){
                mWorld->removeRigidBody(mWorldEntities[mObjectsToRemove[k]]->getRigidBody());
                delete mWorldEntities[mObjectsToRemove[k]];
                mWorldEntities.erase(mObjectsToRemove[k]);
            }

            //remove agent from lists
            int pos = -1;
            for(uint i = 0; i < mMouseAgents.size(); ++i)
                if(mMouseAgents[i] == mObjectsToRemove[k])
                    pos = i;
            if(pos > -1)
                mMouseAgents.erase(mMouseAgents.begin() + pos);
        }
    }

    mCycleCounter++;

    mWorld->stepSimulation(1/(float)mCyclesPerSecond, 1, 1/(float)mCyclesPerSecond);
}

double MouseEscapeSimulation::fitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["FLFitnessWeight"] = 1;
    dblAcc["EVWeight"] = 1;

    for(uint k = 0; k < mMouseAgents.size(); k++)
        pos[mMouseAgents[k]] = getPositionInfo(mMouseAgents[k]);
    intAcc["Positive"] = 1;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["LowerBound"] = 1;
    dblAcc["UpperBound"] = 3;
    dblAcc["Value"] = mMouseAgents.size();
    
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : 1000;

    //change max val
    dblAcc["Collisions"] = mRangefinderVals + mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, dblAcc, intAcc) : 5000;

    return finalFitness;
}

Simulation* MouseEscapeSimulation::getNewCopy(){
    Simulation* sim = new MouseEscapeSimulation(mRangefinderRadius, mNumCycles, mCyclesPerDecision, mCyclesPerSecond, mSolution, mResourceManager, mSeed);
    sim->initialise();
    
    return sim;
}

bool MouseEscapeSimulation::initialise(){
    if(mInitialised)
        return true;

    mFitnessFunctions.push_back(new FinishLineFitness());
    mFitnessFunctions.push_back(new ExpectedValueFitness());
    mFitnessFunctions.push_back(new CollisionFitness());

    mFinishLine.p1 = vector3(50, 0, 100);
    mFinishLine.p2 = vector3(50, 0, -100);

    btQuaternion rotG1(0, 0, 0, 1), rotG2(0, 0, 0, 1);
    rotG1.setEuler(0, 0, 0); rotG2.setEuler(PI, 0, 0);

    vector3 minDimOne(-80, 0, -80), maxDimOne(-30, 0, 80);
    vector3 minDimTwo(20, 0, -80), maxDimTwo(40, 0, 80);

    boost::mt19937 rngonex(mSeed);
    boost::mt19937 rngonez(mSeed + mSeed / 2);
    boost::mt19937 rngtwox(mSeed / 2);
    boost::mt19937 rngtwoz(mSeed * 2);

    boost::uniform_real<double> distxone(minDimOne.x, maxDimOne.x);
    boost::uniform_real<double> distzone(minDimOne.z, maxDimOne.z);
    boost::uniform_real<double> distxtwo(minDimTwo.x, maxDimTwo.x);
    boost::uniform_real<double> distztwo(minDimTwo.z, maxDimTwo.z);

    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxone(rngonex, distxone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genzone(rngonez, distzone);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genxtwo(rngtwox, distxtwo);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> genztwo(rngtwoz, distztwo);

    for(uint k = 0; k < mMouseAgents.size(); ++k){
        mWorldEntities[mMouseAgents[k]] = new MouseAgent(15, 1);
        if(!mWorldEntities[mMouseAgents[k]]->initialise("mouse.mesh", vector3(1, 1, 1), rotG1, mResourceManager, vector3(genxone(), 0, genzone()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mMouseAgents[k]]->getRigidBody());
    }
    
    for(uint k = 0; k < mRobotAgents.size(); ++k){
        mWorldEntities[mRobotAgents[k]] = new WarRobotAgent(10, vector3(10, 0, 10), vector3(-10, 0, -10), 1, 15);
        if(!mWorldEntities[mRobotAgents[k]]->initialise("warrobot.mesh", vector3(1, 1, 1), rotG2, mResourceManager, vector3(genxtwo(), 0, genztwo()), 0.01, mSeed))
            return false;
        mWorld->addRigidBody(mWorldEntities[mRobotAgents[k]]->getRigidBody());
    }
    
    
    mWorldEntities["environment"] = new StaticWorldAgent(0.5, 0.1);
    if(!mWorldEntities["environment"]->initialise("city.mesh", vector3(100, 100, 100), btQuaternion(0, 0, 0, 1), mResourceManager, vector3(0, 0, 0), 0, mSeed))
        return false;
    mWorld->addRigidBody(mWorldEntities["environment"]->getRigidBody());

    mInitialised = true;

    return true;
}

void MouseEscapeSimulation::tick(){
    for(uint k = 0; k < mMouseAgents.size(); ++k)
        mWorldEntities[mMouseAgents[k]]->tick();

    for(uint k = 0; k < mRobotAgents.size(); ++k)
        mWorldEntities[mRobotAgents[k]]->tick();
}

double MouseEscapeSimulation::realFitness(){
    double finalFitness = 0;

    map<string, vector3> pos;
    map<string, long> intAcc;
    map<string, double> dblAcc;
    dblAcc["FLFitnessWeight"] = 1;
    dblAcc["EVWeight"] = 1;

    for(uint k = 0; k < mMouseAgents.size(); k++)
        pos[mMouseAgents[k]] = getPositionInfo(mMouseAgents[k]);
    intAcc["Positive"] = 1;
    pos["LineP1"] = mFinishLine.p1;
    pos["LineP2"] = mFinishLine.p2;

    finalFitness += mFitnessFunctions[0]->evaluateFitness(pos, dblAcc, intAcc);

    dblAcc["LowerBound"] = 1;
    dblAcc["UpperBound"] = 3;
    dblAcc["Value"] = mMouseAgents.size();
    
    finalFitness += finalFitness == 0 ? mFitnessFunctions[1]->evaluateFitness(pos, dblAcc, intAcc) : 1000;

    //change max val
    dblAcc["Collisions"] = mCollisions; 
    dblAcc["ColFitnessWeight"] = 1;
    finalFitness += finalFitness == 0 ? mFitnessFunctions[2]->evaluateFitness(pos, dblAcc, intAcc) : 5000;

    return finalFitness;
}

void MouseEscapeSimulation::checkRayObject(int _groupNum, const btCollisionObject* _obj, int& _team, string& _entityName){
    for(uint k = 0; k < mMouseAgents.size(); k++){
        if(_obj == mWorldEntities[mMouseAgents[k]]->getRigidBody()){
            _team = _groupNum == 0 ? -1 : 1;
            _entityName = mMouseAgents[k];
            return;
        }
    }

    for(uint k = 0; k < mRobotAgents.size(); k++){
        if(_obj == mWorldEntities[mRobotAgents[k]]->getRigidBody()){
            _team = _groupNum == 1 ? -1 : 1;
            _entityName = mRobotAgents[k];
            return;
        }
    }

    _team = 0;
    _entityName = "env";
}

void MouseEscapeSimulation::applyUpdateRules(string _agentName, uint _groupNum){
    btTransform trans;
    mWorldEntities[_agentName]->getRigidBody()->getMotionState()->getWorldTransform(trans);
    const btCollisionObject* obj;
    const btCollisionObject* front;
    string colliderName, otherColliderName;
    int teamInd, frontTeamInd;
    vector3 hitposfront, hitposother;

    map<uint, double> input;

    btBoxShape* agentBox = dynamic_cast<btBoxShape*>(mWorldEntities[_agentName]->getRigidBody()->getCollisionShape());
    if(agentBox == 0){
        cout << "Error: unable to get box to agent, will not apply update" << endl;
        return;
    }

    double d1 = Simulation::getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, agentBox->getHalfExtentsWithMargin().getZ()));
    double d2 = Simulation::getRayCollisionDistance(_agentName, btVector3(100, 0, 0), ENVIRONMENT, vector3(0, 0, -agentBox->getHalfExtentsWithMargin().getZ()));

    double frontDist = d1 > d2 ? d2 : d1;

    //rangefinders
    input[1] = getRayCollisionDistance(_agentName, btVector3(100, 0, 0), front, hitposfront) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, front, frontTeamInd, colliderName);
        input[13] = frontTeamInd;
    }

    input[2] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 0), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[14] = teamInd;
    }

    input[3] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[15] = teamInd;
    }

    input[4] = getRayCollisionDistance(_agentName, btVector3(0, 0, 100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[16] = teamInd;
    }

    input[5] = getRayCollisionDistance(_agentName, btVector3(100, 0, -100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[17] = teamInd;
    }

    input[6] = getRayCollisionDistance(_agentName, btVector3(-100, 0, 100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[18] = teamInd;
    }

    input[7] = getRayCollisionDistance(_agentName, btVector3(-100, 0, -100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[19] = teamInd;
    }

    input[8] = getRayCollisionDistance(_agentName, btVector3(100, 0, 100), obj, hitposother) / 50;
    if(_groupNum == 1){
        checkRayObject(_groupNum, obj, teamInd, otherColliderName);
        input[20] = teamInd;
    }

    //agent position
    input[9] = trans.getOrigin().getX() / 50;
    input[10] = trans.getOrigin().getZ() / 50;

    //velocity
    vector3 agentVel = mWorldEntities[_agentName]->getVelocity();
    input[11] = agentVel.x;
    input[12] = agentVel.z;

    //NB: Find closest point
    if(_groupNum == 0){
        input[13] = mFinishLine.p1.x;
        input[14] = mFinishLine.p1.z;
        input[15] = mFinishLine.p2.x;
        input[16] = mFinishLine.p2.z;
    }


    if(frontDist < 10)
        mWorldEntities[_agentName]->avoidCollisions(d1, d2, mCyclesPerSecond, mCyclesPerDecision, mWorld, mWorldEntities["environment"]->getRigidBody());
    else{
        mWorldEntities[_agentName]->avoided();
        vector<double> output = mSolution->evaluateNeuralNetwork(0, input, _groupNum);
        double frontVal = Simulation::getRayCollisionDistance(_agentName, btVector3(100, 0, 0), AGENT) > 3 ? 1 : 0;
        output.push_back(frontVal);
        mWorldEntities[_agentName]->update(output);
    }

    //shooting logic here
    if(frontTeamInd == 1 && _groupNum == 1 && calcCrossVal(mFinishLine.p1, mFinishLine.p2, getPositionInfo(_agentName)) < 0){
        Line ray;
        //first ray point
        ray.p1 = getPositionInfo(_agentName);
        ray.p2 = hitposfront;

        if(ray.p1.calcDistance(ray.p2) < 40 && dynamic_cast<WarRobotAgent*>(mWorldEntities[_agentName])->shootRay()){
            mRaysShot.push_back(ray);
            mObjectsToRemove.push_back(colliderName);
        }
    }

    /*if(_groupNum == 0){
        if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, getPositionInfo(_agentName)) > 0)
            mCrossed.push_back(_agentName);
    }*/

    //fitness eval code
    //try make aggresors move more/faster
    if(_groupNum == 1)
        mVelocityAcc += mWorldEntities[_agentName]->getVelocity().calcDistance(vector3(0, 0, 0));

    //rangefinder vals
    if((calcCrossVal(mFinishLine.p1, mFinishLine.p2, getPositionInfo(_agentName)) > 0) && mCycleCounter > 10){
        for(uint k = 1; k <= 8; k++)
            if(input[k] * 50 < mRangefinderRadius)
                mRangefinderVals += (mRangefinderRadius - (input[k] * 50))/mRangefinderRadius;
        
        //gets collision data
        int numManifolds = mWorld->getDispatcher()->getNumManifolds();
        for (int i=0;i<numManifolds;i++)
        {
	        btPersistentManifold* contactManifold =  mWorld->getDispatcher()->getManifoldByIndexInternal(i);
            if(contactManifold->getNumContacts() < 1)
                continue;

	        const btCollisionObject* obA = contactManifold->getBody0();
	        const btCollisionObject* obB = contactManifold->getBody1();
            
            if((mWorldEntities[_agentName]->getRigidBody() == obA || mWorldEntities[_agentName]->getRigidBody() == obB))
                mCollisions++;
        }
    }
}

bool MouseEscapeSimulation::crossed(string _agentName){
    bool found = false;
    for(uint k = 0; k < mCrossed.size(); ++k){
        if(mCrossed[k] == _agentName){
            found = true;
            break;
        }
    }

    return found;
}

vector<CompetitiveFitness> MouseEscapeSimulation::competitiveFitness(){
    vector<CompetitiveFitness> fitnesses;

    double maxDist = 100;
    double mouseDistAcc = 0;
    double robotDistAcc = maxDist * mMouseAgents.size();
    vector3 midpoint((mFinishLine.p1.x + mFinishLine.p2.x)/2, (mFinishLine.p1.y + mFinishLine.p2.y)/2, (mFinishLine.p1.z + mFinishLine.p2.z)/2);

    for(uint k = 0; k < mMouseAgents.size(); ++k){
        double dist = 0;
        vector3 agentPos = getPositionInfo(mMouseAgents[k]);
        
        if(calcCrossVal(mFinishLine.p1, mFinishLine.p2, agentPos) < 0)
            midpoint.calcDistance(agentPos);

        mouseDistAcc += dist;
        robotDistAcc = robotDistAcc - 100 + dist;
    }

    double robotFitness = robotDistAcc;
    double mouseFitness = (35 - mMouseAgents.size()) * 100 + mouseDistAcc;

    fitnesses.push_back(make_pair(0, mouseFitness));
    fitnesses.push_back(make_pair(1, robotFitness));

    return fitnesses;
}