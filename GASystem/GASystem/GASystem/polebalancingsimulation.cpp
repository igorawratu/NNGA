#include "polebalancingsimulation.h"

PoleBalancingSimulation::PoleBalancingSimulation(uint _numCycles, uint _cyclesPerDecision, uint _cyclesPerSecond, Solution* _solution, ResourceManager* _resourceManager, int _seed) : Simulation(_numCycles, _cyclesPerDecision, _cyclesPerSecond, _solution, _resourceManager){
    mWorld->setInternalTickCallback(PoleBalancingSimulation::tickCallBack, this, true);
    mComplete = false;
    mLastAppliedForce = 0;
    mSeed = _seed;
}

PoleBalancingSimulation::PoleBalancingSimulation(const PoleBalancingSimulation& other) : Simulation(other.mNumCycles, other.mCyclesPerDecision, other.mCyclesPerSecond, other.mSolution, other.mResourceManager){
    mWorld->setInternalTickCallback(PoleBalancingSimulation::tickCallBack, this, true);
    mComplete = false;
    mLastAppliedForce = 0;
    mSeed = other.mSeed;
}

PoleBalancingSimulation::~PoleBalancingSimulation(){}

bool PoleBalancingSimulation::isCartFine(){
    if(mCart.cart_x_position < MIN_X_POSITION){
        //cout << "cart off rails" << endl;
        return false;
    }

    if(mCart.cart_x_position > MAX_X_POSITION){
        //cout << "cart off rails" << endl;
        return false;
    }

    if(mCart.pole_angle < mCart.min_angle){
        //cout << "pole fell" << endl;
        return false;
    }

    if(mCart.pole_angle > mCart.max_angle){
        //cout << "pole fell" << endl;
        return false;
    }
    
    return true;
}

void PoleBalancingSimulation::iterateCart(){
    float angular_accelaration;
    float x_position_accelaration;

    angular_accelaration = (GRAVITY * sin(mCart.pole_angle) + cos(mCart.pole_angle) * ((-mLastAppliedForce - mCart.cart_mass * mCart.pole_length * mCart.pole_angular_velocity * mCart.pole_angular_velocity * sin(mCart.pole_angle)) / (mCart.cart_mass + mCart.pole_mass))) / (mCart.pole_length * (4.0 / 3.0 - (mCart.pole_mass * cos(mCart.pole_angle) * cos(mCart.pole_angle))) / (mCart.cart_mass + mCart.pole_mass));
    x_position_accelaration = (mLastAppliedForce + mCart.pole_mass * mCart.pole_length * (mCart.pole_angular_velocity * mCart.pole_angular_velocity * sin(mCart.pole_angle) - angular_accelaration * cos(mCart.pole_angle))) / (mCart.cart_mass + mCart.pole_mass);
     
    mCart.cart_x_velocity += ((1/(double)mCyclesPerSecond) * x_position_accelaration);
    mCart.cart_x_position += ((1/(double)mCyclesPerSecond) * mCart.cart_x_velocity);
    mCart.pole_angular_velocity += ((1/(double)mCyclesPerSecond) * angular_accelaration);
    mCart.pole_angle += ((1/(double)mCyclesPerSecond) * mCart.pole_angular_velocity);

    //cout << mCart.cart_x_position << endl;
}

void PoleBalancingSimulation::iterate(){
    if(mCycleCounter >= mNumCycles || mComplete)
        return;

    if(mCycleCounter % mCyclesPerDecision == 0){
        //apply ann decision here
        map<uint, double> input;
        input[1] = mCart.cart_x_position / 15;
        input[2] = mCart.cart_x_velocity / 10;
        input[3] = mCart.pole_angle / PI;
        input[4] = mCart.pole_angular_velocity / 10;

        //cout << mCart.cart_x_velocity << " " << mCart.pole_angular_velocity << endl;

        vector<double> output = mSolution->evaluateNeuralNetwork(0, input);
        mLastAppliedForce = (output[0] * 1000) - 500;
    }

    iterateCart();
    mComplete = !isCartFine();

    mCycleCounter++;
}

double PoleBalancingSimulation::fitness(){
    return mNumCycles - mCycleCounter;
}

Simulation* PoleBalancingSimulation::getNewCopy(){
    Simulation* newsim = new PoleBalancingSimulation(*this);
    newsim->initialise();

    return newsim;
}

void PoleBalancingSimulation::tick(){
}

double PoleBalancingSimulation::realFitness(){
    return mNumCycles - mCycleCounter;
}

bool PoleBalancingSimulation::initialise(){
    if(mInitialised)
        return true;

    mLastAppliedForce = 0;

    boost::mt19937 rng(mSeed);
    boost::uniform_real<double> dist(-30, 30);
    boost::variate_generator<boost::mt19937, boost::uniform_real<double>> gen(rng, dist);

    mCart.pole_length = 5.0;
    mCart.cart_mass = 1.0;
    mCart.pole_mass = 10.0;
    mCart.cart_x_position = 0.0;
    mCart.cart_x_velocity = 0.0;
    mCart.max_angle = PI/2;
    mCart.min_angle = -PI/2;
    mCart.pole_angle = (gen() / 180) * PI;
    mCart.pole_angular_velocity = 0.0;

    mInitialised = true;
    
    return true;
}

vector<Line> PoleBalancingSimulation::getLines(){
    vector<Line> out;

    Line pole;
    pole.p1 = vector3(mCart.cart_x_position, 0, 0);
    pole.p2 = vector3((-mCart.pole_length * sin(-mCart.pole_angle)) + mCart.cart_x_position, mCart.pole_length * cos(-mCart.pole_angle), 0);

    out.push_back(pole);

    Line rail;
    rail.p1 = vector3(MIN_X_POSITION, 0, 0);
    rail.p2 = vector3(MAX_X_POSITION, 0, 0);

    out.push_back(rail);

    return out;
}