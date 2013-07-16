#ifndef FACTORY_H
#define FACTORY_H

#include <map>

using namespace std;

template<class Super, class IdType, class Creator, class ParameterType>
class Factory
{
public:
    bool registerCreator(const IdType& _id, Creator _creator){
        return mRegisteredCreators.insert(map<IdType, Creator>::value_type(_id, _creator)).second;
    }

    bool unregisterCreator(const IdType& _id){
        return mRegisteredCreators.erase(_id) == 1;
    }

    Super* create(const IdType& _id, ParameterType _parameters){
        map<IdType, Creator>::const_iterator iter = mRegisteredCreators.find(_id);
        return (iter != mRegisteredCreators.end()) ? (iter->second)(_parameters) : 0;
    }

    void clear(){
        mRegisteredCreators.clear();
    }

private:
    map<IdType, Creator> mRegisteredCreators;
};

#endif