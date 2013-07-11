#ifndef FACTORY_H
#define FACTORY_H

#include <map>

using namespace std;

template<class Super, class IdType, class Creator>
class Factory
{
public:
    bool reg(const IdType& _id, Creator _creator);
    bool unReg(const IdType& _id);
    Super* create(const IdType& _id);

private:
    map<IdType, Creator> registeredCreators;
};

#endif