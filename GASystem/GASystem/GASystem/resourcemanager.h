#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <string>
#include <map>

#include "common.h"

#include <BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include <OgreRoot.h>
#include <OgreConfigFile.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreResourceGroupManager.h>

using namespace std;

struct ModelInfo
{
    ModelInfo(vector3* _vertices, unsigned long* _indices, size_t _vertexCount, size_t _indexCount){
        vertices = _vertices;
        indices = _indices;
        vertexCount = _vertexCount;
        indexCount = _indexCount;
    }

    ~ModelInfo(){
        delete vertices;
        vertices = 0;
        delete indices;
        indices = 0;
    }
    vector3* vertices;
    unsigned long* indices;
    size_t vertexCount, indexCount;
};

class ResourceManager
{
public:
    ResourceManager();
    ~ResourceManager();

    btCollisionShape* getBulletCollisionShape(string _meshName, bool _static, bool _concave, const vector3& _position = vector3(0, 0, 0), const vector3& _scale = vector3(1, 1, 1),
        const Ogre::Quaternion& _orientation = Ogre::Quaternion::IDENTITY);

private:
    bool getMeshInformation(string _meshName, size_t& _vertexCount, vector3*& _vertices, size_t& _indexCount, unsigned long*& _indices, 
        const vector3& _position = vector3(0, 0, 0), const Ogre::Quaternion& _orientation = Ogre::Quaternion::IDENTITY, const vector3& _scale = vector3(1, 1, 1));

private:
    map<string, ModelInfo*> mCache;

};

#endif