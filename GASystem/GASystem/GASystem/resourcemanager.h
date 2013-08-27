#ifndef RESOURCEMANAGER_H
#define RESOURCEMANAGER_H

#include <string>

#include "common.h"

#include <OgreRoot.h>
#include <OgreConfigFile.h>
#include <OgreEntity.h>
#include <BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreResourceGroupManager.h>

using namespace std;

class ResourceManager
{
public:
    ResourceManager();
    ~ResourceManager();

    bool getMeshInformation(string _meshName, size_t& _vertexCount, vector3*& _vertices, size_t& _indexCount, unsigned long*& _indices, 
        const vector3& _position = vector3(0, 0, 0), const Ogre::Quaternion& _orientation = Ogre::Quaternion::IDENTITY, const vector3& _scale = vector3(1, 1, 1));

    btConvexShape* getBulletCollisionShape(string _meshName, const vector3& _position = vector3(0, 0, 0), const vector3& _scale = vector3(1, 1, 1),
        const Ogre::Quaternion& _orientation = Ogre::Quaternion::IDENTITY);

};

#endif