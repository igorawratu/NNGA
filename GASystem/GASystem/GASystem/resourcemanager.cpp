#include "resourcemanager.h"

ResourceManager::ResourceManager(){
#ifdef _DEBUG
    Ogre::String resourcesCfg = "resources_d.cfg";
#else
    Ogre::String resourcesCfg = "resources.cfg";
#endif
    
    Ogre::ConfigFile cf;
    cf.load(resourcesCfg);

    Ogre::ConfigFile::SectionIterator iter = cf.getSectionIterator();
    Ogre::String secName, typeName, archName;
    while(iter.hasMoreElements()){
        secName = iter.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap * settings = iter.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for(i = settings->begin(); i != settings->end(); ++i){
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }

    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

ResourceManager::~ResourceManager(){

}

bool ResourceManager::getMeshInformation(string _meshName, size_t& _vertexCount, vector3*& _vertices, size_t& _indexCount, unsigned long*& _indices, const vector3& _position,
                                         const Ogre::Quaternion& _orientation, const vector3& _scale){
    _vertexCount = _indexCount = 0;
    bool addedShared = false;
    size_t currentOffset = 0, sharedOffset = 0, nextOffset = 0, indexOffset = 0;
    Ogre::Vector3 position(_position.x, _position.y, _position.z);
    Ogre::Vector3 scale(_scale.x, _scale.y, _scale.z);

    Ogre::MeshManager::getSingleton().load(_meshName, Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
    Ogre::Mesh* mesh = dynamic_cast<Ogre::Mesh*>(Ogre::MeshManager::getSingleton().getByName(_meshName).getPointer());
    if(mesh == 0){
        cerr << "Error: unable to retrieve mesh with name " << _meshName << endl;
        return false;
    }

    //calc num vertices and indices
    for(uint i = 0; i < mesh->getNumSubMeshes(); ++i){
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        if(submesh->useSharedVertices){
            //only add shared vertices once
            if(!addedShared){
                _vertexCount += mesh->sharedVertexData->vertexCount;
                addedShared = true;
            }
        }
        else
            _vertexCount += submesh->vertexData->vertexCount;

        //add index data
        _indexCount += submesh->indexData->indexCount;
    }

    //allocate memory
    _vertices = new vector3[_vertexCount];
    _indices = new unsigned long[_indexCount];

    addedShared = false;

    //loop through submeshes and get data
    for(uint i = 0; i < mesh->getNumSubMeshes(); ++i){
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        Ogre::VertexData* vertexData = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        if((!submesh->useSharedVertices) || (submesh->useSharedVertices && !addedShared)){
            if(submesh->useSharedVertices){
                addedShared = true;
                sharedOffset = currentOffset;
            }

            const Ogre::VertexElement* posElem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            Ogre::HardwareVertexBufferSharedPtr vbuf = vertexData->vertexBufferBinding->getBuffer(posElem->getSource());
            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            float* pReal;

            for(size_t j = 0; j < vertexData->vertexCount; ++j, vertex += vbuf->getVertexSize()){
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
                Ogre::Vector3 currFinal = (_orientation * (pt * scale)) + position;

                _vertices[currentOffset + j] = vector3(currFinal.x, currFinal.y, currFinal.z);
            }

            vbuf->unlock();
            nextOffset += vertexData->vertexCount;
        }

        Ogre::IndexData* indexData = submesh->indexData;
        size_t numTriangles = indexData->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = indexData->indexBuffer;
        
        bool use32bitindices = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
        
        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset = submesh->useSharedVertices ? sharedOffset : currentOffset;
        if(use32bitindices)
            for(size_t k = 0; k < numTriangles * 3; ++k)
                _indices[indexOffset++] = pLong[k] + static_cast<unsigned long>(offset);
        else
            for(size_t k = 0; k < numTriangles * 3; ++k)
                _indices[indexOffset++] = static_cast<unsigned long>(pShort[k]) + static_cast<unsigned long>(offset);

        ibuf->unlock();
        currentOffset = nextOffset;
    }

    return true;
}

btCollisionShape* ResourceManager::getBulletCollisionShape(string _meshName, bool _static, bool _concave, const vector3& _position, const vector3& _scale, const Ogre::Quaternion& _orientation){
    unsigned long* indices;
    vector3* vertices;
    size_t vertexCount = 0, indexCount = 0;

    if(!getMeshInformation(_meshName, vertexCount, vertices, indexCount, indices, _position, _orientation, _scale))
        return NULL;

    btTriangleMesh* triMesh = new btTriangleMesh();

    

    for(size_t i = 0; i < indexCount; i+=3){
        btVector3 v0(vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z);
        btVector3 v1(vertices[indices[i + 1]].x, vertices[indices[i + 1]].y, vertices[indices[i + 1]].z);
        btVector3 v2(vertices[indices[i + 2]].x, vertices[indices[i + 2]].y, vertices[indices[i + 2]].z);

        triMesh->addTriangle(v0, v1, v2);
    }
    
    if(!_concave){
        btConvexShape* unsimpShape = new btConvexTriangleMeshShape(triMesh);
        btShapeHull* hull = new btShapeHull(unsimpShape);
        hull->buildHull(unsimpShape->getMargin());
        btConvexHullShape* out = new btConvexHullShape(&hull->getVertexPointer()->getX(), hull->numVertices());

        delete indices;
        delete vertices;
        delete triMesh;
        delete unsimpShape;
        delete hull;

        return out;
    }
    else if(_concave && _static){
        btBvhTriangleMeshShape* concaveShape = new btBvhTriangleMeshShape(triMesh, true);

        delete indices;
        delete vertices;

        return concaveShape;
    }
    else{
        //convex decomposition here
        return 0;
    }
}