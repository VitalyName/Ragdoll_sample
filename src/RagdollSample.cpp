// RagdollSample.cpp

#include "RagdollSample.h"

#define BIT(x) (1<<(x))
enum collisiontypes   //список взаимодействия тел
    {
    COL_NOTHING = 0, // ничего
    COL_STATIC_GEO = BIT(0), //ландшафт и препядствия
    COL_SHOOT = BIT(1), // выстрелы (ящики)
    COL_RAGDOLL = BIT(2) // Ragdoll
    };
int StaticCollidesWith = COL_SHOOT | COL_RAGDOLL;
int RagdollCollidesWith = COL_STATIC_GEO | COL_SHOOT | COL_RAGDOLL;
int ShootCollidesWith = COL_STATIC_GEO | COL_SHOOT | COL_RAGDOLL;

//-------------------------------------------------------------------------------------
RagdollSample::RagdollSample(void)
{
}
//-------------------------------------------------------------------------------------
RagdollSample::~RagdollSample(void)
{
}
//-------------------------------------------------------------------------------------
void RagdollSample::createScene(void)
{
    //создадим простенькую сцену с камерой
    mSceneMgr->setAmbientLight(Ogre::ColourValue(150, 150, 150));
    Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("CamNode1", Ogre::Vector3(-400, 200, 400));
    node->yaw(Ogre::Degree(-90));
    node = node->createChildSceneNode("PitchNode1");
    node->attachObject(mCamera);
	//node->yaw(Ogre::Degree(90));

    //загрузим меш, от которого будем отталкиваться
    ragdollEntity = mSceneMgr->createEntity("Robot", "robot.mesh");
    ragdollNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("ragdollNode");
    ragdollNode->attachObject(ragdollEntity);
    ragdollNode->setPosition(400,500,600);
  //  ragdollNode->yaw(Ogre::Degree(90));
  //  ragdollNode->pitch(Ogre::Degree(90));
 //   ragdollNode->roll(Ogre::Degree(45));
	ragdollEntity->getMesh()->_setBounds(Ogre::AxisAlignedBox(-9000,-9000,-9000,9000,9000,90000));

    //инициализация физики
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-10,0));

    debugDraw = new CDebugDraw(mSceneMgr, dynamicsWorld);   //
    debugDraw->setDebugMode(btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawWireframe);
	dynamicsWorld->setDebugDrawer(debugDraw);               /// !!!!!!  Debag Drawer !!!!!!!!
	debug=0;                                                //

    mNumEntitiesInstanced=0;

    //создадим из двух плоскостей горку на которую будет падать робот
    Ogre::Vector3 pos(0,-500,1500);
    node = mSceneMgr->getRootSceneNode()->createChildSceneNode("GroundNode", pos);
    node->pitch(Ogre::Degree(-15));
    Ogre::Plane plane1(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane1, 5000, 5000, 20, 20, true, 1, 15, 15, Ogre::Vector3::UNIT_Z);
    Ogre::Entity* entGround = mSceneMgr->createEntity("GroundEntity", "ground");
    node->attachObject(entGround);
    entGround->setMaterialName("Examples/Rockwall");
    entGround->setCastShadows(false);
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
    MyMotionState* groundMotionState = new MyMotionState(btTransform(btQuaternion(btVector3(1,0,0), -0.2616),cvt(pos)), node);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,cvt(pos));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody, COL_STATIC_GEO, StaticCollidesWith);

    Ogre::Vector3 pos2(0,-50,0);
    node = mSceneMgr->getRootSceneNode()->createChildSceneNode("GroundNode2", pos2);
    node->pitch(Ogre::Degree(45));
    Ogre::Plane plane2(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane2, 5000, 5000, 20, 20, true, 1, 15, 15, Ogre::Vector3::UNIT_Z);
    Ogre::Entity* entGround2 = mSceneMgr->createEntity("GroundEntity2", "ground2");
    node->attachObject(entGround2);
    entGround2->setMaterialName("Examples/Rockwall");
    entGround2->setCastShadows(false);
    btCollisionShape* groundShape2 = new btStaticPlaneShape(btVector3(0,1,0),0);
    MyMotionState* groundMotionState2 = new MyMotionState(btTransform(btQuaternion(btVector3(1,0,0), 0.785),cvt(pos2)), node);
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI2(0,groundMotionState2,groundShape2,cvt(pos2));
    btRigidBody* groundRigidBody2= new btRigidBody(groundRigidBodyCI2);
    dynamicsWorld->addRigidBody(groundRigidBody2, COL_STATIC_GEO, StaticCollidesWith);
//=========================================================================================================================

    //создадим информацию о физическом скелете на осове анимационного
    Ogre::Skeleton *skeleton = ragdollEntity->getSkeleton();
    Ogre::Skeleton::BoneIterator it = skeleton->getBoneIterator();

    while (it.hasMoreElements())
    {
        Ogre::Bone *bone = it.getNext();

        if(bone->getParent() == 0) // корневая кость
        {
			bone->setManuallyControlled(true);
            structBoneInfo buffBoneInfo;
            buffBoneInfo.name = bone->getName();
            buffBoneInfo.type = "root";
            buffBoneInfo.baseGraphPos = bone->_getDerivedPosition();
            buffBoneInfo.basePhysPos = bone->_getDerivedPosition();
            buffBoneInfo.baseGraphOrient = bone->_getDerivedOrientation();
            buffBoneInfo.basePhysOrient = bone->_getDerivedOrientation();
            buffBoneInfo.mass = 10;
            buffBoneInfo.lenght = 1;

            boneInfo.push_back(buffBoneInfo);

            Ogre::LogManager::getSingleton().logMessage("Root bone: " + bone->getName());
        }
        else
        {
            if(bone->numChildren() == 0) //крайняя кость
            {
                Ogre::Vector3 posFrom = bone->getParent()->_getDerivedPosition();
                Ogre::Vector3 posTo = bone->_getDerivedPosition();
                Ogre::Vector3 direction = posTo - posFrom;

                direction.normalise();
                Ogre::Quaternion physOrient = Ogre::Vector3::UNIT_Y.getRotationTo(direction);

                structBoneInfo buffBoneInfo;
                buffBoneInfo.name = bone->getName();
                buffBoneInfo.type = "end";
                buffBoneInfo.baseGraphPos = bone->_getDerivedPosition();
                buffBoneInfo.basePhysPos = posTo;
                buffBoneInfo.baseGraphOrient = bone->_getDerivedOrientation();
                buffBoneInfo.basePhysOrient = physOrient;
                buffBoneInfo.mass = 10;
                buffBoneInfo.lenght = 0.1;

                boneInfo.push_back(buffBoneInfo);

                Ogre::LogManager::getSingleton().logMessage("bone end: " + bone->getName());
            }

            if (bone->numChildren() == 1) // обычная кость (одна дочерняя)
            {
				bone->setManuallyControlled(true);
                Ogre::Vector3 posFrom = bone->_getDerivedPosition();
                Ogre::Vector3 posTo = bone->getChild(0)->_getDerivedPosition();
                Ogre::Vector3 physPos = (posFrom + posTo)/2;

                Ogre::Vector3 direction = posTo - posFrom;
                Ogre::Real physLenght = direction.length();

                direction.normalise();
                Ogre::Quaternion physOrient = Ogre::Vector3::UNIT_Y.getRotationTo(direction);

                structBoneInfo buffBoneInfo;
                buffBoneInfo.name = bone->getName();
                buffBoneInfo.type = "normal";
                buffBoneInfo.baseGraphPos = bone->_getDerivedPosition();
                buffBoneInfo.basePhysPos = physPos;
                buffBoneInfo.baseGraphOrient = bone->_getDerivedOrientation();
                buffBoneInfo.basePhysOrient = physOrient;
                buffBoneInfo.mass = 10;
                buffBoneInfo.lenght = physLenght;

                boneInfo.push_back(buffBoneInfo);

                Ogre::LogManager::getSingleton().logMessage("Bone normal: " + bone->getName());
            }

            if (bone->numChildren() > 1) // сложная кость (более одной дочерней кости)
            {
				bone->setManuallyControlled(true);
                structBoneInfo buffBoneInfo;
                buffBoneInfo.name = bone->getName();
                buffBoneInfo.type = "multi";
                buffBoneInfo.baseGraphPos = bone->_getDerivedPosition();
                buffBoneInfo.basePhysPos = bone->_getDerivedPosition();
                buffBoneInfo.baseGraphOrient = bone->_getDerivedOrientation();
                buffBoneInfo.basePhysOrient = bone->_getDerivedOrientation();
                buffBoneInfo.mass = 10;
                buffBoneInfo.lenght = 0.1;

                boneInfo.push_back(buffBoneInfo);

                Ogre::LogManager::getSingleton().logMessage("Bone multi : " + bone->getName());
            }
        }
    }
   //========================================================================================================

    //создадим информацию о соединениях

    Ogre::Skeleton::BoneIterator itt = skeleton->getBoneIterator();

    while (itt.hasMoreElements())
    {
        Ogre::Bone *boneB = itt.getNext();

        if(boneB->getParent())
        {
            Ogre::Node *boneA = boneB->getParent();

            structContactInfo buffContactInfo;
            buffContactInfo.nameA = boneA->getName();
            buffContactInfo.nameB = boneB->getName();

            unsigned int numberA = 0;
            unsigned int numberB = 0;
            for(unsigned int j=0; j< boneInfo.size(); j++)
            {
                if(boneA->getName() == boneInfo[j].name)
                {
                    numberA = j;
                }

                if(boneB->getName() == boneInfo[j].name)
                {
                    numberB = j;
                }
            }

            buffContactInfo.locOffsetPosA = boneInfo[numberA].basePhysOrient.Inverse()* (boneInfo[numberB].baseGraphPos - boneInfo[numberA].basePhysPos);
            buffContactInfo.locOffsetPosB = - Ogre::Vector3::UNIT_Y * boneInfo[numberB].lenght/2;

            Ogre::Quaternion contOrient(1.57,0,0,1);
            buffContactInfo.locOffsetOrientA = boneInfo[numberA].basePhysOrient.Inverse() * ragdollNode->getOrientation().Inverse() * contOrient;
            buffContactInfo.locOffsetOrientB = boneInfo[numberB].basePhysOrient.Inverse() * ragdollNode->getOrientation().Inverse() * contOrient;

            contactInfo.push_back(buffContactInfo);
        }
    }
   //=========================================================================================================

    //создадим физ тело
    for(unsigned int i=0; i< boneInfo.size(); i++)
    {
        btCollisionShape* buffShape = new btCapsuleShape(btScalar(3), boneInfo[i].lenght * 0.9);
        ragShapes.push_back(buffShape);

		btTransform offset; offset.setIdentity(); // Трансформация нода модели
		offset.setOrigin(cvt(ragdollNode->getPosition()));  
		offset.setRotation(cvt(ragdollNode->getOrientation()));

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(cvt(boneInfo[i].basePhysPos));
		transform.setRotation(cvt(boneInfo[i].basePhysOrient));

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform =  offset * transform;

		btVector3 localInertia(0,0,0);
		buffShape->calculateLocalInertia(boneInfo[i].mass, localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo rbInfo(boneInfo[i].mass, myMotionState, buffShape, localInertia);

		btRigidBody* buffBody= new btRigidBody(rbInfo);
		ragBodies.push_back(buffBody);

		dynamicsWorld->addRigidBody(buffBody, COL_RAGDOLL, RagdollCollidesWith);

        buffBody->setGravity(btVector3(0,-100,0));
        buffBody->setDamping(0.3, 0.85);
        buffBody->setDeactivationTime(0.45);
        buffBody->setSleepingThresholds(80.0, 80.0);
    }

// ------------   создадим соединения -----------------------------
    for(unsigned int i=0; i< contactInfo.size(); i++)
    {
		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.setOrigin(cvt(contactInfo[i].locOffsetPosA));
		localB.setOrigin(cvt(contactInfo[i].locOffsetPosB));
		localA.setRotation(cvt(contactInfo[i].locOffsetOrientA));
		localB.setRotation(cvt(contactInfo[i].locOffsetOrientB));

		unsigned int numberA = 0;
		unsigned int numberB = 0;
		for(unsigned int j=0; j< boneInfo.size(); j++)
        {
            if(contactInfo[i].nameA == boneInfo[j].name)
            {
                numberA = j;
            }

            if(contactInfo[i].nameB == boneInfo[j].name)
            {
                numberB = j;
            }
        }

		btHingeConstraint* buffJoint =  new btHingeConstraint(*ragBodies[numberA], *ragBodies[numberB], localA, localB);

		if(boneInfo[numberB].type == "end")
		{
			buffJoint->setLimit(btScalar(-0), btScalar(0));
		}
		else
		{
			buffJoint->setLimit(btScalar(-0.5), btScalar(0.5));
		}
		ragJoints.push_back(buffJoint);
		buffJoint->setDbgDrawSize(7);
		dynamicsWorld->addConstraint(buffJoint, true);

    }
}

//-------------------------------------------------------------------------------------
void RagdollSample::createFrameListener(void)
{
    BaseApplication::createFrameListener();
    mCamNode = mCamera->getParentSceneNode();
    mRotate = 0.13;
    mMove = 250;
    mMouse->setEventCallback(this);
    mKeyboard->setEventCallback(this);
    mDirection = Ogre::Vector3::ZERO;
}
//-------------------------------------------------------------------------------------
bool RagdollSample::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    if (mWindow->isClosed()) return false;
    if (mShutDown) return false;
    mKeyboard->capture();
    mMouse->capture();

    mTrayMgr->frameRenderingQueued(evt);

    mCamNode->translate(mDirection * evt.timeSinceLastFrame, Ogre::Node::TS_LOCAL);

    dynamicsWorld->stepSimulation(evt.timeSinceLastFrame, 100, 0.002);
    debugDraw->Update();



    //--------------- обновим графический скелет ragdoll по физике
    Ogre::Skeleton *skeleton = ragdollEntity->getSkeleton();
    Ogre::Skeleton::BoneIterator it = skeleton->getBoneIterator();

    while (it.hasMoreElements()) //переберем каждую граф кость
    {
        Ogre::Bone *bone = it.getNext();

        unsigned int number = 0;
        for(unsigned int j=0; j< boneInfo.size(); j++) //найдем физ тело, соответствующее граф кости
        {
			if(bone->getName() == boneInfo[j].name && boneInfo[j].type != "end")
            {
                number = j;
				break;
            }
        }

	    btTransform newPhysTrans;
        newPhysTrans.setIdentity();
        newPhysTrans = ragBodies[number]->getWorldTransform();

        Ogre::Quaternion newPhysOrient = cvt(newPhysTrans.getRotation());
        Ogre::Vector3 newPhysPos = cvt(newPhysTrans.getOrigin());

		Ogre::Vector3 newPhysDeriverdPos = ragdollNode->getOrientation().Inverse() * (newPhysPos - ragdollNode->getPosition());
		Ogre::Quaternion newPhysDerivedOrient = ragdollNode->getOrientation().Inverse() * newPhysOrient;

        Ogre::Vector3 newPhysDerivedDirection =  newPhysDerivedOrient  * Ogre::Vector3::UNIT_Y;
        newPhysDerivedDirection.normalise();

		Ogre::Vector3 newGraphDerivedPos;
		Ogre::Quaternion newGraphDerivedOrient;
		newGraphDerivedPos = newPhysDeriverdPos - newPhysDerivedDirection * boneInfo[number].lenght/2;
		newGraphDerivedOrient = newPhysDerivedOrient * boneInfo[number].baseGraphOrient * boneInfo[number].basePhysOrient.Inverse();

		if(bone->getParent() == 0) //корневая кость
        {
			bone->setPosition(newGraphDerivedPos);
			bone->setOrientation(newGraphDerivedOrient);
        }
        else
        {
            bone->_setDerivedPosition(newGraphDerivedPos);
			bone->_setDerivedOrientation(newGraphDerivedOrient);
        }
    }
    return true;
}

//-------------------------------------------------------------------------------------
bool RagdollSample::keyPressed( const OIS::KeyEvent& evt )
{
	BaseApplication::keyPressed(evt);
    switch (evt.key)
    {
    case OIS::KC_ESCAPE:
        mShutDown = true;
        break;
    case OIS::KC_UP:
    case OIS::KC_W:
        mDirection.z = -mMove;
        break;

    case OIS::KC_DOWN:
    case OIS::KC_S:
        mDirection.z = mMove;
        break;

    case OIS::KC_LEFT:
    case OIS::KC_A:
        mDirection.x = -mMove;
        break;

    case OIS::KC_RIGHT:
    case OIS::KC_D:
        mDirection.x = mMove;
        break;

    case OIS::KC_PGDOWN:
    case OIS::KC_E:
        mDirection.y = -mMove;
        break;

    case OIS::KC_PGUP:
    case OIS::KC_Q:
        mDirection.y = mMove;
        break;
    case OIS::KC_N:
        {
        Ogre::Vector3 pos = (mCamera->getDerivedPosition() + mCamera->getDerivedDirection().normalisedCopy() * 10);
        Ogre::Entity *entity = mSceneMgr->createEntity("Box" + Ogre::StringConverter::toString(mNumEntitiesInstanced), "cube.mesh");
        entity->setCastShadows(true);
        entity->setMaterialName("Examples/BumpyMetal");
        Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
        node->attachObject(entity);
        Ogre::Vector3 size;
        size = entity->getBoundingBox().getSize();
        size /= 2.0f;
        node->scale(0.05f, 0.05f, 0.05f);
        size *= 0.05f;
        btCollisionShape* shootShape = new btBoxShape(btBoxShape(btVector3(size.x, size.y, size.z)));
        MyMotionState* mstate = new MyMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(pos.x, pos.y, pos.z)), node);
        btScalar mass = 300;
        btVector3 shootInertia(0,0,0);
        shootShape->calculateLocalInertia(mass,shootInertia);
        btRigidBody::btRigidBodyConstructionInfo shootRigidBodyCI(mass,mstate,shootShape,shootInertia);
        btRigidBody* shootRigidBody = new btRigidBody(shootRigidBodyCI);
        shootRigidBody->setLinearVelocity(btVector3(mCamera->getDerivedDirection().normalisedCopy().x * 700.0f,
			mCamera->getDerivedDirection().normalisedCopy().y * 700.0f, mCamera->getDerivedDirection().normalisedCopy().z * 700.0f));
        dynamicsWorld->addRigidBody(shootRigidBody, COL_SHOOT, ShootCollidesWith);
        mNumEntitiesInstanced++;
        break;
        }
    case OIS::KC_O:
        debug = !debug;
        if(debug)
        {
            debugDraw->setDebugMode(1);
        }
        else
        {
            debugDraw->setDebugMode(0);
        }
        break;
    default:
        break;
    }
    return true;
}

bool RagdollSample::keyReleased( const OIS::KeyEvent& evt )
{
    switch (evt.key)
    {
    case OIS::KC_UP:
    case OIS::KC_W:
        mDirection.z = 0;
        break;

    case OIS::KC_DOWN:
    case OIS::KC_S:
        mDirection.z = 0;
        break;

    case OIS::KC_LEFT:
    case OIS::KC_A:
        mDirection.x = 0;
        break;

    case OIS::KC_RIGHT:
    case OIS::KC_D:
        mDirection.x = 0;
        break;

    case OIS::KC_PGDOWN:
    case OIS::KC_E:
        mDirection.y = 0;
        break;

    case OIS::KC_PGUP:
    case OIS::KC_Q:
        mDirection.y = 0;
        break;
    default:
        break;
    }
    return true;
}

bool RagdollSample::mouseMoved( const OIS::MouseEvent& evt )
{
    if (evt.state.buttonDown(OIS::MB_Right))
    {
    mCamNode->yaw(Ogre::Degree(-mRotate * evt.state.X.rel), Ogre::Node::TS_WORLD);
    mCamNode->pitch(Ogre::Degree(-mRotate * evt.state.Y.rel), Ogre::Node::TS_LOCAL);
    }
    return true;
}
bool RagdollSample::mousePressed( const OIS::MouseEvent& evt, OIS::MouseButtonID id ){return true;}
bool RagdollSample::mouseReleased( const OIS::MouseEvent& evt, OIS::MouseButtonID id ){return true;}
//-------------------------------------------------------------------------------------


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        RagdollSample app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
