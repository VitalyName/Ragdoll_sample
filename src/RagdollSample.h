// RagdollSample.h

#ifndef __RagdollSample_h_
#define __RagdollSample_h_

#include <btBulletDynamicsCommon.h>                     //
#include <btBulletCollisionCommon.h>                   //  библиотеки Bullet'a

#include "BaseApplication.h"

#include "MyMotionState.h"
#include "OgreBulletUtils.h"
#include "DebugDraw.h"

#include <vector>



class RagdollSample : public BaseApplication
{
public:
    RagdollSample(void);
    virtual ~RagdollSample(void);

protected:
    virtual void createScene(void);
    virtual void createFrameListener(void);

    // Ogre::FrameListener
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt );
    // OIS::KeyListener
    virtual bool keyPressed( const OIS::KeyEvent& evt );
    virtual bool keyReleased( const OIS::KeyEvent& evt );
    // OIS::MouseListener
    virtual bool mouseMoved( const OIS::MouseEvent& evt );
    virtual bool mousePressed( const OIS::MouseEvent& evt, OIS::MouseButtonID id );
    virtual bool mouseReleased( const OIS::MouseEvent& evt, OIS::MouseButtonID id );

    Ogre::Real mRotate;          // The rotate constant
    Ogre::Real mMove;            // The movement constant
    Ogre::SceneNode *mCamNode;   // The SceneNode the camera is currently attached to
    Ogre::Vector3 mDirection;    // Value to move in the correct direction

    Ogre::Entity *ragdollEntity;
    Ogre::SceneNode *ragdollNode;

    btDiscreteDynamicsWorld* dynamicsWorld;

    int mNumEntitiesInstanced;

    CDebugDraw * debugDraw; //дебаговая прорисовка для физики
    bool debug;

    struct structBoneInfo  // информация о костях по которой будем строить физические тела
    {
        Ogre::Vector3 basePhysPos;  // позиция физ. кости в момент составления Ragdoll (базовая)
        Ogre::Vector3 baseGraphPos; // позиция графической кости в момент составления Ragdoll (базовая)
        Ogre::Quaternion basePhysOrient; // ориентация физ кости в момент составления Ragdoll (базовая)
        Ogre::Quaternion baseGraphOrient; // ориентация графической кости в момент составления Ragdoll (базовая)
        Ogre::String type;  // тип кости (root, normal, multi, end (корневая, обычная, сложная, конечная))
        Ogre::String name;  // имя кости
        Ogre::Real lenght;  // длинна физ. тела прикрепленного к кости
        Ogre::Real mass; //масса
    };

    struct structContactInfo   // информация о соединении костей
    {
        Ogre::String nameA; // имена соединяемых костей
        Ogre::String nameB; //
        Ogre::Vector3 locOffsetPosA; // локальные отстройки места соединения
        Ogre::Vector3 locOffsetPosB; //
        Ogre::Quaternion locOffsetOrientA; // локальные отстройки ориентации соединения
        Ogre::Quaternion locOffsetOrientB; //
    };

    std::vector<structBoneInfo> boneInfo;
    std::vector<btCollisionShape*> ragShapes;
	std::vector<btRigidBody*> ragBodies;

    std::vector<structContactInfo> contactInfo;
    std::vector<btTypedConstraint*> ragJoints;
};
#endif // #ifndef __RagdollSample_h_
