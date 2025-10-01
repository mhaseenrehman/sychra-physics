#include "phy_engine.h"
#include <memory.h>
#include <assert.h>

#include <cstdlib>
#include <cstdio>

//using namespace Physics;


static inline void _checkInverseInertiaTensor(const Matrix3& iitWorld)
{

}

static inline void _transformInertiaTensor(Matrix3& iitWorld,
    const Quaternion& q,
    const Matrix3& iitBody,
    const Matrix4& rotmat)
{
    real t4 = rotmat.data[0] * iitBody.data[0] +
        rotmat.data[1] * iitBody.data[3] +
        rotmat.data[2] * iitBody.data[6];
    real t9 = rotmat.data[0] * iitBody.data[1] +
        rotmat.data[1] * iitBody.data[4] +
        rotmat.data[2] * iitBody.data[7];
    real t14 = rotmat.data[0] * iitBody.data[2] +
        rotmat.data[1] * iitBody.data[5] +
        rotmat.data[2] * iitBody.data[8];
    real t28 = rotmat.data[4] * iitBody.data[0] +
        rotmat.data[5] * iitBody.data[3] +
        rotmat.data[6] * iitBody.data[6];
    real t33 = rotmat.data[4] * iitBody.data[1] +
        rotmat.data[5] * iitBody.data[4] +
        rotmat.data[6] * iitBody.data[7];
    real t38 = rotmat.data[4] * iitBody.data[2] +
        rotmat.data[5] * iitBody.data[5] +
        rotmat.data[6] * iitBody.data[8];
    real t52 = rotmat.data[8] * iitBody.data[0] +
        rotmat.data[9] * iitBody.data[3] +
        rotmat.data[10] * iitBody.data[6];
    real t57 = rotmat.data[8] * iitBody.data[1] +
        rotmat.data[9] * iitBody.data[4] +
        rotmat.data[10] * iitBody.data[7];
    real t62 = rotmat.data[8] * iitBody.data[2] +
        rotmat.data[9] * iitBody.data[5] +
        rotmat.data[10] * iitBody.data[8];

    iitWorld.data[0] = t4 * rotmat.data[0] +
        t9 * rotmat.data[1] +
        t14 * rotmat.data[2];
    iitWorld.data[1] = t4 * rotmat.data[4] +
        t9 * rotmat.data[5] +
        t14 * rotmat.data[6];
    iitWorld.data[2] = t4 * rotmat.data[8] +
        t9 * rotmat.data[9] +
        t14 * rotmat.data[10];
    iitWorld.data[3] = t28 * rotmat.data[0] +
        t33 * rotmat.data[1] +
        t38 * rotmat.data[2];
    iitWorld.data[4] = t28 * rotmat.data[4] +
        t33 * rotmat.data[5] +
        t38 * rotmat.data[6];
    iitWorld.data[5] = t28 * rotmat.data[8] +
        t33 * rotmat.data[9] +
        t38 * rotmat.data[10];
    iitWorld.data[6] = t52 * rotmat.data[0] +
        t57 * rotmat.data[1] +
        t62 * rotmat.data[2];
    iitWorld.data[7] = t52 * rotmat.data[4] +
        t57 * rotmat.data[5] +
        t62 * rotmat.data[6];
    iitWorld.data[8] = t52 * rotmat.data[8] +
        t57 * rotmat.data[9] +
        t62 * rotmat.data[10];
}

static inline void _calculateTransformMatrix(Matrix4& transformMatrix,
    const Vector3& position,
    const Quaternion& orientation)
{
    transformMatrix.data[0] = 1 - 2 * orientation.j * orientation.j -
        2 * orientation.k * orientation.k;
    transformMatrix.data[1] = 2 * orientation.i * orientation.j -
        2 * orientation.r * orientation.k;
    transformMatrix.data[2] = 2 * orientation.i * orientation.k +
        2 * orientation.r * orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2 * orientation.i * orientation.j +
        2 * orientation.r * orientation.k;
    transformMatrix.data[5] = 1 - 2 * orientation.i * orientation.i -
        2 * orientation.k * orientation.k;
    transformMatrix.data[6] = 2 * orientation.j * orientation.k -
        2 * orientation.r * orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2 * orientation.i * orientation.k -
        2 * orientation.r * orientation.j;
    transformMatrix.data[9] = 2 * orientation.j * orientation.k +
        2 * orientation.r * orientation.i;
    transformMatrix.data[10] = 1 - 2 * orientation.i * orientation.i -
        2 * orientation.j * orientation.j;
    transformMatrix.data[11] = position.z;

}

void RigidBody::calculateDerivedData()
{
    orientation.normalise();

    _calculateTransformMatrix(transformMatrix, position, orientation);

    _transformInertiaTensor(inverseInertiaTensorWorld,
        orientation,
        inverseInertiaTensor,
        transformMatrix);

}

void RigidBody::integrate(real duration)
{
    if (!isAwake) return;

    lastFrameAcceleration = acceleration;
    lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

    Vector3 angularAcceleration =
        inverseInertiaTensorWorld.transform(torqueAccum);

    velocity.addScaledVector(lastFrameAcceleration, duration);

    rotation.addScaledVector(angularAcceleration, duration);

    velocity *= real_pow(linearDamping, duration);
    rotation *= real_pow(angularDamping, duration);

    position.addScaledVector(velocity, duration);

    orientation.addScaledVector(rotation, duration);

    calculateDerivedData();

    clearAccumulators();

    if (canSleep) {
        real currentMotion = velocity.scalarProduct(velocity) +
            rotation.scalarProduct(rotation);

        real bias = real_pow(0.5, duration);
        motion = bias * motion + (1 - bias) * currentMotion;

        if (motion < sleepEpsilon) setAwake(false);
        else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
    }
}

void RigidBody::setMass(const real mass)
{
    assert(mass != 0);
    RigidBody::inverseMass = ((real)1.0) / mass;
}

real RigidBody::getMass() const
{
    if (inverseMass == 0) {
        return REAL_MAX;
    }
    else {
        return ((real)1.0) / inverseMass;
    }
}

void RigidBody::setInverseMass(const real inverseMass)
{
    RigidBody::inverseMass = inverseMass;
}

real RigidBody::getInverseMass() const
{
    return inverseMass;
}

bool RigidBody::hasFiniteMass() const
{
    return inverseMass >= 0.0f;
}

void RigidBody::setInertiaTensor(const Matrix3& inertiaTensor)
{
    inverseInertiaTensor.setInverse(inertiaTensor);
    _checkInverseInertiaTensor(inverseInertiaTensor);
}

void RigidBody::getInertiaTensor(Matrix3* inertiaTensor) const
{
    inertiaTensor->setInverse(inverseInertiaTensor);
}

Matrix3 RigidBody::getInertiaTensor() const
{
    Matrix3 it;
    getInertiaTensor(&it);
    return it;
}

void RigidBody::getInertiaTensorWorld(Matrix3* inertiaTensor) const
{
    inertiaTensor->setInverse(inverseInertiaTensorWorld);
}

Matrix3 RigidBody::getInertiaTensorWorld() const
{
    Matrix3 it;
    getInertiaTensorWorld(&it);
    return it;
}

void RigidBody::setInverseInertiaTensor(const Matrix3& inverseInertiaTensor)
{
    _checkInverseInertiaTensor(inverseInertiaTensor);
    RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensor(Matrix3* inverseInertiaTensor) const
{
    *inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

Matrix3 RigidBody::getInverseInertiaTensor() const
{
    return inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensorWorld(Matrix3* inverseInertiaTensor) const
{
    *inverseInertiaTensor = inverseInertiaTensorWorld;
}

Matrix3 RigidBody::getInverseInertiaTensorWorld() const
{
    return inverseInertiaTensorWorld;
}

void RigidBody::setDamping(const real linearDamping,
    const real angularDamping)
{
    RigidBody::linearDamping = linearDamping;
    RigidBody::angularDamping = angularDamping;
}

void RigidBody::setLinearDamping(const real linearDamping)
{
    RigidBody::linearDamping = linearDamping;
}

real RigidBody::getLinearDamping() const
{
    return linearDamping;
}

void RigidBody::setAngularDamping(const real angularDamping)
{
    RigidBody::angularDamping = angularDamping;
}

real RigidBody::getAngularDamping() const
{
    return angularDamping;
}

void RigidBody::setPosition(const Vector3& position)
{
    RigidBody::position = position;
}

void RigidBody::setPosition(const real x, const real y, const real z)
{
    position.x = x;
    position.y = y;
    position.z = z;
}

void RigidBody::getPosition(Vector3* position) const
{
    *position = RigidBody::position;
}

Vector3 RigidBody::getPosition() const
{
    return position;
}

void RigidBody::setOrientation(const Quaternion& orientation)
{
    RigidBody::orientation = orientation;
    RigidBody::orientation.normalise();
}

void RigidBody::setOrientation(const real r, const real i,
    const real j, const real k)
{
    orientation.r = r;
    orientation.i = i;
    orientation.j = j;
    orientation.k = k;
    orientation.normalise();
}

void RigidBody::getOrientation(Quaternion* orientation) const
{
    *orientation = RigidBody::orientation;
}

Quaternion RigidBody::getOrientation() const
{
    return orientation;
}

void RigidBody::getOrientation(Matrix3* matrix) const
{
    getOrientation(matrix->data);
}

void RigidBody::getOrientation(real matrix[9]) const
{
    matrix[0] = transformMatrix.data[0];
    matrix[1] = transformMatrix.data[1];
    matrix[2] = transformMatrix.data[2];

    matrix[3] = transformMatrix.data[4];
    matrix[4] = transformMatrix.data[5];
    matrix[5] = transformMatrix.data[6];

    matrix[6] = transformMatrix.data[8];
    matrix[7] = transformMatrix.data[9];
    matrix[8] = transformMatrix.data[10];
}

void RigidBody::getTransform(Matrix4* transform) const
{
    memcpy(transform, &transformMatrix.data, sizeof(Matrix4));
}

void RigidBody::getTransform(real matrix[16]) const
{
    memcpy(matrix, transformMatrix.data, sizeof(real) * 12);
    matrix[12] = matrix[13] = matrix[14] = 0;
    matrix[15] = 1;
}

void RigidBody::getGLTransform(float matrix[16]) const
{
    matrix[0] = (float)transformMatrix.data[0];
    matrix[1] = (float)transformMatrix.data[4];
    matrix[2] = (float)transformMatrix.data[8];
    matrix[3] = 0;

    matrix[4] = (float)transformMatrix.data[1];
    matrix[5] = (float)transformMatrix.data[5];
    matrix[6] = (float)transformMatrix.data[9];
    matrix[7] = 0;

    matrix[8] = (float)transformMatrix.data[2];
    matrix[9] = (float)transformMatrix.data[6];
    matrix[10] = (float)transformMatrix.data[10];
    matrix[11] = 0;

    matrix[12] = (float)transformMatrix.data[3];
    matrix[13] = (float)transformMatrix.data[7];
    matrix[14] = (float)transformMatrix.data[11];
    matrix[15] = 1;
}

Matrix4 RigidBody::getTransform() const
{
    return transformMatrix;
}

Vector3 RigidBody::getPointInLocalSpace(const Vector3& point) const
{
    return transformMatrix.transformInverse(point);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3& point) const
{
    return transformMatrix.transform(point);
}

Vector3 RigidBody::getDirectionInLocalSpace(const Vector3& direction) const
{
    return transformMatrix.transformInverseDirection(direction);
}

Vector3 RigidBody::getDirectionInWorldSpace(const Vector3& direction) const
{
    return transformMatrix.transformDirection(direction);
}

void RigidBody::setVelocity(const Vector3& velocity)
{
    RigidBody::velocity = velocity;
}

void RigidBody::setVelocity(const real x, const real y, const real z)
{
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void RigidBody::getVelocity(Vector3* velocity) const
{
    *velocity = RigidBody::velocity;
}

Vector3 RigidBody::getVelocity() const
{
    return velocity;
}

void RigidBody::addVelocity(const Vector3& deltaVelocity)
{
    velocity += deltaVelocity;
}

void RigidBody::setRotation(const Vector3& rotation)
{
    RigidBody::rotation = rotation;
}

void RigidBody::setRotation(const real x, const real y, const real z)
{
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
}

void RigidBody::getRotation(Vector3* rotation) const
{
    *rotation = RigidBody::rotation;
}

Vector3 RigidBody::getRotation() const
{
    return rotation;
}

void RigidBody::addRotation(const Vector3& deltaRotation)
{
    rotation += deltaRotation;
}

void RigidBody::setAwake(const bool awake)
{
    if (awake) {
        isAwake = true;

        motion = sleepEpsilon * 2.0f;
    }
    else {
        isAwake = false;
        velocity.clear();
        rotation.clear();
    }
}

void RigidBody::setCanSleep(const bool canSleep)
{
    RigidBody::canSleep = canSleep;

    if (!canSleep && !isAwake) setAwake();
}

void RigidBody::getLastFrameAcceleration(Vector3* acceleration) const
{
    *acceleration = lastFrameAcceleration;
}

Vector3 RigidBody::getLastFrameAcceleration() const
{
    return lastFrameAcceleration;
}

void RigidBody::clearAccumulators()
{
    forceAccum.clear();
    torqueAccum.clear();
}

void RigidBody::addForce(const Vector3& force)
{
    forceAccum += force;
    isAwake = true;
}

void RigidBody::addForceAtBodyPoint(const Vector3& force,
    const Vector3& point)
{

    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);

}

void RigidBody::addForceAtPoint(const Vector3& force,
    const Vector3& point)
{

    Vector3 pt = point;
    pt -= position;

    forceAccum += force;
    torqueAccum += pt % force;

    isAwake = true;
}

void RigidBody::addTorque(const Vector3& torque)
{
    torqueAccum += torque;
    isAwake = true;
}

void RigidBody::setAcceleration(const Vector3& acceleration)
{
    RigidBody::acceleration = acceleration;
}

void RigidBody::setAcceleration(const real x, const real y, const real z)
{
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

void RigidBody::getAcceleration(Vector3* acceleration) const
{
    *acceleration = RigidBody::acceleration;
}

Vector3 RigidBody::getAcceleration() const
{
    return acceleration;
}

void Contact::setBodyData(RigidBody* one, RigidBody* two,
    real friction, real restitution)
{
    Contact::body[0] = one;
    Contact::body[1] = two;
    Contact::friction = friction;
    Contact::restitution = restitution;
}

void Contact::matchAwakeState()
{

    if (!body[1]) return;

    bool body0awake = body[0]->getAwake();
    bool body1awake = body[1]->getAwake();

    if (body0awake ^ body1awake) {
        if (body0awake) body[1]->setAwake();
        else body[0]->setAwake();
    }
}

void Contact::swapBodies()
{
    contactNormal *= -1;

    RigidBody* temp = body[0];
    body[0] = body[1];
    body[1] = temp;
}

inline
void Contact::calculateContactBasis()
{
    Vector3 contactTangent[2];

    if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
    {

        const real s = (real)1.0f / real_sqrt(contactNormal.z * contactNormal.z +
            contactNormal.x * contactNormal.x);

        contactTangent[0].x = contactNormal.z * s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x -
            contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else
    {

        const real s = (real)1.0 / real_sqrt(contactNormal.z * contactNormal.z +
            contactNormal.y * contactNormal.y);

        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * s;
        contactTangent[0].z = contactNormal.y * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].z -
            contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    contactToWorld.setComponents(
        contactNormal,
        contactTangent[0],
        contactTangent[1]);
}

Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real duration)
{
    RigidBody* thisBody = body[bodyIndex];

    Vector3 velocity =
        thisBody->getRotation() % relativeContactPosition[bodyIndex];
    velocity += thisBody->getVelocity();

    Vector3 contactVelocity = contactToWorld.transformTranspose(velocity);

    Vector3 accVelocity = thisBody->getLastFrameAcceleration() * duration;

    accVelocity = contactToWorld.transformTranspose(accVelocity);

    accVelocity.x = 0;

    contactVelocity += accVelocity;

    return contactVelocity;
}

void Contact::calculateDesiredDeltaVelocity(real duration)
{
    const static real velocityLimit = (real)0.25f;

    real velocityFromAcc = 0;

    if (body[0]->getAwake())
    {
        velocityFromAcc +=
            body[0]->getLastFrameAcceleration() * duration * contactNormal;
    }

    if (body[1] && body[1]->getAwake())
    {
        velocityFromAcc -=
            body[1]->getLastFrameAcceleration() * duration * contactNormal;
    }

    real thisRestitution = restitution;
    if (real_abs(contactVelocity.x) < velocityLimit)
    {
        thisRestitution = (real)0.0f;
    }

    desiredDeltaVelocity =
        -contactVelocity.x
        - thisRestitution * (contactVelocity.x - velocityFromAcc);
}

void Contact::calculateInternals(real duration)
{

    if (!body[0]) swapBodies();
    assert(body[0]);

    calculateContactBasis();

    relativeContactPosition[0] = contactPoint - body[0]->getPosition();
    if (body[1]) {
        relativeContactPosition[1] = contactPoint - body[1]->getPosition();
    }

    contactVelocity = calculateLocalVelocity(0, duration);
    if (body[1]) {
        contactVelocity -= calculateLocalVelocity(1, duration);
    }

    calculateDesiredDeltaVelocity(duration);
}

void Contact::applyVelocityChange(Vector3 velocityChange[2],
    Vector3 rotationChange[2])
{

    Matrix3 inverseInertiaTensor[2];
    body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
    if (body[1])
        body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

    Vector3 impulseContact;

    if (friction == (real)0.0)
    {

        impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
    }
    else
    {

        impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
    }

    Vector3 impulse = contactToWorld.transform(impulseContact);

    Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
    rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
    velocityChange[0].clear();
    velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());

    body[0]->addVelocity(velocityChange[0]);
    body[0]->addRotation(rotationChange[0]);

    if (body[1])
    {

        Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
        rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
        velocityChange[1].clear();
        velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());

        body[1]->addVelocity(velocityChange[1]);
        body[1]->addRotation(rotationChange[1]);
    }
}

inline
Vector3 Contact::calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor)
{
    Vector3 impulseContact;

    Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
    deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

    real deltaVelocity = deltaVelWorld * contactNormal;

    deltaVelocity += body[0]->getInverseMass();

    if (body[1])
    {

        Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
        deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld % relativeContactPosition[1];

        deltaVelocity += deltaVelWorld * contactNormal;

        deltaVelocity += body[1]->getInverseMass();
    }

    impulseContact.x = desiredDeltaVelocity / deltaVelocity;
    impulseContact.y = 0;
    impulseContact.z = 0;
    return impulseContact;
}

inline
Vector3 Contact::calculateFrictionImpulse(Matrix3* inverseInertiaTensor)
{
    Vector3 impulseContact;
    real inverseMass = body[0]->getInverseMass();

    Matrix3 impulseToTorque;
    impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

    Matrix3 deltaVelWorld = impulseToTorque;
    deltaVelWorld *= inverseInertiaTensor[0];
    deltaVelWorld *= impulseToTorque;
    deltaVelWorld *= -1;

    if (body[1])
    {

        impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);

        Matrix3 deltaVelWorld2 = impulseToTorque;
        deltaVelWorld2 *= inverseInertiaTensor[1];
        deltaVelWorld2 *= impulseToTorque;
        deltaVelWorld2 *= -1;

        deltaVelWorld += deltaVelWorld2;

        inverseMass += body[1]->getInverseMass();
    }

    Matrix3 deltaVelocity = contactToWorld.transpose();
    deltaVelocity *= deltaVelWorld;
    deltaVelocity *= contactToWorld;

    deltaVelocity.data[0] += inverseMass;
    deltaVelocity.data[4] += inverseMass;
    deltaVelocity.data[8] += inverseMass;

    Matrix3 impulseMatrix = deltaVelocity.inverse();

    Vector3 velKill(desiredDeltaVelocity,
        -contactVelocity.y,
        -contactVelocity.z);

    impulseContact = impulseMatrix.transform(velKill);

    real planarImpulse = real_sqrt(
        impulseContact.y * impulseContact.y +
        impulseContact.z * impulseContact.z
    );
    if (planarImpulse > impulseContact.x * friction)
    {

        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocity.data[0] +
            deltaVelocity.data[1] * friction * impulseContact.y +
            deltaVelocity.data[2] * friction * impulseContact.z;
        impulseContact.x = desiredDeltaVelocity / impulseContact.x;
        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

void Contact::applyPositionChange(Vector3 linearChange[2],
    Vector3 angularChange[2],
    real penetration)
{
    const real angularLimit = (real)0.2f;
    real angularMove[2];
    real linearMove[2];

    real totalInertia = 0;
    real linearInertia[2];
    real angularInertia[2];

    for (unsigned i = 0; i < 2; i++) if (body[i])
    {
        Matrix3 inverseInertiaTensor;
        body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

        Vector3 angularInertiaWorld =
            relativeContactPosition[i] % contactNormal;
        angularInertiaWorld =
            inverseInertiaTensor.transform(angularInertiaWorld);
        angularInertiaWorld =
            angularInertiaWorld % relativeContactPosition[i];
        angularInertia[i] =
            angularInertiaWorld * contactNormal;

        linearInertia[i] = body[i]->getInverseMass();

        totalInertia += linearInertia[i] + angularInertia[i];

    }

    for (unsigned i = 0; i < 2; i++) if (body[i])
    {

        real sign = (i == 0) ? 1 : -1;
        angularMove[i] =
            sign * penetration * (angularInertia[i] / totalInertia);
        linearMove[i] =
            sign * penetration * (linearInertia[i] / totalInertia);

        Vector3 projection = relativeContactPosition[i];
        projection.addScaledVector(
            contactNormal,
            -relativeContactPosition[i].scalarProduct(contactNormal)
        );

        real maxMagnitude = angularLimit * projection.magnitude();

        if (angularMove[i] < -maxMagnitude)
        {
            real totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = -maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }
        else if (angularMove[i] > maxMagnitude)
        {
            real totalMove = angularMove[i] + linearMove[i];
            angularMove[i] = maxMagnitude;
            linearMove[i] = totalMove - angularMove[i];
        }

        if (angularMove[i] == 0)
        {

            angularChange[i].clear();
        }
        else
        {

            Vector3 targetAngularDirection =
                relativeContactPosition[i].vectorProduct(contactNormal);

            Matrix3 inverseInertiaTensor;
            body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

            angularChange[i] =
                inverseInertiaTensor.transform(targetAngularDirection) *
                (angularMove[i] / angularInertia[i]);
        }

        linearChange[i] = contactNormal * linearMove[i];

        Vector3 pos;
        body[i]->getPosition(&pos);
        pos.addScaledVector(contactNormal, linearMove[i]);
        body[i]->setPosition(pos);

        Quaternion q;
        body[i]->getOrientation(&q);
        q.addScaledVector(angularChange[i], ((real)1.0));
        body[i]->setOrientation(q);

        if (!body[i]->getAwake()) body[i]->calculateDerivedData();
    }
}

ContactResolver::ContactResolver(unsigned iterations,
    real velocityEpsilon,
    real positionEpsilon)
{
    setIterations(iterations, iterations);
    setEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
    unsigned positionIterations,
    real velocityEpsilon,
    real positionEpsilon)
{
    setIterations(velocityIterations);
    setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned iterations)
{
    setIterations(iterations, iterations);
}

void ContactResolver::setIterations(unsigned velocityIterations,
    unsigned positionIterations)
{
    ContactResolver::velocityIterations = velocityIterations;
    ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(real velocityEpsilon,
    real positionEpsilon)
{
    ContactResolver::velocityEpsilon = velocityEpsilon;
    ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(Contact* contacts,
    unsigned numContacts,
    real duration)
{

    if (numContacts == 0) return;
    if (!isValid()) return;

    prepareContacts(contacts, numContacts, duration);

    adjustPositions(contacts, numContacts, duration);

    adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts,
    unsigned numContacts,
    real duration)
{

    Contact* lastContact = contacts + numContacts;
    for (Contact* contact = contacts; contact < lastContact; contact++)
    {

        contact->calculateInternals(duration);
    }
}

void ContactResolver::adjustVelocities(Contact* c,
    unsigned numContacts,
    real duration)
{
    Vector3 velocityChange[2], rotationChange[2];
    Vector3 deltaVel;

    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations)
    {

        real max = velocityEpsilon;
        unsigned index = numContacts;
        for (unsigned i = 0; i < numContacts; i++)
        {
            if (c[i].desiredDeltaVelocity > max)
            {
                max = c[i].desiredDeltaVelocity;
                index = i;
            }
        }
        if (index == numContacts) break;

        c[index].matchAwakeState();

        c[index].applyVelocityChange(velocityChange, rotationChange);

        for (unsigned i = 0; i < numContacts; i++)
        {

            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {

                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
                        deltaVel = velocityChange[d] +
                            rotationChange[d].vectorProduct(
                                c[i].relativeContactPosition[b]);

                        c[i].contactVelocity +=
                            c[i].contactToWorld.transformTranspose(deltaVel)
                            * (b ? -1 : 1);
                        c[i].calculateDesiredDeltaVelocity(duration);
                    }
                }
            }
        }
        velocityIterationsUsed++;
    }
}

void ContactResolver::adjustPositions(Contact* c,
    unsigned numContacts,
    real duration)
{
    unsigned i, index;
    Vector3 linearChange[2], angularChange[2];
    real max;
    Vector3 deltaPosition;

    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations)
    {

        max = positionEpsilon;
        index = numContacts;
        for (i = 0; i < numContacts; i++)
        {
            if (c[i].penetration > max)
            {
                max = c[i].penetration;
                index = i;
            }
        }
        if (index == numContacts) break;

        c[index].matchAwakeState();

        c[index].applyPositionChange(
            linearChange,
            angularChange,
            max);

        for (i = 0; i < numContacts; i++)
        {

            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
            {

                for (unsigned d = 0; d < 2; d++)
                {
                    if (c[i].body[b] == c[index].body[d])
                    {
                        deltaPosition = linearChange[d] +
                            angularChange[d].vectorProduct(
                                c[i].relativeContactPosition[b]);

                        c[i].penetration +=
                            deltaPosition.scalarProduct(c[i].contactNormal)
                            * (b ? 1 : -1);
                    }
                }
            }
        }
        positionIterationsUsed++;
    }
}


void CollisionPrimitive::calculateInternals()
{
    transform = body->getTransform() * offset;
}

bool IntersectionTests::sphereAndHalfSpace(
    const CollisionSphere& sphere,
    const CollisionPlane& plane)
{

    real ballDistance =
        plane.direction *
        sphere.getAxis(3) -
        sphere.radius;

    return ballDistance <= plane.offset;
}

bool IntersectionTests::sphereAndSphere(
    const CollisionSphere& one,
    const CollisionSphere& two)
{

    Vector3 midline = one.getAxis(3) - two.getAxis(3);

    return midline.squareMagnitude() <
        (one.radius + two.radius) * (one.radius + two.radius);
}

static inline real transformToAxis(
    const CollisionBox& box,
    const Vector3& axis
)
{
    return
        box.halfSize.x * real_abs(axis * box.getAxis(0)) +
        box.halfSize.y * real_abs(axis * box.getAxis(1)) +
        box.halfSize.z * real_abs(axis * box.getAxis(2));
}

static inline bool overlapOnAxis(
    const CollisionBox& one,
    const CollisionBox& two,
    const Vector3& axis,
    const Vector3& toCentre
)
{

    real oneProject = transformToAxis(one, axis);
    real twoProject = transformToAxis(two, axis);

    real distance = real_abs(toCentre * axis);

    return (distance < oneProject + twoProject);
}

#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(
    const CollisionBox& one,
    const CollisionBox& two
)
{

    Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

    return (

        TEST_OVERLAP(one.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2)) &&

        TEST_OVERLAP(two.getAxis(0)) &&
        TEST_OVERLAP(two.getAxis(1)) &&
        TEST_OVERLAP(two.getAxis(2)) &&

        TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
        );
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(
    const CollisionBox& box,
    const CollisionPlane& plane
)
{

    real projectedRadius = transformToAxis(box, plane.direction);

    real boxDistance =
        plane.direction *
        box.getAxis(3) -
        projectedRadius;

    return boxDistance <= plane.offset;
}

unsigned CollisionDetector::sphereAndTruePlane(
    const CollisionSphere& sphere,
    const CollisionPlane& plane,
    CollisionData* data
)
{

    if (data->contactsLeft <= 0) return 0;

    Vector3 position = sphere.getAxis(3);

    real centreDistance = plane.direction * position - plane.offset;

    if (centreDistance * centreDistance > sphere.radius * sphere.radius)
    {
        return 0;
    }

    Vector3 normal = plane.direction;
    real penetration = -centreDistance;
    if (centreDistance < 0)
    {
        normal *= -1;
        penetration = -penetration;
    }
    penetration += sphere.radius;

    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->penetration = penetration;
    contact->contactPoint = position - plane.direction * centreDistance;
    contact->setBodyData(sphere.body, NULL,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(
    const CollisionSphere& sphere,
    const CollisionPlane& plane,
    CollisionData* data
)
{

    if (data->contactsLeft <= 0) return 0;

    Vector3 position = sphere.getAxis(3);

    real ballDistance =
        plane.direction * position -
        sphere.radius - plane.offset;

    if (ballDistance >= 0) return 0;

    Contact* contact = data->contacts;
    contact->contactNormal = plane.direction;
    contact->penetration = -ballDistance;
    contact->contactPoint =
        position - plane.direction * (ballDistance + sphere.radius);
    contact->setBodyData(sphere.body, NULL,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndSphere(
    const CollisionSphere& one,
    const CollisionSphere& two,
    CollisionData* data
)
{

    if (data->contactsLeft <= 0) return 0;

    Vector3 positionOne = one.getAxis(3);
    Vector3 positionTwo = two.getAxis(3);

    Vector3 midline = positionOne - positionTwo;
    real size = midline.magnitude();

    if (size <= 0.0f || size >= one.radius + two.radius)
    {
        return 0;
    }

    Vector3 normal = midline * (((real)1.0) / size);

    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + midline * (real)0.5;
    contact->penetration = (one.radius + two.radius - size);
    contact->setBodyData(one.body, two.body,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

static inline real penetrationOnAxis(
    const CollisionBox& one,
    const CollisionBox& two,
    const Vector3& axis,
    const Vector3& toCentre
)
{

    real oneProject = transformToAxis(one, axis);
    real twoProject = transformToAxis(two, axis);

    real distance = real_abs(toCentre * axis);

    return oneProject + twoProject - distance;
}

static inline bool tryAxis(
    const CollisionBox& one,
    const CollisionBox& two,
    Vector3 axis,
    const Vector3& toCentre,
    unsigned index,

    real& smallestPenetration,
    unsigned& smallestCase
)
{

    if (axis.squareMagnitude() < 0.0001) return true;
    axis.normalise();

    real penetration = penetrationOnAxis(one, two, axis, toCentre);

    if (penetration < 0) return false;
    if (penetration < smallestPenetration) {
        smallestPenetration = penetration;
        smallestCase = index;
    }
    return true;
}

void fillPointFaceBoxBox(
    const CollisionBox& one,
    const CollisionBox& two,
    const Vector3& toCentre,
    CollisionData* data,
    unsigned best,
    real pen
)
{

    Contact* contact = data->contacts;

    Vector3 normal = one.getAxis(best);
    if (one.getAxis(best) * toCentre > 0)
    {
        normal = normal * -1.0f;
    }

    Vector3 vertex = two.halfSize;
    if (two.getAxis(0) * normal < 0) vertex.x = -vertex.x;
    if (two.getAxis(1) * normal < 0) vertex.y = -vertex.y;
    if (two.getAxis(2) * normal < 0) vertex.z = -vertex.z;

    contact->contactNormal = normal;
    contact->penetration = pen;
    contact->contactPoint = (two.getTransform() * vertex);
    contact->setBodyData(one.body, two.body,
        data->friction, data->restitution);
}

static inline Vector3 contactPoint(
    const Vector3& pOne,
    const Vector3& dOne,
    real oneSize,
    const Vector3& pTwo,
    const Vector3& dTwo,
    real twoSize,

    bool useOne)
{
    Vector3 toSt, cOne, cTwo;
    real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
    real denom, mua, mub;

    smOne = dOne.squareMagnitude();
    smTwo = dTwo.squareMagnitude();
    dpOneTwo = dTwo * dOne;

    toSt = pOne - pTwo;
    dpStaOne = dOne * toSt;
    dpStaTwo = dTwo * toSt;

    denom = smOne * smTwo - dpOneTwo * dpOneTwo;

    if (real_abs(denom) < 0.0001f) {
        return useOne ? pOne : pTwo;
    }

    mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
    mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

    if (mua > oneSize ||
        mua < -oneSize ||
        mub > twoSize ||
        mub < -twoSize)
    {
        return useOne ? pOne : pTwo;
    }
    else
    {
        cOne = pOne + dOne * mua;
        cTwo = pTwo + dTwo * mub;

        return cOne * 0.5 + cTwo * 0.5;
    }
}

#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(
    const CollisionBox& one,
    const CollisionBox& two,
    CollisionData* data
)
{

    Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

    real pen = REAL_MAX;
    unsigned best = 0xffffff;

    CHECK_OVERLAP(one.getAxis(0), 0);
    CHECK_OVERLAP(one.getAxis(1), 1);
    CHECK_OVERLAP(one.getAxis(2), 2);

    CHECK_OVERLAP(two.getAxis(0), 3);
    CHECK_OVERLAP(two.getAxis(1), 4);
    CHECK_OVERLAP(two.getAxis(2), 5);

    unsigned bestSingleAxis = best;

    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(0), 6);
    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(1), 7);
    CHECK_OVERLAP(one.getAxis(0) % two.getAxis(2), 8);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(0), 9);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(1), 10);
    CHECK_OVERLAP(one.getAxis(1) % two.getAxis(2), 11);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(0), 12);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(1), 13);
    CHECK_OVERLAP(one.getAxis(2) % two.getAxis(2), 14);

    assert(best != 0xffffff);

    if (best < 3)
    {

        fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
        data->addContacts(1);
        return 1;
    }
    else if (best < 6)
    {

        fillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
        data->addContacts(1);
        return 1;
    }
    else
    {

        best -= 6;
        unsigned oneAxisIndex = best / 3;
        unsigned twoAxisIndex = best % 3;
        Vector3 oneAxis = one.getAxis(oneAxisIndex);
        Vector3 twoAxis = two.getAxis(twoAxisIndex);
        Vector3 axis = oneAxis % twoAxis;
        axis.normalise();

        if (axis * toCentre > 0) axis = axis * -1.0f;

        Vector3 ptOnOneEdge = one.halfSize;
        Vector3 ptOnTwoEdge = two.halfSize;
        for (unsigned i = 0; i < 3; i++)
        {
            if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
            else if (one.getAxis(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

            if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
            else if (two.getAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
        }

        ptOnOneEdge = one.transform * ptOnOneEdge;
        ptOnTwoEdge = two.transform * ptOnTwoEdge;

        Vector3 vertex = contactPoint(
            ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
            ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
            bestSingleAxis > 2
        );

        Contact* contact = data->contacts;

        contact->penetration = pen;
        contact->contactNormal = axis;
        contact->contactPoint = vertex;
        contact->setBodyData(one.body, two.body,
            data->friction, data->restitution);
        data->addContacts(1);
        return 1;
    }
    return 0;
}
#undef CHECK_OVERLAP

unsigned CollisionDetector::boxAndPoint(
    const CollisionBox& box,
    const Vector3& point,
    CollisionData* data
)
{

    Vector3 relPt = box.transform.transformInverse(point);

    Vector3 normal;

    real min_depth = box.halfSize.x - real_abs(relPt.x);
    if (min_depth < 0) return 0;
    normal = box.getAxis(0) * ((relPt.x < 0) ? -1 : 1);

    real depth = box.halfSize.y - real_abs(relPt.y);
    if (depth < 0) return 0;
    else if (depth < min_depth)
    {
        min_depth = depth;
        normal = box.getAxis(1) * ((relPt.y < 0) ? -1 : 1);
    }

    depth = box.halfSize.z - real_abs(relPt.z);
    if (depth < 0) return 0;
    else if (depth < min_depth)
    {
        min_depth = depth;
        normal = box.getAxis(2) * ((relPt.z < 0) ? -1 : 1);
    }

    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = point;
    contact->penetration = min_depth;

    contact->setBodyData(box.body, NULL,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndSphere(
    const CollisionBox& box,
    const CollisionSphere& sphere,
    CollisionData* data
)
{

    Vector3 centre = sphere.getAxis(3);
    Vector3 relCentre = box.transform.transformInverse(centre);

    if (real_abs(relCentre.x) - sphere.radius > box.halfSize.x ||
        real_abs(relCentre.y) - sphere.radius > box.halfSize.y ||
        real_abs(relCentre.z) - sphere.radius > box.halfSize.z)
    {
        return 0;
    }

    Vector3 closestPt(0, 0, 0);
    real dist;

    dist = relCentre.x;
    if (dist > box.halfSize.x) dist = box.halfSize.x;
    if (dist < -box.halfSize.x) dist = -box.halfSize.x;
    closestPt.x = dist;

    dist = relCentre.y;
    if (dist > box.halfSize.y) dist = box.halfSize.y;
    if (dist < -box.halfSize.y) dist = -box.halfSize.y;
    closestPt.y = dist;

    dist = relCentre.z;
    if (dist > box.halfSize.z) dist = box.halfSize.z;
    if (dist < -box.halfSize.z) dist = -box.halfSize.z;
    closestPt.z = dist;

    dist = (closestPt - relCentre).squareMagnitude();
    if (dist > sphere.radius * sphere.radius) return 0;

    Vector3 closestPtWorld = box.transform.transform(closestPt);

    Contact* contact = data->contacts;
    contact->contactNormal = (closestPtWorld - centre);
    contact->contactNormal.normalise();
    contact->contactPoint = closestPtWorld;
    contact->penetration = sphere.radius - real_sqrt(dist);
    contact->setBodyData(box.body, sphere.body,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(
    const CollisionBox& box,
    const CollisionPlane& plane,
    CollisionData* data
)
{

    if (data->contactsLeft <= 0) return 0;

    if (!IntersectionTests::boxAndHalfSpace(box, plane))
    {
        return 0;
    }

    static real mults[8][3] = { {1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                               {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1} };

    Contact* contact = data->contacts;
    unsigned contactsUsed = 0;
    for (unsigned i = 0; i < 8; i++) {

        Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
        vertexPos.componentProductUpdate(box.halfSize);
        vertexPos = box.transform.transform(vertexPos);

        real vertexDistance = vertexPos * plane.direction;

        if (vertexDistance <= plane.offset)
        {

            contact->contactPoint = plane.direction;
            contact->contactPoint *= (vertexDistance - plane.offset);
            contact->contactPoint += vertexPos;
            contact->contactNormal = plane.direction;
            contact->penetration = plane.offset - vertexDistance;

            contact->setBodyData(box.body, NULL,
                data->friction, data->restitution);

            contact++;
            contactsUsed++;
            if (contactsUsed == (unsigned)data->contactsLeft) return contactsUsed;
        }
    }

    data->addContacts(contactsUsed);
    return contactsUsed;
}

const Vector3 Vector3::GRAVITY = Vector3(0, -9.81, 0);
const Vector3 Vector3::HIGH_GRAVITY = Vector3(0, -19.62, 0);
const Vector3 Vector3::UP = Vector3(0, 1, 0);
const Vector3 Vector3::RIGHT = Vector3(1, 0, 0);
const Vector3 Vector3::OUT_OF_SCREEN = Vector3(0, 0, 1);
const Vector3 Vector3::X = Vector3(0, 1, 0);
const Vector3 Vector3::Y = Vector3(1, 0, 0);
const Vector3 Vector3::Z = Vector3(0, 0, 1);

/*
 * Definition of the sleep epsilon extern.
 */
real sleepEpsilon = ((real)0.3);

/*
 * Functions to change sleepEpsilon.
 */
void setSleepEpsilon(real value)
{
    sleepEpsilon = value;
}

real getSleepEpsilon()
{
    return sleepEpsilon;
}

real Matrix4::getDeterminant() const
{
    return -data[8] * data[5] * data[2] +
        data[4] * data[9] * data[2] +
        data[8] * data[1] * data[6] -
        data[0] * data[9] * data[6] -
        data[4] * data[1] * data[10] +
        data[0] * data[5] * data[10];
}

void Matrix4::setInverse(const Matrix4& m)
{
    // Make sure the determinant is non-zero.
    real det = getDeterminant();
    if (det == 0) return;
    det = ((real)1.0) / det;

    data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
    data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
    data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;

    data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
    data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
    data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;

    data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
    data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
    data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;

    data[3] = (m.data[9] * m.data[6] * m.data[3]
        - m.data[5] * m.data[10] * m.data[3]
        - m.data[9] * m.data[2] * m.data[7]
        + m.data[1] * m.data[10] * m.data[7]
        + m.data[5] * m.data[2] * m.data[11]
        - m.data[1] * m.data[6] * m.data[11]) * det;
    data[7] = (-m.data[8] * m.data[6] * m.data[3]
        + m.data[4] * m.data[10] * m.data[3]
        + m.data[8] * m.data[2] * m.data[7]
        - m.data[0] * m.data[10] * m.data[7]
        - m.data[4] * m.data[2] * m.data[11]
        + m.data[0] * m.data[6] * m.data[11]) * det;
    data[11] = (m.data[8] * m.data[5] * m.data[3]
        - m.data[4] * m.data[9] * m.data[3]
        - m.data[8] * m.data[1] * m.data[7]
        + m.data[0] * m.data[9] * m.data[7]
        + m.data[4] * m.data[1] * m.data[11]
        - m.data[0] * m.data[5] * m.data[11]) * det;
}

Matrix3 Matrix3::linearInterpolate(const Matrix3& a, const Matrix3& b, real prop)
{
    Matrix3 result;
    for (unsigned i = 0; i < 9; i++) {
        result.data[i] = a.data[i] * (1 - prop) + b.data[i] * prop;
    }
    return result;
}
