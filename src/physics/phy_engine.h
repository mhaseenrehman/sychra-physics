
#include <float.h>
#include <math.h>


#define DOUBLE_PRECISION
typedef double real;
#define REAL_MAX DBL_MAX
#define real_sqrt sqrt
#define real_abs fabs
#define real_sin sin
#define real_cos cos
#define real_exp exp
#define real_pow pow
#define real_fmod fmod
#define real_epsilon DBL_EPSILON
#define R_PI 3.14159265358979


//namespace Physics {

extern real sleepEpsilon;

void setSleepEpsilon(real value);

real getSleepEpsilon();

class Vector3
{
public:

    real x;
    real y;
    real z;

private:
    real pad;

public:

    Vector3() : x(0), y(0), z(0) {}

    Vector3(const real x, const real y, const real z)
        : x(x), y(y), z(z) {
    }

    const static Vector3 GRAVITY;
    const static Vector3 HIGH_GRAVITY;
    const static Vector3 UP;
    const static Vector3 RIGHT;
    const static Vector3 OUT_OF_SCREEN;
    const static Vector3 X;
    const static Vector3 Y;
    const static Vector3 Z;

    real operator[](unsigned i) const
    {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    real& operator[](unsigned i)
    {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    Vector3 operator+(const Vector3& v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    void operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    Vector3 operator-(const Vector3& v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    void operator*=(const real value)
    {
        x *= value;
        y *= value;
        z *= value;
    }

    Vector3 operator*(const real value) const
    {
        return Vector3(x * value, y * value, z * value);
    }

    Vector3 componentProduct(const Vector3& vector) const
    {
        return Vector3(x * vector.x, y * vector.y, z * vector.z);
    }

    void componentProductUpdate(const Vector3& vector)
    {
        x *= vector.x;
        y *= vector.y;
        z *= vector.z;
    }

    Vector3 vectorProduct(const Vector3& vector) const
    {
        return Vector3(y * vector.z - z * vector.y,
            z * vector.x - x * vector.z,
            x * vector.y - y * vector.x);
    }

    void operator %=(const Vector3& vector)
    {
        *this = vectorProduct(vector);
    }

    Vector3 operator%(const Vector3& vector) const
    {
        return Vector3(y * vector.z - z * vector.y,
            z * vector.x - x * vector.z,
            x * vector.y - y * vector.x);
    }

    real scalarProduct(const Vector3& vector) const
    {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    real operator *(const Vector3& vector) const
    {
        return x * vector.x + y * vector.y + z * vector.z;
    }

    void addScaledVector(const Vector3& vector, real scale)
    {
        x += vector.x * scale;
        y += vector.y * scale;
        z += vector.z * scale;
    }

    real magnitude() const
    {
        return real_sqrt(x * x + y * y + z * z);
    }

    real squareMagnitude() const
    {
        return x * x + y * y + z * z;
    }

    void trim(real size)
    {
        if (squareMagnitude() > size * size)
        {
            normalise();
            x *= size;
            y *= size;
            z *= size;
        }
    }

    void normalise()
    {
        real l = magnitude();
        if (l > 0)
        {
            (*this) *= ((real)1) / l;
        }
    }

    Vector3 unit() const
    {
        Vector3 result = *this;
        result.normalise();
        return result;
    }

    bool operator==(const Vector3& other) const
    {
        return x == other.x &&
            y == other.y &&
            z == other.z;
    }

    bool operator!=(const Vector3& other) const
    {
        return !(*this == other);
    }

    bool operator<(const Vector3& other) const
    {
        return x < other.x && y < other.y && z < other.z;
    }

    bool operator>(const Vector3& other) const
    {
        return x > other.x && y > other.y && z > other.z;
    }

    bool operator<=(const Vector3& other) const
    {
        return x <= other.x && y <= other.y && z <= other.z;
    }

    bool operator>=(const Vector3& other) const
    {
        return x >= other.x && y >= other.y && z >= other.z;
    }

    void clear()
    {
        x = y = z = 0;
    }

    void invert()
    {
        x = -x;
        y = -y;
        z = -z;
    }

};

class Quaternion
{
public:
    union {
        struct {

            real r;

            real i;

            real j;

            real k;
        };

        real data[4];
    };

    Quaternion() : r(1), i(0), j(0), k(0) {}

    Quaternion(const real r, const real i, const real j, const real k)
        : r(r), i(i), j(j), k(k)
    {
    }

    void normalise()
    {
        real d = r * r + i * i + j * j + k * k;

        if (d < real_epsilon) {
            r = 1;
            return;
        }

        d = ((real)1.0) / real_sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    void operator *=(const Quaternion& multiplier)
    {
        Quaternion q = *this;
        r = q.r * multiplier.r - q.i * multiplier.i -
            q.j * multiplier.j - q.k * multiplier.k;
        i = q.r * multiplier.i + q.i * multiplier.r +
            q.j * multiplier.k - q.k * multiplier.j;
        j = q.r * multiplier.j + q.j * multiplier.r +
            q.k * multiplier.i - q.i * multiplier.k;
        k = q.r * multiplier.k + q.k * multiplier.r +
            q.i * multiplier.j - q.j * multiplier.i;
    }

    void addScaledVector(const Vector3& vector, real scale)
    {
        Quaternion q(0,
            vector.x * scale,
            vector.y * scale,
            vector.z * scale);
        q *= *this;
        r += q.r * ((real)0.5);
        i += q.i * ((real)0.5);
        j += q.j * ((real)0.5);
        k += q.k * ((real)0.5);
    }

    void rotateByVector(const Vector3& vector)
    {
        Quaternion q(0, vector.x, vector.y, vector.z);
        (*this) *= q;
    }
};

class Matrix4
{
public:

    real data[12];

    Matrix4()
    {
        data[1] = data[2] = data[3] = data[4] = data[6] =
            data[7] = data[8] = data[9] = data[11] = 0;
        data[0] = data[5] = data[10] = 1;
    }

    void setDiagonal(real a, real b, real c)
    {
        data[0] = a;
        data[5] = b;
        data[10] = c;
    }

    Matrix4 operator*(const Matrix4& o) const
    {
        Matrix4 result;
        result.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
        result.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
        result.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

        result.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
        result.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
        result.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

        result.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
        result.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
        result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

        result.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
        result.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
        result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

        return result;
    }

    Vector3 operator*(const Vector3& vector) const
    {
        return Vector3(
            vector.x * data[0] +
            vector.y * data[1] +
            vector.z * data[2] + data[3],

            vector.x * data[4] +
            vector.y * data[5] +
            vector.z * data[6] + data[7],

            vector.x * data[8] +
            vector.y * data[9] +
            vector.z * data[10] + data[11]
        );
    }

    Vector3 transform(const Vector3& vector) const
    {
        return (*this) * vector;
    }

    real getDeterminant() const;

    void setInverse(const Matrix4& m);

    Matrix4 inverse() const
    {
        Matrix4 result;
        result.setInverse(*this);
        return result;
    }

    void invert()
    {
        setInverse(*this);
    }

    Vector3 transformDirection(const Vector3& vector) const
    {
        return Vector3(
            vector.x * data[0] +
            vector.y * data[1] +
            vector.z * data[2],

            vector.x * data[4] +
            vector.y * data[5] +
            vector.z * data[6],

            vector.x * data[8] +
            vector.y * data[9] +
            vector.z * data[10]
        );
    }

    Vector3 transformInverseDirection(const Vector3& vector) const
    {
        return Vector3(
            vector.x * data[0] +
            vector.y * data[4] +
            vector.z * data[8],

            vector.x * data[1] +
            vector.y * data[5] +
            vector.z * data[9],

            vector.x * data[2] +
            vector.y * data[6] +
            vector.z * data[10]
        );
    }

    Vector3 transformInverse(const Vector3& vector) const
    {
        Vector3 tmp = vector;
        tmp.x -= data[3];
        tmp.y -= data[7];
        tmp.z -= data[11];
        return Vector3(
            tmp.x * data[0] +
            tmp.y * data[4] +
            tmp.z * data[8],

            tmp.x * data[1] +
            tmp.y * data[5] +
            tmp.z * data[9],

            tmp.x * data[2] +
            tmp.y * data[6] +
            tmp.z * data[10]
        );
    }

    Vector3 getAxisVector(int i) const
    {
        return Vector3(data[i], data[i + 4], data[i + 8]);
    }

    void setOrientationAndPos(const Quaternion& q, const Vector3& pos)
    {
        data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        data[3] = pos.x;

        data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
        data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
        data[7] = pos.y;

        data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
        data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
        data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        data[11] = pos.z;
    }

    void fillGLArray(float array[16]) const
    {
        array[0] = (float)data[0];
        array[1] = (float)data[4];
        array[2] = (float)data[8];
        array[3] = (float)0;

        array[4] = (float)data[1];
        array[5] = (float)data[5];
        array[6] = (float)data[9];
        array[7] = (float)0;

        array[8] = (float)data[2];
        array[9] = (float)data[6];
        array[10] = (float)data[10];
        array[11] = (float)0;

        array[12] = (float)data[3];
        array[13] = (float)data[7];
        array[14] = (float)data[11];
        array[15] = (float)1;
    }
};

class Matrix3
{
public:

    real data[9];

    Matrix3()
    {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
            data[6] = data[7] = data[8] = 0;
    }

    Matrix3(const Vector3& compOne, const Vector3& compTwo,
        const Vector3& compThree)
    {
        setComponents(compOne, compTwo, compThree);
    }

    Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
        real c6, real c7, real c8)
    {
        data[0] = c0; data[1] = c1; data[2] = c2;
        data[3] = c3; data[4] = c4; data[5] = c5;
        data[6] = c6; data[7] = c7; data[8] = c8;
    }

    void setDiagonal(real a, real b, real c)
    {
        setInertiaTensorCoeffs(a, b, c);
    }

    void setInertiaTensorCoeffs(real ix, real iy, real iz,
        real ixy = 0, real ixz = 0, real iyz = 0)
    {
        data[0] = ix;
        data[1] = data[3] = -ixy;
        data[2] = data[6] = -ixz;
        data[4] = iy;
        data[5] = data[7] = -iyz;
        data[8] = iz;
    }

    void setBlockInertiaTensor(const Vector3& halfSizes, real mass)
    {
        Vector3 squares = halfSizes.componentProduct(halfSizes);
        setInertiaTensorCoeffs(0.3f * mass * (squares.y + squares.z),
            0.3f * mass * (squares.x + squares.z),
            0.3f * mass * (squares.x + squares.y));
    }

    void setSkewSymmetric(const Vector3 vector)
    {
        data[0] = data[4] = data[8] = 0;
        data[1] = -vector.z;
        data[2] = vector.y;
        data[3] = vector.z;
        data[5] = -vector.x;
        data[6] = -vector.y;
        data[7] = vector.x;
    }

    void setComponents(const Vector3& compOne, const Vector3& compTwo,
        const Vector3& compThree)
    {
        data[0] = compOne.x;
        data[1] = compTwo.x;
        data[2] = compThree.x;
        data[3] = compOne.y;
        data[4] = compTwo.y;
        data[5] = compThree.y;
        data[6] = compOne.z;
        data[7] = compTwo.z;
        data[8] = compThree.z;

    }

    Vector3 operator*(const Vector3& vector) const
    {
        return Vector3(
            vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
            vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
            vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
        );
    }

    Vector3 transform(const Vector3& vector) const
    {
        return (*this) * vector;
    }

    Vector3 transformTranspose(const Vector3& vector) const
    {
        return Vector3(
            vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
            vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
            vector.x * data[2] + vector.y * data[5] + vector.z * data[8]
        );
    }

    Vector3 getRowVector(int i) const
    {
        return Vector3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
    }

    Vector3 getAxisVector(int i) const
    {
        return Vector3(data[i], data[i + 3], data[i + 6]);
    }

    void setInverse(const Matrix3& m)
    {
        real t4 = m.data[0] * m.data[4];
        real t6 = m.data[0] * m.data[5];
        real t8 = m.data[1] * m.data[3];
        real t10 = m.data[2] * m.data[3];
        real t12 = m.data[1] * m.data[6];
        real t14 = m.data[2] * m.data[6];

        real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
            t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

        if (t16 == (real)0.0f) return;
        real t17 = 1 / t16;

        data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        data[4] = (m.data[0] * m.data[8] - t14) * t17;
        data[5] = -(t6 - t10) * t17;
        data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        data[7] = -(m.data[0] * m.data[7] - t12) * t17;
        data[8] = (t4 - t8) * t17;
    }

    Matrix3 inverse() const
    {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }

    void invert()
    {
        setInverse(*this);
    }

    void setTranspose(const Matrix3& m)
    {
        data[0] = m.data[0];
        data[1] = m.data[3];
        data[2] = m.data[6];
        data[3] = m.data[1];
        data[4] = m.data[4];
        data[5] = m.data[7];
        data[6] = m.data[2];
        data[7] = m.data[5];
        data[8] = m.data[8];
    }

    Matrix3 transpose() const
    {
        Matrix3 result;
        result.setTranspose(*this);
        return result;
    }

    Matrix3 operator*(const Matrix3& o) const
    {
        return Matrix3(
            data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
            data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
            data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

            data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
            data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
            data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

            data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
            data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
            data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
        );
    }

    void operator*=(const Matrix3& o)
    {
        real t1;
        real t2;
        real t3;

        t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
        t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
        t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
        data[0] = t1;
        data[1] = t2;
        data[2] = t3;

        t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
        t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
        t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
        data[3] = t1;
        data[4] = t2;
        data[5] = t3;

        t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
        t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
        t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
        data[6] = t1;
        data[7] = t2;
        data[8] = t3;
    }

    void operator*=(const real scalar)
    {
        data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
        data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
        data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
    }

    void operator+=(const Matrix3& o)
    {
        data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
        data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
        data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
    }

    void setOrientation(const Quaternion& q)
    {
        data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
        data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
        data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
        data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
        data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
    }

    static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop);
};

class RigidBody
{
public:

protected:

    real inverseMass;

    Matrix3 inverseInertiaTensor;

    real linearDamping;

    real angularDamping;

    Vector3 position;

    Quaternion orientation;

    Vector3 velocity;

    Vector3 rotation;

    Matrix3 inverseInertiaTensorWorld;

    real motion;

    bool isAwake;

    bool canSleep;

    Matrix4 transformMatrix;

    Vector3 forceAccum;

    Vector3 torqueAccum;

    Vector3 acceleration;

    Vector3 lastFrameAcceleration;

public:

    void calculateDerivedData();

    void integrate(real duration);

    void setMass(const real mass);

    real getMass() const;

    void setInverseMass(const real inverseMass);

    real getInverseMass() const;

    bool hasFiniteMass() const;

    void setInertiaTensor(const Matrix3& inertiaTensor);

    void getInertiaTensor(Matrix3* inertiaTensor) const;

    Matrix3 getInertiaTensor() const;

    void getInertiaTensorWorld(Matrix3* inertiaTensor) const;

    Matrix3 getInertiaTensorWorld() const;

    void setInverseInertiaTensor(const Matrix3& inverseInertiaTensor);

    void getInverseInertiaTensor(Matrix3* inverseInertiaTensor) const;

    Matrix3 getInverseInertiaTensor() const;

    void getInverseInertiaTensorWorld(Matrix3* inverseInertiaTensor) const;

    Matrix3 getInverseInertiaTensorWorld() const;

    void setDamping(const real linearDamping, const real angularDamping);

    void setLinearDamping(const real linearDamping);

    real getLinearDamping() const;

    void setAngularDamping(const real angularDamping);

    real getAngularDamping() const;

    void setPosition(const Vector3& position);

    void setPosition(const real x, const real y, const real z);

    void getPosition(Vector3* position) const;

    Vector3 getPosition() const;

    void setOrientation(const Quaternion& orientation);

    void setOrientation(const real r, const real i,
        const real j, const real k);

    void getOrientation(Quaternion* orientation) const;

    Quaternion getOrientation() const;

    void getOrientation(Matrix3* matrix) const;

    void getOrientation(real matrix[9]) const;

    void getTransform(Matrix4* transform) const;

    void getTransform(real matrix[16]) const;

    void getGLTransform(float matrix[16]) const;

    Matrix4 getTransform() const;

    Vector3 getPointInLocalSpace(const Vector3& point) const;

    Vector3 getPointInWorldSpace(const Vector3& point) const;

    Vector3 getDirectionInLocalSpace(const Vector3& direction) const;

    Vector3 getDirectionInWorldSpace(const Vector3& direction) const;

    void setVelocity(const Vector3& velocity);

    void setVelocity(const real x, const real y, const real z);

    void getVelocity(Vector3* velocity) const;

    Vector3 getVelocity() const;

    void addVelocity(const Vector3& deltaVelocity);

    void setRotation(const Vector3& rotation);

    void setRotation(const real x, const real y, const real z);

    void getRotation(Vector3* rotation) const;

    Vector3 getRotation() const;

    void addRotation(const Vector3& deltaRotation);

    bool getAwake() const
    {
        return isAwake;
    }

    void setAwake(const bool awake = true);

    bool getCanSleep() const
    {
        return canSleep;
    }

    void setCanSleep(const bool canSleep = true);

    void getLastFrameAcceleration(Vector3* linearAcceleration) const;

    Vector3 getLastFrameAcceleration() const;

    void clearAccumulators();

    void addForce(const Vector3& force);

    void addForceAtPoint(const Vector3& force, const Vector3& point);

    void addForceAtBodyPoint(const Vector3& force, const Vector3& point);

    void addTorque(const Vector3& torque);

    void setAcceleration(const Vector3& acceleration);

    void setAcceleration(const real x, const real y, const real z);

    void getAcceleration(Vector3* acceleration) const;

    Vector3 getAcceleration() const;

};

class ContactResolver;

class Contact
{

    friend class ContactResolver;

public:

    RigidBody* body[2];

    real friction;

    real restitution;

    Vector3 contactPoint;

    Vector3 contactNormal;

    real penetration;

    void setBodyData(RigidBody* one, RigidBody* two,
        real friction, real restitution);

protected:

    Matrix3 contactToWorld;

    Vector3 contactVelocity;

    real desiredDeltaVelocity;

    Vector3 relativeContactPosition[2];

protected:

    void calculateInternals(real duration);

    void swapBodies();

    void matchAwakeState();

    void calculateDesiredDeltaVelocity(real duration);

    Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);

    void calculateContactBasis();

    void applyImpulse(const Vector3& impulse, RigidBody* body,
        Vector3* velocityChange, Vector3* rotationChange);

    void applyVelocityChange(Vector3 velocityChange[2],
        Vector3 rotationChange[2]);

    void applyPositionChange(Vector3 linearChange[2],
        Vector3 angularChange[2],
        real penetration);

    Vector3 calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor);

    Vector3 calculateFrictionImpulse(Matrix3* inverseInertiaTensor);
};

class ContactResolver
{
protected:

    unsigned velocityIterations;

    unsigned positionIterations;

    real velocityEpsilon;

    real positionEpsilon;

public:

    unsigned velocityIterationsUsed;

    unsigned positionIterationsUsed;

private:

    bool validSettings;

public:

    ContactResolver(unsigned iterations,
        real velocityEpsilon = (real)0.01,
        real positionEpsilon = (real)0.01);

    ContactResolver(unsigned velocityIterations,
        unsigned positionIterations,
        real velocityEpsilon = (real)0.01,
        real positionEpsilon = (real)0.01);

    bool isValid()
    {
        return (velocityIterations > 0) &&
            (positionIterations > 0) &&
            (positionEpsilon >= 0.0f) &&
            (positionEpsilon >= 0.0f);
    }

    void setIterations(unsigned velocityIterations,
        unsigned positionIterations);

    void setIterations(unsigned iterations);

    void setEpsilon(real velocityEpsilon,
        real positionEpsilon);

    void resolveContacts(Contact* contactArray,
        unsigned numContacts,
        real duration);

protected:

    void prepareContacts(Contact* contactArray, unsigned numContacts,
        real duration);

    void adjustVelocities(Contact* contactArray,
        unsigned numContacts,
        real duration);

    void adjustPositions(Contact* contacts,
        unsigned numContacts,
        real duration);
};

class ContactGenerator
{
public:

    virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
};

class IntersectionTests;
class CollisionDetector;

class CollisionPrimitive
{
public:

    friend class IntersectionTests;
    friend class CollisionDetector;

    RigidBody* body;

    Matrix4 offset;

    void calculateInternals();

    Vector3 getAxis(unsigned index) const
    {
        return transform.getAxisVector(index);
    }

    const Matrix4& getTransform() const
    {
        return transform;
    }

protected:

    Matrix4 transform;
};

class CollisionSphere : public CollisionPrimitive
{
public:

    real radius;
};

class CollisionPlane
{
public:

    Vector3 direction;

    real offset;
};

class CollisionBox : public CollisionPrimitive
{
public:

    Vector3 halfSize;
};

class IntersectionTests
{
public:

    static bool sphereAndHalfSpace(
        const CollisionSphere& sphere,
        const CollisionPlane& plane);

    static bool sphereAndSphere(
        const CollisionSphere& one,
        const CollisionSphere& two);

    static bool boxAndBox(
        const CollisionBox& one,
        const CollisionBox& two);

    static bool boxAndHalfSpace(
        const CollisionBox& box,
        const CollisionPlane& plane);
};

struct CollisionData
{

    Contact* contactArray;

    Contact* contacts;

    int contactsLeft;

    unsigned contactCount;

    real friction;

    real restitution;

    real tolerance;

    bool hasMoreContacts()
    {
        return contactsLeft > 0;
    }

    void reset(unsigned maxContacts)
    {
        contactsLeft = maxContacts;
        contactCount = 0;
        contacts = contactArray;
    }

    void addContacts(unsigned count)
    {

        contactsLeft -= count;
        contactCount += count;

        contacts += count;
    }
};

class CollisionDetector
{
public:

    static unsigned sphereAndHalfSpace(
        const CollisionSphere& sphere,
        const CollisionPlane& plane,
        CollisionData* data
    );

    static unsigned sphereAndTruePlane(
        const CollisionSphere& sphere,
        const CollisionPlane& plane,
        CollisionData* data
    );

    static unsigned sphereAndSphere(
        const CollisionSphere& one,
        const CollisionSphere& two,
        CollisionData* data
    );

    static unsigned boxAndHalfSpace(
        const CollisionBox& box,
        const CollisionPlane& plane,
        CollisionData* data
    );

    static unsigned boxAndBox(
        const CollisionBox& one,
        const CollisionBox& two,
        CollisionData* data
    );

    static unsigned boxAndPoint(
        const CollisionBox& box,
        const Vector3& point,
        CollisionData* data
    );

    static unsigned boxAndSphere(
        const CollisionBox& box,
        const CollisionSphere& sphere,
        CollisionData* data
    );
};

struct Box : CollisionBox {
public:

    //MeshNode node;
    bool isOverlapping;

public:
        
    Box() : isOverlapping(false) {
        body = new RigidBody();
    }

    ~Box() {
        delete body;
    }

    void setState(const Vector3& position,
        const Quaternion& orientation,
        const Vector3& extents,
        const Vector3& velocity)
    {
        body->setPosition(position);
        body->setOrientation(orientation);
        body->setVelocity(velocity);
        body->setRotation(Vector3(0, 0, 0));
        halfSize = extents;

        real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
        body->setMass(mass);

        Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        body->setInertiaTensor(tensor);

        body->setLinearDamping(0.95f);
        body->setAngularDamping(0.8f);
        body->clearAccumulators();
        body->setAcceleration(0, -10.0f, 0);

        body->setAwake();

        body->calculateDerivedData();
    }
};
// };