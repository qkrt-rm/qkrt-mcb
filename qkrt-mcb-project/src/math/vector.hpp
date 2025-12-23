#pragma once

#include <cstddef>

namespace qkrt
{

template <size_t L, typename T>
class vec;

template <typename T>
class vec<2, T>
{
private:
    typedef vec<2, T>       class_type;
    typedef T               value_type;
public:
    vec(const value_type& _v = 0)
    {
        x = _v;
        y = _v;
    }

    vec(const value_type& _x, const value_type& _y)
    {
        x = _x;
        y = _y;
    }

    vec(const class_type& o)
    {
        this->x = o.x;
        this->y = o.y;
    }

    class_type& operator=(const class_type& o)
    {
        this->x = o.x;
        this->y = o.y;
        return *this;
    }

    vec(class_type&& o)
    {
        this->x = o.x;
        this->y = o.y;
        o.x = 0;
        o.y = 0;
    }

    class_type& operator=(class_type&& o)
    {
        this->x = o.x;
        this->y = o.y;
        o.x = 0;
        o.y = 0;
        return *this;
    }

public:
    class_type operator+(const class_type&  o) const { return { this->x + o.x, this->y + o.y }; }
    class_type operator-(const class_type&  o) const { return { this->x - o.x, this->y - o.y }; }
    class_type operator*(const value_type& _s) const { return { this->x * _s,  this->y * _s }; }
    class_type operator/(const value_type& _s) const { return { this->x / _s,  this->y / _s }; }

    /**
     * @return dot product
     */
    value_type operator*(const class_type& o) const { return this->x * o.x + this->y * o.y; }

public:
    union
    {
        struct { value_type x, y; };
        struct { value_type u, v; };
    };
};

template<typename T>
class vec<3, T>
{
private:
    typedef vec<3, T>       class_type;
    typedef T               value_type;
public:
    vec(const T& _v = 0)
    {
        x = _v;
        y = _v;
        z = _v;
    }

    vec(const value_type& _x, const value_type& _y, const value_type& _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    vec(const class_type& o)
    {
        this->x = o.x;
        this->y = o.y;
        this->z = o.z;
    }

    class_type& operator=(const class_type& o)
    {
        this->x = o.x;
        this->y = o.y;
        this->z = o.z;
        return *this;
    }

    vec(class_type&& o)
    {
        this->x = o.x;
        this->y = o.y;
        this->z = o.z;
        o.x = 0;
        o.y = 0;
        o.z = 0;
    }

    class_type& operator=(class_type&& o)
    {
        this->x = o.x;
        this->y = o.y;
        this->z = o.z;
        o.x = 0;
        o.y = 0;
        o.z = 0;
        return *this;
    }

public:
    class_type operator+(const class_type&  o) const { return { this->x + o.x, this->y + o.y, this->z + o.z }; }
    class_type operator-(const class_type&  o) const { return { this->x - o.x, this->y - o.y, this->z - o.z }; }
    class_type operator*(const value_type& _s) const { return { this->x * _s,  this->y * _s,  this->z * _s }; }
    class_type operator/(const value_type& _s) const { return { this->x / _s,  this->y / _s,  this->z / _s }; }

    value_type operator*(const class_type& o) const { return this->x * o.x + this->y * o.y + this->z * o.z; }

public:
    union
    {
        struct { value_type x, y, z; };
        struct { value_type r, g, b; };
        struct { value_type u, v, w; };
    };
};

}  // namespace qkrt