/*
* vector3 License
* Copyright (C) 2019 YaweiZhang <yawei.zhang@foxmail.com>.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#pragma once

#include <math.h>
#include "comm_def.h"
#include <cmath>

const f32 RADICAL_2 = 1.414213562373095f;
const f32 NI_PI = 3.1415926535897932f;
const f32 NI_HALF_PI = 0.5f * NI_PI;
const f32 PER_ANGLE_PI = NI_PI/180.0f;//每度是多少PI
const f32 PER_PI_ANGLE = 180.0f/NI_PI;//每PI多少度
const u32 ONE_HUNDRED_CC  = 100;//coordinate conversion factor,100

//浮点数精度
const f32 FLOAT_POINT_PRECISION = 0.0002f;
const f32 FLOAT_PONIT_LOW_PRECISION = 0.1;//运算中用到的低精度
const f32 FLOAT_POINT_MOVE_PRECISION = 2000.0f;

inline bool FLOAT_IS_ZEOR(f32 val){return fabsf(val) < FLOAT_POINT_PRECISION;}
inline bool FLOAT_IS_EQUAL(f32 val1, f32 val2) { return fabsf(val1 - val2) < FLOAT_POINT_PRECISION; }
inline bool FLOAT_IS_EQUAL_LOW_PRECISION(f32 val1, f32 val2) { return fabsf(val1 - val2) <FLOAT_PONIT_LOW_PRECISION;}
inline bool FLOAT_IS_MOVE_EQUAL(f32 val1, f32 val2){return fabsf(val1 - val2) < FLOAT_POINT_MOVE_PRECISION;}
inline s32 FLOAT_TO_PROPER_INT(f32 val) { return fabsf(val) + 0.0001f; }
inline f32 ANGLE_TO_RADIAN(f32 val) { return val / 360.0f * NI_PI * 2.0f; }
inline f32 RADIAN_TO_ANGLE(f32 val) { return val / NI_PI / 2.0f * 360.0f; }
inline f32 INVERSE_SQRT(f32 val);//卡马克快速平方根倒数算法
template<class T>
inline T PRUNING(T v, T min, T max) { return v > max ? max : (v < min ? min : v); }

/* --- -----  空间向量 --------------*/
struct Vector3
{
public:
	f32 x, y, z;
    Vector3(f32 fx, f32 fy, f32 fz):x(fx),y(fy),z(fz){}
    Vector3(const Vector3 & ) = default;
    Vector3() :Vector3(0.0f, 0.0f, 0.0f) {};
    Vector3 & operator =(const Vector3 & v3) = default;

	void Reset() { x = 0.0f; y = 0.0f; z = 0.0f; }
    void Set(f32 fx, f32 fy, f32 fz) { x = fx; y = fy; z = fz; }
    f32 Dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; };
    Vector3 Cross(const Vector3& v) const { return { y * v.z - z * v.y , z * v.x - x * v.z , x * v.y - y * v.x }; };
    f32 SquareLength()const { return Dot(*this); }
    f32 Length()const { return sqrtf(SquareLength()); }
	bool IsZero() const { return FLOAT_IS_ZEOR(x) && FLOAT_IS_ZEOR(y) && FLOAT_IS_ZEOR(z); }
	bool ISLowPrecisionEqual(const Vector3& other ) { return FLOAT_IS_EQUAL_LOW_PRECISION(x,other.x)&&FLOAT_IS_EQUAL_LOW_PRECISION(y,other.y)&&FLOAT_IS_EQUAL_LOW_PRECISION(z,other.z);}
	bool IsValid() const{return !std::isnan(x) && !std::isnan(y) && !std::isnan(z) && !std::isinf(x) && !std::isinf(y) && !std::isinf(z);}

	//角度是x，y平面相对x轴角度（0到360度）
	//x轴正向为0度，y轴正向为90度
	bool ConvertFromAngle(f32 angle)
	{
		if(angle <0 || angle >360)
		{
			return false;
		}

		float radian = angle * PER_ANGLE_PI;
		z = 0.0f;
		y = sin(radian);
		x = cos(radian);
		return true;
	}

	//得到向量的角度,需要转换为0到360
	f32 GetAngle() const
	{
		if(y >0.0f)
		{
			if( FLOAT_IS_ZEOR(x) )
			{
				return 0.0f;
			}

			float angle = atan(y/x) * PER_PI_ANGLE;
			if(angle >= 0.0f)
			{
				return angle;
			}

			return angle + 180;
		}
		else if( y < 0.0f)
		{
			if( FLOAT_IS_ZEOR(x) )
			{
				return 270.0f;
			}

			float angle = atan(y/x) * PER_PI_ANGLE;
			if(angle >= 0.0f)
			{
				return angle+180;
			}

			return angle + 360;
		}

		if(FLOAT_IS_ZEOR(y))
		{
			if(x > 0.0f)
			{
				return 0.0f;
			}
			else if(x < 0.0f)
			{
				return 180.0f;
			}
		}

		return 0.0f;
	}

	//server coordinate  convert to Navmesh coordinate
	void ServerConvertToNAV()
	{
		x = -x*ONE_HUNDRED_CC;
		f32 tem_y = y;
		y = z*ONE_HUNDRED_CC;
		z = -tem_y*ONE_HUNDRED_CC;
	}

	//Navmesh coordinate convert to server coordinate
	void NAVConvertToServer()
	{
		x = -x/ONE_HUNDRED_CC;
		f32 tem_y = y;
		y = -z/ONE_HUNDRED_CC;
		z = tem_y/ONE_HUNDRED_CC;
	}

	//单位化
	bool Normalize()
	{
        f32 square = SquareLength();
        if (FLOAT_IS_ZEOR(square))
        {
            return false;
        }
        * this *= INVERSE_SQRT(square);
        return true;
	}

    template<typename T>
    Vector3& operator=(const T& rhs)
    {
        if (this != (const Vector3*)&rhs)
        {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        return *this;
    }

    template<typename T>
    void AssignTo(T& t) const
    {
        if (this != (const Vector3*)&t)
        {
            t.x = x;
            t.y = y;
            t.z = z;
        }
    }

    inline bool operator == (const Vector3 &v) const { return FLOAT_IS_EQUAL(x, v.x) && FLOAT_IS_EQUAL(y, v.y) && FLOAT_IS_EQUAL(z, v.z); }
    inline bool operator != (const Vector3 &v) const { return !(*this == v); }

    Vector3 operator + (const Vector3 & v) const { return { x + v.x, y + v.y, z + v.z }; }
    Vector3 & operator += (const Vector3 & v) { x += v.x, y += v.y, z += v.z; return *this; }
    Vector3 operator - (const Vector3 & v) const { return { x - v.x, y - v.y, z - v.z }; }
    Vector3 & operator -= (const Vector3 & v) { x -= v.x, y -= v.y, z -= v.z; return *this; }
    Vector3 operator * (const Vector3 & v) const { return { x * v.x, y * v.y, z * v.z }; }
    Vector3 & operator *= (const Vector3 & v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
    Vector3 operator / (const Vector3 & v) const { return { x / v.x, y / v.y, z / v.z }; }
    Vector3 & operator /= (const Vector3 & v) { x /= v.x, y /= v.y, z /= v.z; return *this; }

    Vector3 operator + (f32 scalar) const { return { x + scalar, y + scalar, z + scalar }; }
    Vector3 & operator += (f32 scalar) { x += scalar, y += scalar, z += scalar; return *this; }
    Vector3 operator - (f32 scalar) const { return { x - scalar, y - scalar, z - scalar }; }
    Vector3 & operator -= (f32 scalar) { x -= scalar, y -= scalar, z -= scalar; return *this; }
    Vector3 operator * (f32 scalar) const { return { x * scalar, y * scalar, z * scalar }; }
    Vector3 & operator *= (f32 scalar) { x *= scalar, y *= scalar, z *= scalar; return *this; }
    Vector3 operator / (f32 scalar) const { return { x / scalar, y / scalar, z / scalar }; }
    Vector3 & operator /= (f32 scalar) { x /= scalar, y /= scalar, z /= scalar; return *this; }
};

typedef Vector3 Point3;



const Vector3 POINT3_ZERO = { 0.0f, 0.0f, 0.0f };
const Vector3 POINT3_UNIT_X = { 1.0f, 0.0f, 0.0f };
const Vector3 POINT3_UNIT_Y = { 0.0f, 1.0f, 0.0f };
const Vector3 POINT3_UNIT_Z = { 0.0f, 0.0f, 1.0f };
const Vector3 POINT3_UNIT_ALL = { 1.0f, 1.0f, 1.0f };

f32 INVERSE_SQRT(f32 val)
{
    float xhalf = 0.5f*val;
    u32 i = *(int*)&val;
    i = 0x5f3759df - (i >> 1);
    val = *(float*)&i;
    val = val * (1.5f - xhalf * val*val);
    return val;
}

