#pragma once
#include "Vec3.h"
#include "Element.h"
class Light
{
public:
	Light(Vec3 p, Color c) : pos(p), color(c) {}
	virtual void emit(Ray & ray, Vec3 & flux)
	{
		flux = color * PI * 4;
		ray.dir = Vec3::GetUnitRandVec();
		ray.org = pos;
	}
private:
	Vec3 pos; // ���Դλ��
	Color color; // ��Դ��ɫ
};

class SpotLight : public Light
{
public:
	SpotLight(Vec3 p, Color c) : Light(p, c), pos(p), color(c) {}
private:
	Vec3 pos; // ���Դλ��
	Color color; // ��Դ��ɫ
};