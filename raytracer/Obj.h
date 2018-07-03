#pragma once
#include "Vec3.h"
#include "Element.h"

class Obj
{
protected:
	//Texture
	Texture *texture=nullptr;
public:
	Obj(const Material & mm, const Color &cc) :material(mm), color(cc) {
		diffr = mm.diff.GetPower();
		reflr = mm.refl.GetPower();
		refrr = mm.refr.GetPower();
		allr = diffr + reflr + refrr;
	}
	Material material;
	bool IsDiff() { return material.IsDiff(); }
	bool IsRefl() { return material.IsRefl(); }
	bool IsRefr() { return material.IsRefr(); }
	void GetDiff(Vec3& v) const { v = material.diff; }
	void GetRefl(Vec3& v) const { v = material.refl; }
	void GetRefr(Vec3& v) const { v = material.refr; }
	void GetRefl(Vec3& v, double &rln) const { v = material.refl; rln = material.refln; }
	void GetRefr(Vec3& v, double& rrn) const { v = material.refr; rrn = material.refrn; }

	//Roulette
	enum React { DIFF, REFL, REFR };
	double diffr, reflr, refrr, allr;
	React Roulette()
	{
		//to be finished 
		double r = GetRandu(0., allr);
		if (diffr>r)
		{
			return DIFF;
		}
		else if (diffr + reflr > r)
		{
			return REFL;
		}
		else {
			return REFR;
		}
	}
	Color color;
	virtual ~Obj() = 0 {};
	virtual Collision GetIntersect(const Ray & ray) = 0;
	
	void setTexture(Texture *texture_)
	{
		texture = texture_;
	}
};
class PlaneObj :public Obj
{
public:
	Vec3 P0;//ƽ����һ��(���ѡ���ĵ�)
	Vec3 norm;//��λ������
	PlaneObj(const Vec3& pp, const Vec3 &nn, const Color &cc , const Material & mm) :P0(pp), norm(nn), Obj(mm, cc) {
		norm.Normalize();
	}
	Vec3 texU = Vec3(400, 0, 0);
	Vec3 texV = Vec3(0,0,300 );
	Collision GetIntersect(const Ray & ray) override {
		const Vec3&dir = ray.dir;
		const Vec3&org = ray.org;
		double project = dir.dot(norm);
		if (IsZero(project)) return NoneColl;//û����ײ
		double dist = ((P0 - org).dot(norm)) / project;//��Դ����ײ��ľ��루��ƽ��ľ������cos(�����)��
		//1 ��Դ�����Է��򣬲��ҷ���ͷ������෴��������ײ��,dist>0
		//2 ��Դ�����Է��򣬲��ҷ���ͷ�������ͬ��û����ײ��,dist<0
		//3 ��Դ�ڱ��淽�򣬲��ҷ���ͷ�������ͬ��������ײ��,dist>0,������û�п���
		//4 ��Դ�ڱ��淽�򣬲��ҷ���ͷ������෴��û����ײ��,dist<0

		if (dist > M_EPS)
		{
			//TODO:�����ɫ
			Vec3 P = org + dir * dist;
			return Collision(P, dir, dist, this, norm,GetColor(P));
		}
		else {
			return NoneColl;
		}
	}
	Color GetColor(const Vec3 &P) const
	{
		if(!texture) return this->color;
		int ndir = 0;
		
		for (int i = 0; i < 3; i++)
		{
			if (norm.val[i] != 0) ndir = i;
		}
		int vdex = (ndir +2) % 3;
		int udex = (ndir + 1) % 3;
		double v = 0.5 + (P.val[vdex] - P0.val[vdex]) / texU.Module();
		double u = 0.5 + (P.val[udex] - P0.val[udex]) / texV.Module();
		return texture->colorUV(u, v);
	}
};
class SphereObj :public Obj
{
public:
	Vec3 org;
	double radius;
	Vec3 texU=Vec3(0,3,-3).GetUnit(), texV=Vec3(1,0,0).GetUnit(); // ����������ͼʱ����UV����


	SphereObj(double rradius, const  Vec3& orgg, const Color&cc, const Material& mm) :org(orgg), radius(rradius), Obj(mm, cc) {}
	virtual Collision GetIntersect(const Ray & ray) override {
		const Vec3&d = ray.dir;
		const Vec3&o = ray.org;
		Vec3 L = org - o;//�õ���Դ�����ĵ�����
		double project = L.dot(d);//�����߷����ͶӰ
		double det2 = radius * radius - (L.Module2() - project * project);// ���ҳ�ƽ��
		if (det2 < M_EPS) return NoneColl;//���������ֱ����ô������
		double det = sqrt(det2);
		double dist1 = project - det, dist2 = project + det;//dist1�ǹ�Դ����ײ��ľ��룬dist2�ǹ�Դ����ײ
		//1 ��Դ���⣬������ͬ��dist1>0 ȡdist1 ,��ʱdist2>0
		//2 ��Դ���⣬�����෴��dist1<0,dist2 ҲС��0 ����ȡ
		//3 ��Դ���ڣ�������ͬ��dist1<0 ȡdist2 ,��ʱdist2>0
		//4 ��Դ���ڣ������෴��dist1<0 ȡdist2 ,��ʱdist2>0
		//���ԣ�dist1>0 ��ôȡdist1;
		//������dist2С��0����ô���أ���dist2>0����ôȡdist2
		//ͬʱ����dist1>0����ô��ǹ�Դ���ⲿ�������ǹ�Դ���ڲ�
		if (dist2 < M_EPS) return NoneColl;
		double dist = (dist1 > M_EPS) ? dist1 : dist2;

		Collision ctmp;
		ctmp.Pos = o + d * dist;
		ctmp.From = d;
		ctmp.N = Vec3(ctmp.Pos - org).GetUnit();
		ctmp.obj = this;
		ctmp.dist = dist;
		ctmp.inside = (dist1 < M_EPS);
		ctmp.color = GetColor(ctmp.Pos);
		return ctmp;
	}
	Color GetColor(const Vec3 & P)
	{
		if (!texture) return this->color;
		Vec3 N = Vec3((P - org)).GetUnit();//�õ�����

		double theta = acos(N.dot(texV));
		double t = N.dot(texU) / sin(theta);
		t = (t >= -1) ? t : -1;
		t = (t <= 1) ? t : 1;
		double phi = acos(t);
		double u = theta / PI, v = phi / (2 * PI);
		v = (N.dot(texU.cross(texV)) < 0) ? (1 - v) : v;
		return texture->colorUV(u, v);
	}
};
