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
	Vec3 P0;//平面上一点(最好选中心点)
	Vec3 norm;//单位法向量
	PlaneObj(const Vec3& pp, const Vec3 &nn, const Color &cc , const Material & mm) :P0(pp), norm(nn), Obj(mm, cc) {
		norm.Normalize();
	}
	Vec3 texU = Vec3(400, 0, 0);
	Vec3 texV = Vec3(0,0,300 );
	Collision GetIntersect(const Ray & ray) override {
		const Vec3&dir = ray.dir;
		const Vec3&org = ray.org;
		double project = dir.dot(norm);
		if (IsZero(project)) return NoneColl;//没有碰撞
		double dist = ((P0 - org).dot(norm)) / project;//光源到碰撞点的距离（到平面的距离除以cos(入射角)）
		//1 光源在正对方向，并且方向和法向量相反（正面碰撞）,dist>0
		//2 光源在正对方向，并且方向和法向量相同（没有碰撞）,dist<0
		//3 光源在背面方向，并且方向和法向量相同（背面碰撞）,dist>0,这个情况没有考虑
		//4 光源在背面方向，并且方向和法向量相反（没有碰撞）,dist<0

		if (dist > M_EPS)
		{
			//TODO:添加颜色
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
	Vec3 texU=Vec3(0,3,-3).GetUnit(), texV=Vec3(1,0,0).GetUnit(); // 用于纹理贴图时计算UV坐标


	SphereObj(double rradius, const  Vec3& orgg, const Color&cc, const Material& mm) :org(orgg), radius(rradius), Obj(mm, cc) {}
	virtual Collision GetIntersect(const Ray & ray) override {
		const Vec3&d = ray.dir;
		const Vec3&o = ray.org;
		Vec3 L = org - o;//得到光源到球心的向量
		double project = L.dot(d);//到射线方向的投影
		double det2 = radius * radius - (L.Module2() - project * project);// 半弦长平方
		if (det2 < M_EPS) return NoneColl;//如果几乎垂直，那么不计入
		double det = sqrt(det2);
		double dist1 = project - det, dist2 = project + det;//dist1是光源到碰撞点的距离，dist2是光源到碰撞
		//1 光源在外，方向相同，dist1>0 取dist1 ,此时dist2>0
		//2 光源在外，方向相反，dist1<0,dist2 也小于0 ，不取
		//3 光源在内，方向相同，dist1<0 取dist2 ,此时dist2>0
		//4 光源在内，方向相反，dist1<0 取dist2 ,此时dist2>0
		//所以，dist1>0 那么取dist1;
		//否则，若dist2小于0，那么返回；若dist2>0，那么取dist2
		//同时，若dist1>0，那么标记光源在外部，否则标记光源在内部
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
		Vec3 N = Vec3((P - org)).GetUnit();//得到法向

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
