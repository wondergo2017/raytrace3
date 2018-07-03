#pragma once
#include "Vec3.h"
#include <Eigen\Dense>
#include "Element.h"
#include "Obj.h"
#define MAX_ITER 10
class AABBbox
{
	
public:
	Vec3 pmin = MAX_VEC3, pmax = MIN_VEC3;
	void addpoint(const Vec3 & p)
	{
		for (int i = 0; i < 3; i++)
		{
			if (pmin.val[i] > p.val[i]) pmin.val[i] = p.val[i];
			if (pmax.val[i] < p.val[i]) pmax.val[i] = p.val[i];
		}
	}
	bool intersect(const Vec3 & org,const Vec3 & dir)
	{
		double t = 0;
		Vec3 Colp;//碰撞点
		for (int i = 0; i < 3; i++)
		{
			if (!IsZero(dir.val[i]))
			{
				if (dir.val[i] > 0) t = (pmin.val[i] - org.val[i]) / dir.val[i];
				else t = (pmax.val[i] - org.val[i]) / dir.val[i];
				
				if (t > 0)
				{
					Colp = org + t * dir;
					bool flag = true;
					for (int j = 0; j < 3; j++)
					{
						if (j != i)//根据其他坐标方向判断是不是在盒子内
						{
							if (pmax.val[j]<Colp.val[j] || pmin.val[j]>Colp.val[j])//坐标在外
								flag = false;
						}
					}
					if (flag)
					{
						return true;
					}
				}
			}
		}
		return false;
	}
	void clear()
	{
		pmin = MAX_VEC3;
		pmax = MIN_VEC3;
	}
};

class Bezier3
{
public:
	//1-4 借鉴网上资料
	//1 曲线上点
	//2 曲线上切向量
	//3 曲面上点
	//4 曲面上切向量
	//5 牛顿迭代
	
	static Vec3 evalCurvePoint(const Vec3 P[4], double t)
	{
		double c0 = (1 - t) * (1 - t) * (1 - t);
		double c1 = 3 * t * (1 - t) * (1 - t);
		double c2 = 3 * t * t * (1 - t);
		double c3 = t * t * t;
		return P[0] * c0 + P[1] * c1 + P[2] * c2 + P[3] * c3;
	}
	static Vec3 evalCurveDeriv(const Vec3 P[4], double t)
	{
		double c0 = -3 * (1 - t) * (1 - t);
		double c1 = 3 * (1 - t) * (1 - t) - 6 * t * (1 - t);
		double c2 = 6 * t * (1 - t) - 3 * t * t;
		double c3 = 3 * t * t;
		return P[0] * c0 + P[1] * c1 + P[2] * c2 + P[3] * c3;
	}
	static Vec3 evalPatchPoint(const Vec3 P[16], double u, double v)
	{
		Vec3 curve[4];
		for (int i = 0; i < 4; i++)	curve[i] = evalCurvePoint(P + 4 * i, u);
		return evalCurvePoint(curve, v);
	}
	static Vec3 evalPatchDerivU(const Vec3 P[16], double u, double v)
	{
		Vec3 Pv[4];
		Vec3 curve[4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				Pv[j] = P[i + 4 * j];
			}

			curve[i] = evalCurvePoint(Pv, v);
		}
		return evalCurveDeriv(curve, u);
	}
	static Vec3 evalPatchDerivV(const Vec3 P[16], double u, double v)
	{
		Vec3 curve[4];
		for (int i = 0; i < 4; i++) curve[i] = evalCurvePoint(P + 4 * i, u);
		return evalCurveDeriv(curve, v);
	}
	Vec3 newton(const Vec3 &org, const Vec3 & dir,int maxIter) {

		std::vector<Vec3> xvec;
		for (int o = 0; o < maxIter*5; o++)
		{
			Vec3 x(0, GetRandu(0.,1.), GetRandu(0.,1.)), prevX(-1, -1, -1);
			int iter = 0;
			while (iter < maxIter)
			{
				prevX = x;
				Vec3 L = org + dir * prevX(0);
				Vec3 P_ = evalPatchPoint(P, prevX(1), prevX(2));
				Vec3 derivU = evalPatchDerivU(P, prevX(1), prevX(2));
				Vec3 derivV = evalPatchDerivV(P, prevX(1), prevX(2));
				cv::Matx33d m;
				for (int i = 0; i < 3; i++){m(i, 0) = dir(i);}
				for (int i = 0; i < 3; i++){m(i, 1) = -derivU(i);}
				for (int i = 0; i < 3; i++){m(i, 2) = -derivV(i);}
				m = m.inv();
				x = prevX - m * (L - P_);
				iter++;
				L = org + dir * x(0);
				P_ = evalPatchPoint(P, x(1), x(2));
				if (Vec3(L - P_).Module2() < M_EPS&&x.val[1]<=1&&x.val[1]>=0&&x.val[2]>=0&&x.val[2]<=1)
				{
					xvec.push_back(x);
					//break;
				}
			}
		}
		if (xvec.empty())
		{
			return MAX_VEC3;
		}
		Vec3 minx = Vec3();
		double mindist = MAX_NUM;
		int index = -1;
		for (int i=0;i<xvec.size();i++)
		{
			Vec3& x = xvec[i];
			if (x.val[0] < mindist)
			{
				mindist = x.val[0];
				index = i;
			}
		}
		return xvec[index];
	}

	//1 控制点
	//2 包围盒与建立
	//3 求交
	//4 读取
	Vec3 P[16];
	AABBbox aabb;
	void read(const Vec3 p[16])
	{
		aabb.clear();
		for (int i = 0; i < 16; i++)
		{
			P[i] = p[i];
			aabb.addpoint(P[i]);
		}
	}
	Vec3 intersect(const Vec3 & org, const Vec3 & dir)
	{
		if (aabb.intersect(org, dir))
		{
			return newton(org, dir, MAX_ITER);
		}
		else
		{
			return MAX_VEC3;
		}
	}
};
class Bezier3Obj :public Obj
{
	//1 面vec
	//2 整个的包围盒
	//3 
	std::vector<Bezier3> Bvec;
	AABBbox aabb;
	//1 读取，建立整个包围盒
	//2 求交
	//3 
public:
	Bezier3Obj(const Vec3 &C,double scale,const std::string &file, const Color &cc, const Material & mm,const cv::Matx33d & Trans=cv::Matx33d::eye() ):Obj(mm,cc)
	{
		aabb.clear();
		Bvec.clear();
		std::cerr << "read bezier3::"<<file << std::endl;
		freopen(file.c_str(), "r", stdin);
		int size;
		std::cin >> size;
		
		for (int i = 0; i < size; i++)
		{
			int m, n;
			std::cin >> m >> n;
			Vec3 vtmp[16];
			for (int j = 0; j < m+1; j++)
			{
				for (int k = 0; k < n+1; k++)
				{
					int newv = j * (n + 1) + k;
					for (int l = 0; l < 3; l++)
					{
						std::cin >> vtmp[newv].val[l];
							vtmp[newv].val[l] *= scale;
					}
					vtmp[newv] = Trans * vtmp[newv];
				}
			}
			for (int i = 0; i < 16; i++)
			{
				vtmp[i] += C;
				aabb.addpoint(vtmp[i]);
				//std::cerr << vtmp[i] << std::endl;
			}
			Bezier3 bezier3;
			bezier3.read(vtmp);
			Bvec.push_back(bezier3);
		}
		freopen("CON", "r", stdin);
		std::cerr << "done!size::" << Bvec.size() << std::endl;
		std::cerr << "Bezier::aabb" << aabb.pmax << aabb.pmin << std::endl;
	}
	virtual Collision GetIntersect(const Ray & ray) override
	{
		const Vec3& o = ray.org;
		const Vec3&d = ray.dir;
		if (aabb.intersect(o, d))
		{
			Vec3 para = MAX_VEC3;
			int index = -1;
			for (int i = 0; i < Bvec.size(); i++)
			{
				Vec3 paratmp = Bvec[i].intersect(o, d);
				if (paratmp.val[0]<M_EPS || paratmp.val[0]>para.val[0] || paratmp.val[1] < -M_EPS || paratmp.val[1]>1 + M_EPS || paratmp.val[2] < -M_EPS || paratmp.val[2]>1 + M_EPS)
				{
					continue;
				}
				if (paratmp.val[0] < para.val[0])
				{
					para = paratmp;
					index = i;
				}
			}
			
			if (index == -1) return NoneColl;
			//返回
			Collision ctmp;
			ctmp.Pos = o + d * para.val[0];
			ctmp.From = d;
			Vec3 U = Bvec[index].evalPatchDerivU(Bvec[index].P,para.val[1],para.val[2]);
			Vec3 V = Bvec[index].evalPatchDerivV(Bvec[index].P, para.val[1], para.val[2]);
			Vec3 N = U.cross(V);
			if (N.dot(d) > 0) N = -N;
			
			N.Normalize();
			ctmp.N = N;
			ctmp.obj = this;
			ctmp.dist = para.val[0];
			
			ctmp.inside = false;
			ctmp.color = GetColor(para.val[1],para.val[0]);
			return ctmp;
		}
		return NoneColl;
	}
	Color GetColor(double u,double v)
	{
		if (!texture) return this->color;
		return texture->colorUV(u, v);
	}

};