#pragma once
#include <iostream>
#include <opencv2\highgui.hpp>
#include <algorithm>
static cv::RNG rng;
#define M_EPS 1e-4
#define RAND_R 10000000
#define PI 3.1415926
#define PI_INV 0.3183098862
#define Color Vec3
#define MAX_NUM 1e20
#define MIN_NUM -1e20
#define MAX_VEC3 (Vec3(MAX_NUM,MAX_NUM,MAX_NUM))
#define MIN_VEC3 (Vec3(MIN_NUM,MIN_NUM,MIN_NUM))
inline double GetRandu(double r1, double r2)
{
	return rng.uniform(r1, r2);
}
inline double GetRandu()
{
	return GetRandu(-RAND_R, RAND_R);
}
inline bool IsZero(double i) { return abs(i) < M_EPS; }
int GetRandu(int i, int j)
{
	return rng.uniform(i, j);
}
class Vec3 :public cv::Vec3d
{
	

public:
	using cv::Vec3d::Vec3d;
	Vec3() :cv::Vec3d(0, 0, 0) {}
	Vec3(const cv::Vec3d & v) { for (int i = 0; i < 3; i++) this->val[i] = v.val[i]; }
	Vec3& operator += (double num) { for (int i = 0; i < 3; i++) this->val[i] += num; return *this; }
	Vec3& operator -= (double num) { for (int i = 0; i < 3; i++) this->val[i] -= num; return *this; }
	Vec3 operator + (double num) const { Vec3 vtmp; for (int i = 0; i < 3; i++) vtmp.val[i] = this->val[i] + num; return vtmp; }
	Vec3 operator - (double num) const { Vec3 vtmp; for (int i = 0; i < 3; i++) vtmp.val[i] = this->val[i] - num; return vtmp; }
	Vec3 operator - () { return Vec3(-val[0], -val[1], -val[2]); }
	bool operator <(const Vec3& v)const { return (val[0] < v.val[0] && val[1] < v.val[1] && val[2] < v.val[2]); }
	bool operator <(const double & k)const { return (val[0] < k && val[1] < k && val[2] < k); }
	bool operator >(const Vec3& v)const { return (val[0] > v.val[0] && val[1] > v.val[1] && val[2] > v.val[2]); }
	double Module2()const { double res = 0; for (int i = 0; i < 3; i++) { res += val[i] * val[i]; } return res; }
	double Module() const { return sqrt(Module2()); }
	friend double dist2(const Vec3&v1, const Vec3&v2) { return Vec3(v1 - v2).Module2(); }
	friend double dist(const Vec3&v1, const Vec3&v2) { return sqrt(dist2(v1, v2)); }
	Vec3& Normalize() {
		double mo = Module();
		if (!IsZero(mo)) {
			double mo_ = 1 / mo;
			for (int i = 0; i < 3; i++) this->val[i] *= mo_;
		}
		return *this;
	}
	Vec3 GetUnit() { double mo = Module(); return IsZero(mo) ? *this : *this * (1.0 / mo); }
	static Vec3 GetUnitRandVec() {
			double x, y, z;
			do {
				x = GetRandu(-1., 1.);
				y = GetRandu(-1., 1.);
				z = GetRandu(-1., 1.);
			} while (x * x + y * y + z * z > 1 || x * x + y * y + z * z < M_EPS);
			return Vec3(x, y, z).GetUnit();
	}
	friend void Confine(Vec3 &v1) {//�жϹ����Ƿ�������ֵ
		for (int i = 0; i < 3; i++)
		{
			if (v1.val[i] > 1) v1.val[i] = 1;
		}
	}
	friend bool IsZero(const Vec3 &v1)
	{
		for (int i = 0; i < 3; i++)
		{
			if (IsZero(v1.val[i])) return true;
		}
		return false;
	}
	Vec3 GetRefl(const Vec3& norm)const
	{
		return *this - 2 * this->dot(norm)*norm;
	}
	Vec3 GetUnitRefl(const Vec3& norm) const { return GetRefl(norm).GetUnit(); }
	Vec3 GetAnormal() const
	{
		return (val[0] == 0 && val[1] == 0) ?
			Vec3(1, 0, 0) : Vec3(val[1], -val[0], 0).GetUnit();
	}
	static Vec3 GetUnitRandRefl(const Vec3& norm) {
		// �õ�һ����������ֵ��Ȩ������ֲ��ڵ�λ���ڵĵ�λ����
			// ����ֱ����ļн�theta��������ֵ��Ȩ
			double theta = acos(sqrt(GetRandu(0.,1.)));
			double phi = 2 * PI *GetRandu(0., 1.);
			Vec3 ret = norm.rotated(norm.GetAnormal(), theta);
			return ret.rotated(norm, phi).GetUnit();

	}
	Vec3 rotated(const Vec3 &axis, double angle) const
	{
		if (fabs(angle) < M_EPS) return *this;
		Vec3 ret;
		double cost = cos(angle);
		double sint = sin(angle);
		ret.val[0] += val[0] * (axis.val[0] * axis.val[0] + (1 - axis.val[0] * axis.val[0]) * cost);
		ret.val[0] += val[1] * (axis.val[0] * axis.val[1] * (1 - cost) - axis.val[2] * sint);
		ret.val[0] += val[2] * (axis.val[0] * axis.val[2] * (1 - cost) + axis.val[1] * sint);
		ret.val[1] += val[0] * (axis.val[1] * axis.val[0] * (1 - cost) + axis.val[2] * sint);
		ret.val[1] += val[1] * (axis.val[1] * axis.val[1] + (1 - axis.val[1] * axis.val[1]) * cost);
		ret.val[1] += val[2] * (axis.val[1] * axis.val[2] * (1 - cost) - axis.val[0] * sint);
		ret.val[2] += val[0] * (axis.val[2] * axis.val[0] * (1 - cost) - axis.val[1] * sint);
		ret.val[2] += val[1] * (axis.val[2] * axis.val[1] * (1 - cost) + axis.val[0] * sint);
		ret.val[2] += val[2] * (axis.val[2] * axis.val[2] + (1 - axis.val[2] * axis.val[2]) * cost);
		return ret;
	}
	double GetPower() const
	{
		return (val[0] + val[1] + val[2]) / 3;
	}
	Vec3 refracted(const Vec3 &N, double n_from, double n_to)
	{
		return refracted(N, n_from / n_to);
	}
	Vec3 refracted(const Vec3 &N, double n) const
	{
		//ע��������������ʽ��д���
		//1 �õ�cosR2
		//2 cosR2<0��ȫ���䣬����������
		double cosI = -N.dot(*this);//cos(�����)������ʵ���Ͽ��Բ�ȡ�෴��
		double cosR2 = 1 - (1 - pow(cosI, 2)) * pow(n, 2);//�����n����������ʣ�Ҳ��������������ʱ�����������ʣ����ڴ�������������������Ҫ������п��ǣ��ڵ��õ�ʱ����Ҳ�У�
		return (cosR2 > M_EPS) ? //�����С˵��ȫ����
			*this * n + N * (n * cosI - sqrt(cosR2))
			: this->GetRefl(N);
	}
	double GetInfNorm()//����ֵ�����ֵ
	{
		double t = -1;
		for (int i = 0; i < 3; i++)
		{
			if (abs(val[i]) > t)
			{
				t = val[i];
			}
		}
		return t;
	}
	bool isnan()
	{
		for (int i = 0; i < 3; i++)
		{
			if (!(val[i] < 2) && !(val[i]>1)) return true;
		}
		return false;
	}
};