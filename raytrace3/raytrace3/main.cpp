#include <iostream>
#include <opencv2\highgui.hpp>
#include <algorithm>
using namespace std;
#define M_EPS 1e-10
#define RAND_R 10000000
inline bool IsZero(double i) { return abs(i) < M_EPS; }
#define Color Vec3
#define DEBUG
#define MAX_NUM 1e20
#define MIN_NUM -1e20
#define PI 3.1415926
#define PI_INV 0.3183098862
#define MIN_WGT 1e-3
#define MAX_DEP 20
#define INIT_R2 20
#define MAX_VEC3 (Vec3(MAX_NUM,MAX_NUM,MAX_NUM))
#define MIN_VEC3 (Vec3(MIN_NUM,MIN_NUM,MIN_NUM))
cv::RNG rng;
const double alpha = 0.7;//光子数量更新因子
double GetRandu(double r1, double r2)
{
	return rng.uniform(r1, r2);
}
double GetRandu()
{
	return GetRandu(-RAND_R, RAND_R);
}
class Vec3 :public cv::Vec3d
{
public:
	using cv::Vec3d::Vec;
	Vec3() :cv::Vec3d(0,0,0) {}
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
	Vec3 GetUnit()const { double mo = Module(); return IsZero(mo) ? *this : *this * (1.0 / mo); }
	static Vec3 GetRandVec() { return Vec3(GetRandu(), GetRandu(), GetRandu()); }
	static Vec3 GetUnitRandVec() { return GetRandVec().GetUnit(); }
	friend void Confine(Vec3 &v1) {//判断光亮是否大于最大值
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
		return *this - 2*this->dot(norm)*norm;
	}
	Vec3 GetUnitRefl(const Vec3& norm) const {return GetRefl(norm).GetUnit();}
	static Vec3 GetUnitRandRefl(const Vec3& norm) {
		Vec3 res = GetRandVec();
		while (res.dot(norm) <= 0)
		{
			res = GetRandVec();
		}
		return res.GetUnit();
	}
};
struct Ray { Vec3 org, dir; Ray() {}; Ray(Vec3 o, Vec3 d) : org(o), dir(d) {} }; // 射线
class Obj;
class Camera
{
public:
	int hw, hh;
	Vec3 pos;//相机位置
	double fov_h;//相机张角
	int w, h;//分辨率(w*h)
	double sr;//采样率
	double dis;//相机到画布的距离
	Vec3 dir;//相机朝向(单位矢量)
	Vec3 PicOrg;////相机到画布垂足的矢量
	Vec3 du, dv;//画布横纵单位矢量
	Color **pic;//画布
	Ray emit(int x, int y)
	{
		return Ray(pos, Vec3(PicOrg + du * (x - hh) + dv * (y - hw)).GetUnit());
	}
	void init(int h_ = 500, int w_ = 500, int fovAng = 45, const Vec3 &pos_ = Vec3(25, 58, 200.6), const Vec3&dir_ = Vec3(0, -0.122612, -1)) {
#ifdef DEBUG
		std::cout << "camera::init()!" << std::endl;
#endif // DEBUG
		dir = dir_;
		pos = pos_;
		
		sr = 1.0 / 100;
		fov_h = fovAng * acos(-1.0) / 180;

		h = h_;
		w = w_;

		du = pos.cross(dir);
		du.Normalize();
		du *= sr;
		dv = dir.cross(du);
		dv.Normalize();
		dv *= sr;
		dis = h * sr / 2 / tan(fov_h / 2);
		dir.Normalize();
		PicOrg = dir*dis;
#ifdef DEBUG
		std::cout << "dir:" << dir;
		std::cout << "du:" << du;
		std::cout << "dv:" << dv;
#endif // DEBUG
		pic = new Color*[h];
		for (int i = 0; i < h; i++)
		{
			pic[i] = new Color[w];
		}
		hw = w / 2;
		hh = h / 2;
	}

};
class Material // 材质
{
public:
	Material(Vec3 d, Vec3 s, Vec3 r, double rr = 0, double rl = 1) : diff(d), refl(s), refr(r), refrn(rr), refln(rl) {}
	Vec3 diff; // 漫反射系数
	Vec3 refl; // 镜面反射系数
	Vec3 refr; // 折射系数
	double refrn; // 折射率
	double refln; // 反射发光系数
	bool IsDiff() { return !IsZero(diff); }
	bool IsRefl() { return !IsZero(refl); }
	bool IsRefr() { return !IsZero(refr); }
};
class Collision// 碰撞信息
{
public:
	bool IsValid() { return obj != nullptr; }
	Vec3 Pos;
	Vec3 N;
	Vec3 From;//入射方向
	Obj* obj;//关联物体
	double dist;//距离
	explicit Collision(const Vec3& pos = Vec3(), const Vec3& from = Vec3(), double d = 0, Obj* o = nullptr, const Vec3& n = Vec3()) :Pos(pos), N(n), From(from)
	{
		dist = d;
		obj = o;
	}
	bool inside = 0;
};
enum OBJ_REL { INSIDE, OUTSIDE, ON, UNKNOWN };//和物体的相对位置
class Obj
{
public:
	Obj(const Material & mm, const Color &cc) :material(mm), color(cc) {
		diffr = mm.diff.Module2();
		reflr = mm.refl.Module2();
		refrr = mm.refr.Module2();
		allr = diffr + reflr + refrr;
	}
	Material material;
	bool IsDiff() { return material.IsDiff(); }
	bool IsRefl() { return material.IsRefl(); }
	bool IsRefr() { return material.IsRefr(); }
	void GetDiff(Vec3& v) const  {v=material.diff;}
	void GetRefl(Vec3& v) const {v=material.refl; }
	void GetRefr(Vec3& v) const {v=material.refr; }
	void GetRefl(Vec3& v, double &rln) const { v = material.refl; rln = material.refln; }
	void GetRefr(Vec3& v, double& rrn) const { v = material.refr; rrn = material.refrn; }

	//Roulette
	enum React {DIFF,REFL,REFR};
	double diffr, reflr, refrr,allr;
	React Roulette()
	{
		//to be finished 
		double r = GetRandu(0,allr);
		if (diffr<r)
		{
			return DIFF;
		}
		else if (diffr + reflr < r)
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
};
class PlaneObj :public Obj
{
public:
	Vec3 P0;//平面上一点
	Vec3 norm;//单位法向量
	PlaneObj(const Vec3& pp, const Vec3 &nn, const Material & mm, const Color &cc = Color(1, 1, 1)) :P0(pp), norm(nn), Obj(mm, cc) {}

	Collision GetIntersect(const Ray & ray) override {
		/*double t = -(n.dot(P0) + n.dot(ray.org)) / (n.dot(ray.dir));
		Collision ctemp;
		if (t > 0)
		{
			ctemp.Pos = ray.org + ray.dir*t;
			ctemp.dist = GetDist(ray.org, ray.dir);
			ctemp.From = ray.dir;
			ctemp.N = GetNormal(ctemp.Pos);
			ctemp.obj = this;
		}*/
		Collision ctemp; 
		Vec3 op(ray.org - P0), n;
		if (IsZero(norm.dot(ray.dir))) return ctemp;
		ctemp.From = ray.dir;
		if (!IsZero(norm.dot(op))) { ctemp.N = norm; ctemp.inside = false; }
		else { ctemp.N = norm * -1.; ctemp.inside = true; }
		double dis =ctemp.N.dot(op);
		double project = ctemp.N.dot(ray.dir);
		if (!IsZero(project)) return ctemp;
		ctemp.dist = dis / (-project);
		ctemp.Pos = ray.org + ray.dir * ctemp.dist;
		ctemp.obj = this;
		return ctemp;
	}
	Vec3 GetNormal(const Vec3 & v) {
		return norm;
	}
	double GetDist(const Vec3 & pos, const Vec3 & vec)
	{
		return norm.dot(pos)*norm.dot(vec);
	}
};
class SphereObj :public Obj
{
public:
	Vec3 org;
	double radius;
	SphereObj(double rradius, const  Vec3& orgg, const Material& mm, const Color&cc = Color(1, 1, 1)) :org(orgg), radius(rradius), Obj(mm, cc) {}
	virtual Collision GetIntersect(const Ray & ray) override {
//		//std::cout <<"vec:"<<vec;
//		Collision ctemp;
//		double r2 = radius * radius;
//		Vec3 l = org - ray.org;//光源和球中心的向量
//		double tp = l.dot(ray.dir);//垂足的t
//		OBJ_REL flag = UNKNOWN;//光源与物体相对位置
//							   //判断光源与物体相对位置
//		if (l.Module2() < r2)
//		{
//			flag = INSIDE;
//		}
//		else if (l.Module2() > r2)
//		{
//			flag = OUTSIDE;
//		}
//		else {
//			flag = ON;
//		}
//		//光线与球面不相交
//		if (tp < 0 && flag == OUTSIDE)
//		{
//#ifdef DEBUG
//			//std::cout << "OUTSIDE" << std::endl;
//#endif // DEBUG
//			return ctemp;
//		}
//		double d2 = l.Module2() - tp * tp;// 球心到光线所在直线的距离的平方
//		if (d2 > r2)//光线与球面不相交
//		{
//#ifdef DEBUG
//			//std::cout << "d2>r2" << std::endl;
//#endif // DEBUG
//			return ctemp;
//		}
//		double t2 = r2 - d2;//投影点到光线与球面的交点距离的平方
//		double t = 0;// 最终直线方向参量
//		if (flag == OUTSIDE)
//		{
//			t = tp - sqrt(t2);
//		}
//		else if (flag == INSIDE)
//		{
//			t = tp + sqrt(t2);
//		}
//		else {//在球面上，待处理
//#ifdef DEBUG
//			std::cout << "on the Sphere!" << std::endl;
//#endif // DEBUG
//		}
//		ctemp.Pos = ray.org + ray.dir*t;
//		ctemp.dist = GetDist(ray.org, ray.dir);
//		ctemp.From = ray.dir;
//		ctemp.N = GetNormal(ctemp.Pos);
//		ctemp.obj = this;
		Collision ctemp;
		Vec3 op = org - ray.org;
		if (IsZero(op.dot(op) - radius*radius)) ctemp.inside = true;
		else ctemp.inside = false;
		double t, b = op.dot( ray.dir), det = b * b - op.dot( op) + radius * radius;
		if (det < 0) { ctemp.dist = MAX_NUM; }
		else
		{
			det = sqrt(det);
			ctemp.dist = (t = b - det) > 1e-8 ? t : ((t = b + det) > 1e-8 ? t : MAX_NUM);
			ctemp.Pos = ray.org + ray.dir * t;
			ctemp.N = Vec3(ctemp.Pos - org).GetUnit();
			ctemp.inside = ctemp.N.dot( ray.dir) > 0;
			ctemp.N = ctemp.inside ? ctemp.N * -1 : ctemp.N;
			ctemp.From = ray.dir;
			ctemp.obj = this;
		}
		return ctemp;
	}
	Vec3 GetNormal(const Vec3& v1)//得到法向量(没有单位化)
	{

		return Vec3(v1 - org);
	}
	double GetDist(const Vec3 & pos, const Vec3 & vec)
	{
		//待处理
		return fabs((Vec3(org - pos) - radius).dot(vec));
	}

};
class Light
{
public:
	Light(Vec3 p, Color c) : pos(p), color(c) {}
	virtual void emit(Ray & ray, Vec3 & flux)
	{
		flux = color* PI * 4;
		ray.dir = Vec3::GetUnitRandVec();
		ray.org = pos;
	}
private:
	Vec3 pos; // 点光源位置
	Color color; // 光源颜色
};

class SpotLight : public Light
{
public:
	SpotLight(Vec3 p, Color c) : Light(p, c), pos(p), color(c) {}
private:
	Vec3 pos; // 点光源位置
	Color color; // 光源颜色
};
class Scene
{
public:
	std::vector<Obj*> objvec;
	std::vector<Light*> plvec;
	void init() {
		Color WHITE(1, 1, 1);
		Material WHITE_DIFF(Vec3(.75, .75, .75), Vec3(), Vec3(), 0);
		Material BLACK_DIFF(Vec3(), Vec3(), Vec3(), 0);
		Material RED_DIFF(Vec3(.75, .3, .3), Vec3(), Vec3(), 0);
		Material BLUE_DIFF(Vec3(.3, .3, .75), Vec3(), Vec3(), 0);
		Material LIGHT_BLUE_DIFF(Vec3(0.45, 0.45, .75), Vec3(), Vec3(), 0);
		// Material LIGHT_BLUE_DIFF2(Vec3(0.75, 0.6, .65), Vec3(), Vec3(), 0);
		Material LIGHT_RED_DIFF(Vec3(0.705, 0.45, .45), Vec3(), Vec3(), 0);
		Material MIRROR(Vec3(), Vec3(1, 1, 1)*.999, Vec3(), 0);
		Material MIRROR2(Vec3(.25, .25, .25), Vec3(1, 1, 1)*.999, Vec3(), 0);
		Material REFR0(Vec3(), Vec3(), Vec3(.999, .999, .999), 1.5);
		Material REFR1(Vec3(), Vec3(), Vec3(.75, .999, .999), 1.5);
		Material REFR2(Vec3(), Vec3(), Vec3(.999, .75, .75), 1.5);
		Material REFR3(Vec3(), Vec3(), Vec3(.75, .999, .75), 1.5);
		Material REFR4(Vec3(), Vec3(), Vec3(.999, .75, .999), 1.5);

		Obj* obj[12] = {
			// new SphereObj(1e5, Vec3(-30 + 1e5, 40, 0), RED_DIFF),
			new PlaneObj(Vec3(-30, 0, 125), Vec3(1, 0, 0), RED_DIFF), // 左面
			new PlaneObj(Vec3(80, 0, 0), Vec3(-1, 0, 0), BLUE_DIFF), // 右面
																  // new SphereObj(1e5, Vec3(0, 40, -85 - 1e5), MIRROR),
																  new PlaneObj(Vec3(-30, 0, -85), Vec3(0, 0, 1), WHITE_DIFF), // 背面
																														   // new SphereObj(1e5, Vec3(0, 81.6 - 1e5, 0), MIRROR),
																														   new PlaneObj(Vec3(-30, 81.6, -85), Vec3(0, 1, 0), WHITE_DIFF), // 顶面
																														   new PlaneObj(Vec3(0, 0, 125), Vec3(0, 0, 1), WHITE_DIFF), // 正面

																														   new SphereObj(8, Vec3(32, 8, -20), REFR2),//
																														   new SphereObj(1, Vec3(25, 80, 20), REFR0),// 顶灯
																														   new SphereObj(1, Vec3(-29.5, 80, 0), REFR0),// 顶灯
																														   new SphereObj(1, Vec3(79.5, 80, 0), REFR0),// 顶灯
																														   new SphereObj(7.5, Vec3(24, 7.5, 40), REFR3), //
																																									  // new SphereObj(6.5, Vec3(0, 13, 50), MIRROR), // 玻璃球
																																									  new SphereObj(8, Vec3(48, 8, 0), MIRROR), //
																																									  new SphereObj(8, Vec3(60, 8, 27), MIRROR), //

		};
		for (int i = 0; i < 12; i++)
			objvec.push_back(obj[i]);
		plvec.push_back(new SpotLight(Vec3(-29.5, 81.5, 0), Color(17500, 17500, 17500)));
		plvec.push_back(new SpotLight(Vec3(79.5, 81.5, 0), Color(17500, 17500, 17500)));
		plvec.push_back(new SpotLight(Vec3(25, 81.5, 20), Color(17500, 17500, 17500)));
#ifdef DEBUG
		std::cout << "scene::init()!" << std::endl;
#endif // DEBUG

	}
	Collision NearCollide(const Ray & ray)//找到最近的碰撞点和碰撞物体
	{
		Obj*e = nullptr;
		double mindist = MAX_NUM;
		Collision ctmp;
		for (auto a : objvec) {
			Collision temppoint = a->GetIntersect(ray);
			if (temppoint.IsValid())//没碰到则不计入
			{
				if (temppoint.dist < mindist)
				{
					mindist = temppoint.dist;
					ctmp = temppoint;
				}
			}
		}
		return ctmp;
	}
};

struct HitPoint//视点碰撞点
{
	Vec3 pos;//hit location
	Vec3 n;//normal
	Vec3 v;//ray direction
	Obj * obj;
	int x, y;// pixcel location
	Color wgt;//pixel weight
	double R2;//current photon radius2
	int N;//accumulated photon cnt
	Color tao;//accumulated photon reflected flux
	friend ostream &operator<<(ostream& out, const HitPoint& hp)
	{
		out << "pos:" << hp.pos << "x,y:" << hp.x << " " << hp.y << endl << "R2:" << hp.R2 << " tao" << hp.tao << endl;
		return out;
	}
	explicit HitPoint(const Vec3& pos_ = Vec3(), const Vec3& n_ = Vec3(), const Vec3& v_ = Vec3(), Obj* o = nullptr,int xx=-1,int yy=-1,const Color& wgt_=Color()):pos(pos_),n(n_),v(v_),obj(o),x(xx),y(yy),wgt(wgt_){
		R2 = INIT_R2;
		N = 0;
		tao = Color();

	}
	
};
inline void putxy(int x, int y, int chan, uchar color, cv::Mat& m) {
	m.ptr<uchar>(y)[3 * x + chan] = color;
}
inline void putxy(int x, int y, const Color& c, cv::Mat& m)
{
	for (int i = 0; i < 3; i++)
	{
		putxy(x, y, 0, c.val[i], m);
	}
}
//
bool LtX(HitPoint* a, HitPoint* b) { return a->pos[0] < b->pos[0]; }
bool LtY(HitPoint* a, HitPoint* b) { return a->pos[1] < b->pos[1]; }
bool LtZ(HitPoint* a, HitPoint* b) { return a->pos[2] < b->pos[2]; }
class KdNode
{
public:
	KdNode(HitPoint* hp=nullptr,KdNode* lc_=nullptr,KdNode *rc_=nullptr,int dim_=0,const Vec3 &min_=MAX_VEC3,const Vec3 &max_=MIN_VEC3) :data(hp) {
		data = hp;
		lc = lc_;
		rc = rc_;
		dim = dim_;
		min = min_;
		max = max_;
	}
	HitPoint *data;
	KdNode *lc, *rc;
	int dim;
	Vec3 min, max; // 包围盒
};

class KdTree
{
public:
	void build(vector<HitPoint*> hps)
	{
		root = build(hps, 0);
	}
	void update(const Collision &c, const Color &flux) {
		update(root,c, flux);
	}
	KdNode* root;
	KdNode* build(vector<HitPoint*> hps, int dim)
	{
		//cout << "build:" << dim << endl;
		if (hps.size() < 1) return NULL;
		if (hps.size() < 2) return new KdNode(hps[0]);

		if (dim == 0) sort(hps.begin(), hps.end(), LtX);
		else if (dim == 1) sort(hps.begin(), hps.end(), LtY);
		else sort(hps.begin(), hps.end(), LtZ);

		int mid = hps.size() >> 1;
		vector<HitPoint*> lhp, rhp;
		for (int i = 0; i < mid; i++)
			lhp.push_back(hps[i]);
		for (int i = mid + 1; i < hps.size(); i++)
			rhp.push_back(hps[i]);
		return new KdNode(hps[mid], build(lhp, (dim + 1) % 3), build(rhp, (dim + 1) % 3), dim);
	};
	void update(KdNode* r, const Collision &c, const Color &flux)
	{
		if (!r) return;
		if (dist2(r->data->pos, c.Pos) <= r->data->R2 && !IsZero(r->data->n.dot(c.N)))
		{
			double g = (r->data->N * alpha + alpha) / (r->data->N * alpha + 1.0);
			r->data->R2 = r->data->R2 * g;
			r->data->N++;
			r->data->tao = (r->data->tao + r->data->wgt.mul(flux) * (PI_INV)) * g;
		}
		if (c.Pos[r->dim] >= r->data->pos[r->dim] - sqrt(r->data->R2)) update(r->rc, c, flux);
		if (c.Pos[r->dim] <= r->data->pos[r->dim] + sqrt(r->data->R2)) update(r->lc, c, flux);
	}
};


class RayTracer
{
	int PhotonCnt = 0;
public:
	Camera camera;
	Scene scene;
	vector<HitPoint*> Hvec;//用来保存视点碰撞点
	KdTree kdtree;
	void init()
	{
#ifdef DEBUG
		std::cout << "raytracer init()!" << std::endl;
#endif // DEBUG

		camera.init();
		scene.init();
	}

	void GetHitPoint()//得到视点碰撞点
	{
		cout << "GetHitPoint():"<<camera.pos << endl;
		for (int i = 0; i < camera.h; i++)
		{
			for (int j = 0; j < camera.w; j++)
			{
				//viewtrace
				ViewTrace(camera.emit(i, j), 0, Color(), Color(1, 1, 1), i, j);
			}
		}
		//debug
		cout << "hitpoint::" << Hvec.size() << endl;
	}
	void UpdateB(const Collision&c, const Color& flux)
	{
		//brute force
		for (auto it=Hvec.begin();it!=Hvec.end();it++)
		{
			if (dist2((**it).pos, c.Pos) < (**it).R2)
			{
				//Update
				double g = ((*it)->N * alpha + alpha) / ((*it)->N * alpha + 1.0);
				(*it)->R2 = (*it)->R2 * g;
				(*it)->N++;
				(*it)->tao = ((*it)->tao + (*it)->wgt.mul( flux) * (PI_INV)) * g;
			}
		}
	}
	void UpdateK(const Collision&c, const Color& flux)
	{
		kdtree.update(c, flux);
	}
	void PhotonTrace(const Ray &ray, int dep, Color flux, Color wgt)
	{
		Collision c1 = scene.NearCollide(ray);
		if (c1.IsValid())
		{
			if (wgt<MIN_WGT || dep>MAX_DEP)//control the depths
			{
				return;
			}
			dep++;
			Obj * obj = c1.obj;
			if (obj->IsDiff())//change the flux,no need to go trace again
			{
				Vec3 diff;
				obj->GetDiff(diff);
				Vec3 diff_flux = flux.mul(diff);
				//update
				UpdateK(c1,flux);
				//Roulette
				if (Obj::React::DIFF==obj->Roulette())//go on diff
				{
					PhotonTrace(Ray(c1.Pos, Vec3::GetUnitRandRefl(c1.N)), dep, flux/sqrt(obj->diffr), wgt.mul(diff));
				}
			}
			if (obj->IsRefl())//keep tracing
			{
				Vec3 refl;
				obj->GetRefl(refl);
				PhotonTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, flux.mul(refl), wgt.mul(refl));
			}
			if (obj->IsRefr())
			{
				//to be continued
				Vec3 refr;
				double rrn;
				obj->GetRefr(refr, rrn);
			}
		}
	}
	void PhotonMap(int cnt)
	{
		cout << "PhotonMap:" << cnt << endl;
		Ray ray;
		Color flux;
		for (auto pl:scene.plvec)
		{
			for (int i = 0; i < cnt; i++)
			{
				cout << i << endl;
				pl->emit(ray, flux);
				PhotonTrace(ray, 0, flux, Color(1, 1, 1));
			}
		}
	}
	
	void ViewTrace(const Ray &ray ,int dep, Color flux, Color wgt,int x, int y)
	{
		//cout << "ViewTrace:" << x << " " << y << endl;
		Collision c1= scene.NearCollide(ray);
		if (c1.IsValid())
		{
			if (wgt<MIN_WGT || dep>MAX_DEP)//control the depths
			{
				return;
			}
			dep++;
			Obj * obj = c1.obj;
			if (obj->IsDiff())//change the wgt,no need to go trace again
			{
				Vec3 diff;
				obj->GetDiff(diff);
				Vec3 diff_wgt = wgt.mul(diff);
				HitPoint *hp = new HitPoint(c1.Pos, c1.N, c1.From, c1.obj, x, y, diff_wgt);
				Hvec.push_back(hp);
			}
			if (obj->IsRefl())//keep tracing
			{
				Vec3 refl;
				obj->GetRefl(refl);
				ViewTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, flux.mul(refl), wgt.mul(refl), x, y);
			}
			if (obj->IsRefr())
			{
				//to be continued
				Vec3 refr;
				double rrn;
				obj->GetRefr(refr, rrn);
			}
		}
	}
	void debugHitPoint()
	{
		cout << "HitPoint:size:" << Hvec.size() << endl;
		for (auto e : Hvec)
		{
			cout << *e;
		}
	}
	void GetColor()//计算tao得到L并且得到最终camera的color
	{
		cout << "GetColor()" << endl;
		for (int i = 0; i < camera.h; i++)
		{
			for (int j = 0; j < camera.w; j++)
			{
				camera.pic[i][j] = Color(0, 0, 0);
			}
		}
		for (auto hp : Hvec)
		{
			//if (hp.tao.b != 0) cout << hp.x << " " << hp.y << " " << hp.tao.b << endl;
			
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!need to do
			//camera.pic[hp->x][hp->y] += hp->obj->color.mul(hp->tao*PI_INV / hp->R2);
			camera.pic[hp->x][hp->y] += hp->tao* (PI_INV /(hp->R2*1000));
		}
		double maxnum = 0;
		for (int i = 0; i < camera.h; i++)
		{
			for (int j = 0; j < camera.w; j++)
			{
				double c2 = camera.pic[i][j].Module();
				if (c2 > maxnum)
				{
					maxnum = c2;
				}
			}
		}
		for (int i = 0; i < camera.h; i++)
		{
			for (int j = 0; j < camera.w; j++)
			{
				camera.pic[i][j] /= maxnum*0.8;
			}
		}
	}
	void render()
	{
		GetHitPoint();
		BuildTree();
		//debugHitPoint();
		PhotonMap(3000);
		GetColor();
		show();
	}
	void show()
	{
		cout << "show" << endl;
		cv::Mat* m1 = new cv::Mat(camera.h, camera.w, CV_8UC3);
		for (int i = 0; i < camera.h; i++)
		{
			for (int j = 0; j < camera.w; j++)
			{
				//std::cout << i << " " << j << " " << camera.pic[i][j];
				Confine(camera.pic[i][j]);
				//std::cout << i << " " << j << " " << camera.pic[i][j];
				putxy(i, camera.w - j - 1, camera.pic[i][j] * 255, *m1);
			}
		}
		cv::namedWindow("test");
		cv::imshow("test", *m1);
		cvWaitKey(0);
	}
	void BuildTree()
	{
		cout << "Buildtree" << endl;
		kdtree.build(Hvec);
	}
};
int main()
{
	RayTracer r1;
	r1.init();
	r1.render();
	return 0;
}