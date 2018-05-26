#pragma warning(disable:4996)
#include <iostream>
#include <opencv2\highgui.hpp>
#include <algorithm>
#include <flann\flann.hpp>


using namespace std;
#define M_EPS 1e-10
#define RAND_R 10000000

#define Color Vec3
#define DEBUG
#define MAX_NUM 1e20
#define MIN_NUM -1e20
#define PI 3.1415926
#define PI_INV 0.3183098862
#define MIN_WGT 1e-3
#define MAX_DEP 17
#define INIT_R2 1.08
#define MAX_VEC3 (Vec3(MAX_NUM,MAX_NUM,MAX_NUM))
#define MIN_VEC3 (Vec3(MIN_NUM,MIN_NUM,MIN_NUM))
cv::RNG rng;
inline int rev(const int i, const int p) {
	if (i == 0) return i; else return p - i;
}
int primes[61] = {
	2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71,73,79,
	83,89,97,101,103,107,109,113,127,131,137,139,149,151,157,163,167,173,179,181,
	191,193,197,199,211,223,227,229,233,239,241,251,257,263,269,271,277,281,283
};
double hal(const int b, int j) {
	const int p = primes[b];
	double h = 0.0, f = 1.0 / (double)p, fct = f;
	while (j > 0) {
		h += rev(j % p, p) * fct; j /= p; fct *= f;
	}
	return h;
}
int halcnt = 0;
double qdata[3]{};

const double alpha = 0.7;//光子数量更新因子
double GetRandu(double r1, double r2)
{
	return rng.uniform(r1, r2);
}
double GetRandu()
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
	using cv::Vec3d::Vec;
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
	Vec3 GetUnit(){ double mo = Module(); return IsZero(mo) ? *this : *this * (1.0 / mo); }
	static Vec3 GetRandVec() { return Vec3(GetRandu(-1.,1.), GetRandu(-1., 1.), GetRandu(-1., 1.)); }
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
		return *this - 2 * this->dot(norm)*norm;
	}
	Vec3 GetUnitRefl(const Vec3& norm) const { return GetRefl(norm).GetUnit(); }
	static Vec3 GetUnitRandRefl(const Vec3& norm) {
		Vec3 res = GetRandVec();
		while (res.dot(norm) <= 0)
		{
			res = GetRandVec();
		}
		return res.GetUnit();
	}
	//Vec3 GetRefr(Vec3 N, double n, bool* refracted) {//得到折射方向，并且，判断是否已经折射过（穿过界面会再次折射）
	//	Vec3 V = GetUnit();
	//	double cosI = -N.dot(V), cosT2 = 1 - (n * n) * (1 - cosI * cosI);
	//	if (cosT2 > M_EPS) {
	//		if (refracted != NULL)
	//			*refracted ^= true;
	//		return Vec3(V * n + N * (n * cosI - sqrt(cosT2))).GetUnit();
	//	}//否则全反射
	//	return V.GetUnitRefl(N);
	//}
};
struct Ray { Vec3 org, dir; Ray() {}; Ray(Vec3 o, Vec3 d) : org(o), dir(d) {} }; // 射线
class Obj;
class Camera
{
public:
	Vec3 pos;//相机位置
			 //double fov_h;//相机张角
	double len_w, len_h;
	int w, h;//分辨率(w*h)
			 //double sr;//采样率
			 //double dis;//相机到画布的距离
	Vec3 dir;//相机朝向(单位矢量)
			 //Vec3 PicOrg;////相机到画布垂足的矢量
	Vec3 du, dv;//画布横纵单位矢量
	Color **pic;//画布
	Ray emit(int y,int x)
	{
		Vec3 d = du * ((x + 0.5) / w - 0.5) + dv * (-(y + 0.5) / h + 0.5) + dir;
		return Ray(pos + d * 140, d.GetUnit());
	}
	void init(int w_ = 1024, int h_ = 768, int fovAng = 45, const Vec3 &pos_ = Vec3(50, 48, 295.6), const Vec3&dir_ = Vec3(0, -0.042612, -1)) {
#ifdef DEBUG
		std::cout << "camera::init()!" << std::endl;
#endif // DEBUG
		len_w = .5135;
		len_h = .5135;
		dir = dir_;
		pos = pos_;
		dir.Normalize();

		//sr = 1.0 / 1;
		//fov_h = fovAng * acos(-1.0) / 180;

		h = h_;
		w = w_;

		du = Vec3(w*len_w / h);
		dv = Vec3(du.cross(dir)).GetUnit()*len_h;

		/*du.Normalize();
		du *= sr;

		dv = du.cross(dir);
		dv.Normalize();
		dv *= sr;
		dis = h * sr / 2 / tan(fov_h / 2);*/
		//PicOrg = dir*dis;
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
	bool IsFront() { return !inside; }
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
		double dis = ctemp.N.dot(op);
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
	SphereObj(double rradius, const  Vec3& orgg, const Color&cc, const Material& mm) :org(orgg), radius(rradius), Obj(mm, cc) {}
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
		if (IsZero(op.dot(op) - radius * radius)) ctemp.inside = true;
		else ctemp.inside = false;
		double t, b = op.dot(ray.dir), det = b * b - op.dot(op) + radius * radius;
		if (det < 0) { ctemp.dist = MAX_NUM; }
		else
		{
			det = sqrt(det);
			ctemp.dist = (t = b - det) > 1e-8 ? t : ((t = b + det) > 1e-8 ? t : MAX_NUM);
			ctemp.Pos = ray.org + ray.dir * t;
			ctemp.N = Vec3(ctemp.Pos - org).GetUnit();
			ctemp.inside = ctemp.N.dot(ray.dir) > 0;
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
		flux = color * PI * 4;
		ray.dir = Vec3::GetUnitRandVec();
		ray.org = pos;
	}
	virtual void emit(Ray & ray, Vec3 & flux, int i)//i用halton产生球状均匀的随机向量
	{
		flux = color * PI * 4;
		double p = 2.*PI*hal(0, i), t = 2.*acos(sqrt(1. - hal(1, i)));
		double st = sin(t);
		ray.dir = Vec3(cos(p)*st, cos(t), sin(p)*st);
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

		Obj* obj[] = {
			new SphereObj(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(.75,.25,.25),WHITE_DIFF),//Left
			new SphereObj(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(.25,.25,.75),WHITE_DIFF),//Right
			new SphereObj(1e5, Vec3(50,40.8, 1e5),     Vec3(.75,.75,.75),WHITE_DIFF),//Back
			new SphereObj(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),           WHITE_DIFF),//Front
			new SphereObj(1e5, Vec3(50, 1e5, 81.6),    Vec3(.75,.75,.75),WHITE_DIFF),//Bottomm
			new SphereObj(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(.75,.75,.75),WHITE_DIFF),//Top
			new SphereObj(16.5,Vec3(27,16.5,47),       Vec3(1,1,1)*.999, MIRROR),//Mirror
			new SphereObj(16.5,Vec3(73,16.5,88),       Vec3(1,1,1)*.999, REFR0),//Glass
			new SphereObj(8.5, Vec3(50,8.5,60),        Vec3(1,1,1)*.999, WHITE_DIFF)
		};
		
		for (int i = 0; i < 9; i++)
			objvec.push_back(obj[i]);
		plvec.push_back(new SpotLight(Vec3(50, 60, 85), Color(2500, 2500, 2500)));
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
	explicit HitPoint(const Vec3& pos_ = Vec3(), const Vec3& n_ = Vec3(), const Vec3& v_ = Vec3(), Obj* o = nullptr, int xx = -1, int yy = -1, const Color& wgt_ = Color()) :pos(pos_), n(n_), v(v_), obj(o), x(xx), y(yy), wgt(wgt_) {
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
int toInt(double x) {
	return int(pow(1 - exp(-x), 1 / 2.2) * 255 + .5);
}
class RayTracer
{
	int PhotonCnt = 0;
public:
	flann::Index<flann::L2<double> > index;
	RayTracer():index(flann::Matrix<double>(qdata, 1, 3), flann::KDTreeSingleIndexParams())
	{
	}
	Camera camera;
	Scene scene;
	vector<HitPoint*> Hvec;//用来保存视点碰撞点
	void GetHitPoint()//得到视点碰撞点
	{
		cout << "GetHitPoint():" << camera.pos << endl;
		for (int i = 0; i < camera.h; i++)
		{
			fprintf(stderr, "\rHitPointPass %5.2f%%", 100.0*i / (camera.h - 1));
			for (int j = 0; j < camera.w; j++)
			{
				//viewtrace
				ViewTrace(camera.emit(i, j), 0, Color(), Color(1, 1, 1), i, j);
			}
		}
		//debug
		cout << "hitpoint::" << Hvec.size() << endl;
	}
	void PhotonTrace(const Ray &ray, int dep, Color flux, Color wgt,int i)
	{
		Collision c1 = scene.NearCollide(ray);
		if (c1.IsValid())
		{
			if (dep>MAX_DEP)//control the depths
			{
				return;
			}
			dep++;
			// use QMC to sample the next direction
			int d3 = dep * 3;
			double r1 = 2.*PI*hal(d3 - 1, i), r2 = hal(d3, i);
			double r2s = sqrt(r2);
			Vec3 w = c1.IsFront()?c1.N:-c1.N, u = Vec3((fabs(w[0])>.1 ? Vec3(0, 1) : Vec3(1)).cross( w)).GetUnit();
			Vec3 v = w .cross( u), d = Vec3(u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).GetUnit();

			Obj * obj = c1.obj;
			if (obj->IsDiff())//change the flux,no need to go trace again
			{
				//update
				//UpdateK(c1,flux);
				Vec3 x = c1.Pos;
				for (int i = 0; i < 3; i++)
				{
					qdata[i] = x[i];
				}
				flann::Matrix<double> query(qdata, 1, 3);
				std::vector<std::vector< int> >indices;
				std::vector<std::vector< double> >dists;
				index.radiusSearch(query, indices, dists, INIT_R2, flann::SearchParams());
				for (auto e : indices)
				{
					for (auto e2 : e)
					{
						HitPoint *hp = Hvec[e2];
						Vec3 v = hp->pos - x;
						// check normals to be closer than 90 degree (avoids some edge brightning)
						if ((hp->n.dot(c1.N) > 1e-3) && (v.dot(v) <= hp->R2)) {
							// unlike N in the paper, hp->n stores "N / ALPHA" to make it an integer value
							double g = (hp->N*alpha + alpha) / (hp->N*alpha + 1.0);
							hp->R2 = hp->R2*g;
							hp->N++;
							hp->tao = (hp->tao + hp->wgt.mul(flux)*(1. / PI))*g;
						}
					}
				}

				//Roulette

				//again
				double p = 0;
				for (int i = 0; i < 3; i++)
				{
					if (c1.obj->color[i] > p) p = c1.obj->color[i];
				}
				if (GetRandu(0.0, 1.0)<p)  PhotonTrace(Ray(c1.Pos, d), dep, c1.obj->color.mul(flux)*(1. / p), wgt,i);
				//if (Obj::React::DIFF==obj->Roulette())//go on diff
				//{
				//	PhotonTrace(Ray(c1.Pos, Vec3::GetUnitRandRefl(c1.N)), dep, flux/sqrt(obj->diffr), wgt.mul(diff));
				//}
			}
			if (obj->IsRefl())//keep tracing
			{
				PhotonTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, c1.obj->color.mul(flux), c1.obj->color.mul(wgt),i);
			}
			if (obj->IsRefr())
			{
				//to be continued
				Ray lr(c1.Pos, c1.From.GetUnitRefl(c1.N));//   得到反射ray
				Vec3 refr;
				double rrn;
				obj->GetRefr(refr, rrn);
				bool into = c1.IsFront();
				double nt = c1.obj->material.refrn;

				double nc = 1.0, nnt = into ? nc / nt : nt / nc, ddn = ray.dir.dot(into ? c1.N : c1.N*-1), cos2t;//计算相对折射率,ddn为入射方向和法线方向的夹角，也即cos（入射角）																							 // total internal reflection
				if ((cos2t = 1 - nnt * nnt*(1 - ddn * ddn))<0) return PhotonTrace(lr, dep, flux, wgt,i);//全反射，cos2t为出射角的cos值的平方，即(cos^2)（出射角）

				Vec3 td = Vec3(ray.dir*nnt - c1.N*(into ? 1 : -1) * ((ddn*nnt + sqrt(cos2t)))).GetUnit();//折射定律得到出射方向
				double a = nt - nc, b = nt + nc, R0 = a * a / (b*b), c = 1 - (into ? -ddn : td.dot(c1.N));
				double Re = R0 + (1 - R0)*c*c*c*c*c, P = Re; Ray rr(c1.Pos, td); Vec3 fa = c1.obj->color.mul(wgt);
				
				(GetRandu(0.0, 1.0)<P) ? PhotonTrace(lr, dep, flux, fa,i) : PhotonTrace(rr, dep, flux, fa,i);
			}
		}
	}
	void PhotonMap(int cnt)
	{
		cout << "PhotonMap:" << cnt << endl;
		Ray ray;
		Color flux;
		for (int i = 0; i < cnt; i++)
		{
			double p = 100.*(i + 1) / cnt;

			fprintf(stderr, "\rPhotonPass %5.2f%%", p);
			int onetime = 1000;
			for (auto pl : scene.plvec)
			{
				for (int j = 0; j < onetime; j++)
				{
					int halindex = i * onetime + j;
					pl->emit(ray, flux,halindex);
					PhotonTrace(ray, 0, flux, Color(1, 1, 1),halindex);
				}
			}
			if (i % 100 == 1)
			{
				// density estimation
				for (int i1 = 0; i1 < camera.h; i1++)
				{
					for (int j = 0; j < camera.w; j++)
					{
						camera.pic[i1][j] = Vec3();
					}
				}
				for (auto hp : Hvec)
				{
					int x = hp->x;
					int y = hp->y;
					camera.pic[x][y] = camera.pic[x][y] + hp->tao*(1.0 / (PI*hp->R2*i*onetime));
				}
				cout << i << endl;
				// save the image after tone mapping and gamma correction
				cv::Mat m1(camera.h, camera.w, CV_8UC3, cvScalar(0, 0, 0));
				for (int i1 = 0; i1 < camera.h; i1++)
				{
					for (int j = 0; j < camera.w; j++)
					{
						for (int k = 0; k < 3; k++)
						{
							m1.ptr<uchar>(i1)[j * 3 + k] = toInt(camera.pic[i1][j][k]);
						}
					}
				}
				std::string p = std::string("halimage") + std::to_string(i / 100) + std::string(".jpg");
				cv::imwrite(p.c_str(), m1);
			}
		}
	}
	void ViewTrace(const Ray &ray, int dep, Color flux, Color wgt, int x, int y)
	{
		//cout << "ViewTrace:" << x << " " << y << endl;
		Collision c1 = scene.NearCollide(ray);
		if (c1.IsValid())
		{
			if (dep>MAX_DEP)//control the depths
			{
				return;
			}
			dep++;
			Obj * obj = c1.obj;
			if (obj->IsDiff())//change the wgt,no need to go trace again
			{
				HitPoint *hp = new HitPoint(c1.Pos, c1.N, c1.From, c1.obj, x, y, c1.obj->color.mul(wgt));
				Hvec.push_back(hp);
			}
			if (obj->IsRefl())//keep tracing
			{
				ViewTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, c1.obj->color.mul(flux), c1.obj->color.mul(wgt), x, y);
			}
			if (obj->IsRefr())
			{
				//to be continued
				Ray lr(c1.Pos, ray.dir.GetUnitRefl(c1.N));//   得到反射ray
				Vec3 refr;
				double rrn;
				obj->GetRefr(refr, rrn);
				bool into = c1.IsFront();
				double nt = c1.obj->material.refrn;
				double nc = 1.0, nnt = into ? nc / nt : nt / nc, ddn = ray.dir.dot(into ? c1.N : c1.N*-1), cos2t;//计算相对折射率,ddn为入射方向和法线方向的夹角，也即cos（入射角）

																										  // total internal reflection
				if ((cos2t = 1 - nnt * nnt*(1 - ddn * ddn))<0) return ViewTrace(lr, dep, flux, wgt, x, y);//全反射，cos2t为出射角的cos值的平方，即(cos^2)（出射角）

				Vec3 td = Vec3(ray.dir*nnt - c1.N*(into? 1:-1) * ((ddn*nnt + sqrt(cos2t)))).GetUnit();//折射定律得到出射方向
				double a = nt - nc, b = nt + nc, R0 = a * a / (b*b), c = 1 - (into ? -ddn : td.dot(c1.N));
				double Re = R0 + (1 - R0)*c*c*c*c*c, P = Re; Ray rr(c1.Pos, td); Vec3 fa = c1.obj->color.mul(wgt);
				// eye ray (trace both rays)
				ViewTrace(lr, dep, flux, fa*Re, x, y);
				ViewTrace(rr, dep, flux, fa*(1.0 - Re), x, y);

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
	void render()
	{
		GetHitPoint();
		BuildTree();
		//debugHitPoint();
		PhotonMap(10000);
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
		double* rdata = new double[3 * Hvec.size()];
		for (int i = 0; i < Hvec.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				rdata[i * 3 + j] = Hvec[i]->pos[j];
			}
		}
		flann::Matrix<double> dataset(rdata, Hvec.size(), 3);
		//flann::Index<flann::L2<double> > index(dataset, flann::KDTreeSingleIndexParams());
		index.buildIndex(dataset);
		std::cout << "build index done" << std::endl;
	}
};
int main()
{
	RayTracer r1;
	r1.camera.init();
	r1.scene.init();
	r1.render();
	return 0;
}