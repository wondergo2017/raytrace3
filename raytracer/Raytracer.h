#pragma once
#include <flann\flann.hpp>
#include <iostream>
#include <vector>
#include "Scene.h"
#include "Camera.h"
using namespace std;

#define DEBUG

#define MIN_WGT 1e-3
#define MAX_DEP 13
#define INIT_R2 2.0
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
inline int toInt(double x) {
	return int(pow(1 - exp(-x), 1 / 2.2) * 255 + .5);
}

/*
inline Vector3 RANDONSPHERE() {
float u, v, r2;
do {
u = RAND() * 2 - 1;
v = RAND() * 2 - 1;
} while (u*u + v*v > 1);
r2 = u * u + v * v;
float x = 2 * u * sqrt(1 - r2);
float y = 2 * v * sqrt(1 - r2);
float z = 1 - 2 * r2;
return Normalize(Vector3(x, y, z));
}
*/

double qdata[3]{};

const double alpha = 0.7;//����������������

struct HitPoint//�ӵ���ײ��
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
	int newN=0;//�¼���Ĺ���
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
	void update()
	{
		//1 �õ�˥��ϵ��
		//2 ���°뾶����ͨ�����ۼƹ�������
		//3 ����¼ӹ���
		if (N <= 0 || newN <= 0) return;//���û���ۼ�����������ӵĹ���������ô���أ��������³���	
		double k = (N + alpha * newN) / (N + newN);	// �뾶˥������
		R2 *= k; tao *= k;		// �뾶��С����ͨ��������ɱ�������С
		N += alpha*newN;
		newN = 0;
	}
};

class RayTracer
{
	int PhotonCnt = 0;
	double searchR2 = INIT_R2;
	void updateR2(double r2)
	{
		searchR2 = r2;
	}

public:
	flann::Index<flann::L2<double> > index;
	std::vector<std::vector< int> >indices;
	std::vector<std::vector< double> >dists;
	flann::SearchParams searchparams = flann::SearchParams();
	RayTracer() :index(flann::Matrix<double>(qdata, 1, 3), flann::KDTreeSingleIndexParams())
	{
	}
	Camera *camera;
	Scene *scene;
	vector<HitPoint*> Hvec;//���������ӵ���ײ��
	void GetHitPoint()//�õ��ӵ���ײ��
	{
		cout << "GetHitPoint():" << camera->pos << endl;
		for (int i = 0; i < camera->h; i++)
		{
			fprintf(stderr, "\rHitPointPass %5.2f%%", 100.0*i / (camera->h - 1));
			for (int j = 0; j < camera->w; j++)
			{
				//viewtrace
				ViewTrace(camera->emit(i, j), 0, Color(1, 1, 1), i, j);
			}
		}
		//debug
		cout << "hitpoint::" << Hvec.size() << endl;
	}
	void PhotonTrace(const Ray &ray, int dep, Color flux)
	{
		//PASS 2
		//0 ������ȣ�����
		//1 �ҵ���������壬�õ���ײ��Ϣ�����û�У���ô����
		//2 ��������������壬��ô������ײ�㣨ֱ����ײ������������ɫ��
		//3 ��������������ɫ���ݹ���Ӵ���������ɫ,�����Ѿ����̶ģ����������ϵ����
		//4 ���̶ģ�����������ײ������Ϣ���У�������+�߹⣩�����߾��淴�䣬�������䣨��

		Collision c1 = scene->NearCollide(ray);
		if (c1.IsValid())
		{
			if (dep>MAX_DEP)//control the depths
			{
				return;
			}
			dep++;

			Obj * obj = c1.obj;
			if (obj->IsDiff())//change the flux,no need to go trace again
			{
				//update
				Vec3 x = c1.Pos;
				for (int i = 0; i < 3; i++)
				{
					qdata[i] = x[i];
				}
				flann::Matrix<double> query(qdata, 1, 3);

				index.radiusSearch(query, indices, dists, searchR2, searchparams);
				for (auto e : indices)
				{
					for (auto e2 : e)
					{
						HitPoint *hp = Hvec[e2];
						Vec3 v = hp->pos - x;
						// check normals to be closer than 90 degree (avoids some edge brightning)
						if ((hp->n.dot(c1.N) > 1e-3) && (v.dot(v) <= hp->R2)) {
							hp->newN++;
							hp->tao += hp->wgt.mul(flux)*PI_INV;
						}
					}
				}
			}
			//Roulette
			Obj::React Next = obj->Roulette();
			if (Next == Obj::React::DIFF)
			{
			PhotonTrace(Ray(c1.Pos, Vec3::GetUnitRandRefl(c1.N)), dep, c1.color.mul(flux));
			}
			/*if (obj->IsDiff())
			{
				double p = 0;
				for (int i = 0; i < 3; i++)
				{
					if (c1.obj->color[i] > p) p = c1.obj->color[i];
				}
				if (GetRandu(0.0, 1.0)<p)  PhotonTrace(Ray(c1.Pos, Vec3::GetUnitRandRefl(c1.N)), dep, c1.color.mul(flux)*(1. / p));
			}
*/
			if (Next == Obj::React::REFL)//keep tracing
				//if (obj->IsRefl())//keep tracing
			{
				PhotonTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, c1.color.mul(flux));
			}
			if (Next == Obj::React::REFR)
			/*if (obj->IsRefr())*/
			{
				Vec3 Refr;
				obj->GetRefr(Refr);
				double refrn = (c1.IsFront() ? 1 / obj->material.refrn : obj->material.refrn);//���������
				Vec3 refr = c1.From.refracted((c1.IsFront() ? c1.N : -c1.N), refrn);
				refr.Normalize();
				PhotonTrace(Ray(c1.Pos, refr), dep, c1.color.mul(flux));
				//to be continued
				//Ray lr(c1.Pos, c1.From.GetUnitRefl(c1.N));//   �õ�����ray
				//Vec3 refr;
				//double rrn;
				//obj->GetRefr(refr, rrn);
				//bool into = c1.IsFront();
				//double nt = c1.obj->material.refrn;

				//double nc = 1.0, nnt = into ? nc / nt : nt / nc, ddn = ray.dir.dot(into ? c1.N : c1.N*-1), cos2t;//�������������,ddnΪ���䷽��ͷ��߷���ļнǣ�Ҳ��cos������ǣ�																							 // total internal reflection
				//if ((cos2t = 1 - nnt * nnt*(1 - ddn * ddn))<0) return PhotonTrace(lr, dep, flux);//ȫ���䣬cos2tΪ����ǵ�cosֵ��ƽ������(cos^2)������ǣ�

				//Vec3 td = Vec3(ray.dir*nnt - c1.N*(into ? 1 : -1) * ((ddn*nnt + sqrt(cos2t)))).GetUnit();//���䶨�ɵõ����䷽��
				//double a = nt - nc, b = nt + nc, R0 = a * a / (b*b), c = 1 - (into ? -ddn : td.dot(c1.N));
				//double Re = R0 + (1 - R0)*c*c*c*c*c, P = Re; Ray rr(c1.Pos, td); Vec3 fa = c1.obj->color.mul(flux);

				//(GetRandu(0.0, 1.0)<P) ? PhotonTrace(lr, dep, fa) : PhotonTrace(rr, dep, fa);
			}
		}
	}
	void PhotonMap(int cnt)
	{
		//1 ����cnt��
		//2 ÿ�ַ���onetime ������
		//3 ÿ�ַ�����󣬸�������hitpoint ��tao�����Ҹ���kdtree�����뾶�����ͼ��
		cout << "PhotonMap:" << cnt << endl;
		Ray ray;
		Color flux;
		int onetime = 100;
		cout << "onetime:" << onetime << endl;
		for (int i = 0; i < cnt; i++)
		{
			double p = 100.*(i + 1) / cnt;

			fprintf(stderr, "\rPhotonPass %5.2f%%", p);
			
			for (auto pl : scene->plvec)
			{
				for (int j = 0; j < onetime; j++)
				{
					pl->emit(ray, flux);
					PhotonTrace(ray, 0, flux);
				}
			}
			//std::cerr << "update" << std::endl;
			//��������hitpoint����Ϣ,���������뾶
			
			double maxr2 = 0;
			for (auto hp : Hvec)
			{
				hp->update();
				if (hp->R2>maxr2)
				{
					maxr2 = hp->R2;
				}
			}
			//updateR2(maxr2);
			//std::cerr << "change r2 to " << maxr2 << std::endl;
			//std::cerr << "save" << std::endl;
			// density estimation
			/*for (int i1 = 0; i1 < camera->h; i1++)
			{
				for (int j = 0; j < camera->w; j++)
				{
					camera->pic[i1][j] = Vec3();
				}
			}
			for (auto hp : Hvec)
			{
				int x = hp->x;
				int y = hp->y;
				camera->pic[x][y] = camera->pic[x][y] + hp->tao*(1.0 / PI / (hp->R2*i*onetime));
			}
			cout << i*onetime << endl;
*/
			// save the image after tone mapping and gamma correction
			/*cv::Mat m1(camera->h, camera->w, CV_8UC3, cvScalar(0, 0, 0));
			for (int i1 = 0; i1 < camera->h; i1++)
			{
				for (int j = 0; j < camera->w; j++)
				{
					for (int k = 0; k < 3; k++)
					{
						m1.ptr<uchar>(camera->h - 1 - i1)[j * 3 + k] = toInt(camera->pic[i1][j][k]);
					}
				}
			}

			std::string pic = std::string("aabezir") + std::to_string(i / 100) + std::string(".jpg");
			cv::imwrite(pic.c_str(), m1);*/
		}
		for (int i1 = 0; i1 < camera->h; i1++)
		{
			for (int j = 0; j < camera->w; j++)
			{
				camera->pic[i1][j] = Vec3();
			}
		}
		for (auto hp : Hvec)
		{
			int x = hp->x;
			int y = hp->y;
			camera->pic[x][y] = camera->pic[x][y] + hp->tao*(1.0 / PI / (hp->R2*cnt*onetime));
		}
		cout << cnt * onetime << endl;
	}
	void ViewTrace(const Ray &ray, int dep, Color wgt, int x, int y)
	{
		//PASS 1
		//0 ���������ȣ�����
		//1 �ҵ���������壬�õ���ײ��Ϣ��û����ײ���򷵻�
		//2 �����ײ��������Ȩ�أ����ݷ����ϵ�������ݹ�ͼ�¼��ֱ�ӻ��������ɫϵ����
		//3 ��ײ���������������߸߹����ԣ���ô������ײ�㣬��������ݹ飨���淴������䣩
		Collision c1 = scene->NearCollide(ray);
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
				Vec3 diff;
				obj->GetDiff(diff);
				HitPoint *hp = new HitPoint(c1.Pos, c1.N, c1.From, obj, x, y, c1.color.mul(wgt).mul(diff));
				//HitPoint *hp = new HitPoint(c1.Pos, c1.N, c1.From, obj, x, y, c1.color.mul(wgt));
				Hvec.push_back(hp);
			}
			if (obj->IsRefl())//keep tracing
			{
				Vec3 Refl;
				obj->GetRefl(Refl);
				ViewTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, c1.color.mul(wgt).mul(Refl), x, y);
				//ViewTrace(Ray(c1.Pos, c1.From.GetUnitRefl(c1.N)), dep, c1.color.mul(wgt), x, y);
			}
			if (obj->IsRefr())
			{
				//to be continued
				Vec3 Refr;
				obj->GetRefr(Refr);
				double refrn=(c1.IsFront()?1/obj->material.refrn: obj->material.refrn);//���������
				Vec3 refr = c1.From.refracted((c1.IsFront()?c1.N:-c1.N), refrn);
				refr.Normalize();
				ViewTrace(Ray(c1.Pos, refr), dep, c1.color.mul(wgt).mul(Refr), x, y);
				
				////to be continued
				//Ray lr(c1.Pos, ray.dir.GetUnitRefl(c1.N));//   �õ�����ray
				//Vec3 refr;
				//double rrn;
				//obj->GetRefr(refr, rrn);
				//bool into = c1.IsFront();
				//double nt = c1.obj->material.refrn;
				//double nc = 1.0, nnt = into ? nc / nt : nt / nc, ddn = ray.dir.dot(into ? c1.N : c1.N*-1), cos2t;//�������������,ddnΪ���䷽��ͷ��߷���ļнǣ�Ҳ��cos������ǣ�

				//																								 // total internal reflection
				//if ((cos2t = 1 - nnt * nnt*(1 - ddn * ddn))<0) return ViewTrace(lr, dep, wgt, x, y);//ȫ���䣬cos2tΪ����ǵ�cosֵ��ƽ������(cos^2)������ǣ�

				//Vec3 td = Vec3(ray.dir*nnt - c1.N*(into ? 1 : -1) * ((ddn*nnt + sqrt(cos2t)))).GetUnit();//���䶨�ɵõ����䷽��
				//double a = nt - nc, b = nt + nc, R0 = a * a / (b*b), c = 1 - (into ? -ddn : td.dot(c1.N));
				//double Re = R0 + (1 - R0)*c*c*c*c*c, P = Re; Ray rr(c1.Pos, td); Vec3 fa = c1.obj->color.mul(wgt);
				//// eye ray (trace both rays)
				//ViewTrace(lr, dep, fa*Re, x, y);
				//ViewTrace(rr, dep, fa*(1.0 - Re), x, y);
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
		//build tree
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
		
		//debugHitPoint();
		PhotonMap(10000);

		delete[] rdata;
	}

	void show()
	{
		cout << "show" << endl;
		cv::Mat* m1 = new cv::Mat(camera->h, camera->w, CV_8UC3);
		for (int i = 0; i < camera->h; i++)
		{
			for (int j = 0; j < camera->w; j++)
			{
				//std::cout << i << " " << j << " " << camera->pic[i][j];
				Confine(camera->pic[i][j]);
				//std::cout << i << " " << j << " " << camera->pic[i][j];
				putxy(i, camera->w - j - 1, camera->pic[i][j] * 255, *m1);
			}
		}
		cv::namedWindow("test");
		cv::imshow("test", *m1);
		cvWaitKey(0);
	}
	~RayTracer()
	{
		for (auto hp:Hvec)
		{
			delete hp;
			hp = nullptr;
		}
		vector<HitPoint*>().swap(Hvec);
		camera = nullptr;
		scene = nullptr;
	}
	
};

class SPPMRayTracer {
public:
	void render(Scene scene,Camera camera)
	{
		int sppmtime = 100000;
		Camera c[4];
		for (int i = 0; i < sppmtime; i++)
		{
			RayTracer rt[4];
			for (int i = 0; i < 4; i++)
			{
				c[i] = camera;
				//cerr << "camera org:" << camera.pos << camera.dir << endl;
				c[i].MoveTo(camera.pos + 0.00015*Vec3::GetUnitRandVec());
				//c[i].MoveTo(camera.pos);
				c[i].lookAt(camera.look, 1.);

				//cerr << "camera c[i]:" << c[i].pos << c[i].dir << endl;
				rt[i].camera = &c[i];
				rt[i].scene = &scene;
			}
			#pragma omp parallel for num_threads(4)
			for (int j = 0; j < 4; j++)
			{
				rt[j].render();
				std::cerr << "Thread " << j << " done" << std::endl;
			}
			// save the image after tone mapping and gamma correction
			for (int i = 0; i < 4; i++)
			{
				for (int i1 = 0; i1 <camera.h; i1++)
				{
					for (int j = 0; j < camera.w; j++)
					{
						camera.pic[i1][j] += c[i].pic[i1][j];
					}
				}
			}
			
			cv::Mat m1(camera.h, camera.w, CV_8UC3, cvScalar(0, 0, 0));
			for (int i1 = 0; i1 < camera.h; i1++)
			{
			for (int j = 0; j < camera.w; j++)
			{
			for (int k = 0; k < 3; k++)
			{
			m1.ptr<uchar>(camera.h - 1 - i1)[j * 3 + k] = toInt(camera.pic[i1][j][k]/((i+1)*4.));
			}
			}
			}

			std::string pic = std::string("aaanew") + std::to_string(i/10) + std::string(".jpg");
			std::cerr << "save:" << pic << std::endl;
			cv::imwrite(pic.c_str(), m1);
		}
	}
};