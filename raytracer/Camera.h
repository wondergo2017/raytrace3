#pragma once
#include "Vec3.h"
#include "Element.h"
class Camera
{
public:
	Vec3 pos;//相机位置
	Vec3 look;
	double len_w, len_h;
	int w, h;//分辨率(w*h)
	Vec3 dir;//相机朝向(单位矢量)
	Vec3 du, dv;//画布横纵单位矢量
	Color **pic=nullptr;//画布
	Camera() {};
	static const int DEFAULT_LENS =100;
	static const int 	 DEFAULT_HEIGHT = 1024;
	static const int 	 DEFAULT_WIDTH = 1024;
	Ray emit(int y, int x)
	{
		Vec3 d = du * ((x+0.5) / w - 0.5) + dv * ((y+0.5)/ h -0.5) + dir;
		return Ray(pos  , d.GetUnit());
	}
	void init(int w_=DEFAULT_WIDTH , int h_=DEFAULT_HEIGHT) {
		h = h_;
		w = w_;
		
	}
	void MoveTo(const Vec3 &pos_)
	{
		pos = pos_;
	}
	void lookAt(const Vec3 &P, double scale)
	{
		look = P;
		const Vec3 up(0, 0, 1);
		w *= scale;
		h *= scale;
		dir = Vec3(look - pos).GetUnit();
		du = dir.cross(up);
		du.Normalize();
		dv = -dir.cross(du);
		dv.Normalize();

		double fov = 50.*PI/180.;//视角
		dir *= .5 / tan(fov / 2.);
		if (!pic)
		{
			pic = new Color*[h];
			for (int i = 0; i < h; i++)
			{
				pic[i] = new Color[w];
			}
		}
	}
	Camera(const Camera & c)
	{
		look = c.look;
		pos = c.pos;
		len_w = c.len_w;
		len_h = c.len_h;
		w = c.w;
		h = c.h;
		dir = c.dir;
		du = c.du;
		dv = c.dv;
		if (!pic)
		{
			pic = new Color*[h];
			for (int i = 0; i < h; i++)
			{
				pic[i] = new Color[w];
			}
		}
	}
	Camera & operator=(const Camera& c)
	{
		look = c.look;
		pos = c.pos;
		len_w = c.len_w;
		len_h = c.len_h;
		w = c.w;
		h = c.h;
		dir = c.dir;
		du = c.du;
		dv = c.dv;
		if (!pic)
		{
			pic = new Color*[h];
			for (int i = 0; i < h; i++)
			{
				pic[i] = new Color[w];
			}
		}
		return *this;
	}
	void clear()
	{
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				pic[i][j] = Color();
			}
		}
	}
	~Camera()
	{
	for (int i = 0; i < h; i++)
		{
			delete[] pic[i];
		}
	delete[] pic;
	}
};