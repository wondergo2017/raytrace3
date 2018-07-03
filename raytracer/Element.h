#pragma once
#include <opencv2\highgui.hpp>
#include <vector>
#include <string>
extern class Obj;
struct Ray { Vec3 org, dir; Ray() {}; Ray(Vec3 o, Vec3 d) : org(o), dir(d) {} }; // 射线
class Material // 材质
{
public:
	Material(Vec3 d, Vec3 s, Vec3 r, double rr = 1.5, double rl = 1) : diff(d), refl(s), refr(r), refrn(rr), refln(rl) {}
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
	Color color = Vec3();
	double dist;//距离
	explicit Collision(const Vec3& pos = Vec3(), const Vec3& from = Vec3(), double d = 0, Obj* o = nullptr, const Vec3& n = Vec3(),const Vec3& c=Vec3()) :Pos(pos), N(n), From(from)
	{
		dist = d;
		obj = o;
		color = c;
	}
	bool inside = 0;
};
const Collision NoneColl=Collision();//表示没有碰撞
enum OBJ_REL { INSIDE, OUTSIDE, ON, UNKNOWN };//和物体的相对位置

//借鉴网上资料
struct Texture
{
public:
	int rows, cols;
	Texture(const std::string &fileName)
	{
		cv::Mat_<cv::Vec3b> map = cv::imread(fileName);
		rows = map.rows;
		cols = map.cols;
		m_map.resize(rows);
		for (int i = 0; i < rows; i++) m_map[i].resize(cols);

		for (int i = 0; i < rows; i++) for (int j = 0; j < cols; j++)
			for (int co = 0; co < 3; co++)
				m_map[i][j][co] = map(i, j)[co] / 255.0;
		std::cout << "Texture:" << fileName << "::row:" << rows << " cols:" << cols << std::endl;
	}
	
	Color colorUV(double u, double v) const
	{
		double row = (u - floor(u)) * rows, col = (v - floor(v)) * cols;
		int r1 = (int)floor(row + 1e-10), r2 = r1 + 1;
		int c1 = (int)floor(col + 1e-10), c2 = c1 + 1;
		double detR = r2 - row, detC = c2 - col;
		r1 = (r1 >= 0) ? (r1 >= rows ? 0 : r1) : (rows - 1);
		c1 = (c1 >= 0) ? (c1 >= cols ? 0 : c1) : (cols - 1);
		r2 = (r2 < rows) ? r2 : 0;
		c2 = (c2 < cols) ? c2 : 0;
		return m_map[r1][c1] * detR * detC+ m_map[r1][c2] * detR * (1 - detC)+ m_map[r2][c1] * (1 - detR) * detC+ m_map[r2][c2] * (1 - detR) * (1 - detC);
	}
private:
	std::vector<std::vector<Vec3>> m_map;//保存纹理信息
};



