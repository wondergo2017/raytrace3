#pragma once
#include <vector>
#include "Element.h"
#include "Vec3.h"
#include "Obj.h"
#include "Light.h"
#include "Bezier.h"
//class Scene
//{
//public:
//	std::vector<Obj*> objvec;
//	std::vector<Light*> plvec;
//	void init() {
//		Color WHITE(1, 1, 1);
//		Material WHITE_DIFF(Vec3(.75, .75, .75), Vec3(), Vec3(), 0);
//		Material BLACK_DIFF(Vec3(), Vec3(), Vec3(), 0);
//		Material RED_DIFF(Vec3(.75, .3, .3), Vec3(), Vec3(), 0);
//		Material BLUE_DIFF(Vec3(.3, .3, .75), Vec3(), Vec3(), 0);
//		Material LIGHT_BLUE_DIFF(Vec3(0.45, 0.45, .75), Vec3(), Vec3(), 0);
//		// Material LIGHT_BLUE_DIFF2(Vec3(0.75, 0.6, .65), Vec3(), Vec3(), 0);
//		Material LIGHT_RED_DIFF(Vec3(0.705, 0.45, .45), Vec3(), Vec3(), 0);
//		Material MIRROR(Vec3(), Vec3(1, 1, 1)*.999, Vec3(), 0);
//		Material MIRROR2(Vec3(.25, .25, .25), Vec3(1, 1, 1)*.999, Vec3(), 0);
//		Material REFR0(Vec3(), Vec3(), Vec3(1, 1, 1)*.999, 1.2);
//		Material REFR1(Vec3(), Vec3(), Vec3(.75, .999, .999), 1.5);
//		Material REFR2(Vec3(), Vec3(), Vec3(.999, .75, .75), 1.5);
//		Material REFR3(Vec3(), Vec3(), Vec3(.75, .999, .75), 1.5);
//		Material REFR4(Vec3(), Vec3(), Vec3(.999, .75, .999), 1.5);
//
//		//Obj* obj[] = {
//		////new SphereObj(1e5, Vec3(1e5 + 1,40.8,81.6), Vec3(.75,.25,.25),WHITE_DIFF),//Left
//		//new SphereObj(1e5, Vec3(-1e5 + 99,40.8,81.6),Vec3(.25,.75,.25),WHITE_DIFF),//Right
//		//new SphereObj(1e5, Vec3(50,40.8, 1e5),     Vec3(1,1,1),WHITE_DIFF),//Back
//		////new SphereObj(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),           WHITE_DIFF),//Front
//		//new SphereObj(1e5, Vec3(50, 1e5, 81.6),    Vec3(1,1,1),WHITE_DIFF),//Bottomm
//		////new SphereObj(1e5, Vec3(50,-1e5 + 81.6,81.6),Vec3(.25,.25,.75),WHITE_DIFF),//Top
//		//new SphereObj(16.5,Vec3(50,16.5,47),       Vec3(1,1,1)*.999, MIRROR),//Mirror
//		//new SphereObj(12.5,Vec3(90,12.5,88),       Vec3(1,1,1)*.999, REFR0),//Glass
//		//new SphereObj(4.5, Vec3(30,4.5,60),        Vec3(0,1,1)*.999, MIRROR2),
//		//new SphereObj(4.5, Vec3(30,13.5,60),        Vec3(0,0,1)*.999, RED_DIFF)
//		//};
//		
//		double sratio = 1;
//		objvec.push_back(new PlaneObj(sratio*Vec3(100, 100, 0), Vec3(0, 0, 1), WHITE_DIFF, Vec3(.75, .75, .75)));//bottom
//		//Texture *wood = new Texture("wood2.jpg");
//		//objvec[0]->setTexture(wood);
//		
//		//objvec.push_back(new SphereObj(10, Vec3(100, 100, 10), WHITE, MIRROR));
//		//objvec.push_back(new SphereObj(20, Vec3(130, 130,20 ), WHITE, WHITE_DIFF));
//		//Texture *blue = new Texture("blue.jpg");
//		//objvec[2]->setTexture(blue);
//	
//		objvec.push_back(new PlaneObj(sratio*Vec3(100, 100, 200), Vec3(0, 0, -1), WHITE_DIFF, Vec3(.75, .75, .75)));//up
//		objvec.push_back(new PlaneObj(sratio*Vec3(0, 100, 100), Vec3(1, 0, 0), WHITE_DIFF, Vec3(0.25,0.25,.75)));//left
//		objvec.push_back(new PlaneObj(sratio*Vec3(200,100, 100), Vec3(-1, 0, 0), WHITE_DIFF, Vec3(.75,0.25,0.25)));//right
//		objvec.push_back(new PlaneObj(sratio*Vec3(100, 200, 100), Vec3(0, -1, 0), WHITE_DIFF, Vec3(.75, .75, .75)));//front
//		//objvec.push_back(new PlaneObj(Vec3(100, 0, 100), Vec3(0, 1, 0), WHITE_DIFF, WHITE));//back
//
//		//objvec.push_back(new SphereObj(1e5,Vec3(100, 100, 1e5+200),WHITE, WHITE_DIFF));//up
//		//objvec.push_back(new SphereObj(1e5,Vec3(0-1e5, 100, 100), Vec3(0, 0, .75),WHITE_DIFF));//left
//		//objvec.push_back(new SphereObj(1e5,Vec3(200+1e5,100, 100), Vec3(.75, 0, 0), WHITE_DIFF));//right
//		//objvec.push_back(new SphereObj(1e5,Vec3(100, 200+1e5, 100), WHITE, WHITE_DIFF));//front
//		//objvec.push_back(new SphereObj(1e5,Vec3(100, 0-1e5, 100), WHITE, WHITE_DIFF));//back
//
//		objvec.push_back(new SphereObj(sratio*30, sratio*Vec3(150,80, 30), WHITE*0.99, REFR0));
//
//		plvec.push_back(new SpotLight(sratio*Vec3(100, 100, 150), .5*2500 * Color(1, 1, 1)*0.8));
//		plvec.push_back(new SpotLight(sratio*Vec3(70, 130, 80), .5*2500 * Color(1, 1, 1)*0.8));
//#ifdef DEBUG
//		std::cout << "scene::init()!" << std::endl;
//#endif // DEBUG
//
//	}
//	Collision NearCollide(const Ray & ray)//找到最近的碰撞点和碰撞物体
//	{
//		Obj*e = nullptr;
//		double mindist = MAX_NUM;
//		Collision ctmp;
//		for (auto a : objvec) {
//			Collision temppoint = a->GetIntersect(ray);
//			if (temppoint.IsValid())//没碰到则不计入
//			{
//				if (temppoint.dist < mindist)
//				{
//					mindist = temppoint.dist;
//					ctmp = temppoint;
//				}
//			}
//		}
//		return ctmp;
//	}
//};
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
		Material REFR4(Vec3(.2,.2,.2), Vec3(), Vec3(0, .5, .5), 1.5);

		//场景坐标系：x左（越大越左），y上（越大越竖直往上），z里（越大越往视角反方向），左手系
		Obj* obj[] = {
			new PlaneObj(Vec3(1,40.8,81.6),Vec3(1,0,0), Vec3(.75,.25,.25),WHITE_DIFF),//Left
			new PlaneObj(Vec3(99,40.8,81.6),Vec3(1,0,0),Vec3(.25,.25,.75),WHITE_DIFF),//Right
			new PlaneObj(Vec3(50,40.8, 0),Vec3(0,0,1),Vec3(.75,0.75,.75),MIRROR),//Back
			//new SphereObj(1e5, Vec3(50,40.8,-1e5 + 170), Vec3(),           WHITE_DIFF),//Front
			new PlaneObj( Vec3(50, 0, 81.6),Vec3(0,-1,0),    Vec3(.75,.75,.75),WHITE_DIFF),//Bottomm
			new PlaneObj(Vec3(50,81.6,81.6),Vec3(0,1,0),Vec3(.75,.75,.75),WHITE_DIFF),//Top
			new SphereObj(16.5,Vec3(27,16.5,47),       Vec3(1,1,1)*.999, MIRROR),//Mirror
			new SphereObj(16.5,Vec3(73,16.5,88),       Vec3(1,1,1)*.999, REFR0),//Glass
			new SphereObj(8.5, Vec3(50,8.5,60),        Vec3(1,1,1)*.999, WHITE_DIFF)
			
		};
		
		for (int i = 0; i < 8; i++)
			objvec.push_back(obj[i]);
		Texture *timg = new Texture("timg.jpg");
		Texture *stone = new Texture("wall.jpg");
		
		objvec[0]->setTexture(stone);
		objvec[1]->setTexture(stone);
		objvec[4]->setTexture(stone);

		Texture *planet = new Texture("planet.jpg");
		objvec[7]->setTexture(planet);
		
		objvec[3]->setTexture(timg);
		cv::Matx33d Trans;
		Trans(0, 0) = 1;
		Trans(1, 2) = 1;
		Trans(2, 1) = 1;
		cv::Matx33d Trans2;
		double theta = (90. / 180.)*PI;
		Trans2(0, 0) = cos(theta);
		Trans2(0, 2) = sin(theta);
		Trans2(1, 1) = 1;
		Trans2(2, 0) = -sin(theta);
		Trans2(2, 2) = cos(theta);
		Bezier3Obj* test=new Bezier3Obj(Vec3(20,0,120),4,"teapot.bpt", Vec3(0, 1, 1)*.999,WHITE_DIFF,Trans2*Trans);
		objvec.push_back(test);
		Texture *blue = new Texture("blue.jpg");
		objvec[8]->setTexture(blue);
		plvec.push_back(new SpotLight(Vec3(50, 60, 85), 5000*Color(1, 1, 1)));
		//plvec.push_back(new SpotLight(Vec3(10, 40, 140), 2000 * Color(0, 1, 1)));
		//plvec.push_back(new SpotLight(Vec3(50, 50, 85), 2500 * Color(1, 1, 1)));
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
