#pragma warning(disable:4996)


#include "Vec3.h"
#include "Obj.h"
#include "Camera.h"
#include "Element.h"
#include "Light.h"
#include "Scene.h"
#include "Raytracer.h"








int main()
{
	
	RayTracer r1;
	Camera camera;
	Vec3 org = Vec3(50, 35, 230);
	camera.init();
	camera.MoveTo(org);
	camera.lookAt(org + Vec3(0, 0.042612, -1), 1 / 1.);
	Scene scene;
	scene.init();

	/*r1.camera = &camera;
	r1.scene = &scene;
	r1.render();*/
	
	SPPMRayTracer r2;
	r2.render(scene, camera);

	

	system("pause");
	return 0;
}