## 1 实验内容

### 1.1 算法选型：SPPM

​	PPM通过一次从视点方向的光线追踪获取碰撞点图，再通过多次光子映射更新碰撞点的光通量，最终通过每个碰撞点的光通量加权和得到画布的颜色。

​	![1530380903707](C:\Users\35742\AppData\Local\Temp\1530380903707.png)

​	每一个碰撞点都有自己的半径，接受领域范围内的光子，当光子越多，半径范围越小，最终收敛到很小的点以至于颜色值基本不变。因此，可以通过不断地发射光子，使得渲染的精度不断得到提升。

![1530381286460](C:\Users\35742\AppData\Local\Temp\1530381286460.png)

​	相比古老的RT，PPM能做到焦散等效果。

​	相比PM，PPM不需要用过多的内存保存光子，相反，只需要保存数量固定的碰撞点图。因此，PPM能通过不断地撒光子，只要时间充足，就能达到更好的效果。

​	而SPPM将相机射线随机抖动，通过多次发射PASS1，从而达到良好的抗锯齿效果。这个相当于一个在球上采样的过程，并且天然适合多线程加速。

### 1.2 附加功能

1. 三次Bezier曲面求交（牛顿迭代）
2. kdtree加速
3. 包围盒加速
4. OpenMP加速
5. 纹理贴图
6. 抗锯齿

### 1.3 附加功能对应代码

#### 1.3.1 三次Bezier曲面求交（牛顿迭代）

路径：Bezier.h

外循环maxIter*5次

内循环maxiter 次

内循环负责对于特定的初值，得到一个合理的迭代值

外层循环负责随机初值

最终得到对所有得到的值进行比较，返回距离最小的那个

Jacobi矩阵根据习题课课件写出，求逆使用opencv矩阵实现

```c++
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
```

#### 1.3.2 kdtree

路径:RayTracer.h

使用flann库实现

1）建树

```c++
double* rdata = new double[3 * Hvec.size()];
for (int i = 0; i < Hvec.size(); i++)
{
    for (int j = 0; j < 3; j++)
    {
        rdata[i * 3 + j] = Hvec[i]->pos[j];
    }
}
flann::Matrix<double> dataset(rdata, Hvec.size(), 3);
index.buildIndex(dataset);
```

2)查询

```c++
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
        if ((hp->n.dot(c1.N) > 1e-3) && (v.dot(v) <= hp->R2)) {
            hp->newN++;
            hp->tao += hp->wgt.mul(flux)*PI_INV;
        }
    }
}
```

#### 1.3.3 包围盒加速

路径：Bezier.h

简单的AABB包围盒，由于自己给Bezier迭代次数还比较多（最多50*10次），所以只对Bezier物体做包围盒，来避免过多无用的迭代。

每一个Bezier面都有一个包围盒，Bezier物体有一个大包围盒

求交先判断Bezier物体的包围盒有没有求交，再判断物体的每一个Bezier面的包围盒，再与Bezier迭代求交。

```c++
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
```

#### 1.3.4 OpenMP加速

路径：RayTracer.h

Scene是只读（并不对场景修改），所以每一个RayTracer只需以指针使用它

Camera保存了画布，而每一个RayTracer的摄像头不一样，所以给每一个RayTracer都按上自己的摄像头，4个线程结束后，再将图片融合然后保存，循环往复，这样就可以保证多个线程正常运转。

```c++
class SPPMRayTracer {
public:
	void render(Scene scene,Camera camera)
	{
		int sppmtime = 100000;
		Camera c[4];
		for (int i = 0; i < sppmtime; i++)
		{
			
            //...省略:生成临时的4个Raytracer//
			#pragma omp parallel for num_threads(4)
			for (int j = 0; j < 4; j++)
			{
				rt[j].render();
				std::cerr << "Thread " << j << " done" << std::endl;
			}
			//...省略：保存图片//
		}
	}
};
```

#### 1.3.5 纹理贴图

平面贴图只实现了法向是轴向的平面

球面贴图使用球极坐标变换

Bezier贴图将uv值直接得出就行了

贴图的插值借鉴了网上资料

##### 1.3.5.1 平面贴图

```c++
Color GetColor(const Vec3 &P) const
	{
		if(!texture) return this->color;
    //确定轴方向
		int ndir = 0;
		for (int i = 0; i < 3; i++)
		{
			if (norm.val[i] != 0) ndir = i;
		}
		int vdex = (ndir +2) % 3;
		int udex = (ndir + 1) % 3;
    //
		double v = 0.5 + (P.val[vdex] - P0.val[vdex]) / texU.Module();
		double u = 0.5 + (P.val[udex] - P0.val[udex]) / texV.Module();
		return texture->colorUV(u, v);
	}
```

##### 1.3.5.2 球面贴图

```c++
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
```

##### 1.3.5.3 Bezier 贴图

```c++
Color GetColor(double u,double v)
	{
		if (!texture) return this->color;
		return texture->colorUV(u, v);//通过Bezier的uv值直接得出
	}
```

#### 1.3.6 抗锯齿

通过随机晃动摄像机，从而得到随机抖动的碰撞点图，分别作出光子映射，最后融合得到图像，从而使得锯齿减少。

##### 1.3.6.1 前后对比效果

PPM：

![1530386013604](C:\Users\35742\AppData\Local\Temp\1530386013604.png)

SPPM:

![1530386093614](C:\Users\35742\AppData\Local\Temp\1530386093614.png)

##### 1.3.6.2 代码

```c++
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
				c[i].MoveTo(camera.pos + 0.00015*Vec3::GetUnitRandVec());//随机扰动
				c[i].lookAt(camera.look, 1.);
				rt[i].camera = &c[i];
				rt[i].scene = &scene;
			}
            //..省略：多线程render//
        }
    }
```

## 2 最终效果图

### 2.1 参数

1）初始收敛半径，这会对最后的噪声情况有影响

2）光源亮度，由于考虑焦散，不能通过归一化来调整亮度，需要手动设置一个合理的亮度

3）相机抖动的幅度，过小会导致无法抗锯齿，过大会使得图片模糊

4）Bezier迭代次数，过少会使得Bezier收敛错误，过大则速度过慢

5）光子数

6）画布大小与相机位置

最终设定

1)初始收敛半径1.（场景借鉴自smallppm，差不多80* 80* 80）

2)光源亮度5000

3)相机抖动幅度0.00015

4)Bezier迭代次数50*10

5)光子数总计50M

6)画布1024*1024

### 2.2 图

![](/raytracer/大理石.jpg)
$$
大理石.jpg
$$

## 3 参考

smallppm.cpp

ppm.pdf 

https://www.scratchapixel.com/index.php?redirect 

经典的teapot

以及其他各种网上资料

