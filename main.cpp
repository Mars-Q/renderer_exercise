#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include<vector>
#include<iostream>
#include<algorithm>
#include<random>
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 1000;
const int height = 1000;
Model* model = nullptr;
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {

	//Bresenham’s line algorithm(推导过程基于0<k<1)
	//key:(x,y)后下一个点只有可能是(x,y)或(x,y±1)
	bool steep = false;
	if (std::abs(x1 - x0) < std::abs(y1 - y0)) {
		steep = true;
		std::swap(x0, y0);
		std::swap(y1, x1);
	}

	if (x0 > x1) {//下面for循环是从x0到x1的，确保x0<x1
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = y1 - y0;
	int delta_y = 1;
	if (dy < 0) {//斜率为负的情况也统一视作为正斜率来处理，只要让y的变化量为-1
		dy = -dy;
		delta_y = -1;
	}
	int error = 0;
	int y = y0;
	for (int x = x0; x <= x1; x++) {
		if (steep) {
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
		error += dy;
		if (error * 2 >= dx) {
			y += delta_y;
			error -= dx;
		}
	}
}
inline void line(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color) {
	return line(t0.x, t0.y, t1.x, t1.y, image, color);
}

Vec3f barycentric(Vec2i* tri,Vec2i point) {//返回该点的重心坐标（x，y，z） 几何意义：（Sa/（Sa+Sb+Sc），...，...） 可用于插值
	Vec3f res = Vec3f(tri[1].x - tri[0].x, tri[2].x - tri[0].x, tri[0].x - point.x) ^ Vec3f(tri[1].y - tri[0].y, tri[2].y - tri[0].y, tri[0].y - point.y);
	if (std::abs(res.z) < 1) {//防止res.z为0作为除数
		return Vec3f(-1, -1, -1);//返回一个非法的（在三角形外的点的）重心坐标
	}
	return Vec3f(1.f - (res.x + res.y) / res.z, res.x / res.z, res.y/ res.z);//重心坐标(1-u-v,u,v) res是和(u,v,1)平行的向量
}
void triangle(Vec2i* tri, TGAImage &image, TGAColor color) {
	line(tri[0] ,tri[1], image, color);
	line(tri[1], tri[2], image, color);
	line(tri[2], tri[0], image, color);
	//计算三角形的矩形边界
	int x_min = std::min(tri[0].x, std::min(tri[1].x, tri[2].x));
	int y_min = std::min(tri[0].y, std::min(tri[1].y, tri[2].y));
	int x_max = std::max(tri[0].x, std::max(tri[1].x, tri[2].x));
	int y_max = std::max(tri[0].y, std::max(tri[1].y, tri[2].y));
	for (int x = x_min; x <= x_max; x++) {
		for (int y = y_min; y <= y_max; y++) {
			Vec3f bct = barycentric(tri, Vec2i (x, y));
			if (bct.x < 0 || bct.y < 0 || bct.z < 0) continue;
			image.set(x, y, color);
		}
	}
}

int main(int argc, char** argv) {
	TGAImage image1(200,200, TGAImage::RGB);
	Vec2i t0[3] = { Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80) };
	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
	Vec2i t2[3] = { Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180) };
	triangle(t0, image1, red);
	triangle(t1, image1, white);
	triangle(t2, image1, green);
	image1.flip_vertically();
	image1.write_tga_file("test.tga");

	model = new Model("obj/african_head/african_head.obj");
	Vec3f light(0, 0, -1);
	TGAImage image(width, height, TGAImage::RGB);
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		Vec2i tri[3];
		Vec3f tri_withZ[3];
		for (int j = 0; j < 3; j++) {
			Vec3f v0 = model->vert(face[j]);
			tri_withZ[j] = v0;
			//Vec3f v1 = model->vert(face[(j + 1) % 3]);
			//model里的顶点都在[-1,1]立方体内了，相当于已经经过透视变换
			//之后再乘以视口变换矩阵即可
			int x0 = (v0.x + 1.) * width / 2.;
			int y0 = (v0.y + 1.) * height / 2.;
			//int x1 = (v1.x + 1.) * width / 2.;
			//int y1 = (v1.y + 1.) * height / 2.;
			//line(x0, y0, x1, y1, image, white); 在线上的点重心坐标也非负，在triangle里也会画，就不必重复画了
			tri[j].x = x0;
			tri[j].y = y0;
		}
		Vec3f normal_vector =(tri_withZ[2]-tri_withZ[0])^ (tri_withZ[1] - tri_withZ[0]);
		normal_vector.normalize();
		float k = light * normal_vector.normalize();
		if (k > 0) {
			triangle(tri, image, TGAColor(k * 255, k * 255, k * 255, 255));
		}
	}
	image.flip_vertically();
	image.write_tga_file("face.tga");
	delete model;
	return 0;
}