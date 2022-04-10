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
const int width = 1024;
const int height = 1024;
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

Vec3f barycentric(Vec3f* tri,Vec2i point) {//返回该点的重心坐标（x，y，z） 几何意义：（Sa/（Sa+Sb+Sc），...，...） 可用于插值
	Vec3f res = Vec3f(tri[1].x - tri[0].x, tri[2].x - tri[0].x, tri[0].x - point.x) ^ Vec3f(tri[1].y - tri[0].y, tri[2].y - tri[0].y, tri[0].y - point.y);
	if (std::abs(res.z) < 1e-2) {//防止res.z为0作为除数
		return Vec3f(-1, -1, -1);//返回一个非法的（在三角形外的点的）重心坐标
	}
	return Vec3f(1.f - (res.x + res.y) / res.z, res.x / res.z, res.y/ res.z);//重心坐标为(1-u-v,u,v)。res是和(u,v,1)平行的向量
}
//Vec3f barycentric(Vec3f* v, Vec2i point) { 验证了一下games101中 公式法求重心坐标 也正确
//	float x = point.x;
//	float y = point.y;
//	float c1 = (x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*y + v[1].x*v[2].y - v[2].x*v[1].y) / (v[0].x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*v[0].y + v[1].x*v[2].y - v[2].x*v[1].y);
//	float c2 = (x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*y + v[2].x*v[0].y - v[0].x*v[2].y) / (v[1].x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*v[1].y + v[2].x*v[0].y - v[0].x*v[2].y);
//	float c3 = (x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*y + v[0].x*v[1].y - v[1].x*v[0].y) / (v[2].x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*v[2].y + v[0].x*v[1].y - v[1].x*v[0].y);
//	return Vec3f(c1,c2,c3);
//}
void triangle(Vec3f* tri,Vec2i* txe,TGAImage &image, float light_k,float *z_buffer) {//这里把参数Vec2i改成Vec3i，把深度这一属性也带上，才能得到每个点插值后的深度
	//计算三角形的矩形边界
	int x_min = std::min(tri[0].x, std::min(tri[1].x, tri[2].x));
	int y_min = std::min(tri[0].y, std::min(tri[1].y, tri[2].y));
	int x_max = std::max(tri[0].x, std::max(tri[1].x, tri[2].x));
	int y_max = std::max(tri[0].y, std::max(tri[1].y, tri[2].y));
	for (int x = x_min; x <= x_max; x++) {
		for (int y = y_min; y <= y_max; y++) {
			Vec3f bct = barycentric(tri, Vec2i (x, y));
			float z = 0.0;
			if (bct.x < 0 || bct.y < 0 || bct.z < 0) continue;
			z += bct.x*tri[0].z;
			z += bct.y*tri[1].z;
			z += bct.z*tri[2].z;
			float u = 0.0;
			float v = 0.0;
			for (int i = 0; i < 3; i++) {
				u += bct[i] * txe[i].x;
				v += bct[i] * txe[i].y;
			}
			TGAColor c = model->diffuse(Vec2i(static_cast<int>(u), static_cast<int>(v)));
			if (z > z_buffer[x + y * width]) {
				z_buffer[x + y * width] = z;
				image.set(x, y,TGAColor(c.r*light_k, c.g*light_k, c.b*light_k,255));
			}
		}
	}
}

int main(int argc, char** argv) {
	model = new Model("obj/african_head/african_head.obj");
	Vec3f light(0, 0, -1);
	TGAImage image(width, height, TGAImage::RGB);
	float* z_buffer = new float[width*height];
	for (int i = 0; i < width*height; i++) {
		z_buffer[i] = -std::numeric_limits<float>::max();
	}
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		Vec3f screen_tri[3];//把深度属性z带上
		Vec2i tex_arr[3];
		Vec3f world_tri[3];
		for (int j = 0; j < 3; j++) {
			Vec3f v0 = model->vert(face[j]);
			world_tri[j] = v0;
			tex_arr[j] = model->uv(i, j);
			//model里的顶点都在[-1,1]立方体内了，相当于已经经过透视变换
			//之后再乘以视口变换矩阵即可
			int x0 = (v0.x + 1.) * width / 2.;
			int y0 = (v0.y + 1.) * height / 2.;
			screen_tri[j].x = x0;
			screen_tri[j].y = y0;
			screen_tri[j].z = v0.z;
		}
		Vec3f normal_vector =(world_tri[2]- world_tri[0])^ (world_tri[1] - world_tri[0]);
		normal_vector.normalize();
		float k = light * normal_vector.normalize();
		if (k > 0) {
			triangle(screen_tri, tex_arr, image,k,z_buffer);
		}
	}
	image.flip_vertically();
	image.write_tga_file("face.tga");
	delete model;
	return 0;
}