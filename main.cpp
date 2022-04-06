#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include<vector>
#include<iostream>
#include<algorithm>
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 1000;
const int height = 1000;
Model* model = nullptr;
std::vector<int> line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color,bool is_record=false) {
	std::vector<int> res;
	if (is_record) {
		res.resize(std::max(y0,y1)+1);
	}

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
			if (is_record) {
				res[x] = y;
			}
		}
		else {
			image.set(x, y, color);
			if (is_record) {
				res[y] = x;
			}
		}
		error += dy;
		if (error * 2 >= dx) {
			y += delta_y;
			error -= dx;
		}
	}
	return res;
}
inline std::vector<int> line(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color, bool is_record = false) {
	return line(t0.x, t0.y, t1.x, t1.y, image, color,is_record);
}
void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
	if (t0.y > t1.y) std::swap(t0, t1);
	if (t0.y > t2.y) std::swap(t0, t2);
	if (t1.y > t2.y) std::swap(t1, t2); 
	std::vector<int> y_low=line(t0 ,t1, image, color, true);//在上面重载一下line函数
	std::vector<int> y_high=line(t1, t2, image, color, true);//修改：让line函数返回一个数组，记录绘制的坐标点
	std::vector<int> y_pub=line(t2, t0, image, color, true);
	for (int y = t0.y; y < t1.y; y++) {
		line(y_pub[y], y, y_low[y], y,image,color);
	}
	for (int y = t1.y; y <= t2.y; y++) {
		line(y_pub[y], y, y_high[y], y, image, color);
	}

}

//void pop_sort(Vec2i* t) { 这段冒泡排序可以放triangle里
//	for (int i = 0; i < 3; i++) {
//		for (int j = i + 1; j < 3; j++) {
//			if (t[i].raw[1] > t[j].raw[1]) {
//				std::swap(t[i], t[j]);
//			}
//		}
//	}
//}
//
int main(int argc, char** argv) {
	TGAImage image1(200,200, TGAImage::RGB);
	Vec2i t0[3] = { Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80) };
	Vec2i t1[3] = { Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180) };
	Vec2i t2[3] = { Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180) };
	//pop_sort(t0);
	//pop_sort(t1);
	//pop_sort(t2);
	triangle(t0[0], t0[1], t0[2], image1, red);
	triangle(t1[0], t1[1], t1[2], image1, white);
	triangle(t2[0], t2[1], t2[2], image1, green);
	image1.flip_vertically();
	image1.write_tga_file("test.tga");

	model = new Model("obj/african_head/african_head.obj");

	TGAImage image(width, height, TGAImage::RGB);
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		for (int j = 0; j < 3; j++) {
			Vec3f v0 = model->vert(face[j]);
			Vec3f v1 = model->vert(face[(j + 1) % 3]);
			//model里的顶点都在[-1,1]立方体内了，相当于已经经过透视变换
			//之后再乘以视口变换矩阵即可
			int x0 = (v0.x + 1.) * width / 2.;
			int y0 = (v0.y + 1.) * height / 2.;
			int x1 = (v1.x + 1.) * width / 2.;
			int y1 = (v1.y + 1.) * height / 2.;
			line(x0, y0, x1, y1, image, white);
		}
	}
	image.flip_vertically();
	image.write_tga_file("face.tga");
	delete model;
	return 0;
}