#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include<vector>
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 1000;
const int height = 1000;
Model* model = nullptr;
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
	//Bresenham��s line algorithm(�Ƶ����̻���0<k<1)
	//key:(x,y)����һ����ֻ�п�����(x,y)��(x,y��1)
	bool steep = false;
	if (std::abs(x1 - x0) < std::abs(y1 - y0)) {
		steep = true;
		std::swap(x0, y0);
		std::swap(y1, x1);
	}
	if (x0 > x1) {//����forѭ���Ǵ�x0��x1�ģ�ȷ��x0<x1
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = y1 - y0;
	int delta_y = 1;
	if (dy < 0) {//б��Ϊ�������Ҳͳһ����Ϊ��б��������ֻҪ��y�ı仯��Ϊ-1
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
void test(TGAImage& image) {
	line(13, 20, 80, 40, image, white);
	line(20, 13, 40, 80, image, red);
	line(80, 40, 13, 20, image, red);
	line(0, 0, 100, 100, image, green);
}
int main(int argc, char** argv) {
	//TGAImage image(100, 100, TGAImage::RGB);
	//test(image);
	model = new Model("obj/african_head/african_head.obj");

	TGAImage image(width, height, TGAImage::RGB);
	for (int i = 0; i < model->nfaces(); i++) {
		std::vector<int> face = model->face(i);
		for (int j = 0; j < 3; j++) {
			Vec3f v0 = model->vert(face[j]);
			Vec3f v1 = model->vert(face[(j + 1) % 3]);
			//�����൱�ڳ����ӿڱ任����
			int x0 = (v0.x + 1.) * width / 2.;
			int y0 = (v0.y + 1.) * height / 2.;
			int x1 = (v1.x + 1.) * width / 2.;
			int y1 = (v1.y + 1.) * height / 2.;
			line(x0, y0, x1, y1, image, white);
		}
	}
	image.flip_vertically();
	image.write_tga_file("face.tga");
	return 0;
}