#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include"our_gl.h"
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
Vec3f camera(2.f, 1.f, 3.f);
Vec3f center(0, 0.f, 1.f);//摄像机看向center
Vec3f light=Vec3f(0, 0, 1).normalize();
Vec3f up(0, 1, 0);

inline Matrix to_four_d(Vec3f v) {
	Matrix m(4, 1);
	m[0][0] = v.x;
	m[1][0] = v.y;
	m[2][0] = v.z;
	m[3][0] = 1.f;
	return m;
}
inline Vec3f to_v3(Matrix m) {
	return Vec3f(m[0][0]/ m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
}

struct GouraudShader : public IShader {
	Vec3f varying_intensity; // written by vertex shader, read by fragment shader
	Vec2i varying_uv[3];//u，v分别进行插值

	virtual Matrix vertex(int iface,int nthvert) {
		Matrix gl_Vertex = to_four_d(model->vert(iface,nthvert)); // read the vertex from .obj file
		gl_Vertex = Viewport * Perspective*ModelView*gl_Vertex;     // transform it to screen coordinates
		varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert)*light); // get diffuse lighting intensity
		varying_uv[nthvert] = model->uv(iface, nthvert);
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor &color) {
		float intensity = varying_intensity * bar;   // interpolate intensity for the current pixel
		float u = 0.0;
		float v = 0.0;
		for (int i = 0; i < 3; i++) {
			u += bar[i] * varying_uv[i].x;
			v += bar[i] * varying_uv[i].y;
		}
		color = model->diffuse(Vec2i(u,v))*intensity; // well duh
		return false;                              // no, we do not discard this pixel
	}
};
int main(int argc, char** argv) {
	model = new Model("obj/african_head/african_head.obj");

	view_trans(camera, center, up);
	//model里的顶点都在[-1,1]立方体内，相当于已经经过MVP变换
	//此项目透视变换：1、将所有点投影至z=center.z平面上。2、透视变换矩阵与摄像机位置有关。另：新点的z值与原来点的z值是正相关的，仍可用新点的z值来反映真实z值的大小，作zbuffer
	//与games101不同：1、透视矩阵是将棱台挤压成长方体，棱台内点也相应变化。2、games101在进行MVP变换后，才将顶点移至[-1,1]立方体内，摄像机的位置与角度在View transformation中已经确定
	//进行完模型变换视角变换后，模型的点已经不处于[-1,1]立方体内了，视口变换不能沿用之前的
	//原项目将视口矩阵改为从[-1,1]映射到[100,700](注:width=800)，这样即使带入大于1的值，得到的x也可能落在[700,800]这一段画布内,让顶点尽量不超出画布
	//坏处:x值域过大时,x仍会超出画布
	viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
	projection(-1.f / (camera - center).norm());
	TGAImage image(width,height, TGAImage::RGB);
	float* z_buffer = new float[width*height];
	for (int i = 0; i < width*height; i++) {
		z_buffer[i] = -std::numeric_limits<float>::max();
	}
	GouraudShader shader;
	for (int i = 0; i < model->nfaces(); i++) {
		Matrix screen_coords[3];
		std::vector<int> face = model->face(i);
		for (int j = 0; j < 3; j++) {
			screen_coords[j] = shader.vertex(i,j);
		}
		triangle(screen_coords, shader, image, z_buffer);
	}
	image.flip_vertically();
	image.write_tga_file("face.tga");
	delete model;
	return 0;
}