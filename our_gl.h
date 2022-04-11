#ifndef __OUR_GL_H__
#define __OUR_GL_H__

#include "tgaimage.h"
#include "geometry.h"
#include<algorithm>

extern Matrix ModelView;
extern Matrix Viewport;
extern Matrix Perspective;

void viewport(int x, int y, int w, int h);
void projection(float coeff = 0.f); // coeff = -1/c
void view_trans(Vec3f camera, Vec3f center, Vec3f up);
struct IShader {
	virtual ~IShader();
	virtual Matrix vertex(int iface, int nthvert) = 0;//顶点着色器
	virtual bool fragment(Vec3f bar, TGAColor &color) = 0;//像素着色器，接收像素的质心坐标对其进行插值着色
};

void triangle(Matrix* m, IShader &shader, TGAImage &image, float* zbuffer);
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);
inline void line(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color) {
	line(t0.x, t0.y, t1.x, t1.y, image, color);
	return;
}


#endif //__OUR_GL_H__
