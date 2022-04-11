#include <cmath>
#include <limits>
#include <cstdlib>
#include "our_gl.h"

Matrix ModelView;
Matrix Viewport;
Matrix Perspective;

IShader::~IShader() {}
void viewport(int x, int y, int w, int h) {
	Viewport = Matrix::identity(4);
	Viewport[0][3] = x + w / 2.f;
	Viewport[1][3] = y + h / 2.f;
	Viewport[2][3] = 255.f / 2.f;
	Viewport[0][0] = w / 2.f;
	Viewport[1][1] = h / 2.f;
	Viewport[2][2] = 255.f / 2.f;
}

void projection(float coeff) {
	Perspective = Matrix::identity(4);
	Perspective[3][2] = coeff;
}

void view_trans(Vec3f camera, Vec3f center,Vec3f up) {
	//对位于(0,0,(camera-center).norm())看向(0,0,0)的摄像机进行旋转，沿center向量平移后可得程序所设置的camera及center
	//可先求出旋转矩阵平移矩阵，再对物体乘以平移矩阵的逆，旋转矩阵的逆，视作摄像机还位于(0,0,(camera-center).norm())看向(0,0,0)
	Matrix move_inverse = Matrix::identity(4);
	for (int i = 0; i < 3; i++) {
		move_inverse[i][3] = -center[i];
	}
	//以下三个向量表示新摄像机的标准正交基，由标准正交基可得旋转矩阵（camera_x，camera_y，camera_z）注：camera_x为列向量
	Vec3f camera_z = (camera - center).normalize();//摄像机看向-z方向
	Vec3f camera_x = (up^camera_z).normalize();
	Vec3f camera_y = (camera_z^camera_x).normalize();
	//旋转矩阵是正交矩阵，正交矩阵的逆等于其转置
	Matrix rotate_inverse = Matrix::identity(4);
	for (int i = 0; i < 3; i++) {
		rotate_inverse[0][i] = camera_x[i];
		rotate_inverse[1][i] = camera_y[i];
		rotate_inverse[2][i] = camera_z[i];
	}
	ModelView=rotate_inverse * move_inverse;
}

Vec3f barycentric(Vec3f* tri, Vec2i point) {//返回该点的重心坐标（x，y，z） 几何意义：（Sa/（Sa+Sb+Sc），...，...） 可用于插值
	Vec3f res = Vec3f(tri[1].x - tri[0].x, tri[2].x - tri[0].x, tri[0].x - point.x) ^ Vec3f(tri[1].y - tri[0].y, tri[2].y - tri[0].y, tri[0].y - point.y);
	if (std::abs(res.z) < 1e-2) {//防止res.z为0作为除数
		return Vec3f(-1, -1, -1);//返回一个非法的（在三角形外的点的）重心坐标
	}
	return Vec3f(1.f - (res.x + res.y) / res.z, res.x / res.z, res.y / res.z);//重心坐标为(1-u-v,u,v)。res是和(u,v,1)平行的向量
}
//Vec3f barycentric(Vec3f* v, Vec2i point) { 验证了一下games101中 公式法求重心坐标 也正确
//	float x = point.x;
//	float y = point.y;
//	float c1 = (x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*y + v[1].x*v[2].y - v[2].x*v[1].y) / (v[0].x*(v[1].y - v[2].y) + (v[2].x - v[1].x)*v[0].y + v[1].x*v[2].y - v[2].x*v[1].y);
//	float c2 = (x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*y + v[2].x*v[0].y - v[0].x*v[2].y) / (v[1].x*(v[2].y - v[0].y) + (v[0].x - v[2].x)*v[1].y + v[2].x*v[0].y - v[0].x*v[2].y);
//	float c3 = (x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*y + v[0].x*v[1].y - v[1].x*v[0].y) / (v[2].x*(v[0].y - v[1].y) + (v[1].x - v[0].x)*v[2].y + v[0].x*v[1].y - v[1].x*v[0].y);
//	return Vec3f(c1,c2,c3);
//}

void triangle(Matrix* m, IShader &shader, TGAImage &image, float *z_buffer) {//这里把参数Vec2i改成Vec3i，把深度这一属性也带上，才能得到每个点插值后的深度
	Vec3f tri[3];
	for (int i = 0; i < 3; i++) {
		tri[i]= Vec3f(m[i][0][0] / m[i][3][0], m[i][1][0] / m[i][3][0], m[i][2][0] / m[i][3][0]);
	}
	//计算三角形的矩形边界
	int x_min = std::min(tri[0].x, std::min(tri[1].x, tri[2].x));
	int y_min = std::min(tri[0].y, std::min(tri[1].y, tri[2].y));
	int x_max = std::max(tri[0].x, std::max(tri[1].x, tri[2].x));
	int y_max = std::max(tri[0].y, std::max(tri[1].y, tri[2].y));
	TGAColor c;
	for (int x = x_min; x <= x_max; x++) {
		for (int y = y_min; y <= y_max; y++) {
			if (x<0 || x>image.get_width() || y<0 || y>image.get_height()) continue;//超出画布的跳过
			Vec3f bct = barycentric(tri, Vec2i(x, y));
			if (bct.x < 0 || bct.y < 0 || bct.z < 0) continue;
			Vec3f bct_revised(0, 0, 0);//经透视变换后，屏幕上的点已经是非线性，对屏幕上像素进行插值存在误差，需要进行透视矫正插值
			for (int i = 0; i < 3; ++i)
			{
				bct_revised[i] = bct[i] / m[i][3][0];
			}
			float Z_n = 1. / (bct_revised[0] + bct_revised[1] + bct_revised[2]);
			for (int i = 0; i < 3; ++i)
			{
				bct_revised[i] *= Z_n;
			}//至此，得到了一准确的质心坐标
			float z = 0.0;
			z += bct_revised.x*tri[0].z;
			z += bct_revised.y*tri[1].z;
			z += bct_revised.z*tri[2].z;
			if (z > z_buffer[x + y * image.get_width()]) {
				bool discard = shader.fragment(bct_revised, c);
				if (!discard) {
					z_buffer[x + y * image.get_width()] = z;
					image.set(x, y, c);
				}
			}
		}
	}
}

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

