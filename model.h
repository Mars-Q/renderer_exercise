#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts_;//顶点
	std::vector<std::vector<Vec3i> > faces_; // attention, this Vec3i means vertex/uv/normal
	std::vector<Vec3f> norms_;//对应的法向量
	std::vector<Vec2f> uv_;//对应的纹理顶点
	TGAImage diffusemap_;
	TGAImage normalmap_;
	void load_texture(std::string filename, const char *suffix, TGAImage &img);
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	Vec3f normal(int iface, int nthvert);
	Vec3f normal(Vec2f uv);
	Vec3f vert(int i);
	Vec3f vert(int iface, int nthvert);
	Vec2i uv(int iface, int nvert);
	TGAColor diffuse(Vec2i uv);
	std::vector<int> face(int idx);
};

#endif //__MODEL_H__