///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include "libs/Eigen/StdVector"
#include "libs/Eigen_unsupported/Eigen/BVH"
#include <limits>
const float INF = (std::numeric_limits<float>::max)();
typedef Eigen::AlignedBox<float, 3> Bbox;
const float epsilon = 1.f / pow(10.f, 4.f);
const float epsilon_2 = 1.f / pow(10.f, 6.f);

class Intersection;
class Shape;

const float PI = 3.14159f;
enum BRDFType {
	PHONG,
	GGX,
	BECKMAN
};
class Slab
{
public:
	Slab() {
		N = Vector3f(0.f, 0.f, 0.f), d0 = 0.f, d1 = 0.f;
	};
	Slab(Vector3f normal, float d_0, float d_1) {
		N = normal, d0 = d_0, d1 = d_1;
	};
	Vector3f N; // normal
	float d0;
	float d1;
};
class Intersection
{
public:
	Intersection() {
		t = INF, intersect_obj = nullptr, theta = 0.f, UV_theta = 0.f,
			intersect_point = Vector3f(0.f, 0.f, 0.f),
			intersect_normal = Vector3f(0.f, 0.f, 0.f);
		UV_coord = Vector2f(0.f, 0.f);
	};
	Intersection(Intersection& in) {
		t = in.t, intersect_obj = in.intersect_obj, theta = in.theta, UV_theta = in.UV_theta,
			intersect_point = in.intersect_point,
			intersect_normal = in.intersect_normal;
		UV_coord = in.UV_coord;
	};
	Intersection& operator=(const Intersection& rhs) {
		t = rhs.t;
		intersect_obj = rhs.intersect_obj;
		intersect_point = rhs.intersect_point;
		intersect_normal = rhs.intersect_normal;
		theta = rhs.theta;
		UV_theta = rhs.UV_theta;
		UV_coord = rhs.UV_coord;
		return *this;
	}
	float t;
	Shape* intersect_obj;
	Vector3f intersect_point;
	Vector3f intersect_normal;

	// later
	float theta;
	float UV_theta;
	Vector2f UV_coord;
};
////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
	 Vector3f Kd, Ks, emission, Kt; // Kt = index of refraction
	 float alpha, ior;
    unsigned int texid;
	std::vector<Color> pixels;
	int tex_width, tex_height;

    virtual bool isLight() { return false; }
	bool isTexture() { return pixels.size() != 0; }

	Material() : Kd(Vector3f(1.0f, 1.0f, 1.0f)), Ks(Vector3f(1.0, 1.0, 1.0)), alpha(1.0), texid(0), emission(1.f, 1.f, 1.f), Kt(1.0f, 1.0f, 1.0f), ior(1.f), tex_width(0), tex_height(0) {}
	Material(const Vector3f d, const Vector3f s, const float a)
		: Kd(d), Ks(s), alpha(a), texid(0), tex_width(0), tex_height(0) {}
	Material(Material& o) {
		Kd = o.Kd;  Ks = o.Ks;  alpha = o.alpha;  texid = o.texid; Kt = o.Kt; ior = o.ior; tex_width = o.tex_width; tex_height = o.tex_height;
		pixels = o.pixels;
	}

    void setTexture(const std::string path);
	void setCheckboardTexture();
    //virtual void apply(const unsigned int program);
};
////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light : public Material
{
public:
	Light() {};
	Light(const Vector3f e) : Material() { emission = e; }
	virtual bool isLight() { return true; };
	//Vector3f phong_illumination(Vector3f point, Ray* ray);
};
class BRDF : public Material
{
public:
	BRDF() {};
	BRDF(Vector3f diffuse_, Vector3f specular_, float shiness_) {
		Kd = diffuse_, Ks = specular_, alpha = shiness_; Kt = Vector3f(1.0f, 1.0f, 1.0f); ior = 1.f;
	};
	BRDF(Vector3f diffuse_, Vector3f specular_, float shiness_, Vector3f(tm_), float ior_) {
		Kd = diffuse_, Ks = specular_, alpha = shiness_; Kt = tm_; ior = ior_;
	};
	virtual bool isLight() { return false; };
};
////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (TriData: consisting of three
// indices into the vertex array).
typedef Eigen::Matrix<unsigned int, 3, 1 > TriData;
    
class VertexData
{
 public:
    Vector3f pnt;
    Vector3f nrm;
    Vector2f tex;
    Vector3f tan;
    VertexData(const Vector3f& p, const Vector3f& n, const Vector2f& t, const Vector3f& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<TriData> triangles;
    Material *mat;
	bool isVertNorm;
	bool isTexCoord;
	std::vector<Bbox> boxes;
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class Realtime;

class Camera
{
public:
	Camera(Vector3f eye_, Quaternionf or_, float ry_) { eye = eye_, orient = or_, ry = ry_; };
	Vector3f GetEye() { return eye; }
	Quaternionf GetOrientation() { return orient; }
	float GetRatio() { return ry; }
	void DepthOfField(float& r_x, float& r_y, float focal_d, const float W); // W = size of the circle of confusion
private:
	Vector3f eye;      // Position of eye for viewing scene
	Quaternionf orient;   // Represents rotation of -Z to view direction
	float ry;
};
class Ray
{
public:
	Ray(Vector3f start, Vector3f dir) { point = start, direction = dir; };
	Vector3f eval(float t)
	{
		return point + t * direction.normalized();
	};
	Vector3f GetStartPoint() { return point; }
	Vector3f GetDirection() { return direction; }
	void SetStartPoint(Vector3f vec) { point = vec; };
	void SetDirection(Vector3f dir) { direction = dir; };
private:
	Vector3f point;
	Vector3f direction;
};

class Shape
{
public:
	Shape() : mesh_data(nullptr) {};
	virtual Intersection intersect(Ray* ray) { return Intersection(); };
	virtual bool HasTriangle() { return false; };
	virtual Material* GetMat() = 0;
	MeshData* mesh_data;
	virtual Vector3f GetPosition() = 0;
	virtual Bbox bbox() = 0;
	virtual bool isMesh() = 0;
	virtual bool HasBbox() = 0;
	virtual float GetRadius() = 0;
	Intersection SampleSphere(Vector3f center_, float radius_);
};
class Sphere : public Shape
{
public:
	Sphere(Vector3f center_, float radius_, Material* r_mat) { center = center_, radius = radius_, mat_ = r_mat; };
	Sphere(Vector3f center_, float radius_, Light* light) { center = center_, radius = radius_, light_ = light; };
	virtual Intersection intersect(Ray* ray);
	virtual bool HasTriangle() { return false; };
	virtual Material* GetMat() { return mat_; };
	virtual Vector3f GetPosition() { return center; };
	virtual float GetRadius() { return radius; };
	Bbox m_box;
	virtual Bbox bbox() {
		return m_box;
	}
	virtual bool isMesh() { return false; };
	virtual bool HasBbox() { return true; };
private:
	Light* light_;
	Material* mat_;
	Vector3f center;
	float radius;
};
class Box : public Shape
{
public:
	Box(Vector3f base_corner, Vector3f diagonal, Material* r_mat) { corner = base_corner, diagonal_vec = diagonal, mat_ = r_mat; };
	virtual Intersection intersect(Ray* ray);
	virtual bool HasTriangle() { return false; };
	virtual Material* GetMat() { return mat_; };
	virtual Vector3f GetPosition() { return corner; };
	Bbox m_box;
	virtual Bbox bbox() {
		return m_box;
	}
	virtual bool isMesh() { return false; };
	virtual bool HasBbox() { return true; };
	virtual float GetRadius() { return 0.f; };
private:
	Material* mat_;
	Vector3f corner;
	Vector3f diagonal_vec;
};


class Interval
{
public:
	Interval() : t0(0.f), t1(INF) {
		n0 = Vector3f(0.f, 0.f, 0.f);
		n1 = Vector3f(0.f, 0.f, 0.f);
	}
	Interval(float t0_, float t1_, Vector3f n0_, Vector3f n1_)
	{
		if (t0_ <= t1_)
		{
			t0 = t0_; t1 = t1_;
		}
		else
		{
			t1 = t0_; t0 = t1_;
		}
		n0 = n0_; n1 = n1_;
	}
	void SetEmpty() { t0 = 0.f, t1 = -1.f; }
	float Intersect_check(Vector3f& norm); // return the smallest positive value of t0, t1
	void Intersect_oneslab(Ray ray, Slab slab);
	void Intersect_slablist(Ray ray, std::vector<Slab>& slabs);
	float Get_T0() { return t0; };
	float Get_T1() { return t1; };
	Vector3f Get_Normal0() { return n0; };
	Vector3f Get_Normal1() { return n1; };
	void Set_T0(float t0_) { t0 = t0_; };
	void Set_T1(float t1_) { t1 = t1_; };
private:
	float t0, t1; // beginning and ending point along a ray
	Vector3f n0, n1; // surface normals at t0 and t1 respectively
};
float Vec_length(Vector3f vec);
class Cylinder : public Shape
{
public:
	Cylinder(Vector3f point, Vector3f axis_vec, float radius_, Material* r_mat) { base_point = point, axis = axis_vec, radius = radius_, mat_ = r_mat; };
	virtual Intersection intersect(Ray* ray);
	virtual bool HasTriangle() { return false; };
	virtual Material* GetMat() { return mat_; };
	Interval Quadratic_eq(float dx, float dy, float qx, float qy, bool& no_intersection);
	virtual Vector3f GetPosition() { return base_point; };
	Vector3f GetAxis() { return axis; };
	Bbox m_box;
	virtual Bbox bbox() {
		return m_box;
	}
	virtual bool isMesh() { return false; };
	virtual bool HasBbox() { return true; };
	virtual float GetRadius() { return radius; };
private:
	Material* mat_;
	Vector3f base_point;
	Vector3f axis;
	float radius;
};
class Mesh : public Shape
{
public:
	Mesh(std::string file, Vector3f trans, Vector3f scale_, Quaternionf orient, Material* r_mat) {
		file_path = file;
		translation = trans;
		m_scale = scale_;
		orientation = orient;
		mat_ = r_mat;
	};
	virtual Intersection intersect(Ray* ray);
	Intersection intersect_triangle(Ray* ray, Vector3f vert0, Vector3f vert1, Vector3f vert2, Vector3f vert_norm0, Vector3f vert_norm1, Vector3f vert_norm2, 
		Vector2f tex0, Vector2f tex1, Vector2f tex2, bool isNorm, bool isTex);
	virtual bool HasTriangle() { return true; };
	virtual Material* GetMat() { return mat_; };
	virtual Vector3f GetPosition() { return translation; };
	Matrix4f MTV() {
		return translate(translation) * scale(m_scale) * toMat4(orientation);
	};
	Bbox m_box;
	virtual Bbox bbox() {
		return m_box;
	}
	virtual bool isMesh() { return true; };
	virtual bool HasBbox() {
		if (m_box.isEmpty()) return false;
		else return true;
	};
	virtual float GetRadius() { return 0.f; };
private:
	Material* mat_;
	std::string file_path;
	Vector3f translation;
	Vector3f m_scale;
	Quaternionf orientation;

};
class Minimizer
{
public:
	typedef float Scalar;
	Ray ray;
	Intersection intersect_;
	Minimizer(const Ray& r) : ray(r) { intersect_.t = INF; };
	float minimumOnObject(Shape* obj)
	{
		Intersection curr_result = obj->intersect(&ray);
		if (curr_result.t == INF)
			return INF;
		// Keep track the minimal intersection and object
		else if (curr_result.t > 0.f && curr_result.t < intersect_.t)
		{
			intersect_.intersect_obj = obj;
			intersect_.intersect_normal = curr_result.intersect_normal;
			intersect_.intersect_point = curr_result.intersect_point;
			intersect_.t = curr_result.t;
			intersect_.theta = curr_result.theta;
			intersect_.UV_coord = curr_result.UV_coord;
			intersect_.UV_theta = curr_result.UV_theta;
		}
		return intersect_.t;
	}
	float minimumOnVolume(const Bbox& box)
	{
		Vector3f L = (box.min)();
		Vector3f U = (box.max)();
		std::vector<Slab> slab_list;
		slab_list.push_back(Slab(Vector3f(1.0f, 0.0f, 0.0f), -L.x(), -U.x()));
		slab_list.push_back(Slab(Vector3f(0.0f, 1.0f, 0.0f), -L.y(), -U.y()));
		slab_list.push_back(Slab(Vector3f(0.0f, 0.0f, 1.0f), -L.z(), -U.z()));

		if (box.contains(ray.GetStartPoint()))
			return 0.f;
		Interval interval_result;
		interval_result.Intersect_slablist(ray, slab_list);
		Vector3f temp_norm;
		float result_t = interval_result.Intersect_check(temp_norm);
		if(result_t == INF)
			return INF;
		return result_t;
	}
};
class Scene {
public:
    int width, height;
    Realtime* realtime;         // Remove this (realtime stuff)
    Material* currentMat;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const Matrix4f& M,
		std::string st, Vector3f trans, Vector3f scale, Quaternionf orient, Material* r_mat);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

	//Color CalculateLight(Vector3f p, Vector3f norm, Vector3f Kd);
	void AverageColor(Color* image);
	void TraceRay(Scene* scn, Ray* ray);
	Color TracePath(Ray* ray);
    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);
	void InsertObject(Shape* obj) { shapes.push_back(obj); };

	/* For Light obj, not for BRDF */
	Intersection SampleLight(float& r);
	float PdfLight(Intersection Q, float r); // return 1/(AreaOfLight * NumberOfLight);

	/* For BRDF, not for Light */
	Vector3f Scene::SampleBrdf(Vector3f w_0_vec, Vector3f N, float p_d, float p_r, float alpha_, float ior_, BRDFType type);
private:
	Camera* cam;
	std::vector<BRDF*> materials;
	std::vector<Shape*> shapes;
	std::vector<Light*> lights;

	Vector3f ambient;

	int num_calculation;
	KdBVH<float, 3, Shape*> Tree;
};

inline float NearlyEqual(float a, float b, float epsilon = std::numeric_limits<float>::epsilon())
{
	return std::abs(a - b) < epsilon;
}
inline float Sign(float val)
{
	return static_cast<float>((0 < val) - (val < 0));
}
float GeometryFactor(Intersection A, Intersection B);

/* For Light obj, not for BRDF */
Vector3f EvalRadiance(Intersection Q); // RGB radiance
//virtual void apply(const unsigned int program);

/* For BRDF, not for Light */
float PdfBrdf(Vector3f w_0, Vector3f N, Vector3f w_i, float p_d, float p_r, float p_t, float alpha_, float ior_, BRDFType type);
Vector3f EvalScattering(Vector3f w_0_vec, Vector3f N, Vector3f w_i, Vector3f k_d, Vector3f k_s, Vector3f k_t, float alpha_, float ior_, float dist, BRDFType type);
Vector3f SampleLobe(Vector3f N, float cos_, float phi);

float char_func(float d);
Vector3f F_term(Vector3f L, Vector3f H, Vector3f k_s);
float D_term(Vector3f H, Vector3f N, float alpha, BRDFType type);
float G_sub(Vector3f v, Vector3f m, float alpha_, Vector3f N, BRDFType type);
float G_factor(Vector3f L, Vector3f V, Vector3f H, float alpha_, Vector3f N, BRDFType type);

//float IndexOfRefraction(Vector3f w0, Vector3f N, float ior_);
Vector3f BeersLaw(Vector3f w_0, Vector3f N, Vector3f k_t, float distance);
Vector3f computePrimaryTexDir(Vector3f norm);