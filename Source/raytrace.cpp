//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
//#include "libs/stb_image.h"
#include <stb_image.h>

#define sign(x) ((x >= 0.f) ? 1.f : -1.f)
// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0f, 1.0f);

const float Russianroulette = 0.8f;
const float e_value = 2.71828183f;

// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].
Scene::Scene()
{ 
    realtime = new Realtime(); 
}

void Scene::Finit()
{
	num_calculation = 10;
}

void Scene::triangleMesh(MeshData* mesh)
{
}

Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int n;
    stbi_set_flip_vertically_on_load(true);
	float* image = stbi_loadf(path.c_str(), &tex_width, &tex_height, &n, STBI_rgb);
	pixels.resize(tex_width * tex_height);
	for (unsigned i = 0; i < pixels.size(); ++i)
	{
		for (unsigned o = 0; o < 3; ++o)
			pixels[i][o] = static_cast<float>(image[i * 3 + o]);
	}
}
void Material::setCheckboardTexture()
{
	pixels.resize(100); // 10 x 10
	for (unsigned i = 0; i < 10; ++i)
	{
		for (unsigned j = 0; j < 10; ++j)
		{
			if ((i + j) % 2 == 0) pixels[j + (i * 10)] = Color(1.f, 1.f, 1.f);
			else pixels[j + (i * 10)] = Color(0.f, 0.f, 0.f);
		}
	}
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        //realtime->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        //realtime->setCamera(Vector3f(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); 
		cam = new Camera(Vector3f(f[1], f[2], f[3]), Orientation(5, strings, f), f[4]);
	}

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        //realtime->setAmbient(Vector3f(f[1], f[2], f[3])); 
		ambient = Vector3f(f[1], f[2], f[3]);
	}
    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        //currentMat = new Material(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
		if (f.size() == 8) // 7 + 1
		{
			currentMat = new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
			BRDF* brdf_ = new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]);
			materials.push_back(brdf_);
		}
		else if (f.size() == 12) // 11 + 1
		{
			currentMat = new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], Vector3f(f[8], f[9], f[10]), f[11]);
			BRDF* brdf_ = new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], Vector3f(f[8], f[9], f[10]), f[11]);
			materials.push_back(brdf_);
		}
		else
			std::cout << "BRDF has illegal components\n";
	}
    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3]));
		Light* light_ = new Light(Vector3f(f[1], f[2], f[3]));
		lights.push_back(light_);
	}
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        //realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
		if(f[5] == 1)
			currentMat->setTexture("wood.png");
		Sphere* sp = new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
		sp->m_box = Bbox(Vector3f(f[1] - f[4], f[2] - f[4], f[3] - f[4]), Vector3f(f[1] + f[4], f[2] + f[4], f[3] + f[4]));
		shapes.push_back(sp);
	}
    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        //realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
		if (f[7] == 1)
			//currentMat->setCheckboardTexture();
			currentMat->setTexture("checkboard.jpg");
		Box* bx = new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat);
		bx->m_box = Bbox(Vector3f(f[1], f[2], f[3]), Vector3f(f[1] + f[4], f[2] + f[5], f[3] + f[6]));
		shapes.push_back(bx);
	}
    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        //realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
		Cylinder* cy = new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat);
		//B+-(0,0,r) (0,r,0) (r,0,0)
		//B+A +- (0,0,r) (0,r,0) (r,0,0)
		// Find min, max
		Vector3f lb = cy->GetPosition() - Vector3f(f[7], f[7], f[7]);
		Vector3f rb = cy->GetPosition() + Vector3f(f[7], f[7], f[7]);
		Vector3f rt = cy->GetPosition() + cy->GetAxis() + Vector3f(f[7], f[7], f[7]);
		Vector3f lt = cy->GetPosition() + cy->GetAxis() - Vector3f(f[7], f[7], f[7]);
		Vector3f mb = cy->GetPosition() - cy->GetAxis() + Vector3f(f[7], f[7], f[7]);
		Vector3f ma = cy->GetPosition() - cy->GetAxis() - Vector3f(f[7], f[7], f[7]);
		float min_x = min(min(lb.x(), rt.x()), min(rb.x(), lt.x()));
		float max_x = max(max(lb.x(), rt.x()), max(rb.x(), lt.x()));
		float min_y = min(min(lb.y(), rt.y()), min(rb.y(), lt.y()));
		float max_y = max(max(lb.y(), rt.y()), max(rb.y(), lt.y()));
		float min_z = min(min(lb.z(), rt.z()), min(rb.z(), lt.z()));
		float max_z = max(max(lb.z(), rt.z()), max(rb.z(), lt.z()));
		cy->m_box = Bbox(Vector3f(min_x, min_y, min_z), Vector3f(max_x, max_y, max_z));
		shapes.push_back(cy);
	}
    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
		Vector3f translation = Vector3f(f[2], f[3], f[4]);
		Vector3f m_scale = Vector3f(f[5], f[5], f[5]);
		Quaternionf orientation = Orientation(6, strings, f);
		Matrix4f modelTr = translate(translation) * scale(m_scale) * toMat4(orientation);
		currentMat->setTexture("checkboard.png");
        ReadAssimpFile(strings[1], modelTr, strings[1], Vector3f(f[2], f[3], f[4]), Vector3f(f[5], f[5], f[5]), Orientation(6, strings, f), currentMat);
	}
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}
float IndexOfRefraction(Vector3f w0, Vector3f N, float ior_)
{
	if (w0.dot(N) >= 0.f) // inside to outside
		return 1.0f / ior_;
	return ior_;
}
Vector3f SampleLobe(Vector3f N, float cos_, float phi)
{
	float s = sqrtf(1.f - (cos_ * cos_));
	Vector3f K = Vector3f(s * cosf(phi), s * sinf(phi), cos_);
	Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), N);
	return q._transformVector(K);
}
Intersection Scene::SampleLight(float& r)
{
	Intersection result;
	for (auto light_ : shapes)
	{
		if (light_->GetMat()->isLight())
		{
			r = light_->GetRadius();
			result = light_->SampleSphere(light_->GetPosition(), light_->GetRadius());
			break;
		}
	}

	return result;
}
float GeometryFactor(Intersection A, Intersection B)
{
	Vector3f dir_ = A.intersect_point - B.intersect_point;
	float div = dir_.dot(dir_);
	return fabsf((A.intersect_normal.dot(dir_) * B.intersect_normal.dot(dir_)) / (div*div));
}
float Scene::PdfLight(Intersection Q, float r)
{
	return 1.f / (4.f * PI * r * r * static_cast<float>(lights.size()));
}
Vector3f Scene::SampleBrdf(Vector3f w_0_vec, Vector3f N, float p_d, float p_r, float alpha_, float ior_, BRDFType type)
{
	float rand_ = myrandom(RNGen);
	if (rand_ < p_d) // diffuse
		return SampleLobe(N, sqrtf(myrandom(RNGen)), 2.f * PI * myrandom(RNGen));
	// Phong
	float cos_theta_m = 0.f;
	if (type == PHONG)
		cos_theta_m = pow(myrandom(RNGen), (1.f / (alpha_ + 1.f)));
	// reflection
	else if (type == GGX)
	{
		float rand_num = myrandom(RNGen);
		cos_theta_m = cosf(atanf((alpha_ * sqrtf(rand_num))/(sqrtf(1.f - rand_num))));
	}
	else if (type == BECKMAN)
		cos_theta_m = cosf(atanf(sqrtf(-(alpha_ * alpha_) * log(1.f - myrandom(RNGen)))));

	Vector3f m = SampleLobe(N, cos_theta_m, 2.f * PI * myrandom(RNGen));
	//m_.normalize();
	if (rand_ < (p_d + p_r)) // reflection
		return 2.f * (w_0_vec.dot(m) * m) - w_0_vec;

	float new_ior = IndexOfRefraction(w_0_vec, N, ior_);//1.0f / ior_;//IndexOfRefraction(w_0_vec, N, ior_);
	const float radicand = 1.0f - ((new_ior * new_ior) * (1.0f - pow(w_0_vec.dot(m), 2)));
	if (radicand < 0.f) // declare total internal reflection
		return 2.f * (w_0_vec.dot(m) * m) - w_0_vec;
	return (((new_ior * w_0_vec.dot(m)) - (sign(w_0_vec.dot(N)) * sqrtf(radicand))) * m) - (new_ior * w_0_vec);
}
Vector3f EvalRadiance(Intersection Q)
{ 
	return Q.intersect_obj->GetMat()->emission;
}
Vector3f BeersLaw(Vector3f w_0, Vector3f N, Vector3f k_t, float distance)
{
	if (isinf(distance) || isnan(distance)) std::cout << "distance is INF or nan\n";
	Vector3f result(1.f,1.f,1.f);
	if (w_0.dot(N) < 0.f)
	{
		for(int i = 0; i < 3; ++i)
			result[i] = pow(e_value, distance * log(k_t[i]));//e ^ distance * log(k_t);
		return result;
	}
	return result;
}
float char_func(float d)
{
	return (d > 0.f) ? 1.f : 0.f;
}
Vector3f F_term(Vector3f L, Vector3f H, Vector3f k_s) // L = w_i, H = m
{
	return k_s + (Vector3f(1.f,1.f,1.f) - k_s) * pow(1.f - fabs(L.dot(H)), 5);
}
float D_term(Vector3f H, Vector3f N, float alpha, BRDFType type) // H = m, N = N
{
	// Phong
	if (type == PHONG)
		return char_func(H.dot(N)) * ((alpha + 2.f) / (2.f * PI)) * pow(H.dot(N), alpha);
	else if (type == GGX)
	{
		float tan_m = sqrtf(1.f - (H.dot(N) * H.dot(N))) / H.dot(N);
		float middle = (alpha * alpha) / (PI * pow(N.dot(H), 4) * pow((alpha * alpha) + (tan_m * tan_m), 2));
		return char_func(H.dot(N)) * middle;
	}
	else if (type == BECKMAN)
	{
		float tan_m = sqrtf(1.f - (H.dot(N) * H.dot(N))) / H.dot(N);
		float middle = 1.f / (PI * alpha * alpha * pow(N.dot(H), 4));
		float last = pow(e_value, (-(tan_m * tan_m) / (alpha * alpha)));
		return char_func(H.dot(N)) * middle * last;
	}
	return 0.f;
}
float G_sub(Vector3f v, Vector3f m, float alpha_, Vector3f N, BRDFType type)
{
	float tan_v = sqrtf(1.f - (v.dot(N) * v.dot(N))) / v.dot(N);
	if (tan_v == 0.f)
		return 1.f;
	if (v.dot(N) > 1.f)
		return 1.f;
	if (type == PHONG)
	{
		float a = sqrtf((alpha_ / 2.f) + 1.f) / tan_v;
		float back = (a < 1.6f) ? ((3.535f * a + 2.181f * a * a) / (1.f + 2.276f * a + 2.577f * a * a)) : 1.f;
		return (a < 1.6) ? (char_func(v.dot(m) / v.dot(N)) * back) : 1.f;
	}
	else if (type == GGX)
	{
		float back = 2.f / (1.f + sqrtf(1.f + (alpha_ * alpha_ * tan_v * tan_v)));
		return char_func(v.dot(m) / v.dot(N)) * back;
	}
	else if (type == BECKMAN)
	{
		float a = 1.f / (alpha_ * tan_v);
		float back = (a < 1.6f) ? ((3.535f * a + 2.181f * a * a) / (1.f + 2.276f * a + 2.577f * a * a)) : 1.f;
		return char_func(v.dot(m) / v.dot(N)) * back;
	}
	return 0.f;
}
float G_factor(Vector3f L, Vector3f V, Vector3f H, float alpha_, Vector3f N, BRDFType type) // L = w_i, V = w_0, H = m
{
	return G_sub(L, H, alpha_, N, type) * G_sub(V, H, alpha_, N, type);
}
float PdfBrdf(Vector3f w_0, Vector3f N, Vector3f w_i, float p_d, float p_r, float p_t, float alpha_, float ior_, BRDFType type)
{
	const float P_d_new = fabs(w_i.dot(N)) / PI;
	const Vector3f m = (w_0 + w_i).normalized();
	const float P_r_new = D_term(m, N, alpha_, type) * fabs(m.dot(N)) / (4.f * fabs(w_i.dot(m)));

	const float new_ior = IndexOfRefraction(w_0, N, ior_);//1.0f / ior_;//IndexOfRefraction(w_0, N, ior_);
	float new_ior_0 = ior_;
	float new_ior_i = 1.f;
	if (w_0.dot(N) < 0.f)
	{
		new_ior_0 = 1.f;
		new_ior_i = ior_;
	}
	const Vector3f m_ = -(new_ior_0 * w_i + new_ior_i * w_0).normalized();
	const float radicand = 1.0f - ((new_ior * new_ior) * (1.0f - pow(w_0.dot(m_), 2)));
	float P_t_new = 0.f;
	if (radicand < 0.f)
		P_t_new = D_term(m, N, alpha_, type) * fabs(m.dot(N)) / (4.f * fabs(w_i.dot(m)));
	else
		P_t_new = (D_term(m_, N, alpha_, type) * fabs(m_.dot(N)) * (pow(new_ior_0, 2) * fabs(w_i.dot(m_)))) / pow((new_ior_0 * w_i.dot(m_)) + (new_ior_i * w_0.dot(m_)), 2);
	return p_d * P_d_new + p_r * P_r_new + p_t * P_t_new;
}
Vector3f EvalScattering(Vector3f w_0_vec, Vector3f N, Vector3f w_i, Vector3f k_d, Vector3f k_s, Vector3f k_t, float alpha_, float ior_, float dist, BRDFType type) // V = w_0_vec, L = w_i, H = m
{
	const Vector3f E_d = k_d / PI;
	const Vector3f m = (w_0_vec + w_i).normalized();
	const Vector3f E_r = (D_term(m, N, alpha_, type) * G_factor(w_i, w_0_vec, m, alpha_, N, type) * F_term(w_i, m, k_s)) / (4.f * fabs(w_i.dot(N)) * fabs(w_0_vec.dot(N)));

	const float new_ior = IndexOfRefraction(w_0_vec, N, ior_);//1.0f / ior_;//IndexOfRefraction(w_0_vec, N, ior_);
	float new_ior_0 = ior_;
	float new_ior_i = 1.f;
	if (w_0_vec.dot(N) < 0.f)
	{
		new_ior_0 = 1.f;
		new_ior_i = ior_;
	}
	const Vector3f m_ = -(new_ior_0 * w_i + new_ior_i * w_0_vec).normalized();
	const float radicand = 1.0f - ((new_ior * new_ior) * (1.0f - pow(w_0_vec.dot(m_), 2)));
	Vector3f E_t(0.f, 0.f, 0.f);
	const Vector3f A_t = BeersLaw(w_0_vec, N, k_t, dist);
	if (radicand < 0.f)
		E_t = A_t.cwiseProduct((D_term(m, N, alpha_, type) * G_factor(w_i, w_0_vec, m, alpha_, N, type) * F_term(w_i, m, k_s)) / (4.f * fabs(w_i.dot(N)) * fabs(w_0_vec.dot(N))));
	else
	{
		const Vector3f Front = (D_term(m_, N, alpha_, type) * G_factor(w_i, w_0_vec, m_, alpha_, N, type) * (Vector3f(1.f, 1.f, 1.f) - F_term(w_i, m_, k_s))) / (fabs(w_i.dot(N)) * fabs(w_0_vec.dot(N)));
		const float Back = (fabs(w_i.dot(m_)) * fabs(w_0_vec.dot(m_)) * pow(new_ior_0, 2)) / pow((new_ior_0 * w_i.dot(m_)) + (new_ior_i * w_0_vec.dot(m_)), 2);
		E_t = A_t.cwiseProduct(Front * Back);
	}
	return fabs(N.dot(w_i)) * (E_d + E_r + E_t);
}
Intersection Shape::SampleSphere(Vector3f center_, float radius_)
{
	float z = 2.f * myrandom(RNGen) - 1.f;
	float r = sqrtf(1.f - (z * z));
	float a = 2.f * PI * myrandom(RNGen);
	Intersection result;
	result.intersect_normal = Vector3f(r * cosf(a), r * sinf(a), z);
	result.intersect_normal.normalize();
	result.intersect_point = center_ + radius_ * result.intersect_normal;
	result.intersect_obj = this;
	return result;
}
Intersection Sphere::intersect(Ray* ray)
{
	Vector3f Q_ = ray->GetStartPoint() - center;
	Intersection intersect_ = Intersection();
	float Q_dot_D = Q_.dot(ray->GetDirection().normalized());
	float Q_dot_Q = Q_.dot(Q_);
	float sqrt_val = sqrtf(Q_dot_D * Q_dot_D - Q_dot_Q + radius * radius);
	if (Q_dot_D * Q_dot_D - Q_dot_Q + radius * radius < epsilon)
		return intersect_; // no intersection
	float small_t = -(Q_dot_D) - sqrt_val;
	float large_t = -(Q_dot_D) + sqrt_val;
	if (small_t < 0.f && large_t < 0.f) // no intersection
		return intersect_; // no intersection
	else
	{
		if (small_t < 0.f && large_t >= 0.f) intersect_.t = large_t;
		else if (small_t >= 0.f && large_t < 0.f) intersect_.t = small_t;
		else intersect_.t = (small_t < large_t) ? small_t : large_t;
		intersect_.intersect_point = ray->eval(intersect_.t);
		intersect_.intersect_normal = intersect_.intersect_point - center;
		intersect_.intersect_normal.normalize();
		intersect_.theta = atan2f(intersect_.intersect_normal.y(), intersect_.intersect_normal.x());
		intersect_.UV_theta = acos(intersect_.intersect_normal.z());
		intersect_.UV_coord = Vector2f(intersect_.theta / (2.f * PI), intersect_.UV_theta / PI);
	}
	return intersect_;
}
void Interval::Intersect_oneslab(Ray ray, Slab slab)
{
	float NdotD = slab.N.dot(ray.GetDirection().normalized());
	if (!NearlyEqual(NdotD, 0.0f)) // ray intersects both slab planes
	{
		int index = 0;
		for (int i = 0; i < 3; ++i)	{
			if (slab.N[i] != 0.0f) {
				index = i;
				break;
			}
		}
		const auto directionSign = Sign(ray.GetDirection()[index]);
		if (directionSign > 0)
		{
			float t0_ = -(slab.d0 + slab.N.dot(ray.GetStartPoint())) / NdotD;
			float t1_ = -(slab.d1 + slab.N.dot(ray.GetStartPoint())) / NdotD;
			if (t0_ > t1_)
			{
				float temp = t1_;
				t1_ = t0_;
				t0_ = temp;
			}
			t0 = t0_, t1 = t1_;
			n0 = -slab.N, n1 = slab.N;
		}
		else
		{
			float t0_ = -(slab.d1 + slab.N.dot(ray.GetStartPoint())) / NdotD;
			float t1_ = -(slab.d0 + slab.N.dot(ray.GetStartPoint())) / NdotD;
			if (t0_ > t1_)
			{
				float temp = t1_;
				t1_ = t0_;
				t0_ = temp;
			}
			t0 = t0_, t1 = t1_;
			n0 = slab.N, n1 = -slab.N;
		}
	}
	else // ray is parallel to slab planes
	{
		float NdotQ = slab.N.dot(ray.GetStartPoint());
		float s0 = NdotQ + slab.d0;
		float s1 = NdotQ + slab.d1;
		if (s0 * s1 < 0.f) // diff sign
			t0 = 0.f, t1 = INF; // between planes
		else
			t0 = 1.f, t1 = 0.f;
	}
}
void Interval::Intersect_slablist(Ray ray, std::vector<Slab>& slabs)
{
	std::pair<float, Vector3f> t0n0 = std::make_pair(-INF, Vector3f(0.f,0.f,0.f));
	std::pair<float, Vector3f> t1n1 = std::make_pair(INF, Vector3f(0.f, 0.f, 0.f));
	for(int i = 0; i < 3; ++i)
	{
		Interval isIntersect;
		isIntersect.Intersect_oneslab(ray, slabs[i]);
		if (!NearlyEqual(slabs[i].N.dot(ray.GetDirection().normalized()), 0.0f))
		{
			t0n0.first = max(t0n0.first, isIntersect.t0);
			t1n1.first = min(t1n1.first, isIntersect.t1);
			if (t0n0.first == isIntersect.t0)
				t0n0.second = isIntersect.n0;
			if (t1n1.first == isIntersect.t1)
				t1n1.second = isIntersect.n1;
		}
	}
	t0 = t0n0.first;
	t1 = t1n1.first;
	n0 = t0n0.second;
	n1 = t1n1.second;
}
float Interval::Intersect_check(Vector3f& norm)
{
	float result = INF;
	if (t0 > t1) // off the corner
		return result;
	if ((t0 < epsilon_2) && (t1 < epsilon_2)) return result;
	else if ((t1 >= 0.f) && (t0 >= 0.f))
	{
		result = abs(t0);
		norm = n0;
	}
	else
	{
		result = (abs(t0) <= abs(t1)) ? abs(t0) : abs(t1);
		if (result == t0)
			norm = n0;
		else
			norm = n1;
	}
	return result;
}
Vector3f computePrimaryTexDir(Vector3f norm)
{
	Vector3f a = norm.cross(Vector3f(1.f, 0.f, 0.f));
	Vector3f b = norm.cross(Vector3f(0.f, 1.f, 0.f));
	Vector3f c = norm.cross(Vector3f(0.f, 0.f, 1.f));
	Vector3f max_ab = fabs(a.dot(a)) < fabs(b.dot(b)) ? b : a; // should be absolute value of dot product?
	Vector3f max_abc = fabs(max_ab.dot(max_ab)) < fabs(c.dot(c)) ? c : max_ab;
	return max_abc;
}
Intersection Box::intersect(Ray* ray)
{
	Intersection intersect_ = Intersection();
	Vector3f L = (m_box.min)();
	Vector3f U = (m_box.max)();
	std::vector<Slab> slab_list;
	slab_list.push_back(Slab(Vector3f(1.0f, 0.0f, 0.0f), -L.x(), -U.x()));
	slab_list.push_back(Slab(Vector3f(0.0f, 1.0f, 0.0f), -L.y(), -U.y()));
	slab_list.push_back(Slab(Vector3f(0.0f, 0.0f, 1.0f), -L.z(), -U.z()));
	
	Interval interval_result;
	interval_result.Intersect_slablist(*ray, slab_list);
	Vector3f temp_norm;
	intersect_.t = interval_result.Intersect_check(temp_norm);

	if (intersect_.t != INF && intersect_.t > epsilon)
	{
		intersect_.intersect_point = ray->eval(intersect_.t);
		intersect_.intersect_normal = temp_norm;
		Vector3f u_ = computePrimaryTexDir(intersect_.intersect_normal);
		Vector3f v_ = intersect_.intersect_normal.cross(u_);
		intersect_.UV_coord = Vector2f(u_.dot(intersect_.intersect_point), v_.dot(intersect_.intersect_point));
	}
	else
	{
		intersect_.t = INF;
		return intersect_;
	}
	return intersect_;
}
Interval Cylinder::Quadratic_eq(float dx, float dy, float qx, float qy, bool& no_intersection)
{
	float a = dx * dx + dy * dy;
	float b = 2.f * (dx * qx + dy * qy);
	float c = qx * qx + qy * qy - radius * radius;
	Interval result;
	if (b * b - 4.f * a * c < epsilon) //no intersection
	{
		no_intersection = true;
		return result;
	}
	float small_t = (-b - sqrtf(b * b - 4.f * a * c)) / (2.f * a);
	float large_t = (-b + sqrtf(b * b - 4.f * a * c)) / (2.f * a);
	result = Interval(small_t, large_t, Vector3f(0.f, 0.f, 1.f), Vector3f(0.f, 0.f, 1.f));
	return result;
}
float Vec_length(Vector3f vec)
{
	return sqrtf(vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z());
}
Intersection Cylinder::intersect(Ray* ray)
{
	Quaternionf q = Quaternionf::FromTwoVectors(axis, Vector3f::UnitZ());
	Ray temp_ray(q._transformVector(ray->GetStartPoint() - base_point), q._transformVector(ray->GetDirection().normalized()));
	// a0 = 0, a1 = INF
	Slab s(Vector3f(0.f,0.f,1.f), 0, -Vec_length(axis));
	Interval ray_slab_intersection;
	ray_slab_intersection.Intersect_oneslab(temp_ray, s); // b0, b1

	float dx = temp_ray.GetDirection().x(), dy = temp_ray.GetDirection().y();
	float qx = temp_ray.GetStartPoint().x(), qy = temp_ray.GetStartPoint().y(); // quadratic equation

	bool no_intersection = false;
	Interval c_ray_intersection = Quadratic_eq(dx, dy, qx, qy, no_intersection);
	Intersection intersect_ = Intersection();//= new Intersection();
	if (no_intersection) return intersect_;

	float t0_ = max(max(-INF, ray_slab_intersection.Get_T0()), c_ray_intersection.Get_T0()); // a0, b0, c0
	float t1_ = min(min(INF, ray_slab_intersection.Get_T1()), c_ray_intersection.Get_T1()); // a0, b0, c0

	if (t0_ > t1_) return intersect_; // off the corner
	if (t0_ < 0.f && t1_ < 0.f) return intersect_;
	else if (t0_ >= 0.f && t1_ >= 0.f) intersect_.t = t0_;
	else intersect_.t = t1_;
	/*if (t1_ < 0.f)
		intersect_.t = t0_;
	else
		intersect_.t = (fabs(t0_) < fabs(t1_)) ? t0_ : t1_;*/
	intersect_.intersect_point = ray->eval(intersect_.t);
	Vector3f norm;
	if (s.N.dot(ray->GetDirection()) == 0.0f)
	{
		if (t0_ == ray_slab_intersection.Get_T0())
			norm = -Vector3f::UnitZ();
		if (t1_ == ray_slab_intersection.Get_T1())
			norm = Vector3f::UnitZ();
	}
	else
	{
		norm = Vector3f(temp_ray.eval(intersect_.t).x(), temp_ray.eval(intersect_.t).y(), 0.f);
		norm.normalize();
	}
	intersect_.intersect_normal = q.conjugate()._transformVector(norm);
	intersect_.UV_theta = atan2f(norm.y(), norm.x());
	intersect_.UV_coord = Vector2f(intersect_.UV_theta / (2.f*PI), norm.z()/ -Vec_length(axis));
	return intersect_;
}
Intersection Mesh::intersect(Ray* ray)
{
	float smallest_t = INF;
	Vector3f vert_0 = mesh_data->vertices[0].pnt;
	Vector3f vert_1 = mesh_data->vertices[1].pnt;
	Vector3f vert_2 = mesh_data->vertices[2].pnt;
	Vector3f norm_0 = mesh_data->vertices[0].nrm;
	Vector3f norm_1 = mesh_data->vertices[1].nrm;
	Vector3f norm_2 = mesh_data->vertices[2].nrm;
	Vector2f tex_0 = mesh_data->vertices[0].tex;
	Vector2f tex_1 = mesh_data->vertices[1].tex;
	Vector2f tex_2 = mesh_data->vertices[2].tex;
	Intersection intersect_ = intersect_triangle(ray, vert_0, vert_1, vert_2,
		norm_0, norm_1, norm_2, tex_0, tex_1, tex_2, mesh_data->isVertNorm, mesh_data->isTexCoord);
	// closest triangle
	if (intersect_.t == INF)
		return intersect_;
	if (intersect_.t < smallest_t)
		smallest_t = intersect_.t;
	return intersect_;
}
Intersection Mesh::intersect_triangle(Ray* ray, Vector3f vert0, Vector3f vert1, Vector3f vert2, Vector3f vert_norm0, Vector3f vert_norm1, Vector3f vert_norm2,
	Vector2f tex0, Vector2f tex1, Vector2f tex2, bool isNorm, bool isTex)
{
	Vector3f E1 = vert1 - vert0;
	Vector3f E2 = vert2 - vert0;
	Vector3f p = ray->GetDirection().normalized().cross(E2);
	float d = p.dot(E1);
	if (d == 0) return Intersection(); // no intersection, ray is parallel to triangle
	
	Vector3f S = ray->GetStartPoint() - vert0;
	float u = (p.dot(S)) / d;
	if (u < 0.f || u > 1.f) return Intersection(); //no intersection, ray intersects plane, but outside E2 edge
	Vector3f q = S.cross(E1);
	float v = (ray->GetDirection().normalized().dot(q)) / d;
	if (v < 0 || (u + v > 1.f)) return Intersection(); //no intersection, intersects plane, but outside other edges
	float t_ = E2.dot(q) / d;
	if (t_ < 0) return Intersection(); // ray's negative half intersects triangle
	Intersection result = Intersection();
	result.t = t_;
	result.intersect_point = ray->eval(t_);
	
	if(isNorm)
		result.intersect_normal = (1.f - u - v) * vert_norm0 + u * vert_norm1 + v * vert_norm2;
	else
		result.intersect_normal = E2.cross(E1);
	result.intersect_normal.normalize();
	float abs_max = result.intersect_normal.x();
	if (abs_max < result.intersect_normal.y())
		abs_max = result.intersect_normal.y();
	if (abs_max < result.intersect_normal.z())
		abs_max = result.intersect_normal.z();
	if (isTex)
		result.UV_coord = (1.f - u - v) * tex0 + u * tex1 + v * tex2;
	else
	{
		Vector3f u_ = computePrimaryTexDir(result.intersect_normal);
		Vector3f v_ = result.intersect_normal.cross(u_);
		result.UV_coord = Vector2f(u_.dot(result.intersect_point), v_.dot(result.intersect_point));
	}
	return result;
}
Bbox bounding_box(Shape* obj)
{
	return obj->bbox();
}
void Scene::AverageColor(Color* image)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			if(isnan(image[y * width + x].x()) || isnan(image[y * width + x].y()) || isnan(image[y * width + x].z()) || 
				isinf(image[y * width + x].x()) || isinf(image[y * width + x].y()) || isinf(image[y * width + x].z()))
			{
				Vector3f save_neighbor = Vector3f(0.f,0.f,0.f);
				int neighbor_count = 0;
				for (int col = -1; col <= 1; ++col)
				{
					for (int row = -1; row <= 1; ++row)
					{
						if (col == 0 && row == 0)
							continue;
						if (y + col < 0 || x + row < 0 || y + col >= height || x + row >= width)
							continue;
						if (!isnan(image[(y + col) * width + (x + row)].x()) && !isnan(image[(y + col) * width + (x + row)].y()) && !isnan(image[(y + col) * width + (x + row)].z())
							&& !isinf(image[y * width + x].x()) && !isinf(image[y * width + x].y()) && !isinf(image[y * width + x].z()))
						{
							save_neighbor += Vector3f(image[(y + col) * width + (x + row)]);
							++neighbor_count;
						}
					}
				}
				if(neighbor_count)
					image[y * width + x] = Color(save_neighbor / neighbor_count);
				//std::cout << "(" << x << ", " << y << ") : neighbor_count : " << neighbor_count << image[y * width + x] << std::endl;
			}
		}
	}
}
void Scene::TraceImage(Color* image, const int pass)
{
	Tree = KdBVH<float, 3, Shape*>(shapes.begin(), shapes.end());
	const float rx = cam->GetRatio() * width / height;
	const Vector3f X = rx * cam->GetOrientation()._transformVector(Vector3f::UnitX()); // eigen to vector
	const Vector3f Y = cam->GetRatio() * cam->GetOrientation()._transformVector(Vector3f::UnitY());
	const Vector3f Z = -1.0f * cam->GetOrientation()._transformVector(Vector3f::UnitZ());
	for (int curr_pass = 1; curr_pass <= pass; ++curr_pass)
	{
//#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				const float focal_len = 7.f;
				float r_x = 0.f, r_y = 0.f;
				cam->DepthOfField(r_x, r_y, focal_len, 0.5f);
				float dx = (2.0f * (static_cast<float>(x) + myrandom(RNGen)) / width) - 1.0f;
				float dy = (2.0f * (static_cast<float>(y) + myrandom(RNGen)) / height) - 1.0f;
				//Vector3f dir_ = dx * X + dy * Y + Z;
				//dir_.normalize();
				Vector3f dir_ = (focal_len * dx - r_x) * X + (focal_len * dy - r_y) * Y + focal_len * Z;
				Ray ray(cam->GetEye() + r_x * X + r_y * Y, dir_);
				//Ray ray(cam->GetEye(), dir_);
				Color curr_color = Color(0.0f, 0.0f, 0.0f);
				// check intersect
				Minimizer minimizer(ray);
				float minDist = BVMinimize(Tree, minimizer); // return smallest t and intersection in minimizer
				if (minDist == INF)
					continue;
				//Vector3f L = pos - minimizer.intersect_.intersect_point;
				//curr_color = minimizer.intersect_.intersect_normal.dot(L) * minimizer.intersect_.intersect_obj->GetMat()->Kd / PI;
				
				//curr_color = Color(abs(minimizer.intersect_.intersect_normal.x()),
				//	abs(minimizer.intersect_.intersect_normal.y()),
				//	abs(minimizer.intersect_.intersect_normal.z())); // normal

				//curr_color = Color((minDist - 5.f) / 12.f,
				//	(minDist - 5.f) / 12.f,
				//	(minDist - 5.f) / 12.f); // value t
				/*if (minimizer.intersect_.intersect_obj->GetMat()->isTexture())
				{
					Material* temp = minimizer.intersect_.intersect_obj->GetMat();
					int tex_width_ = static_cast<int>(static_cast<float>(temp->tex_width) * minimizer.intersect_.UV_coord.x());
					int tex_height_ = static_cast<int>(static_cast<float>(temp->tex_height) * minimizer.intersect_.UV_coord.y());
					int px = tex_width_ + (tex_height_ * temp->tex_width);//tex_height_ + (tex_width_ * temp->tex_width);
					if (px > temp->pixels.size())
						px = px % temp->pixels.size();
					else if (px < 0)
						px = abs(px);
					curr_color = temp->pixels[px];
				}
				else
					curr_color = minimizer.intersect_.intersect_obj->GetMat()->Kd;*/ // diffuse color

				image[y * width + x] += TracePath(&ray);
				//image[y * width + x] = curr_color;
			}
		}
		if (curr_pass % 10 == 0)
		{
			std::cout << "curr pass : " << curr_pass << std::endl;
		}
	}
	// ocasionally:
	// write image[x,y] /= pass
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			image[y * width + x] /= static_cast<float>(pass);
		}
	}

	AverageColor(image);

    fprintf(stderr, "\n");
}
// Start with the given ray and iterate it through a series of bounces,
// recording and returning any light found.
Color Scene::TracePath(Ray* ray)
{
	BRDFType b_type = PHONG; // GGX, BECKMAN : alpha = 0.01 good.
	Vector3f C = Vector3f(0.f,0.f,0.f);
	Vector3f W = Vector3f(1.f, 1.f, 1.f);

	// Trace ray into the scene to first intersection
	Ray P_ray = Ray(ray->GetStartPoint(), ray->GetDirection()); // ??

	Minimizer P(P_ray);
	float minDist = BVMinimize(Tree, P); // return smallest t and intersection in minimizer
	if (minDist == INF) return Color(C);
	if (P.intersect_.intersect_obj->GetMat()->isLight())
		return P.intersect_.intersect_obj->GetMat()->emission; // return radiance. Should implement radiance method
	Vector3f w_0 = -P_ray.GetDirection().normalized();
	while (myrandom(RNGen) <= Russianroulette) // random() <= 0.8
	{
		Vector3f N = P.intersect_.intersect_normal.normalized();

		// EXPLICIT LIGHT CONNECTION
		// Choose a random point on (random) light
		float l_radius = 0.f;
		Intersection L = SampleLight(l_radius); // ??
		float p_light = PdfLight(L, l_radius) / GeometryFactor(P.intersect_, L);

		// EXTEND PATH
		// Genreate a new ray from P in some random direction (using importance sampling)
		float kd_len = Vec_length(P.intersect_.intersect_obj->GetMat()->Kd);
		float kr_len = Vec_length(P.intersect_.intersect_obj->GetMat()->Ks);
		float kt_len = Vec_length(P.intersect_.intersect_obj->GetMat()->Kt);
		float s = kd_len + kr_len + kt_len;
		float p_d = kd_len / s;
		float p_r = kr_len / s;
		float p_t = kt_len / s;
		
		Vector3f w_i_light_vec = (L.intersect_point - P.intersect_.intersect_point).normalized();
		Ray w_i_ray_light(P.intersect_.intersect_point, w_i_light_vec);
		Minimizer I(w_i_ray_light);
		float w_i_minDist = BVMinimize(Tree, I);
		float q = PdfBrdf(w_0, N, w_i_light_vec, p_d, p_r, p_t, P.intersect_.intersect_obj->GetMat()->alpha, P.intersect_.intersect_obj->GetMat()->ior, b_type) * Russianroulette;
		float w_mis = (p_light * p_light) / (p_light * p_light + q * q);
		if ((p_light > 0.f) && (w_i_minDist != INF) && I.intersect_.intersect_obj->GetMat()->isLight())
		{
			Vector3f diffuse_color = P.intersect_.intersect_obj->GetMat()->Kd;
			if (P.intersect_.intersect_obj->GetMat()->isTexture())
			{
				Material* temp = P.intersect_.intersect_obj->GetMat();
				int tex_width_ = static_cast<int>(static_cast<float>(temp->tex_width) * P.intersect_.UV_coord.x());
				int tex_height_ = static_cast<int>(static_cast<float>(temp->tex_height) * P.intersect_.UV_coord.y());
				int px = tex_width_ + (tex_height_ * temp->tex_width);
				if (px > temp->pixels.size())
					px = px % temp->pixels.size();
				else if (px < 0)
					px = abs(px);
				diffuse_color = temp->pixels[px];
			}
			Vector3f f_ = EvalScattering(w_0, N, w_i_light_vec, diffuse_color, P.intersect_.intersect_obj->GetMat()->Ks, P.intersect_.intersect_obj->GetMat()->Kt,
				P.intersect_.intersect_obj->GetMat()->alpha, P.intersect_.intersect_obj->GetMat()->ior, w_i_minDist, b_type);
			Vector3f front = W.cwiseProduct(w_mis * f_ / p_light);
			C = C + front.cwiseProduct(EvalRadiance(L));
		}

		Vector3f w_i_vec = SampleBrdf(w_0, N, p_d, p_r, P.intersect_.intersect_obj->GetMat()->alpha, P.intersect_.intersect_obj->GetMat()->ior, b_type);
		w_i_vec.normalize();
		Ray w_i_ray(P.intersect_.intersect_point, w_i_vec);
		// Q = Trace that new ray into the scene to first intersection
		Minimizer Q(w_i_ray);
		float q_minDist = BVMinimize(Tree, Q);

		// if (Q is no intersection) break;
		if (q_minDist == INF) break;
		Vector3f diffuse_color = P.intersect_.intersect_obj->GetMat()->Kd;
		if (P.intersect_.intersect_obj->GetMat()->isTexture())
		{
			Material* temp = P.intersect_.intersect_obj->GetMat();
			int tex_width_ = static_cast<int>(static_cast<float>(temp->tex_width) * P.intersect_.UV_coord.x());
			int tex_height_ = static_cast<int>(static_cast<float>(temp->tex_height) * P.intersect_.UV_coord.y());
			int px = tex_width_ + (tex_height_ * temp->tex_width);
			if (px > temp->pixels.size())
				px = px % temp->pixels.size();
			else if (px < 0)
				px = abs(px);
			diffuse_color = temp->pixels[px];
		}
		const Vector3f f = EvalScattering(w_0, N, w_i_vec, diffuse_color, P.intersect_.intersect_obj->GetMat()->Ks, P.intersect_.intersect_obj->GetMat()->Kt,
			P.intersect_.intersect_obj->GetMat()->alpha, P.intersect_.intersect_obj->GetMat()->ior, q_minDist,b_type);
		const float p = PdfBrdf(w_0, N, w_i_vec, p_d, p_r, p_t, P.intersect_.intersect_obj->GetMat()->alpha, P.intersect_.intersect_obj->GetMat()->ior, b_type) * Russianroulette;

		if(p < epsilon_2) break;
		W = W.cwiseProduct(f/p);

		// IMPLICIT LIGHT CONNECTION
		if(Q.intersect_.intersect_obj->GetMat()->isLight())
		{
			q = PdfLight(Q.intersect_, l_radius) / GeometryFactor(P.intersect_, Q.intersect_);
			w_mis = (p * p) / (p * p + q * q);
			W = W * w_mis;
			C = C + W.cwiseProduct(EvalRadiance(Q.intersect_));
			break;
		}
		// STEP FORWARD
		P.intersect_ = Q.intersect_;
		w_0 = -w_i_vec;
	}
	return Color(C);
}
void Camera::DepthOfField(float& r_x, float& r_y, float focal_d, const float W) // W = size of the circle of confusion
{
	const float r = W * sqrtf(myrandom(RNGen));
	const float theta = 2.f * PI * r * myrandom(RNGen);
	r_x = r * cosf(theta);
	r_y = r * sinf(theta);
}