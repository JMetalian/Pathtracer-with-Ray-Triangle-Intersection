#define MODEL "tetraeder.off"
#include <stdlib.h>     // single-source path tracer based on smallpt by Kevin Beason 
#include <stdio.h>      // Make: g++ -O3 -fopenmp cgrpt1.cpp -o cgrpt
#include <omp.h>        // Usage: ./cgrpt <samplesPerPixel> <y-resolution>, e.g.: ./cgrpt 4000 600     
#include <random>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>

using namespace std;

// uniform double random generator function
double rand01() {
	static std::default_random_engine generator;
	static std::uniform_real_distribution<double> distr(0.0, 1.0);
	return distr(generator);
}
inline double clamp(double x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + 0.5); }		// performs gamma correction!
const double PI = 3.141592653589793;

// 3d vector class
struct Vec {
	double x, y, z;

	Vec(double x_ = 0, double y_ = 0, double z_ = 0) { x = x_; y = y_; z = z_; }
	Vec operator+(const Vec& b) const { return Vec(x + b.x, y + b.y, z + b.z); }
	Vec operator-(const Vec& b) const { return Vec(x - b.x, y - b.y, z - b.z); }
	Vec operator*(double b) const { return Vec(x*b, y*b, z*b); }
	Vec operator/(double b) const { double ib = 1.0 / b; return Vec(x*ib, y*ib, z*ib); }
	Vec mult(const Vec& b) const { return Vec(x*b.x, y*b.y, z*b.z); }
	Vec& normalize() { return *this = *this * (1 / sqrt(x*x + y * y + z * z)); }
	double dot(const Vec& b) const { return x * b.x + y * b.y + z * b.z; }
	Vec cross(const Vec& b) const { return Vec(y*b.z - z * b.y, z*b.x - x * b.z, x*b.y - y * b.x); }
	double length() const { return sqrt(x*x + y * y + z * z); }
};

// 3d ray class
struct Ray {
	Vec o, d;		// origin and direction 
	Ray(Vec o_, Vec d_) : o(o_), d(d_) {}
};

// material types
enum Refl_t { DIFF, SPEC, REFR };

// base class for geometric primitives
struct Primitive {
	Vec e, c;      // emission, color
	Refl_t refl;   // reflection type (DIFFuse, SPECular, REFRactive)
	
    virtual double intersect(const Ray& r, Vec& x, Vec& n) const = 0;   // returns distance, 0 if no hit
};


// simple sphere object class
struct Sphere : public Primitive {
	Vec p;			  // position
	double rad;       // radius

	Sphere(double rad_, const Vec& p_, const Vec& e_, const Vec& c_, Refl_t refl_) {
		rad = rad_, p = p_, e = e_, c = c_, refl = refl_;
	}

	virtual double intersect(const Ray& r, Vec& x, Vec& n) const {
		// Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
		Vec op = p - r.o;
		double t, eps = 1e-4, b = op.dot(r.d), det = b * b - op.dot(op) + rad * rad;
		if (det < 0) return 0; else det = sqrt(det);
		if ((t = b - det) < eps && (t = b + det) < eps) return 0;
		x = r.o + r.d*t;
		n = (x - p).normalize();
		return t;
	}
};
struct Triangle:public Primitive
{
	Vec vertex0;
	Vec vertex1;
	Vec vertex2;

	Triangle(
			const Vec& vert0,
			const Vec& vert1,
			const Vec& vert2,
			const Vec& emission,
			const Vec& color,
			Refl_t reflectionType
			)
	{
		vertex0=vert0;
		vertex1=vert1;
		vertex2=vert2;
		c=color;
		e=emission;
		refl=reflectionType;

	}

//REAL TIME RENDERING 4th Edition, Pseuodocode Möller-Trumbore Algorithm
// RayTriIntersect(o, d, p0, p1, p2)
// returns ({REJECT, INTERSECT}, u, v, t);
// 1 : e1 = p1 − p0
// 2 : e2 = p2 − p0
// 3 : q = d × e2
// 4 : a = e1 · q
// 5 : if(a > −ǫ and a < ǫ) return (REJECT, 0, 0, 0);
// 6 : f = 1/a
// 7 : s = o − p0
// 8 : u = f(s · q)
// 9 : if(u < 0.0) return (REJECT, 0, 0, 0);
// 10 : r = s × e1
// 11 : v = f(d · r)
// 12 : if(v < 0.0 or u + v > 1.0) return (REJECT, 0, 0, 0);
// 13 : t = f(e2 · r)
// 14 : return (INTERSECT, u, v, t);

	virtual double intersect(const Ray &ray, Vec& x, Vec& n) const
	{
		const Vec& rayOrigin=ray.o;
		const Vec& directionOfRay=ray.d;
		const float eps = 0.0000001;

		Vec edge1 = vertex1 - vertex0;
		Vec edge2 = vertex2 - vertex0;
		n = directionOfRay.cross(edge2);
		float det = edge1.dot(n);
		
		if (det > -eps && det < eps)
		{
			return 0.0;    // Ray and Triangle are parallel. (Determinant is close to zero)
						 //or missed
		}


		float inverseDet = 1.0/det;
		Vec s = rayOrigin - vertex0;
		float u = s.dot(n) * inverseDet;

		if (u < 0.0 || u > 1.0)
		{
			return 0.0;
		}

		Vec q = s.cross(edge1);
		float v = directionOfRay.dot(q)*inverseDet ;

		if (v < 0.0 || u + v > 1.0 )
		{
			return 0.0;
		}
		
		float t = edge2.dot(q) * inverseDet;
		if (t > eps) // ray intersection
		{
			x = rayOrigin + directionOfRay * t;
			n=edge1.cross(edge2).normalize();
			return t;
		}
		else //line intersection
			return 0.0;
	}
};


//---- setup scene (length units in in meters)
#define BOX_HX	2.6
#define BOX_HY	2
#define BOX_HZ	2.8
std::vector<Primitive*> primitives =
{
	// Cornell Box centered in the origin (0, 0, 0)
	new Sphere(1e5, Vec(-1e5 - BOX_HX, 0, 0), Vec(), Vec(0.85, 0.25, 0.25), DIFF),  // Left
	new Sphere(1e5, Vec(1e5 + BOX_HX, 0, 0),  Vec(), Vec(0.25, 0.35, 0.85), DIFF),  // Right
	new Sphere(1e5, Vec(0, 1e5 + BOX_HY, 0),  Vec(), Vec(0.75, 0.75, 0.75), DIFF),  // Top
	new Sphere(1e5, Vec(0,-1e5 - BOX_HY, 0),  Vec(), Vec(0.75, 0.75, 0.75), DIFF),  // Bottom
	new Sphere(1e5, Vec(0, 0, -1e5 - BOX_HZ), Vec(), Vec(0.75, 0.75, 0.75), DIFF),  // Back 
	new Sphere(1e5, Vec(0, 0,  1e5 + 3 * BOX_HZ - 0.5), Vec(), Vec(), DIFF),        // Front
	// Objects
	new Sphere(0.8, Vec(-1.3, -BOX_HY + 0.8, -1.3), Vec(), Vec(1,1,1) * 0.999, SPEC), // mirroring
	new Sphere(0.8, Vec(1.3, -BOX_HY + 0.8, -0.2), Vec(), Vec(1,1,1) * 0.999, REFR),  // refracting
    // The ceiling area light source (slightly yellowish color)
	new Sphere(10, Vec(0, BOX_HY + 10 - 0.04, 0), Vec(0.98, 1., 0.9) * 15, Vec(0.0, 0.0, 0.0), DIFF),
};


inline bool intersect(const Ray &r, int& id, double &t, Vec& x, Vec& n) 
{
	Vec xmin, nmin;
	double d, inf = t = 1e20;
	for (int i = primitives.size(); i--;)
	{
		if ((d = primitives[i]->intersect(r, xmin, nmin)) && d < t)
		{
			t = d;
			id = i;
			x = xmin;
			n = nmin;
		}
	}
	return t < inf;
}


#define MAX_DEPTH 12
Vec radiance(const Ray &r, int depth) 
{
	double t;		// distance to intersection point
	int id = 0;		// id of intersected object
	Vec x, n;		// intersected surface point and normal

	if (!intersect(r, id, t, x, n))
		return Vec();	// if ray doesn't hit anything, return black

	const Primitive &obj = *primitives[id]; // the hit object
	Vec f = obj.c;							// surface color;
	double p = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; // max refl
	Vec nl = n.dot(r.d) < 0 ? n : n * -1;	// normal facing towards ray
	
	//--- Russian Roulette Ray termination
	if (++depth > 5) {
		if (rand01() < p && depth < MAX_DEPTH)
			f = f * (1 / p);
		else
			return obj.e;	// R.R.
	}

	//--- Ideal DIFFUSE reflection
	if (obj.refl == DIFF) 
    {
		// cosinus-weighted importance sampling
		double r1 = 2 * PI * rand01(), r2 = rand01(), r2s = sqrt(r2);
		Vec w = nl, u = ((fabs(w.x)>.1 ? Vec(0, 1) : Vec(1)).cross(w)).normalize(), v = w.cross(u);
		Vec d = (u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).normalize();

		return obj.e + f.mult(radiance(Ray(x, d), depth));
	}
	//--- Ideal SPECULAR reflection
	else if (obj.refl == SPEC) 
    {
		return obj.e + f.mult(radiance(Ray(x, r.d - n * 2 * n.dot(r.d)), depth));
	}
	//--- Ideal dielectric REFRACTION
	else {
		Ray reflRay(x, r.d - n * 2 * n.dot(r.d));
		bool into = n.dot(nl) > 0;					// Ray from outside going in?
		double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = r.d.dot(nl), cos2t;

		// Total internal reflection
		if ((cos2t = 1 - nnt * nnt*(1 - ddn * ddn)) < 0)
			return (obj.e + f.mult(radiance(reflRay, depth)));

		Vec tdir = (r.d*nnt - n * ((into ? 1 : -1)*(ddn*nnt + sqrt(cos2t)))).normalize();
		double a = nt - nc, b = nt + nc, R0 = a*a/(b*b), c = 1 - (into ? -ddn : tdir.dot(n));
		double Re = R0 + (1 - R0)*c*c*c*c*c, Tr = 1 - Re, P = .25 + .5*Re, RP = Re / P, TP = Tr / (1 - P);

		Vec L = depth > 2 ?
			(rand01() < P ? radiance(reflRay, depth)*RP : radiance(Ray(x, tdir), depth) * TP) :	// Russian roulette
			radiance(reflRay, depth)*Re + radiance(Ray(x, tdir), depth)*Tr;

		return obj.e + f.mult(L);
	}
}


int main(int argc, char *argv[]) 
{
	fstream theShape(MODEL);
	string token;
	vector<string> allOfContent;

	while (getline(theShape,token))
	{
		istringstream stringStream(token);
		while (stringStream >> token)
		{
			allOfContent.push_back(token);
		}
	}
	for(int i=1; i<allOfContent.size();i++)
	{
		allOfContent.at(i-1)=allOfContent.at(i);
	}

	int vertexStoppingCriteria=stoi(allOfContent.at(0)); //4
	int indexStoppingCriteria=stoi(allOfContent.at(1)); //4

	vector<float> vertexVector;
	
	for (int i = 0; i < vertexStoppingCriteria*3; i++)
	{
		vertexVector.push_back(std::stof(allOfContent[i+3]));
	}

	int indexStartingPoint=3+vertexStoppingCriteria*3;

	vector<int> indexVector;
	for (int i = 0; i < indexStoppingCriteria*4; i++)
	{
		indexVector.push_back(std::stoi(allOfContent.at(indexStartingPoint+i)));
	}
	for(int i=0;i<indexVector.size();i+=3)
	{
		indexVector.erase(indexVector.begin() + i );
	}


	vector<Vec> groupVertexVector;
	for(int i=0; i <= vertexVector.size()-3; i+=3)
	{
		groupVertexVector.push_back(Vec(vertexVector.at(i),vertexVector.at(i+1),vertexVector.at(i+2)));
	}
	
	for (int i = 0; i <= indexVector.size()-3; i+=3)
	{
		primitives.push_back(new Triangle(groupVertexVector.at(indexVector.at(i)), 
										  groupVertexVector.at(indexVector.at(i+1)), 
										  groupVertexVector.at(indexVector.at(i+2)),Vec(), Vec(1, 1., 0.9),SPEC));
	}


    //-- parameter info
	if (argc >= 2 && *argv[1] == '?') {
        printf("./cgrpt1 <samplesPerPixel = 4000> <y-resolution = 600>\n");
        exit (0);
    }

	//-- set number of threads
	omp_set_num_threads(8);
	#pragma omp parallel
	#pragma omp master
	{ fprintf(stderr, "using %d threads\n", omp_get_num_threads()); }

	//-- setup sensor 
	Vec so(0, 0.26 * BOX_HY, 3 * BOX_HZ - 1.0);	// sensor origin
	Vec sd = Vec(0, -0.06, -1).normalize();		// sensor view direction (normal to sensor plane)
	double sw = 0.036;							// sensor width
	double sh = 0.024;							// sensor height
	double f  = 0.035;							// focal length in m
	int resy = argc == 3 ? atoi(argv[2]) : 600;	// vertical pixel resolution
	int resx = resy * 3 / 2;					// horizontal pixel resolution
	Vec* pixels = new Vec[resx * resy];			// pixel buffer

	//-- orthogonal axes spanning the sensor plane
	Vec su = sd.cross(fabs(sd.y) < 0.9 ? Vec(0, 1, 0) : Vec(0, 0, 1)).normalize();
	Vec sv = su.cross(sd);

	//-- number of samples per pixel
	int nSamplesPerPixel = argc >= 2 ? atoi(argv[1]) : 4000;
	
	auto tstart = std::chrono::system_clock::now();		// take start time
	#pragma omp parallel for schedule(dynamic, 1)		// OpenMP 
	for (int y = 0; y < resy; y++) 						// loop over image rows 
	{					
		fprintf(stderr, "\rRendering (%d spp) %5.2f%%", nSamplesPerPixel, 100.0 * y / (resy - 1));
		for (int x = 0; x < resx; x++)		// loop over image columns 
		{	
			Vec r;
			for (int s = 0; s < nSamplesPerPixel; s++)	// shoot n samples per pixel
			{
				int ysub = (s / 2) % 2, xsub = s % 2;	// map to 2x2 subpixel rows and cols
				
				// sample sensor subpixel in [-1,1]
				double r1 = 2 * rand01(), xfilter = r1<1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
				double r2 = 2 * rand01(), yfilter = r2<1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);

				// x and y sample position on sensor plane
				double sx = ((x + 0.5 * (0.5 + xsub + xfilter)) / resx - 0.5) * sw;
				double sy = ((y + 0.5 * (0.5 + ysub + yfilter)) / resy - 0.5) * sh;

				Vec spos = so + su * sx + sv * sy;	  // 3d sample position on sensor
				Vec lc = so + sd * f;				  // lens center (pinhole point)
				Ray ray(lc, (lc - spos).normalize()); // ray through pinhole

				r = r + radiance(ray, 0);		// evaluate radiance from this ray and accumulate
			}
			r = r / nSamplesPerPixel;			// normalize radiance by number of samples

			int i = (resy - y - 1)*resx + x;	// buffer location of this pixel
			pixels[i] = pixels[i] + Vec(clamp(r.x), clamp(r.y), clamp(r.z));
		}
	}
	auto tend = std::chrono::system_clock::now();

	//-- write inverse sensor image to file
	FILE *file = fopen("image.ppm", "w");
	fprintf(file, "P3\n");
	fprintf(file, "# spp: %d\n", nSamplesPerPixel);
	fprintf(file, "# rendering time: %f s\n", std::chrono::duration_cast<std::chrono::duration<double>>(tend - tstart).count());
	fprintf(file, "%d %d\n%d\n", resx, resy, 255);
	for (int i = resx * resy; i--;)
		fprintf(file, "%d %d %d ", toInt(pixels[i].x), toInt(pixels[i].y), toInt(pixels[i].z));
}