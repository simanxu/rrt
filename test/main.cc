#include <cmath>
#include <iostream>

using namespace std;

struct Point3D {
  double x, y, z;
};

struct Sphere {
  Point3D center;
  double radius;
};

double distance(Point3D p1, Point3D p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

bool lineIntersectsSphere(Point3D p1, Point3D p2, Sphere s) {
  double d1 = distance(p1, s.center);
  double d2 = distance(p2, s.center);
  double d3 = distance(p1, p2);
  double a = (d1 * d1 - d2 * d2 + s.radius * s.radius) / (2 * d3);
  double h = sqrt(s.radius * s.radius - a * a);
  Point3D p3 = {p1.x + a * (p2.x - p1.x) / d3, p1.y + a * (p2.y - p1.y) / d3, p1.z + a * (p2.z - p1.z) / d3};
  double d4 = distance(p3, s.center);
  if (d4 <= h) {
    return true;
  } else {
    return false;
  }
}

int main() {
  Point3D p1 = {5.16578, 4.79874, 2.7062};
  Point3D p2 = {6.34863, 6.66789, 5.39642};
  Sphere s = {{5, 5, 5}, 2.0};
  if (lineIntersectsSphere(p1, p2, s)) {
    cout << "The line intersects the sphere." << endl;
  } else {
    cout << "The line does not intersect the sphere." << endl;
  }
  return 0;
}
