#include <iostream>
#include <vector>

using namespace std;

struct Point {
    float x;
    float y;
};

class Spline {
	private:
		std::vector<Point> points;
		bool bLooped;
	public:
		Spline(std::vector<Point> points) {
			this->points = points;
			this->bLooped = true;
		}

		Spline(std::vector<Point> points, bool bLooped) {
			this->points = points;
			this->bLooped = bLooped;
		}

		Point GetSplinePoint(float t) {
			int p0, p1, p2, p3;

			if (this->bLooped) {
				p1 = (int)t + 1;
				p2 = p1 + 1;
				p3 = p2 + 1;
				p0 = p1 - 1;
			} else {
				p1 = (int)t;
				p2 = (p1 + 1) % points.size();
				p3 = p2 == points.size()-1 ? p2 : (p2 + 1) % points.size();
				p0 = p1 >= 1 ? p1 - 1 : p1 == 0 ? p1 : points.size() - 1;
			}
		
			t = t - (int)t;

			float tt = t * t;
			float ttt = tt * t;

			float q1 = -ttt + 2.0f*tt - t;
			float q2 = 3.0f*ttt - 5.0f*tt + 2.0f;
			float q3 = -3.0f*ttt + 4.0f*tt + t;
			float q4 = ttt - tt;

			float tx = 0.5f * (points[p0].x * q1 + points[p1].x * q2 + points[p2].x * q3 + points[p3].x * q4);
			float ty = 0.5f * (points[p0].y * q1 + points[p1].y * q2 + points[p2].y * q3 + points[p3].y * q4);

			return { tx, ty };
		}

		std::vector<Point> GetPoints(float stepSize) {
			std::vector<Point> points;

			for (float t = 0; t < this->points.size()-1; t += stepSize) {
				Point p = this->GetSplinePoint(t);
				points.push_back(p);
			}

			Point lastPoint = points[points.size()-1];
			Point lastTargetPoint = this->points[this->points.size()-1];

			if(lastPoint.x != lastTargetPoint.x && lastPoint.y != lastTargetPoint.y) {
				points.push_back(lastTargetPoint);
			}

			return points;
		}
};

int main() {
	std::vector<Point> inputPoints = {{0,0}, {1, 2}, {2, 3}, {3, 2}, {4, 6}};
    Spline spline(inputPoints,false);

    std::vector<Point> generatedPoints = spline.GetPoints(0.1f);

    for (const auto &point : generatedPoints) {
        std::cout << point.x << "," << point.y << std::endl;
    }
}
