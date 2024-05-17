// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
#ifndef GAME_MATH_H
#define GAME_MATH_H

#include <cstdlib>
#include <SDL_rect.h>

static const float PI = 3.14159265358979323846;

inline float toRadians(float deg) {
	return deg * PI/180;
}

inline float toDegrees(float rad) {
	return rad * 180/PI;
}

struct Vector2f {
	float x;
	float y;

	Vector2f() : Vector2f(0.0f, 0.0f) {}
	Vector2f(float x, float y) : x(x), y(y) {}

    // Vector2f + Vector2f | OPTIMISE this can be inefficient for more than two vectors
    Vector2f operator + (const Vector2f& v) const { return {x + v.x, y + v.y}; }
    // Vector2f - Vector2f | OPTIMISE this can be inefficient for more than two vectors
    Vector2f operator - (const Vector2f& v) const { return {x - v.x, y - v.y}; }

    // Vector2f += Vector2f
    Vector2f& operator += (const Vector2f& v) { x += v.x; y += v.y; return *this; }
    // Vector2f -= Vector2f
    Vector2f& operator -= (const Vector2f& v) { x -= v.x; y -= v.y; return *this; }

    // Vector2f * float
    Vector2f operator * (float scalar) const { return {x * scalar, y * scalar}; }
};

struct Vector2i {
	int x;
	int y;

	Vector2i() : Vector2i(0, 0) {}
	Vector2i(int x, int y) : x(x), y(y) {}
};

struct Point2 {
	int x, y;

	Point2() : Point2(0, 0) {}
	Point2(int x, int y) : x(x), y(y) {}

	Point2& operator+=(const Vector2f& v) {
		x += v.x;
		y += v.y;
		return *this;
	}

	Point2& operator-=(const Vector2f& v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}
};

struct Line2i {
	Point2 start, end;

	Line2i() : Line2i(Point2(), Point2()) {}
	Line2i(const Point2& start, const Point2& end) : start(start), end(end) {}
};

struct Rectangle2 {
	int x, y, w, h;

	Rectangle2(int x, int y, int w, int h) : x(x), y(y), w(w), h(h) {}

	SDL_Rect getSDLRect() const {
		SDL_Rect rect = { x, y, w, h };
		return rect;
	}

	inline bool contains(const Point2& p) {
		return p.x >= x && p.x <= x + w
			&& p.y >= y && p.y <= y + h;
	}

	inline bool intersects(const Rectangle2& other) {
		SDL_Rect rect1 = getSDLRect();
		SDL_Rect rect2 = other.getSDLRect();

		return SDL_HasIntersection(&rect1, &rect2) == SDL_TRUE;
	}

	inline bool intersects(const Line2i& line) {
		int x1 = line.start.x, y1 = line.start.y, x2 = line.end.x, y2 = line.end.y;
		SDL_Rect rect = { x, y, w, h };
		return SDL_IntersectRectAndLine(&rect, &x1, &y1, &x2, &y2) == SDL_TRUE;
	}
};

struct Rectangle2f {
	float x, y, w, h;

	Rectangle2f(float x, float y, float w, float h) : x(x), y(y), w(w), h(h) {}

	SDL_Rect getSDLRect() const {
		SDL_Rect rect = { x, y, w, h };
		return rect;
	}

	inline bool contains(const Point2& p) {
		return p.x >= x && p.x <= x + w
			&& p.y >= y && p.y <= y + h;
	}

	inline bool intersects(const Rectangle2f& other) {
		SDL_Rect rect1 = getSDLRect();
		SDL_Rect rect2 = other.getSDLRect();

		return SDL_HasIntersection(&rect1, &rect2) == SDL_TRUE;
	}

	inline bool intersects(const Line2i& line) {
		int x1 = line.start.x, y1 = line.start.y, x2 = line.end.x, y2 = line.end.y;
		SDL_Rect rect = { x, y, w, h };
		return SDL_IntersectRectAndLine(&rect, &x1, &y1, &x2, &y2) == SDL_TRUE;
	}
};

typedef Rectangle2 Rect;
typedef Rectangle2f Rectf;

struct Dimension2i {
	int w, h;

	Dimension2i() : Dimension2i(0, 0) {}
	Dimension2i(int w, int h) : w(w), h(h) {}
};

/**
* Init srand() before to get different random values every time you start program
*
* @return
*			a random integer value between "min" and "max", both inclusive
*/
inline int getRandom(int min, int max) {
	return (int)(rand() % (max - min)) + min;
}

#endif
