// all code resources are taken from https://github.com/AlmasB/xcube2d/tree/master
#ifndef GAME_MATH_H
#define GAME_MATH_H

#include <cmath>
#include <cstdlib>
#include <SDL_rect.h>
#include <cmath>

static const float PI = 3.14159265358979323846;

inline float toRadians(float deg) {
	return deg * PI/180;
}

inline float toDegrees(float rad) {
	return rad * 180/PI;
}

struct Vector2I;
struct Vector2F {
	float x;
	float y;

	Vector2F() : Vector2F(0.0f, 0.0f) {}
	Vector2F(float x, float y) : x(x), y(y) {}
    explicit Vector2F(float s) : x(s), y(s) {}

    /**
     * Get the magnitude of this Vector2F (the scalar distance represented by the Vector2F)
     * @return  The magnitude of this Vector2F
     */
    [[nodiscard]] float getMagnitude() const { return sqrtf( powf(x, 2.f) + powf(y, 2.f) ); }

    /**
     * Get the unit vector of this Vector2F (the vector with the same direction but a magnitude of 1)
     * @return  The unit vector of this Vector2F
     */
    [[nodiscard]] Vector2F getUnitVector() const { return *this / this->getMagnitude(); }

    /**
     * Get the dot product of this Vector2F and the other, given Vector2F
     * @param other The Vector2F to get the dot product of with this Vector2F
     * @return  The dot product of this Vector2F and the other, given Vector2F
     */
    [[nodiscard]] float dot(Vector2F other) const { return x * other.x + y * other.y; }

    /**
     * Get the magnitude of the cross product of this Vector2F and the other, given Vector2F
     * @param other The Vector2F to get the cross product of with this Vector2F
     * @return  The magnitude of the cross product of this Vector2F and the other, given Vector2F
     */
    [[nodiscard]] float cross(Vector2F other) const { return x * other.y - other.x * y; }

    /**
     * Get the scalar distance from this Vector2F to the other, given Vector2F
     * @param other The Vector2F to get the distance to from this Vector2F
     * @return  The scalar distance from this Vector2F to the other, given Vector2F
     */
    [[nodiscard]] float getDistanceTo(Vector2F other) const
    {
        return std::sqrt( powf(other.x - x, 2) + powf(other.y - y, 2) );
    }

    /**
     * Get the scalar distance squared from this Vector2F to the other, given Vector2F
     * @param other The Vector2F to get the distance squared to from this Vector2F
     * @return  The scalar distance squared from this Vector2F to the other, given Vector2F
     */
    [[nodiscard]] float getDistanceSquaredTo(Vector2F other) const
    {
        return powf(other.x - x, 2) + powf(other.y - y, 2);
    }

    /**
     * Get the Vector2F from rotating this Vector2F clockwise round a given origin by a given angle in radians
     * @param angleInRads The angle in radians to rotate this Vector2F by
     * @param origin The origin to rotate this Vector2F around
     * @return The Vector2F given from rotating this Vector2F clockwise round a given origin by a given angle in radians
     */
    [[nodiscard]] Vector2F rotate(float angleInRads, Vector2F origin = {}) const {
        // Calculate sine and cosine of the angle
        float sinAngle = std::sin(angleInRads);
        float cosAngle = std::cos(angleInRads);

        // Get the vector from the origin to the point
        Vector2F translatedVector = *this - origin;

        // Rotate the translated point/vector
        Vector2F rotatedVector{ // Rotate the vector
            translatedVector.x * cosAngle - translatedVector.y * sinAngle,
            translatedVector.x * sinAngle - translatedVector.y * cosAngle
        };

        // Translate the point/vector back to the original position
        return rotatedVector + origin;
    }

    /* Arithmetic Operators */

    // Assignment operators
    //Vector2F & operator = (const Vector2F & v) { x = v.x; y = v.y; return *this; }
    Vector2F& operator = (const float & s) { x = s; y = s; return *this; }
    Vector2F& operator - () { x = -x; y = -y; return *this; }

    //equality operators
    bool operator == (const Vector2F & v) const { return (x == v.x) && (y == v.y); }
    bool operator != (const Vector2F & v) const { return !(*this == v); }

    // Vector2F to this
    Vector2F& operator += (const Vector2F& v) { x += v.x; y += v.y; return *this; }
    Vector2F& operator -= (const Vector2F& v) { x -= v.x; y -= v.y; return *this; }
    Vector2F& operator *= (const Vector2F& v) { x *= v.x; y *= v.y; return *this; }
    Vector2F& operator /= (const Vector2F& v) { x /= v.x; y /= v.y; return *this; }

    // Vector2F to Vector2F
    Vector2F operator + (const Vector2F& v) const { return Vector2F(*this) += v; }
    Vector2F operator - (const Vector2F& v) const { return Vector2F(*this) -= v; }
    Vector2F operator * (const Vector2F& v) const { return Vector2F(*this) *= v; }
    Vector2F operator / (const Vector2F& v) const { return Vector2F(*this) /= v; }

    // Scalar to this
    Vector2F& operator += (const float& s) { x += s; y += s; return *this; }
    Vector2F& operator -= (const float& s) { x -= s; y -= s; return *this; }
    Vector2F& operator *= (const float& s) { x *= s; y *= s; return *this; }
    Vector2F& operator /= (const float& s) { x /= s; y /= s; return *this; }

    //Scalar to Vector2F
    Vector2F operator + (float s) const { return Vector2F(*this) += s; }
    Vector2F operator - (float s) const { return Vector2F(*this) -= s; }
    Vector2F operator * (float s) const { return Vector2F(*this) *= s; }
    Vector2F operator / (float s) const { return Vector2F(*this) /= s; }

    explicit operator Vector2I() const;
};

struct Vector2I {
	int x;
	int y;

	Vector2I() : Vector2I(0, 0) {}
	Vector2I(int x, int y) : x(x), y(y) {}
    explicit Vector2I(int s) : x(s), y(s) {}

    static Vector2I zero() { return Vector2I{}; }

    /**
     * Get the magnitude of this Vector2I (the scalar distance represented by the Vector2I)
     * @return  The magnitude of this Vector2I
     */
    [[nodiscard]] float getMagnitude() const
        { return sqrtf(powf(static_cast<float>(x), 2.f) + powf(static_cast<float>(y), 2.f)); }

    /**
    * Get the unit vector of this Vector2I (the vector with the same direction but a magnitude of 1)
    * @return  The unit vector of this Vector2I
    */
    [[nodiscard]] Vector2F getUnitVector() const { return static_cast<Vector2F>(*this) / this->getMagnitude(); }

    /**
     * Get the dot product of this Vector2I and the other, given Vector2I
     * @param other The Vector2I to get the dot product of with this Vector2I
     * @return  The dot product of this Vector2I and the other, given Vector2I
     */
    [[nodiscard]] int dot(Vector2I other) const { return x * other.x + y * other.y; }

    /**
     * Get the magnitude of the cross product of this Vector2I and the other, given Vector2I
     * @param other The Vector2I to get the cross product of with this Vector2I
     * @return  The magnitude of the cross product of this Vector2I and the other, given Vector2I
     */
    [[nodiscard]] int cross(Vector2I other) const { return x * other.y - other.x * y; }

    /**
     * Get the scalar distance from this Vector2I to the other, given Vector2I
     * @param other The Vector2I to get the distance to from this Vector2I
     * @return  The scalar distance from this Vector2I to the other, given Vector2I
     */
    [[nodiscard]] float getDistanceTo(Vector2I other) const
    {
        return sqrtf( powf(static_cast<float>(other.x - x), 2) + powf(static_cast<float>(other.y - y), 2));
    }

    /**
     * Get the scalar distance squared from this Vector2I to the other, given Vector2I
     * @param other The Vector2I to get the distance squared to from this Vector2I
     * @return  The scalar distance squared from this Vector2I to the other, given Vector2I
     */
    [[nodiscard]] float getDistanceSquaredTo(Vector2I other) const
    {
        return powf(static_cast<float>(other.x - x), 2) + powf(static_cast<float>(other.y - y), 2);
    }

    /* Arithmetic Operators */

    // Assignment operators
    Vector2I& operator = (const int & s) { x = s; y = s; return *this; }
    Vector2I operator - () { x = -x; y = -y; return *this; }

    // Equality operators
    bool operator == (const Vector2I & v) const { return (x == v.x) && (y == v.y); }
    bool operator != (const Vector2I & v) const { return !(*this == v); }

    // Vector2I to this
    Vector2I& operator += (const Vector2I& v) { x += v.x; y += v.y; return *this; }
    Vector2I& operator -= (const Vector2I& v) { x -= v.x; y -= v.y; return *this; }

    // Vector2I to Vector2I
    Vector2I operator + (const Vector2I& v) const { return Vector2I(*this) += v; }
    Vector2I operator - (const Vector2I& v) const { return Vector2I(*this) -= v; }

    // Scalar to this
    Vector2I& operator *= (const int& s) { x *= s; y *= s; return *this; }
    Vector2I& operator /= (const int& s) { x /= s; y /= s; return *this; }

    //Scalar to Vector2F
    Vector2I operator * (int s) const { return Vector2I(*this) *= s; }
    Vector2I operator / (int s) const { return Vector2I(*this) /= s; }

    explicit operator Vector2F() const {
        return {static_cast<float>(x), static_cast<float>(y)};
    }
};

inline Vector2F::operator Vector2I() const {
    return {static_cast<int>(x), static_cast<int>(y)};
}

struct Line2i {
    Vector2I start, end;

    Line2i() : Line2i(Vector2I(), Vector2I()) {}
    Line2i(const Vector2I& start, const Vector2I& end) : start(start), end(end) {}

    [[nodiscard]] Vector2I getNormal() const
    {
        int dx = end.x - start.x;
        int dy = end.y - start.y;
        return {-dy, dx};
    }
};

struct Line2f {
	Vector2F start, end;

	Line2f() : Line2f(Vector2F(), Vector2F()) {}
	Line2f(const Vector2F& start, const Vector2F& end) : start(start), end(end) {}

    [[nodiscard]] Vector2F getNormal() const
    {
        float dx = end.x - start.x;
        float dy = end.y - start.y;
        return {-dy, dx};
    }
};

struct Rectangle2F;
struct Rectangle2I {
	int x, y, w, h;

    Rectangle2I() : x(0), y(0), w(0), h(0) {}
	Rectangle2I(int x, int y, int w, int h) : x(x), y(y), w(w), h(h) {}

	[[nodiscard]] SDL_Rect getSDLRect() const {
		return { x, y, w, h };
	}

	[[nodiscard]] inline bool contains(const Vector2I& p) const {
		return p.x >= x && p.x <= x + w
			&& p.y >= y && p.y <= y + h;
	}

	[[nodiscard]] inline bool intersects(const Rectangle2I& other) const {
		SDL_Rect rect1 = getSDLRect();
		SDL_Rect rect2 = other.getSDLRect();

		return SDL_HasIntersection(&rect1, &rect2) == SDL_TRUE;
	}

	inline bool intersects(const Line2i& line) {
		int x1 = line.start.x, y1 = line.start.y, x2 = line.end.x, y2 = line.end.y;
		SDL_Rect rect = { x, y, w, h };
		return SDL_IntersectRectAndLine(&rect, &x1, &y1, &x2, &y2) == SDL_TRUE;
	}

    // Rectangle2I += Vector2I
    Rectangle2I& operator += (const Vector2I& v) { x += v.x; y += v.y; return *this; }
    // Rectangle2I -= Vector2I
    Rectangle2I& operator -= (const Vector2I& v) { x -= v.x; y -= v.y; return *this; }

    // Rectangle2I + Vector2I
    Rectangle2I operator + (const Vector2I& v) const { return Rectangle2I(*this) += v; }
    // Rectangle2I - Vector2I
    Rectangle2I operator - (const Vector2I& v) const { return Rectangle2I(*this) -= v; }

    explicit operator Rectangle2F() const;

};

struct Rectangle2F {
	float x, y, w, h;

    Rectangle2F() : x(0.f), y(0.f), w(0.f), h(0.f) {}
	Rectangle2F(float x, float y, float w, float h) : x(x), y(y), w(w), h(h) {}

	[[nodiscard]] SDL_FRect getSDLRect() const {
		SDL_FRect rect = { x, y, w, h };
		return rect;
	}

	[[nodiscard]] inline bool contains(const Vector2F& p) const {
		return p.x >= x && p.x <= x + w
			&& p.y >= y && p.y <= y + h;
	}

	[[nodiscard]] inline bool intersects(const Rectangle2F& other) const {
		SDL_FRect rect1 = getSDLRect();
		SDL_FRect rect2 = other.getSDLRect();

		return SDL_HasIntersectionF(&rect1, &rect2) == SDL_TRUE;
	}

	inline bool intersects(const Line2f& line) {
		float x1 = line.start.x, y1 = line.start.y, x2 = line.end.x, y2 = line.end.y;
		SDL_FRect rect = { x, y, w, h };
		return SDL_IntersectFRectAndLine(&rect, &x1, &y1, &x2, &y2) == SDL_TRUE;
	}

    // Rectangle2F += Vector2F
    Rectangle2F& operator += (const Vector2F& v) { x += v.x; y += v.y; return *this; }
    // Rectangle2F -= Vector2F
    Rectangle2F& operator -= (const Vector2F& v) { x -= v.x; y -= v.y; return *this; }

    // Rectangle2F + Vector2F
    Rectangle2F operator + (const Vector2F& v) const { return Rectangle2F(*this) += v; }
    // Rectangle2F - Vector2F
    Rectangle2F operator - (const Vector2F& v) const { return Rectangle2F(*this) -= v; }

    explicit operator Rectangle2I() const {
        return Rectangle2I{static_cast<int>(x),
                           static_cast<int>(y),
                           static_cast<int>(w),
                           static_cast<int>(h)};
    }
};

inline Rectangle2I::operator Rectangle2F() const {
    return Rectangle2F{static_cast<float>(x),
                       static_cast<float>(y),
                       static_cast<float>(w),
                       static_cast<float>(h)};
}

typedef Rectangle2I Rect;
typedef Rectangle2F RectF;

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
