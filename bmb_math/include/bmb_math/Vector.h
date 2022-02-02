#pragma once

#include <array>
#include <initializer_list>
#include <cmath>
#include <functional>
#include <string>
#include <sstream>
#include <fstream>

template<typename T, size_t n>
class Vector {
public:
    using iterator = typename std::array<T, n>::iterator;
    using const_iterator = typename std::array<T, n>::const_iterator;
protected:
    std::array<T, n> data{};
public:
    Vector() = default;

    Vector(std::initializer_list<T> elements) {
        // Function caller must ensure the number of arguments matches the template argument
        // Excess arguments will be ignored
        size_t i = 0;
        for (auto it = elements.begin(); i < n && it != elements.end(); it++)
            data[i++] = *it;
    }

    explicit Vector(const T *arr) {
        // initializes this Vector with data from the raw C++ array
        // this should only be used when inputting data from MATLAB
        for (size_t i = 0; i < n; i++) data[i] = arr[i];
    }

    Vector& operator=(const Vector<T, n> &other) {
        for (size_t i = 0; i < n; i++) data[i] = other[i];
        return *this;
    }

    Vector(const Vector<T, n> &other) {
        *this = other;
    }

    template<typename OStream>
    void toCSV(OStream& out) const {
        for (int i = 0; i < n; i++) out << data[i] << std::endl;
    }

    T magnitudeSquared() const {
        T sum{};
        for (size_t i = 0; i < n; i++) sum += data[i] * data[i];
        return sum;
    }

    T magnitude() const {
        return sqrt(magnitudeSquared());
    }

    void normalize() {
        T size = this->magnitude();
        for (size_t i = 0; i < n; i++) data[i] /= size;
    }

    T dot(const Vector<T, n> &other) const {
        T sum{};
        for (size_t i = 0; i < n; i++) sum += data[i] * other[i];
        return sum;
    }

    template<int m>
    Vector<T, n + m> concatenate(const Vector<T, m> &other) const {
        Vector<T, n + m> concat;
        for (size_t i = 0; i < n; i++) concat[i] = data[i];
        for (size_t i = n; i < n + m; i++) concat[i] = other[i];
        return concat;
    }

    template<typename R>
    Vector<R, n> applyFunc(const std::function<R(const T &)> &func) const {
        Vector<R, n> result;
        for (size_t i = 0; i < n; i++) result[i] = func(data[i]);
        return result;
    }

    Vector<T, n> operator+(const Vector<T, n> &other) const {
        Vector<T, n> sum;
        for (size_t i = 0; i < n; i++) sum[i] = data[i] + other[i];
        return sum;
    }

    void operator+=(const Vector<T, n> &other) {
        for (size_t i = 0; i < n; i++) data[i] += other[i];
    }

    Vector<T, n> operator+(const double &scalar) {
        Vector<T, n> sum;
        for (size_t i = 0; i < n; i++) sum[i] = data[i] + scalar;
        return sum;
    }

    void operator+=(const double &scalar) {
        for (size_t i = 0; i < n; i++) data[i] += scalar;
    }

    Vector<T, n> operator-() const {
        Vector<T, n> negative;
        for (size_t i = 0; i < n; i++) negative[i] = -data[i];
        return negative;
    }

    Vector<T, n> operator-(const Vector<T, n> &other) const {
        Vector<T, n> sum;
        for (size_t i = 0; i < n; i++) sum[i] = data[i] - other[i];
        return sum;
    }

    void operator-=(const Vector<T, n> &other) {
        for (size_t i = 0; i < n; i++) data[i] -= other[i];
    }

    Vector<T, n> operator-(const double &scalar) {
        Vector<T, n> sum;
        for (size_t i = 0; i < n; i++) sum[i] = data[i] - scalar;
        return sum;
    }

    void operator-=(const double &scalar) {
        for (size_t i = 0; i < n; i++) data[i] -= scalar;
    }

    Vector<T, n> operator*(const T &scalar) const {
        Vector<T, n> product;
        for (size_t i = 0; i < n; i++)
            product[i] = data[i] * scalar; // respect operator order in case the underlying type is non-commutative
        return product;
    }

    void operator*=(const T &scalar) {
        for (size_t i = 0; i < n; i++) data[i] *= scalar;
    }

    Vector<T, n> operator*(const Vector<T, n> &other) const {
        // elementwise multiplication
        Vector<T, n> product;
        for (size_t i = 0; i < n; i++) product[i] = data[i] * other[i];
        return product;
    }

    void operator*=(const Vector<T, n> &other) {
        // elementwise multiplication
        for (size_t i = 0; i < n; i++) data[i] *= other[i];
    }

    Vector<T, n> operator/(const T &scalar) const {
        Vector<T, n> product;
        for (size_t i = 0; i < n; i++) product[i] = data[i] / scalar;
        return product;
    }

    void operator/=(const T &scalar) {
        for (size_t i = 0; i < n; i++) data[i] /= scalar;
    }

    Vector<T, n> operator/(const Vector<T, n> &other) const {
        // elementwise division
        Vector<T, n> quotient;
        for (size_t i = 0; i < n; i++) quotient[i] = data[i] / other[i];
        return quotient;
    }

    void operator/=(const Vector<T, n> &other) {
        // elementwise division
        for (size_t i = 0; i < n; i++) data[i] /= other[i];
    }

    T &operator[](const size_t &index) {
        return this->data[index];
    }

    const T &operator[](const size_t &index) const {
        return this->data[index];
    }

    iterator begin() {
        return data.begin();
    }

    iterator end() {
        return data.end();
    }

    const_iterator begin() const {
        return data.begin();
    }

    const_iterator end() const {
        return data.end();
    }

    [[nodiscard]] std::string toStr() const {
        if (n == 0) return "{}";
        std::stringstream out;
        out << "{" << data[0];
        for (size_t i = 1; i < n; i++) out << ", " << data[i];
        out << "}";
        return out.str();
    }
};

template<typename T, size_t n>
Vector<T, n> operator+(const T &scalar, const Vector<T, n> &vec) {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++)
        sum[i] = scalar + vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, size_t n>
Vector<T, n> operator-(const T &scalar, const Vector<T, n> &vec) {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++)
        sum[i] = scalar - vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, size_t n>
Vector<T, n> operator*(const T &scalar, const Vector<T, n> &vec) {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++)
        sum[i] = scalar * vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}

template<typename T, size_t n>
Vector<T, n> operator/(const T &scalar, const Vector<T, n> &vec) {
    Vector<T, n> sum;
    for (size_t i = 0; i < n; i++)
        sum[i] = scalar / vec[i]; // respect operator order in case underlying type is non-commutative
    return sum;
}