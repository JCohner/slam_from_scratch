#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1 - d2) < epsilon){
            return true;
        } else {
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        double rad = deg * PI / 180.0;
        return rad; 
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        double deg = rad * 180.0 / PI;
        return deg;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;
    };

    //I tried to make this nice, but it mainly seems contrived, would like some feedback on how to make it better
    class NormVector2D : Vector2D {
        Vector2D norm;
        double mag; 
        void normalize(Vector2D vec){
            mag = sqrt((pow(norm.x,2) + pow(norm.y,2)));
            norm.x = vec.x / mag;
            norm.y = vec.y / mag;

        }
        public:
            NormVector2D();
            explicit NormVector2D(const Vector2D & dir){
                normalize(dir);
            }
            Vector2D reorient(Vector2D dir){
                normalize(dir);
                return norm;
            }
            Vector2D get(){
                return norm;
            }
    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);


    struct Twist2D
    {
        //Angular rotation (oh boy did this as a class at first and learned why there are reasons one does not do that)
        double omega;
        Vector2D vel;
    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// dtheta (degrees/s): 90 vx: 3 vy: 5
    /// \param os - an output stream
    /// \param v - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & V);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees/s, vx, vy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Twist2D & V);




    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        Transform2D(double theta, double ctheta, double stheta, double x, double y); //angle, sin, cos, x ,y
        double theta, ctheta, stheta, x, y;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief from Transform2D calculates adjoint
        /// applies adjoint to Twist2D to return Twist2D in new frame
        /// \param Twist2D to convert
        /// \returns Twist2D in a new frame defined by the adjoint of this Transform2D 
        Twist2D adjoint(Twist2D V) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
        
        /// \brief \see operator>>(...) (declared outside this class)
        /// for a description
        friend std::istream & operator>>(std::istream & is, Transform2D & tf);
        
        /// \brief \see operator*(...) (declared outside this class)
        /// for a description
        friend Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
        friend Transform2D& operator*=(Transform2D lhs, const Transform2D & rhs);
    };

    /*MATT QUESTION: why does the redeclaration of this function not cause an error, how is this unique to 'friend'*/
    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function can be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
    Transform2D& operator*=(Transform2D& lhs, const Transform2D & rhs);

}

#endif