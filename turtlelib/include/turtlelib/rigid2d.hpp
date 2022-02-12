#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1-d2) < epsilon){
            return true;
        }
        else{
            return false;
        }
    }
    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        double rad = (PI * deg)/(180.0);
        return rad;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        double deg = (180.0 * rad)/(PI);
        return deg;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0.0, 0.0), "is_zero failed");
    static_assert(almost_equal(5.0, 5.0), "is_zero failed");
    static_assert(almost_equal(PI, PI), "is_zero failed");
    // static_assert(almost_equal(1.0, 0), "is_zero failed"); //should fail

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(90.0), PI/2), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");
    // static_assert(almost_equal(deg2rad(0), PI), "deg2rad failed"); //should fail

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg) failed");
    static_assert(almost_equal(rad2deg((3*PI)/2), 270.0), "rad2deg) failed");
    // static_assert(almost_equal(rad2deg((3*PI)/2), 0.0), "rad2deg) failed"); //should fail

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(0.0)), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(5.0)), 5.0), "deg2rad failed");
    // static_assert(almost_equal(deg2rad(rad2deg(2.5)), 1.0), "deg2rad failed");//should fail

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief add this vector with another vector and store the result 
        /// in this object
        /// \param rhs - the vector to add to the object
        /// \return a reference to the newly added vector
        Vector2D & operator+=(const Vector2D & rhs);

        /// \brief subtract this vector with another vector and store the result 
        /// in this object
        /// \param rhs - the vector to subtract from the object
        /// \return a reference to the newly subtracted vector
        Vector2D & operator-=(const Vector2D & rhs);

        /// \brief compose this vector with another vector and store the result 
        /// in this object
        /// \param rhs - the vector to multiplty by the object
        /// \return a reference to the newly multiplied vector
        Vector2D & operator*=(const double & scalar);

    };
    /// \brief add two vectors together, returning their composition
    /// \param lhs - the left hand vector
    /// \param rhs - the right hand vector
    /// \return the composition of the two vectors after being added
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract two vectors together, returning their composition
    /// \param lhs - the left hand vector
    /// \param rhs - the right hand vector
    /// \return the composition of the two vectors after the subtraction
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief multiply a scalar by a vector with scalar on the right
    /// \param lhs - the left hand vector
    /// \param scalar - the value to multiply the vector by
    /// \return the composition of the vectors and scalar after being multiplied
    Vector2D operator*(Vector2D lhs, const double & scalar);

    /// \brief multiply a scalar by a vector with scalar on the left 
    /// \param scalar - the value to multiply the vector by
    /// \param lhs - the left hand vector
    /// \return the composition of the vectors and scalar after being multiplied
    Vector2D operator*(const double & scalar, Vector2D lhs);

    /// \brief compute the dot product between two vectors
    /// \param vector1 - the first vector
    /// \param vector2 - the second vector
    /// \return - the dot product of both vectors
    double dot(Vector2D vector1, Vector2D vector2);

    /// \brief compute the magnitude of a vector
    /// \param vector - the vector to get the magnitude of
    /// \return - the magnitude of the vector
    double magnitude(Vector2D vector);

    /// \brief compute the angle between two vectors
    /// \param vector1 - the first vector
    /// \param vector2 - the second vector
    /// \return - the angle between both vectors
    double angle(Vector2D vector1, Vector2D vector2);

    /// \brief create a normalized 2D vector as [x_normalized y_normalized]
    /// \param vector - the Vector2D vector to normalize
    /// \return the normalized 2D vector
    Vector2D NormalizeVector(Vector2D vector);

    /// \brief normalize an angle to be between (-PI,PI]
    /// \param angle - the angle, in radians, to normalize
    /// \return the normalized angle
    double normalize_angle(double rad);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief A twist made up of theta_dot,x_dot,y_dot
    struct Twist2D
    {
        double theta_dot = 0.0;
        double x_dot = 0.0;
        double y_dot = 0.0;
    };

    /// \brief output a twist [theta_dot x_dot y_dot]
    /// os - stream to output to
    /// t - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a twist made up of theta_dot, x_dot, y_dot in this order
    /// is - stream from which to read
    /// t [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & t);
    
    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;


        /// \brief invert the transformation
        /// \return the inverse transformation of the transformation
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radiansVector2D
        double rotation() const;

        /// \brief apply a transformation to a Twist2D
        /// \param twist - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D twist) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
        
    private:
        Vector2D translational_component;
        double angular_displacement;
    };

    /// \brief Integrate a twist to get Tbbprime
    /// \param twist - a twist of x_dot,y_dot and theta_dot
    /// \return Transformation relative to the world frame
    Transform2D integrate_twist(const Twist2D & twist);

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
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
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

}

#endif
