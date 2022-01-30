# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   1. Create a member function inside of the Vector2D struct.
   2. Create a function that is not a part of a class or struct but instead is of type Vector2D
   3. Create a class or struct who's function is to normalize a vector.
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      Makes organization simple by having a function that uses the Vector2D struct be inside. 
      C.4 states that one should only make a function a member function if it needs direct access to representation of class. The function does need any private numbers or information other than that it is of a Vector2D data type.

      The function does not need direct access to the representation of the struct and it can be in the same namespace as the struct which according to C.5, is the definition of a helper function. Additionally, being in the same namespace shows the obvious relationship between the function and the struct.
      Since it is not inside a class/struct, the relationship might be as apparent to users looking at the function. 

      Referencing C.1, it is advised to use strutures for easy 
      of comprehension and allows data to be used within the structure.
      Creating an entire structure for the implementation of one function would be adding too much complexity for a single function.

   - Which of the methods would you implement and why?

   I chose to implement creating a function that is a helper function to Vector2D and lies in the same namespace but not inside the class/struct. To me, I found it simpler to implement and had more success using it in the main function.

2. What is the difference between a class and a struct in C++?

   There is no real difference between a class and a struct besides if the members are public or private. For classes, the members are private whereas members are public for structs by default. [by convention classes are used when their are invariants]

3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

   The most apparent reason that Vector2D is a struct and not a class is because all of the members are public. This is according to C.8. Private members were introduced in Transform2D. 

   Another reason refers to C.4 where Transfrom2D has member functions that need access to private members in the class.
   [also invariants.  There are no restraints on the values of x, and y in Vector2D, they vary independently, but for transform2d there potentially are restrictions.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   According to C.46, one should declase single argument constructors explicit. This is to prevent implicit conversions and when a function can take either a double or Vector2D, implicit conversion can occur unless specificed othewise.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   Inv() is const because it is not modifying the members of Transform2D and is instead using those members to return another variable. The operator *= does change the private members of the class.
# Sample Run of frame_main
```
Enter transform T_{a,b}: 
90 0 1
Enter transform T_{b,c}: 
90 1 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b: 
1 1
v_bhat[0.707107 0.707107]
v_a [-1 2]
v_b [1 1]
v_c [1 1.11022e-16]
Enter twist V_b: 
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]
```
