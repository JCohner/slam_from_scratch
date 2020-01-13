# Answers to C002

1. What is the difference between a class and a struct in C++?
A struct defines a type comprised of other already defined types. 
Structs are inhereted from C, C++ classes take this data packaging (C structs are very handy at specifying just how to package data in memory) to the OOP domain. However, in C++ the only difference between a class and struct is that the member variables and functions in a class are by deault private. In a struct they are public. 

inspo taken from: https://blogs.mentor.com/colinwalls/blog/2014/06/02/struct-vs-class-in-c/  


2. Why is Vector2D a struct and Transform2DClass?
There could be many motivations. I would argue readability and intended user development. However, from the c++ guidlines we can make this a little more explicit. **C.2**, our data does not have an invariant, x and y are orthoganal axis and can do whatever they want in relation to one another. **C.8** since there are no non-public members we are more than free to use a struct. I would say since it is such a simple datatype the usage of struct suggests to future developers that the data type should remain simple. 

3. Why are some of the constructors in Transform2D explicit?
My initial guess:
In C++ the compiler is allowed to make 1 implicit conversion to resolve parameters to a function. We are defining 2 different overloadings of the constructor and what to make sure the compiler treats these as distinct, therefore we do not want to allow the constructor to change the type of our input argument behind the scene.
C++ guidelines:
**C.46** says this explicitly "to avoid unintended conversions"


4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):

	* Propose three different designs for implementing the normalize functionality

	* Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies)

	* Which of the methods would you implement and why?


5. Implement the normalize functionality using the method you chose.

6. Why is Transform2D::inv() declared const while Transform2D::operator\*=() is not?=

