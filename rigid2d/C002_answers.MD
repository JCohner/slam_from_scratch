# Answers to C.002

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
		1. Implement as a member function of the Vector2D struct
		2. Switch Vector2D to a class, make x and y values private and make a public normalize function
		3. Implement a new class NormVector2D, this class has the invariant the the norm of the vector described by components x and y is 1. 

	* Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies)
		Pros:
		C.2 seems to suggest that since an invariant exists a class does make sense. 
		C.3 says we can represent the distinction between an interface and an implementation using a class. 
		C.8 related to the two above, we should make our invariant parameters private such that a user cannot break the invariance by setting one of them individually
		Cons:
		It goes against my intuition. If Vector2D is already defined as a struct it seems messy to make a seperate class to deal with its normalized type. Maybe inheritence could play a role here but how would inheritance work between a struct and a class?
		

	* Which of the methods would you implement and why?
		Though it goes against my intuition, the guidelines  seem to suggest making it a class with invariant x and y variables private. 

5. Implement the normalize functionality using the method you chose. 
	--Done

6. Why is Transform2D::inv() declared const while Transform2D::operator\*=() is not?=
	Josh answer: 
	The const declaration ensures the object represented by *this* pointer does not change. In the case of the inv() method we want to make sure not to change the underlaying data in the object, merely return a version that is the inverse. The \*= has a much different purpose where we want to change the Transform2D object into the result of that Transform2D object left leftmultiplying the Transform2D object that is on the rhs of this operator. 
	Backing it up:
	Con.2 By default make member functions const. "A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities." - https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rconst-const
	Con.4 use const to define objects with values that do not change after construction. 


