g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
if cmp -s ./rigid2d_test < test1_input.txt test1_answer.txt; then
	echo Failure!
else 
	echo Success!
fi