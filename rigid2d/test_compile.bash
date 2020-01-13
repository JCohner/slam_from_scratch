g++ -Wall -Wextra -g -std=c++17 -o rigid2d_test main.cpp rigid2d.cpp
./rigid2d_test < test1_input.txt > output.txt
# cat output.txt
# cat test1_answer.txt
cmp -s output.txt test1_answer.txt && echo "Success!"
cmp -s output.txt test1_answer.txt || echo "Failure!" #maurice helped me with this