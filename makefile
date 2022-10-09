all: part_c part_d part_e bonus

part_c: part_c.cpp
	c++ -fPIC -g -Wall -I/usr/local/Aria/include part_c.cpp -o part_c -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt
part_d: part_d.cpp
	c++ -fPIC -g -Wall -I/usr/local/Aria/include part_d.cpp -o part_d -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt
part_e: part_e.cpp
	c++ -fPIC -g -Wall -I/usr/local/Aria/include part_e.cpp -o part_e -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt
bonus: bonus.cpp
	c++ -fPIC -g -Wall -I/usr/local/Aria/include bonus.cpp -o bonus -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

clean:
	rm result


tmp: tmp.cpp
	c++ -fPIC -g -Wall -I/usr/local/Aria/include tmp.cpp -o tmp -L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt
