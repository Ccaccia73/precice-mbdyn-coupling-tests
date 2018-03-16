
#include <vector>
#include <iostream>
#include <fstream>

#include "localfunctions.h"


void readFoil(std::vector<double>& x, std::vector<double>& y, const float c, const double *shift, const char* filename){

	std::string line;
	float data[2];

	std::ifstream infile(filename);

	if(infile.fail()){
		std::cout << "File not open" << std::endl;
		return;
	}

	// read first line which is title
	std::getline(infile, line);
	std::cout << "Reading foil data for " << line << std::endl;

	while(infile >> data[0] >> data[1]){
		x.push_back(data[0]*c+shift[0]);
		y.push_back(data[1]*c+shift[1]);
	}

	/*
	std::cout << "X coord: " << x.size() << std::endl;
	std::cout << "Y coord: " << y.size() << std::endl;

	for(std::vector<float>::iterator it = y.begin(); it != y.end(); ++it) {
	    std::cout << *it << " ";
	}
	*/
}


