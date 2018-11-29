#ifndef READBVH_H
#define READBVH_H
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#define PI 3.14159265358979
#define DEBUG false

typedef std::vector< float > Frame;
typedef std::vector< Frame > Motion;

class EndSite {
public:
	float offset[3];
};

class Joint {
public:
	std::string name;
	float offset[3];
	float rotation[3];
	int channelOrder[6];
	int channelCount;
	int channelCountAll;
	int channelNum;
	Joint* parent;
	std::vector< Joint * > joints;
	std::vector< EndSite * > ends;

	Joint(std::string s) {
		name = s;
		channelCountAll = 0;
	}
};

class BVH {
public:
	std::string name;
	float offset[3];
	float position[3];
	float rotation[3];
	int channelOrder[6];
	int channelCount;
	int channelCountAll;
	int channelNum;
	std::vector< Joint * > joints;
	std::vector< EndSite * > ends;
	std::map< std::string, Joint* > jointMap;

	int frameCount;
	float frameTime;
	std::vector< std::vector< float > > frames;

	BVH(std::string s) {
		name = s;
		channelCountAll = 0;
	}
};


class Parser {

private:
	std::vector< std::string > tokens;
	int tokenIndex = 0;
	std::string getToken() {
		return tokens.at(tokenIndex);
	}
	std::string nextToken() {
		tokenIndex++;
		if(tokenIndex >= tokens.size()) {
			if (DEBUG) std::cout << "DEBUG: token empty" << std::endl;
			return "";
		}
		if (DEBUG) std::cout << "DEBUG: next: " << tokens.at(tokenIndex) << std::endl;
		return tokens.at(tokenIndex);
	}
	bool readAssert(std::string s1, std::string const s2);
public:

	BVH* parseRoot();
	Joint* parseJoint(int chan_num);
	void parseMotion(BVH* root);
	BVH* parse(const char* fileName);
	void jointMapping(std::map< std::string, Joint* >* m, Joint* current);
};

#endif
