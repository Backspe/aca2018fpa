#include "readBvh.h"

bool Parser::readAssert(std::string s1, std::string const s2) {
	if (s1.compare(s2) == 0) {
		return true;
	} else {
		std::cout << "assert" << s1 << "!=" << s2 << std::endl;
		exit(1);
		return false;
	}		
}

BVH* Parser::parseRoot() {
	readAssert(getToken(), "ROOT");
	BVH* current = new BVH(nextToken());
	current->channelNum = 0;
	readAssert(nextToken(), "{");
	nextToken();
	while (getToken().compare("}") != 0) {
		if (getToken().compare("OFFSET") == 0) {
			current->offset[0] = std::stof(nextToken());
			current->offset[1] = std::stof(nextToken());
			current->offset[2] = std::stof(nextToken());
			nextToken();
		} else if (getToken().compare("CHANNELS") == 0) { //TODO CHANNEL 6에 대응
			current->channelCount = std::stoi(nextToken());
			current->channelCountAll += current->channelCount;
			for(int i = 0; i < current->channelCount; i++) {
				std::string s = nextToken();
				if (s.compare("Xrotation") == 0) {
					current->channelOrder[i] = 0;
				} else if (s.compare("Yrotation") == 0) {
					current->channelOrder[i] = 1;
				} else if (s.compare("Zrotation") == 0) {
					current->channelOrder[i] = 2;
				} else if (s.compare("Xposition") == 0) {
					current->channelOrder[i] = 3;
				} else if (s.compare("Yposition") == 0) {
					current->channelOrder[i] = 4;
				} else if (s.compare("Zposition") == 0) {
					current->channelOrder[i] = 5;
				}
			}
			nextToken();
		} else if (getToken().compare("End") == 0) {
			readAssert(nextToken(), "Site");
			readAssert(nextToken(), "{");
			EndSite* endsite = new EndSite();
			readAssert(nextToken(), "OFFSET");
			endsite->offset[0] = std::stof(nextToken());
			endsite->offset[1] = std::stof(nextToken());
			endsite->offset[2] = std::stof(nextToken());
			readAssert(nextToken(), "}");
			current->ends.push_back(endsite);
			nextToken();
		} else if (getToken().compare("JOINT") == 0) {
			Joint* newJoint = parseJoint(current->channelNum + current->channelCountAll);
			newJoint->parent = NULL;
			current->joints.push_back(newJoint);
			current->channelCountAll += newJoint->channelCountAll;
		} else if (getToken().compare("") == 0) {
			std::cout << "Parse Error: file ended in " << current->name << std::endl;
			exit(1);
		} else {
			std::cout << "Assert Error: " << getToken() << " is not for ROOT " << current->name << std::endl;
			exit(1);
		}
	}
	nextToken();
	return current;
}

Joint* Parser::parseJoint(int chan_num) {
	readAssert(getToken(), "JOINT");
	Joint* current = new Joint(nextToken());
	current->channelNum = chan_num;
	//std::cout << current->name << " : " << current->channelNum << std::endl;
	readAssert(nextToken(), "{");
	nextToken();
	while (getToken().compare("}") != 0) {
		if (getToken().compare("OFFSET") == 0) {
			current->offset[0] = std::stof(nextToken());
			current->offset[1] = std::stof(nextToken());
			current->offset[2] = std::stof(nextToken());
			nextToken();
		} else if (getToken().compare("CHANNELS") == 0) {
			current->channelCount = std::stoi(nextToken());
			current->channelCountAll += current->channelCount;
			for(int i = 0; i < current->channelCount; i++) {
				std::string s = nextToken();
				if (s.compare("Xrotation") == 0) {
					current->channelOrder[i] = 0;
				} else if (s.compare("Yrotation") == 0) {
					current->channelOrder[i] = 1;
				} else if (s.compare("Zrotation") == 0) {
					current->channelOrder[i] = 2;
				}
			}
			nextToken();
		} else if (getToken().compare("End") == 0) {
			readAssert(nextToken(), "Site");
			readAssert(nextToken(), "{");
			EndSite* endsite = new EndSite();
			readAssert(nextToken(), "OFFSET");
			endsite->offset[0] = std::stof(nextToken());
			endsite->offset[1] = std::stof(nextToken());
			endsite->offset[2] = std::stof(nextToken());
			readAssert(nextToken(), "}");
			current->ends.push_back(endsite);
			nextToken();
		} else if (getToken().compare("JOINT") == 0) {
			Joint* newJoint = parseJoint(current->channelNum + current->channelCountAll);
			newJoint->parent = current;
			current->joints.push_back(newJoint);
			current->channelCountAll += newJoint->channelCountAll;
		} else if (getToken().compare("") == 0) {
			std::cout << "Parse Error: file ended in " << current->name << std::endl;
			exit(1);
		} else {
			std::cout << "Assert Error: " << getToken() << " is not for JOINT " << current->name << std::endl;
			exit(1);
		}
	}
	nextToken();
	return current;
}

void Parser::parseMotion(BVH* root) {
	readAssert(getToken(), "MOTION");
	readAssert(nextToken(), "Frames:");
	root->frameCount = std::stoi(nextToken());
	readAssert(nextToken(), "Frame");
	readAssert(nextToken(), "Time:");
	root->frameTime = std::stof(nextToken());
	for(int i = 0; i < root->frameCount; i++) {
		std::vector<float> frame;
		for(int j = 0; j < root->channelCountAll; j++) {
			frame.push_back(std::stof(nextToken()));
		}
		root->frames.push_back(frame);
	}
	nextToken();
}

BVH* Parser::parse(const char* fileName) {
	printf("start reading file %s\n", fileName);
	std::ifstream ifs(fileName);
	if (ifs.fail()) {
		printf("ERROR: no such file : %s\n", fileName);
		exit(0);
	}

	printf("end reading\n");
	float x, y, z, a;
	char buf[1024];
	std::string line;
	while (!ifs.eof()) {
		getline(ifs, line);
		line = line.substr(0, line.find("#"));
		std::stringstream ss(line);
		while (ss >> buf) tokens.push_back(buf);
	}

	tokenIndex = 0;
	readAssert(getToken(), "HIERARCHY");
	readAssert(nextToken(), "ROOT");
	BVH* root = parseRoot();
	parseMotion(root);
	std::cout << "model has " << root->channelCountAll << " channels and " << root->frameCount << " frames" << std::endl;
	ifs.close();
	return root;
}

/*
int main() {
	char fileName[30] = "MotionData/Trial001.bvh";
	Parser parser;
	BVH* bvh = parser.parse(fileName);
	return 0;
}
*/
