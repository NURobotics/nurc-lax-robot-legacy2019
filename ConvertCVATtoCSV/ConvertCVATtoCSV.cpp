
#include "pch.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "rapidxml_utils.hpp"

using namespace std;
using namespace rapidxml;

struct box {
	int frame;
	double x;
	double y;
	double width;
	double height;
};

void walk(const rapidxml::xml_node<>* node, map<int, vector<box>>* balls) {
	const rapidxml::node_type t = node->type();
	if (t == rapidxml::node_element)
	{
		if (node->name_size() == 3 && node->name()[0] == 'b') {

			int f;
			double xtl;
			double ytl;
			double xbr;
			double ybr;

			xml_attribute<>* a = node->first_attribute();

			f = stoi(a->value());

			a = a->next_attribute();

			xtl = stoll(a->value());

			a = a->next_attribute();

			ytl = stoll(a->value());

			a = a->next_attribute();

			xbr = stoll(a->value());

			a = a->next_attribute();

			ybr = stoll(a->value());

			box newBall = box();

			newBall.frame = f;

			newBall.x = ((xtl + xbr) / 2) / 1280;
			newBall.y = ((ytl + ybr) / 2) / 720;

			newBall.width = -(xtl - xbr) / 1280;
			newBall.height = -(ytl - ybr) / 720;

			if (balls->find(f) == balls->end()) {
				vector<box> newFrame;
				newFrame.push_back(newBall);
				balls->insert({ f, newFrame });
			}
			else {
				balls->at(f).push_back(newBall);
			}
		}

		for (const rapidxml::xml_node<>* n = node->first_node()
			; n
			; n = n->next_sibling()
			) {
			walk(n, balls);
		}
	}
}

int main()
{
	string files[] = { "8M_TightLeft1", "8M_Center3", "8M_TightLeft", "8M_TightLeft1", "8M_TightRight", "8M_TightRight1", "8M_TightRight2", "4M_Center", "4M_Center1" };

	for (string file : files) {
		const int n = file.length() + 4;
		char * char_array = new char[n + 1];
		strcpy_s(char_array, n + 1, string(file + ".xml").c_str());

		rapidxml::file<> xmlFile(char_array); // Default template is char
		rapidxml::xml_document<> doc;
		doc.parse<0>(xmlFile.data());

		map<int, vector<box>> balls;
		walk(doc.first_node(), &balls);

		for (map<int, vector<box>>::iterator it = balls.begin(); it != balls.end(); ++it) {
			string outputName = file + "_frame_" + to_string(it->first) + ".txt";
			outputName.replace(1, 1, "Meters");

			ofstream outputFile("labels/" + outputName);
			cout << outputName << endl;

			for (box b : it->second) {
				outputFile << "0 " << b.x << " " << b.y << " " << b.width << " " << b.height << endl;
			}
			outputFile.close();
			cout << endl;
		}
	}

	return 0;
}
