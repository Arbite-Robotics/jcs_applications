// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
#include <iostream>//cout
#include "../cmd_input_parser.h"

int main(int argc, char* argv[]) {
	cmd_input_parser cmd_parser(argc, argv);
	if (cmd_parser.cmd_option_exists("-h")) {
		std::cout << "Got -h\n";
	}
	if (cmd_parser.cmd_option_exists("-w")) {
		std::cout << "Got -w\n";
	}

	std::string const filename = cmd_parser.cmd_option_get("-f");
	if (!filename.empty()) {
		std::cout << "Got filename " << filename << "\n";
	} else {
		std::cout << "No filename given\n";
	}

	return 0;
}
