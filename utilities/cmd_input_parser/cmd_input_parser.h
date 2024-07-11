// Copyright (c) 2024 Arbite Robotics Pty Ltd
// https://arbite.io
//
// Command line parsing utility
// Inspiration from:
// https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
//
#ifndef CMD_INPUT_PARSER_H_
#define CMD_INPUT_PARSER_H_

#include <string>
#include <vector>
#include <algorithm>//std::find

class cmd_input_parser {
public:
	cmd_input_parser(int const& argc, char** argv) {
		for (int i=1; i<argc; ++i) {
			this->tokens.push_back( std::string(argv[i]) );
		}
	}
	~cmd_input_parser() {}

	std::string const cmd_option_get(std::string const& option) {
		std::vector<std::string>::const_iterator itr;
		itr = std::find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr !=this->tokens.end()) {
			return *itr;
		}
		static std::string const empty_string("");
		return empty_string;
	}

	bool cmd_option_exists(std::string const& option) {
		return std::find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
	}

private:
	std::vector<std::string> tokens;

};

#endif