#include "FileHelper.hpp"
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "pystring.h"
#include "StringHelper.hpp"
namespace Utility
{
	std::string FileHelper::readText(const std::string& filename)
	{
		std::ifstream file{ filename };
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + filename);
		}

		std::stringstream buffer;
		buffer << file.rdbuf();
		return buffer.str();
	}

	std::string FileHelper::readTextW(const std::wstring& filename)
	{
		std::ifstream file{ filename };
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + StringHelper::ws2s(filename));
		}

		std::stringstream buffer;
		buffer << file.rdbuf();
		return buffer.str();
	}


	std::vector<std::string> FileHelper::readLines(const std::string& filename)
	{
		std::ifstream file{ filename };

		if (!file.is_open()) {
			throw std::runtime_error("Could not open file:" + filename);
		}
		else{
			std::vector<std::string> lines;
			std::string line;
			// Read the file line by line
			while (getline(file, line)) {
				auto strippedline = pystring::strip(line);
				if(strippedline.size())
					lines.emplace_back(strippedline);
			}
			// Close the file
			file.close();

			return lines;
		}	
	}

	std::vector<char> FileHelper::readBinary(const std::string& filename)
	{
		std::ifstream file{ filename, std::ios::binary | std::ios::ate };
		if (!file.is_open()) {
			throw std::runtime_error("Could not open file: " + filename);
		}

		std::streamsize size = file.tellg();
		file.seekg(0, std::ios::beg);

		std::vector<char> buffer(size);
		file.read(buffer.data(), size);
		return buffer;
	}

	//namespace fs = std::filesystem;
	//bool FileHelper::Exist(const std::string& filepath)
	//{
	//	//path to wstring
	//	auto wstr = StringHelper::s2ws(filepath);
	//	return (fs::exists(wstr));
	//};

}