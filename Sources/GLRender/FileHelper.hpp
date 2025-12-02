#ifndef _FILE_HELPER_H_
#define _FILE_HELPER_H_

#include <string>
#include <vector>
#include <filesystem>
namespace Utility
{
	class FileHelper
	{
	public:
		/// <summary>
		/// read file into std::string
		/// </summary>
		/// <param name="filename"></param>
		/// <returns></returns>
		static std::string readText(const std::string& filename);


		static std::string readTextW(const std::wstring& filename);

		/// <summary>
		/// read file into lines
		/// </summary>
		/// <param name="filename"></param>
		/// <returns></returns>
		static std::vector<std::string> readLines(const std::string& filename);

		/// <summary>
		/// read file into binary(buffer)
		/// </summary>
		/// <param name="filename"></param>
		/// <returns></returns>
		static std::vector<char> readBinary(const std::string& filename);

		/// <summary>
		/// check file exist (with unicode support)
		/// </summary>
		/// <param name="filepath"></param>
		/// <returns></returns>
		static bool Exist(const std::string& filepath);
	};
}


#endif