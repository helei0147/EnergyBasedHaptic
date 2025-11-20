#ifndef _STRING_HELPER_H_
#define _STRING_HELPER_H_


#include <string>

#include <codecvt>
#include <locale>

namespace Utility
{
	class StringHelper
	{
	public:
#if _WIN32
		static std::string convertToUTF8(const std::wstring& wstr);
		static std::wstring convertToUTF16(const std::string& str);
#endif // _WIN32

		static std::wstring s2ws(const std::string& str)
		{
			using convert_typeX = std::codecvt_utf8<wchar_t>;
			std::wstring_convert<convert_typeX, wchar_t> converterX;

			return converterX.from_bytes(str);
		}

		static std::string ws2s(const std::wstring& wstr)
		{
			using convert_typeX = std::codecvt_utf8<wchar_t>;
			std::wstring_convert<convert_typeX, wchar_t> converterX;

			return converterX.to_bytes(wstr);
		}
	};
}

#endif