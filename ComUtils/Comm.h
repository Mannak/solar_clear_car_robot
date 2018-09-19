#ifndef _COMM_H_
#define _COMM_H_

#include <string>
#include <vector>


class cComm {
public:
	cComm(void) {
	}
	~cComm(void) {
	}

	static int SplitString(const std::string& input, const std::string& delimiter, std::vector<std::string>& results);

	static void StringUpper(std::string& strDes);

	template<typename T>
	static bool num_valid(const char* str);

	//static void Find_files(const boost::filesystem::path& dir, const std::string& filename, std::vector<path>& v);

	static std::string GetRunPath();

	static bool FileExist(std::string strPath);

	static std::string Get_FileName(std::string strPath);

};

#endif
