#include "Comm.h"
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
//查找文件
#include <boost/algorithm/string.hpp> 
#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/filesystem.hpp>
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"

#include <fstream>
#include <map>

#include <cctype>


using namespace boost;
using namespace boost::xpressive;
using namespace boost::filesystem;

using namespace std;
//using namespace boost::BOOST_FILESYSTEM_NAMESPACE;

int cComm::SplitString(const string& input, const string& delimiter, vector<string>& results) {
	int iPos = 0;
	int newPos = -1;
	int sizeS2 = (int)delimiter.size();
	int isize = (int)input.size();
	results.clear();

	if ((isize == 0) || (sizeS2 == 0)) {
		return 0;
	}

	vector<int> positions;

	int numFound = 0;

	while ((newPos = input.find(delimiter, iPos)) > 0) {
		positions.push_back(newPos);
		iPos = newPos + sizeS2;
		numFound++;
	}

	if (numFound == 0) {
		if (input.size() > 0) {
			results.push_back(input);
		}
		return 1;
	}

	if (positions.back() != isize) {
		positions.push_back(isize);
	}

	int offset = 0;
	string s("");

	for (int i = 0; i < (int)positions.size(); ++i) {

		s = input.substr(offset, positions[i] - offset);

		offset = positions[i] + sizeS2;

		if (s.length() > 0 && !all(s, is_space())) {
			results.push_back(s);
		}

	}
	return numFound;
}

void cComm::StringUpper(string& strDes) {
	std::transform(strDes.begin(), strDes.end(), strDes.begin(), ::toupper); //转大写
}

template<typename T>
bool cComm::num_valid(const char* str) {
	try {
		lexical_cast<T> (str);
		return true;
	}
	catch (bad_lexical_cast) {
		return false;
	}
}

void /*cComm::*/Find_files(const path& dir, const string& filename,
	vector<path>& v) {



	typedef recursive_directory_iterator rd_iterator;

	//cout<<"dir" << dir << " , filename = " << filename <<endl;

	typedef vector<path> result_type;
	if (!exists(dir) || !is_directory(dir)) {
		return;
	}



	rd_iterator end;
	for (rd_iterator pos(dir); pos != end; ++pos) {


		if (is_directory(*pos))
			continue;
#ifdef _WIN32

#if _MSC_VER >= 1800
		string file_name = pos->path().filename().string();
#else
		string file_name = pos->path().filename();
#endif

#else
		string file_name = pos->path().filename().generic_string();
#endif
		if (file_name.size() < filename.size())
			continue;
		else if (file_name.size() == filename.size()) {
			if (file_name == filename)
				v.push_back(pos->path());
		}
		else {
			file_name = file_name.substr(
				file_name.size() - filename.size(), filename.size());
			//cout<<"##file_name="<<file_name<<"--"<<endl;
			if (file_name == filename){
				v.push_back(pos->path());
			}
		}
	}
}

string cComm::GetRunPath() {
	return boost::filesystem::initial_path<boost::filesystem::path>().string();
}

bool cComm::FileExist(string strPath) {
	return exists(strPath);
}

string cComm::Get_FileName(string strPath) {
	vector<string> v_path;
	SplitString(strPath, ".", v_path);
	if (v_path.size() > 0) {
		return v_path[v_path.size() - 1];
	}
	else {
		return "";
	}
}