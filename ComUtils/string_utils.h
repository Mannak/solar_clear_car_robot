#ifndef _STRING_UTILS_H_
#define _STRING_UTILS_H_

#include <string>


namespace string_utils {
	std::string replace_all_substr(std::string ori_str, std::string replaced_sub_str, std::string new_sub_str);
}

#endif