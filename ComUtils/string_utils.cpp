#include "string_utils.h"


namespace string_utils {
	std::string replace_all_substr(std::string ori_str, std::string replaced_sub_str, std::string new_sub_str) {
		std::string::size_type pos = ori_str.find_first_of(replaced_sub_str);
		while (pos != ori_str.npos) {
			ori_str.replace(pos, replaced_sub_str.length(), new_sub_str);

			pos = ori_str.find_first_of(replaced_sub_str);
		}

		return ori_str;
	}
	
}