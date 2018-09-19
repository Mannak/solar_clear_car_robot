/*
 * EasySplit.hpp
 *
 *  Created on: 2012-3-14
 *      Author: Administrator
 */

#ifndef INTEGRATED_NAVIGATION_EASY_SPLIT_SWORD_20120314_HPP_
#define INTEGRATED_NAVIGATION_EASY_SPLIT_SWORD_20120314_HPP_

#include "ConfigFile.hpp"
#include <boost/tokenizer.hpp>

class EasySplit {
public:
	EasySplit() {}
	~EasySplit() {}

	static void split(const string &str,const string &token, vector<string> &data) {
		boost::algorithm::split(data, str, boost::algorithm::is_any_of(token.c_str()));	//boost::algorithm::is_punct()
	}

	static void split(const string &str, vector<string> &data) {
		boost::tokenizer<> toks(str);
		for(boost::tokenizer<>::iterator beg = toks.begin(); beg != toks.end(); ++beg) {
			data.push_back(*beg);
		}
	}

	static void split(const string &str, vector<string> &data, vector<int> &clen) {
		boost::tokenizer<> toks(str);
		int cc = 0;
		for(boost::tokenizer<>::iterator beg = toks.begin(); beg != toks.end(); ++beg) {
			data.push_back(*beg);
			clen.push_back(cc);
			cc += (*beg).length();
		}
	}
};

#endif /* INTEGRATED_NAVIGATION_EASY_SPLIT_SWORD_20120314_HPP_ */
