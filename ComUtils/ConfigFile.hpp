/***************************************************************************
 *                              Supcon SuroOS Team
 *                     -------------------------------------------------
 * Copyright (c) Supcon , ZheJiang, China.
 * All rights reserved.
 *
 * @author bobshenhui@gmail.com
 *
 ****************************************************************************/
/*! @file ConfigFile.hpp
 *	配置文件类，可以从文件中查找属性键值，并且可以保存成文件
 */

#ifndef __CONFIGFILE_H_
#define __CONFIGFILE_H_

#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

using namespace std;

namespace std {
typedef std::basic_ifstream<char, char_traits<char> > tifstream;
}
;

//#define CONF_DEFAULT_NAME "*****____s"

#define CONF_DEFAULT_VALUE "CONF_NULL"

typedef std::pair<int, std::string> ConfigPair;

class ConfigFile_Section {
private:
	std::map<std::string, ConfigPair> m_datas;

	std::vector<std::string> m_lines;

	std::string m_name;

public:
	std::map<std::string, ConfigPair> getData(){
		return m_datas;
	}

private:
	bool func2(const string & context) {
		int c = 0;
		for (size_t i = 0; i < context.size(); i++) {
			if (context[i] == '='){
				c++;
				break;
			}
		}

		if (c != 1) {
			return false;
		}

		size_t posEqual = context.find('=');

		assert(posEqual != context.size());

		string name = context.substr(0, posEqual); //读取从0位到“=”返回位的内容。
		name = boost::trim_copy(name); //boost库去掉前后的空格

		string value = context.substr(posEqual + 1);//读取从“=”返回位下一位到最后的内容
		value = boost::trim_copy(value);

		ConfigPair pair(m_lines.size() - 1, value);
		m_datas.insert(make_pair(name, pair));
		return true;
	}

public:
	ConfigFile_Section(const string & name) {
		m_name = name;
	}

	inline string getName() {
		return m_name;
	}

	void InsertLine(const string& lineStr) {
		m_lines.push_back(lineStr);
		string line = boost::trim_copy(lineStr);

		if (line.size() == 0)
			return;

		if (line[0] == '#')
			return;
		func2(line);
	}

	void RemoveLastLine() {
		m_lines.erase(m_lines.begin() + m_lines.size() - 1);
	}

	void AddContext(const string& name, const string& value) {

		map<string, ConfigPair>::iterator it = m_datas.find(name);
		if (it == m_datas.end()) {
			stringstream ss;
			ss << name << " = " << value;
			ConfigPair pair(m_lines.size(), value);
			m_datas.insert(make_pair(name, pair));
			m_lines.push_back(ss.str());
		} else {
			int line = it->second.first;
			stringstream ss;
			ss << name << " = " << value;
			m_lines[line] = ss.str();
			it->second.second = value;
		}

	}

	void RemoveContext(const string & name) {
		map<string, ConfigPair>::iterator it = m_datas.find(name);
		if (it != m_datas.end()) {
			int line = it->second.first;
			m_lines.erase(m_lines.begin() + line);
			m_datas.erase(it);

			map<string, ConfigPair >::iterator it2 =
				m_datas.begin();

			for(;it2!=m_datas.end();++it2)  
			{
				if(it2->second.first >= line)
					it2->second.first -= 1;
			}
			
		}
	}

	bool GetContext(const string & name, string & value) {
		map<string, ConfigPair>::iterator it = m_datas.find(name);
		if (it != m_datas.end()) {
			value = it->second.second;
			return true;
		}

		return false;
	}

	void Write2File(ofstream& stream) {
		stream << "[" << m_name << "]" << endl;
		for (int i = 0; i < (int) m_lines.size(); i++) {
			stream << m_lines[i] << endl;
		}
	}

};

class ConfigFile {
private:
	map<string, boost::shared_ptr<ConfigFile_Section> > m_datas;
	vector<boost::shared_ptr<ConfigFile_Section> > m_sections;
	map<string, string> m_default;

	boost::shared_ptr<ConfigFile_Section> m_current_section;
	boost::mutex m_mutex;

	std::string m_file_name;

public:
	ConfigFile(string fileName) {

		std::tifstream file(fileName.c_str());

		m_file_name = fileName;

		init(file);
	}

	map<string, boost::shared_ptr<ConfigFile_Section> > getAllData(){
		return m_datas;
	}

	vector<boost::shared_ptr<ConfigFile_Section> > getVectorData(){
		return m_sections;
	}

private:

	bool func1(string context) {
		if (context[0] == '[' && context[context.size() - 1] == ']') {
			for (size_t i = 1; i < context.size() - 1; i++) {
				if (context[i] == '[' || context[i] == ']')
					return false;
			}

			return true;
		}

		return false;
	}

	void init(tifstream& file) {
		string line;
		int count = 0;

		while (std::getline(file, line)) {

			if (m_current_section.get() != 0)
				m_current_section->InsertLine(line);

			count++;
			line = boost::trim_copy(line);

			if (line.size() == 0)
				continue;

			if (line[0] == '#')
				continue;

			if (func1(line)) {
				if (m_current_section.get() != 0)
					m_current_section->RemoveLastLine();

				string head = line.substr(1, line.size() - 2);
				head = boost::trim_copy(head);

				m_current_section = boost::shared_ptr<ConfigFile_Section>(
						new ConfigFile_Section(head));
				m_sections.push_back(m_current_section);
				m_datas.insert(make_pair(head, m_current_section));
			}

		}

	}

public:

	/*!
	 *  把ConfigFile內存保存成文件
	 *   @param fileName 保存的文件名
	 */

	void Save2File(const std::string & fileName) {
		boost::mutex::scoped_lock lock(m_mutex);
		ofstream stream;
		stream.open(fileName.c_str(), ios::out);
		for (int i = 0; i < (int) m_sections.size(); i++) {
			m_sections[i]->Write2File(stream);
		}

		stream.close();
	}

	void Save2File() {
		Save2File(m_file_name);
	}

	/*!
	 *  查询ConfigFile中的信息
	 *   @param name 键名
	 *   @param inSection 域的名字: [section]
	 *   @param default_value 默认的值，如果对应inSection中没有name的键，者返回default_value的值
	 */
	std::string GetValue(std::string const& name, std::string const& inSection,
			std::string const & default_value = CONF_DEFAULT_VALUE) {

		boost::mutex::scoped_lock lock(m_mutex);
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(inSection);

		if (it == m_datas.end()) {
			return default_value;
		}

		string value;
		if (it->second->GetContext(name, value)) {
			return value;
		} else
			return default_value;
	}

	bool getValue(std::string &value, const std::string & name,
			const std::string & inSection) {
		boost::mutex::scoped_lock lock(m_mutex);
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(inSection);

		if (it == m_datas.end()) {
			return false;
		}

		if (it->second->GetContext(name, value)) {
			return true;
		} else
			return false;
	}

	/*!
	 * 增加一个域
	 * @param section 域名*/
	void AddSection(const string& section) {
		boost::mutex::scoped_lock lock(m_mutex);
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(section);
		if (it == m_datas.end()) {
			boost::shared_ptr<ConfigFile_Section> s = boost::shared_ptr<
					ConfigFile_Section>(new ConfigFile_Section(section));
			m_sections.push_back(s);
			m_datas.insert(make_pair(section, s));
		}
	}

	/*!
	 * 增加一个键值对
	 * @param name 名字
	 * @param value 值
	 * @param 域名
	 * */
	void AddContext(const string & name, const string & value,
			const string & section) {
		boost::mutex::scoped_lock lock(m_mutex);
		boost::shared_ptr<ConfigFile_Section> my_section;
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(section);
		if (it == m_datas.end()) {
			boost::shared_ptr<ConfigFile_Section> s = boost::shared_ptr<
					ConfigFile_Section>(new ConfigFile_Section(section));
			m_sections.push_back(s);
			m_datas.insert(make_pair(section, s));
			my_section = s;
		} else {
			my_section = it->second;
		}

		my_section->AddContext(name, value);
	}

	/*!
	 * 删除整个域
	 * @param section 域名
	 * */
	void RemoveSection(const string & section) {
		boost::mutex::scoped_lock lock(m_mutex);
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(section);
		if (it != m_datas.end()) {
			m_datas.erase(it);
		}

		for (vector<boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_sections.begin(); it != m_sections.end();) {
			if ((*it)->getName() == section) {
				it = m_sections.erase(it);
			} else {
				it++;
			}
		}
	}

	/*!
	 * 删除一个Context
	 * @param name 键名
	 * @param section 域名
	 * */
	void RemoveContext(const string & name, const string & section) {
		boost::mutex::scoped_lock lock(m_mutex);
		map<string, boost::shared_ptr<ConfigFile_Section> >::iterator it =
				m_datas.find(section);
		if (it == m_datas.end()) {
			return;
		}

		boost::shared_ptr<ConfigFile_Section> my_section = it->second;
		my_section->RemoveContext(name);

	}

};
#endif
