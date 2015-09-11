#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>
#include "../../../path.h"

using namespace std;

struct cPair{
	std::string key;
	std::string value;
	
	cPair(string a, string b){
		key=a;
		value=b;
	}
};

struct Section{
	string name;
	vector<cPair> constantPairs;
	
	Section(string a){
		name=a;
	}
};
vector<cPair> common;
vector<Section> sections;
//~ vector<cPair> core;
//~ vector<cPair> vision;
//~ vector<cPair> manipulation;
//~ vector<cPair> bimanual;
//~ vector<cPair> utilities;

string line;

string file_string = (string)PROJECT_PATH + "/src/core/AllConstants.ini";

/**read a section, record all pairs.
* start below [**], when finished, read an extra line
* */
void readSection(fstream &file, vector<cPair> &pairs){
	
	while(true){
		getline(file,line);
		
		//next section
		if(line[0] =='['){
			printf("%s\n",line.c_str());
			break;
		}
		
		printf("%s\n",line.c_str());
		//comment
		if(line[0] ==';'){
			cPair temp(";",line.substr(1,string::npos) );
			pairs.push_back(temp);
		}
		//empty line
		else if(line.empty()){
			cPair temp("\n","" );
			pairs.push_back(temp);
		}
		//constants data
		else{
			size_t found = line.find("=");
			if(found == std::string::npos){
				printf("\n\n!!!!!.ini format!!!\n\n");
				cout<<line;
			}
			cPair temp( line.substr(0,found), line.substr(found+1,string::npos) );
			pairs.push_back(temp);
		}
			
	}
}

int main( int argc, char** argv ){
	
	///open file
	printf("openning AllConstants.ini: %s\n",file_string.c_str());
    fstream file(file_string.c_str());
    if (!file.is_open()){
        printf("ERROR: Unable to open ini file\n");
        return 1;
    }
	
	///read
	//skip to [common]
	while(true){
		getline(file,line);
		printf("%s\n",line.c_str());
		if(line =="[common]")
			break;
	}
	//record common
	readSection(file, common);
	
	//read rest sections
	while(line.find("[end]") == std::string::npos){
		string name = line.substr(1,line.length()-2);
		Section newSection(name);
		readSection(file, newSection.constantPairs);
		sections.push_back(newSection);
	}
	
	///write to files
	for(size_t i=0; i< sections.size(); i++){
		///write .h
		string file_string = (string)PROJECT_PATH + "/src/" + sections[i].name + "/src/Constants.h" ;
		printf("writing to:%s\n", file_string.c_str());
		ofstream outputH(file_string.c_str());
		
		//write common constants
		outputH<<"/**Generated by core/generateAllConstants\n */\n";
		outputH<<"\n///Common\n";
		for(size_t j=0; j< common.size(); j++){
			string key = common[j].key;
			string value = common[j].value;
			
			if(key == ";"){
				outputH<<"//"<<value<<"\n";
			}
			else if (key == "\n"){
				outputH<<"\n";
			}
			else{
				outputH<<"#define "<<key<<" "<<value<<"\n";
			}
			
		}
		
		//write module constants
		outputH<<"\n///Module specific\n";
		for(size_t j=0; j< sections[i].constantPairs.size(); j++){
			string key = sections[i].constantPairs[j].key;
			string value = sections[i].constantPairs[j].value;
			
			if(key == ";"){
				outputH<<"//"<<value<<"\n";
			}
			else if (key == "\n"){
				outputH<<"\n";
			}
			else{
				outputH<<"#define "<<key<<" "<<value<<"\n";
			}
			
		}
		
		///write .py
		file_string = (string)PROJECT_PATH + "/src/" + sections[i].name + "/scripts/Constants.py" ;
		printf("writing to:%s\n", file_string.c_str());
		ofstream outputPy(file_string.c_str());
		
		//write common constants
		outputPy<<"#Generated by core/generateAllConstants\n";
		outputPy<<"\n###Common\n";
		for(size_t j=0; j< common.size(); j++){
			string key = common[j].key;
			string value = common[j].value;
			
			if(key == ";"){
				outputPy<<"#"<<value<<"\n";
			}
			else if (key == "\n"){
				outputPy<<"\n";
			}
			else{
				outputPy<<key<<" = "<<value<<"\n";
			}
			
		}
		
		//write module constants
		outputPy<<"\n###Module specific\n";
		for(size_t j=0; j< sections[i].constantPairs.size(); j++){
			string key = sections[i].constantPairs[j].key;
			string value = sections[i].constantPairs[j].value;
			
			if(key == ";"){
				outputPy<<"#"<<value<<"\n";
			}
			else if (key == "\n"){
				outputPy<<"\n";
			}
			else{
				outputPy<<key<<" = "<<value<<"\n";
			}
			
		}
		
	}

	
	return 0;	
}
