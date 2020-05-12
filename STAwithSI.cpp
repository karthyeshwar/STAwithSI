/************************************************************************
README
Code consists of Implementation for performing Static Timing Analysis incorporating Signal Integrity Effects 
For running the code use the following command

Make  -> object file cad2 is created

./cad2 ./NLDM/saed32_hvt_ss0p95v125c.lib ./NLDM/saed32_rvt_ss0p95v125c.lib ./NLDM/saed32_lvt_ss0p95v125c.lib  ./NLDM/saed32io_wb_ss0p95v125c_2p25v.lib ./c1908/c1908.vg ./c1908/c1908.spef.min ./c1908/c1908.spef.max 5 0.2 

where 
argv[1] : HVT NLDM file 
argv[2] : RVT NLDM file 
argv[3] : LVT NLDM file 
argv[4] : IO NLDM file 
argv[5] : Placed Netlist
argv[6] : SPEF min file  
argv[7] : SPEF max file  
argv[8] : clock 
argv[9] : input slew 

Code Authors: Kartheshwar, Akhil
************************************************************************/

#include <iostream>  // basic C++ input/output (e.g., cin, cout)
#include <fstream>	// needed to open files in C++
#include <sstream>   // needed if you are using sstream (C++)
#include <vector>
#include <map>
#include <algorithm>  //for converting to upper case
#include <queue>
#include <stack>
#include <string>
using namespace std;

struct cell
{
	string cellName;
	double capacitance;
	vector<double> index_1;
	vector<double> index_2;
	vector<vector<vector<double> > >    delay_rise;
	vector<vector<vector<double> > >    delay_fall;

	vector<vector<vector<double> > >    slew_rise;
	vector<vector<vector<double> > >    slew_fall;
	int inverting=0;
}node;



struct circuit
{
	double cload = 0;
	vector<string> fanin;
	vector<string> fanout;
	string gatename;
	double req_arr_time_rise;// = INT_MAX;
	double req_arr_time_fall;// = INT_MAX;
	double gate_delay_rise;// = 0;
	double gate_delay_fall;// = 0;
	double slack_rise;// = 0;
	double slack_fall;// = 0;
	double output_arrival_rise;// = 0;
	double output_arrival_fall;// = 0;
	double output_slew_rise;// = 0;
	double output_slew_fall;// = 0;

}ckt;

struct spef
{
	string nodeName;
	double couplingCap=0;
	double loadCap=0;
}spefObj;
map<string, cell> NldmMap;
map<string, bool> VisitedMap;
vector<double> slew_vec_rise;
vector<double> slew_vec_fall;
ofstream myfile;
double CLock;
ofstream cktFile;
//delimeters for parsing 
struct ParenCommaEq_is_space : std::ctype<char> {
	ParenCommaEq_is_space() : std::ctype<char>(get_table()) {}
	static mask const* get_table()
	{
		static mask rc[table_size];
		rc['('] = std::ctype_base::space;
		rc[')'] = std::ctype_base::space;
		rc[','] = std::ctype_base::space;
		rc['='] = std::ctype_base::space;
		rc[' '] = std::ctype_base::space;
		rc['\t'] = std::ctype_base::space;
		rc['"'] = std::ctype_base::space;
		rc[':'] = std::ctype_base::space;
		rc['\0'] = std::ctype_base::space;
		rc['*'] = std::ctype_base::space;
		rc[';'] = std::ctype_base::space;
		rc['.'] = std::ctype_base::space;
		return &rc[0];
	}
};

int parseCktFile(int argc, char* argv[], map<string, circuit >   & ckt_vector);
int parseLibFile(int argc, char* argv[]);
void topoSort(char* argv[], map<string, circuit >& ckt_vector, string & riseNode, string & fallNode, stack<string> & Stack1);
void topoSortUtil(string v, queue<string>& Queue, map<string, circuit >   & ckt_vector);
double sumCapacitance(vector<string> fanout, map<string, circuit >  &  ckt_vector);
void calculateOutputArrivalTime(string NodeName, map<string, circuit >  &  ckt_vector);
void delay_i(string NodeName, double slew_in_rise, double slew_in_fall, double cl, int Num,string str);
double maxArrivalTime(vector<double> arrival_time);
double findmin(vector<double>comp);
void printRiseCriticalPath(string value, map<string, circuit >   & ckt_vector);
void printFallCriticalPath(string value, map<string, circuit >   & ckt_vector);
void parseSpefMin(char* argv[], map<string, circuit > &   ckt_vector);
void parseSpefMax(char* argv[], map<string, circuit > &   ckt_vector);
void timingWindow(char* argv[], map<string, circuit > & ckt_vector, map<string, circuit >& ckt_vectorEAT, map<string, circuit >& ckt_vectorWorstLAT);

void parseVerilogCktFile(char* argv[]);



int main(int argc, char* argv[])
{
	if (argc < 8)
	{
		cout << "I need atleast 6 paramter : 1) Library File Name 2)Library File Name 3) Library File Name 4) IO Liberty File Name 5) Verilog file name 6) Spef Min file 7) Spef max file 8) clock frequency 9) input slew"  << endl;
		return -1;
	}
	CLock = stod(argv[8]);
	stack<string> StackEAT, StackWorstLAT,StackLAT;
	string NodeName;

	myfile.open("Output.txt");
	myfile << "The NLDM Liberty files which are used are as below: " << endl << argv[1] << endl << argv[2] << endl << argv[3] << endl << argv[4]<< endl;
	myfile << "The verilog Circuit File under test is ----" << argv[5] << endl << endl;
	myfile << "The SPEF files for RC calculation are as below " << endl << argv[6] << endl << argv[7] << endl << endl;
	myfile << "The clock which is used for considerstion is -- " << argv[8] << " ns" << endl << endl;
	myfile << "The clock which is used for considerstion is -- " << argv[8] << " ns" << endl << endl;
	cout << "The input slew which is used for considerstion is -- " << argv[9] << " ns" << endl << endl;

	string riseNodeEAT, fallNodeEAT, riseNodeLAT, fallNodeLAT;
	map<string, circuit >    ckt_vectorEAT;
	map<string, circuit >    ckt_vector;

	slew_vec_rise.clear();
	slew_vec_fall.clear();
	parseVerilogCktFile(argv); //create a circuit file that is easier to read
	parseLibFile(argc, argv);
	parseCktFile(argc, argv, ckt_vectorEAT);
	parseSpefMin(argv, ckt_vectorEAT);
	topoSort(argv, ckt_vectorEAT, riseNodeEAT, fallNodeEAT, StackEAT);

	map<string, circuit >    ckt_vectorWorstLAT;
	VisitedMap.clear();
	ckt = {};
	parseCktFile(argc, argv, ckt_vectorWorstLAT);
	parseSpefMax(argv, ckt_vectorWorstLAT);
	topoSort(argv, ckt_vectorWorstLAT, riseNodeLAT, fallNodeLAT, StackWorstLAT);

	VisitedMap.clear();
	ckt = {};
	parseCktFile(argc, argv, ckt_vector);
	timingWindow(argv, ckt_vector, ckt_vectorEAT, ckt_vectorWorstLAT);
	topoSort(argv, ckt_vector, riseNodeLAT, fallNodeLAT, StackLAT);

	myfile << "Circuit delay " << endl;
	myfile << "Rise delay: " << ((ckt_vector[riseNodeLAT].output_arrival_rise)) << "ns" << endl;
	myfile << "Fall delay: " << ((ckt_vector[fallNodeLAT].output_arrival_fall)) << "ns" << endl << endl;
	myfile << "Gate slacks :" << endl;
	while (StackLAT.empty() == false)
	{
		NodeName = StackLAT.top();
		myfile << NodeName << "   " << ckt_vector[NodeName].gatename << "  Rise " << ":  " << (ckt_vector[NodeName].slack_rise) << " ns" << endl;
		myfile << NodeName << "  " << ckt_vector[NodeName].gatename << "  Fall " << ":  " << (ckt_vector[NodeName].slack_fall) << " ns" << endl;

		StackLAT.pop();
	}
	myfile << endl << "Critical path RISE:" << endl;
	printRiseCriticalPath(riseNodeLAT, ckt_vector);
	myfile << endl << "Critical path FALL:" << endl;
	printFallCriticalPath(fallNodeLAT, ckt_vector);
	myfile << endl << endl;



	return 0;
}

void parseVerilogCktFile(char* argv[])
{
	vector <string> inpVec;
	char* VerilogFileName = argv[5];
	char lineBuf[1024];
	string lineStr,gatename;
	cout << "Parsing Verilog file " << VerilogFileName << endl;
	ifstream ifs(VerilogFileName);
	if (ifs.is_open() == 0) { // or we could say if (!ifs)
		cout << "Error opening Verilog file " << VerilogFileName << endl;
	}
	string str;
	
	cktFile.open("ParsedVerilogCktFile.txt");
	while (ifs.good())
	{
		ifs.getline(lineBuf, 1023);	// read one line
		lineStr = (lineBuf); // convert to C++ string
		if (lineStr.empty())	// is empty line?
			continue;
		istringstream iss(lineStr); //declare string stream
		iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
		
		string  firstWord,secondWord;
		iss >> firstWord;
		//transform(firstWord.begin(), firstWord.end(), firstWord.begin(), ::toupper);
		if (firstWord == "endmodule")
			break;
		if (firstWord == "//" || firstWord == "module" || firstWord == "wire" ||  firstWord == "") {
			continue; //ignore all comments which starts with #
		}
		if (firstWord == "input")
		{
			iss >> secondWord;
			//while (iss.tellg() >0)
			//{
				//secondWord.erase(0, 1); //Removing the first letter "N" or 
				cktFile << "INPUT  " << secondWord << endl;
			//	iss >> secondWord;
			//}
			continue;
		}
		if (firstWord == "inout")
		{
			iss >> secondWord;
			//while (iss.tellg() >0)
			//{
				//secondWord.erase(0, 1);
				cktFile << "OUTPUT  " << secondWord << endl;
			//	iss >> secondWord;
			//}
			continue;
		}
		if (firstWord == "TIEH_HVT"|| firstWord == "TIEH_LVT" || firstWord == "TIEH_RVT" || firstWord == "TIEL_HVT" || firstWord == "TIEL_RVT" || firstWord == "TIEL_LVT" )
		{
			continue;
		}


		iss >> secondWord; // It tells name of of the gate eg u22, we are not considering that
		int count = 1;
		int num;

		while (iss.tellg() > 0)
		{
			count++;
			iss >> secondWord;

			if (count % 2)
			{
					if (secondWord.find("SYNOPSYS_UNCONNECTED") != std::string::npos) {
						continue;
					}
					/*if (secondWord.find("TIEH") != std::string::npos) {
						continue;
					}
					if (secondWord.find("TIEL") != std::string::npos) {
						continue;
					}*/
					inpVec.push_back(secondWord);
			}
			else
			{
				if (secondWord == "DOUT" || secondWord == "Y" || secondWord == "SO" || secondWord == "S")
				{
					iss >> gatename;
					count++;
				}
				if ((firstWord == "D4I1025_NS" || firstWord == "D4I1025_EW") && (secondWord=="PADIO"))
				{
					iss >> gatename;
					count++;
				}
				if (secondWord == "EN" || secondWord == "R_EN" )
				{
					iss >> secondWord;
					count++;
				}
			}
		}
		int ele = inpVec.size();
		cktFile  << gatename   << " = " << firstWord;

		for (int i = 0; i < ele; i++)
		{
			cktFile << "  " << inpVec[i];
		}
		inpVec.clear();
		cktFile << endl;
	}
}



void parseSpefMin(char* argv[], map<string, circuit >  &  ckt_vector)
{
	char* SpeffName = argv[6];
	char lineBuf[1024];
	string lineStr;
	map<int, spef> SpefMap;
	map<int, spef>::iterator it2;
	map<string, circuit>::iterator iT3;
	cout << "Parsing MIN SPEF file " << SpeffName << endl;
	ifstream ifs(SpeffName);
	if (ifs.is_open() == 0) { // or we could say if (!ifs)
		cout << "Error opening SPEF file " << SpeffName << endl;
	}
	while (ifs.good())
	{
		ifs.getline(lineBuf, 1023);	// read one line
		lineStr = (lineBuf); // convert to C++ string
		if (lineStr.empty())	// is empty line?
			continue;

		istringstream iss(lineStr); //declare string stream
		iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
		string str, firstWord;
		int nodeNumber;
		double LoadCapvalue;
		iss >> firstWord;
		if (firstWord == "NAME_MAP") //store all the wire name's key 
		{
			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string

			while (!lineStr.empty())	// read all name map
			{
				istringstream iss(lineStr); //declare string stream
				iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
				iss >> nodeNumber;
				iss >> str;
				spefObj.nodeName = str;
				SpefMap.insert(std::make_pair(nodeNumber, spefObj));
				spefObj = {};
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string
			}
		}
		if (firstWord == "D_NET")
		{
			double couplingCap = 0;
			iss >> nodeNumber;
			iss >> LoadCapvalue;
			it2 = SpefMap.find(nodeNumber);
			spefObj = {};
			it2->second.loadCap = LoadCapvalue; //updating the loadcap value in map
			//************************************************************///
			string str;
			string  oneWord;
			unsigned int count = 0;
			//ignore part before capacitance
			while (str != "CAP")
			{
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string
				istringstream iss(lineStr); //declare string stream
				iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
				iss >> str;
			}

			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string
			//add all coupling capacitance present in the design
			while (!lineStr.empty())
			{
				stringstream  stream(lineStr);
				count = 0;
				while (stream >> oneWord) { ++count; }
				if (count == 4) // in case of coupling capacitance there will be 4 fields seperated by space - identifier, two capacitance nodes, capacitance values 
				{
					couplingCap += stod(oneWord); //the last oneWord contains coupling capacitance, so convert to double and add that to couplingCap
					//cases where there is a load cap present 
				}
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string

			}
			it2->second.couplingCap = couplingCap; //update couplingCap values into map
			//********************************************************************//
		}
	}
	for (it2 = SpefMap.begin(); it2 != SpefMap.end(); ++it2)
	{
		iT3 = ckt_vector.find(it2->second.nodeName);
		if (iT3 != ckt_vector.end())
		{
			ckt_vector[it2->second.nodeName].cload += it2->second.loadCap;   //if there are other cells ignore that
			ckt_vector[it2->second.nodeName].cload -= it2->second.couplingCap;

		}
	}
	return;
}

void parseSpefMax(char* argv[], map<string, circuit >  &  ckt_vector)
{
		char* SpeffName = argv[7];
		char lineBuf[1024];
		string lineStr;
		map<int, spef> SpefMap;
		map<int, spef>::iterator it2;
		map<string, circuit>::iterator iT3;

		cout << "Parsing MAX SPEF file " << SpeffName << endl;
		ifstream ifs(SpeffName);
		if (ifs.is_open() == 0) { // or we could say if (!ifs)
			cout << "Error opening SPEF file " << SpeffName << endl;
		}
		while (ifs.good())
		{
			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string
			if (lineStr.empty())	// is empty line?
				continue;

			istringstream iss(lineStr); //declare string stream
			iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
			string str, firstWord;
			int nodeNumber;
			double LoadCapvalue;
			iss >> firstWord;
			if (firstWord == "NAME_MAP")
			{
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string

				while (!lineStr.empty())	// is empty line?
				{
					istringstream iss(lineStr); //declare string stream
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
					iss >> nodeNumber;
					iss >> str;
					spefObj.nodeName = str;
					SpefMap.insert(std::make_pair(nodeNumber, spefObj));
					spefObj = {};
					ifs.getline(lineBuf, 1023);	// read one line
					lineStr = (lineBuf); // convert to C++ string
				}
			}
			if (firstWord == "D_NET")
			{
				double couplingCap=0;
				iss >> nodeNumber;
				iss >> LoadCapvalue;
				it2 = SpefMap.find(nodeNumber);
				spefObj = {};
				it2->second.loadCap = LoadCapvalue;
				//************************************************************///
				string str;
				string  oneWord;
				unsigned int count = 0;
				//ignore part before capacitance
				while (str!="CAP")	
				{
					ifs.getline(lineBuf, 1023);	// read one line
					lineStr = (lineBuf); // convert to C++ string
					istringstream iss(lineStr); //declare string stream
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
					iss >> str;
				}

				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string
				//add all coupling capacitance present in the design
				while (!lineStr.empty())
				{
					stringstream  stream(lineStr);
					count = 0;
					while (stream >> oneWord) { ++count; }
					if (count == 4) // in case of coupling capacitance there will be 4 fields seperated by space - identifier, two capacitance nodes, capacitance values 
					{
						couplingCap+=stod(oneWord); //the last oneWord contains coupling capacitance, so convert to double and add that to couplingCap
						//cases where there is a load cap present 
					}
					ifs.getline(lineBuf, 1023);	// read one line
					lineStr = (lineBuf); // convert to C++ string

				}
				it2->second.couplingCap = couplingCap; //update couplingCap values into map
				//********************************************************************//
			}
		}
		//Adding load capacitance and 2*coupling capacitance values into the main ckt_vector
		for (it2 = SpefMap.begin(); it2 != SpefMap.end(); ++it2)
		{
			iT3 = ckt_vector.find(it2->second.nodeName);
			if (iT3 != ckt_vector.end())
			{
				ckt_vector[it2->second.nodeName].cload += it2->second.loadCap;
				ckt_vector[it2->second.nodeName].cload += it2->second.couplingCap ;
			}
		}

}


void timingWindow(char* argv[], map<string, circuit >& ckt_vector, map<string, circuit >& ckt_vectorEAT, map<string, circuit >& ckt_vectorWorstLAT)
{

	char* SpeffName = argv[7];
	char lineBuf[1024];
	string lineStr;
	map<int, spef> SpefMap;
	map<int, spef>::iterator it2;
	map<string, circuit>::iterator iT3;

	cout << "Parsing MAX SPEF file for timing window calculation" << SpeffName << endl;
	ifstream ifs(SpeffName);
	if (ifs.is_open() == 0) { // or we could say if (!ifs)
		cout << "Error opening SPEF file " << SpeffName << endl;
	}
	while (ifs.good())
	{
		ifs.getline(lineBuf, 1023);	// read one line
		lineStr = (lineBuf); // convert to C++ string
		if (lineStr.empty())	// is empty line?
			continue;

		istringstream iss(lineStr); //declare string stream
		iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
		string str, firstWord;
		int node1, node2;
		int nodeNumber,nodeNumber1;
		double LoadCapvalue;
		iss >> firstWord;
		if (firstWord == "NAME_MAP")
		{
			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string

			while (!lineStr.empty())	// is empty line?
			{
				istringstream iss(lineStr); //declare string stream
				iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
				iss >> nodeNumber;
				iss >> str;
				spefObj.nodeName = str;
				SpefMap.insert(std::make_pair(nodeNumber, spefObj));
				spefObj = {};
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string
			}
		}
		if (firstWord == "D_NET")
		{
			double couplingCap = 0;
			iss >> nodeNumber;
			iss >> LoadCapvalue;
			it2 = SpefMap.find(nodeNumber);
			spefObj = {};
			it2->second.loadCap = LoadCapvalue;
			//************************************************************///
			string str;
			string  oneWord;
			unsigned int count = 0;
			//ignore part before capacitance
			while (str != "CAP")
			{
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string
				istringstream iss(lineStr); //declare string stream
				iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
				iss >> str;
			}

			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string
			//add all coupling capacitance present in the design
			while (!lineStr.empty())
			{
				stringstream  stream(lineStr);
				count = 0;
				while (stream >> oneWord) { ++count; }
				if (count == 4) // in case of coupling capacitance there will be 4 fields seperated by space - identifier, two capacitance nodes, capacitance values 
				{
					if ((ckt_vectorEAT[SpefMap[nodeNumber].nodeName].output_arrival_rise > ckt_vectorWorstLAT[SpefMap[nodeNumber].nodeName].output_arrival_rise)&&  (ckt_vectorEAT[SpefMap[nodeNumber].nodeName].output_arrival_fall > ckt_vectorWorstLAT[SpefMap[nodeNumber].nodeName].output_arrival_fall))
						couplingCap -= stod(oneWord); 
					else
						couplingCap += stod(oneWord);

				}
				ifs.getline(lineBuf, 1023);	// read one line
				lineStr = (lineBuf); // convert to C++ string

			}
			it2->second.couplingCap = couplingCap; //update couplingCap values into map
			//********************************************************************//
		}
	}
	//Adding load capacitance and 2*coupling capacitance values into the main ckt_vector
	for (it2 = SpefMap.begin(); it2 != SpefMap.end(); ++it2)
	{
		iT3 = ckt_vector.find(it2->second.nodeName);
		if (iT3 != ckt_vector.end())
		{
			ckt_vector[it2->second.nodeName].cload += it2->second.loadCap;
			ckt_vector[it2->second.nodeName].cload += it2->second.couplingCap;
		}
	}

}


int parseLibFile(int argc, char* argv[])
{
	int noofinputpins = 0;
	int currentpin = -1;
	int flag = 0;
	int resizedone = 0;
	struct ParenCommaEq_is_space : std::ctype<char> {
		ParenCommaEq_is_space() : std::ctype<char>(get_table()) {}
		static mask const* get_table()
		{
			static mask rc[table_size];
			rc['('] = std::ctype_base::space;
			rc[')'] = std::ctype_base::space;
			rc[','] = std::ctype_base::space;
			rc['='] = std::ctype_base::space;
			rc[' '] = std::ctype_base::space;
			rc['\t'] = std::ctype_base::space;
			rc['"'] = std::ctype_base::space;
			rc[':'] = std::ctype_base::space;
			rc['\0'] = std::ctype_base::space;
			rc['*'] = std::ctype_base::space;
			rc[';'] = std::ctype_base::space;

			return &rc[0];
		}
	};
	//Parse Library File

	//	char* CktfName = argv[2];
	char lineBuf[1024];
	string lineStr, str;
	int delayTableRow = 7;
	int delayTableColumn = 7;
	double val;
	string getVal;
	string cellName;
	vector<double> v1;
	vector<double>    index(7);
	vector<vector<vector<double> > >   rise_delay;
	vector<vector<vector<double> > >   rise_slew;
	vector<vector<vector<double> > >   fall_delay;
	vector<vector<vector<double> > >   fall_slew;

	map<string, cell>::iterator it2;

	for (int files = 1; files < 5; files++) {
		char* LibfName = argv[files ];

		cout << "Parsing input library file " << LibfName << endl;
		ifstream ifs(LibfName);
		if (ifs.is_open() == 0) { // or we could say if (!ifs)
			cout << "Error opening file " << LibfName << endl;
			return -1;
		}
		while (ifs.good())
		{
			ifs.getline(lineBuf, 1023);	// read one line
			lineStr = (lineBuf); // convert to C++ string
			if (lineStr.empty())	// is empty line?
				continue;
			istringstream iss(lineStr); //declare string stream
			string firstWord;
			iss >> firstWord;
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if (firstWord == "cell") // found the word cell
			{
				char c;
				iss >> c; // read '('
				if (c != '(') {
					continue; // this is not an error. Happens for example when we have cell_delay(
				}
				iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));  //remove delimiters
				iss >> cellName;  //read next string whole till eol
				node.cellName = cellName;
				if (cellName.find("AOI") != string::npos || cellName.find("INV") != string::npos || cellName.find("NAND") != string::npos || cellName.find("NOR") != string::npos || cellName.find("XNOR") != string::npos) {
					node.inverting = 1;
				}
				noofinputpins = 0;
				currentpin = -1;
				flag = 1;

			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (firstWord == "capacitance" && flag)
			{
				char c;
				iss >> c;
				iss >> val;
				node.capacitance = val;
				noofinputpins++;
				flag = 2;
				resizedone = 0;
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if (firstWord.find("cell_rise") != string::npos && flag == 2)
			{
				currentpin++;
				if (currentpin == noofinputpins) {
					node.delay_rise = rise_delay;
					node.slew_rise = rise_slew;
					node.delay_fall = fall_delay;
					node.slew_fall = fall_slew;
					//insert into map cellnamne and (delay table,slew table) as a vector  
					NldmMap.insert(std::make_pair(cellName, node));
					flag = 0;
					continue;
				}
				if (!resizedone && string(LibfName).find("io") != string::npos) {
					rise_delay.resize(noofinputpins, vector<vector<double>>(6, vector<double>(6)));
					rise_slew.resize(noofinputpins, vector<vector<double>>(6, vector<double>(6)));
					fall_delay.resize(noofinputpins, vector<vector<double>>(6, vector<double>(6)));
					fall_slew.resize(noofinputpins, vector<vector<double>>(6, vector<double>(6)));
					resizedone = 1;
					delayTableRow = 6;
					delayTableColumn = 6;
				}
				else if (!resizedone) {
					rise_delay.resize(noofinputpins, vector<vector<double>>(7, vector<double>(7)));
					rise_slew.resize(noofinputpins, vector<vector<double>>(7, vector<double>(7)));
					fall_delay.resize(noofinputpins, vector<vector<double>>(7, vector<double>(7)));
					fall_slew.resize(noofinputpins, vector<vector<double>>(7, vector<double>(7)));
					resizedone = 1;
				}


				while (firstWord != "index_1")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;
					istringstream iss1(lineStr);
					iss1.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss1 >> firstWord;
				}
				index.clear();
				lineStr = (lineBuf);
				istringstream iss2(lineStr);
				iss2.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
				iss2 >> str;
				for (int i = 0; i < delayTableRow; i++)
				{
					iss2 >> val;
					index.push_back(val);

				}
				node.index_1 = index;
				index.clear();

				while (firstWord != "index_2")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;
					istringstream iss1(lineStr);
					iss1.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss1 >> firstWord;
				}

				lineStr = (lineBuf);
				istringstream iss3(lineStr);
				iss3.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
				iss3 >> str;
				for (int i = 0; i < delayTableRow; i++)
				{
					iss3 >> val;
					index.push_back(val);
				}
				node.index_2 = index;
				index.clear();

				//loop till the first word of line is "value"
				while (firstWord != "values")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;
					istringstream iss1(lineStr);
					iss1.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss1 >> firstWord;
				}
				//extract delay table
				for (int i = 0; i < delayTableRow; i++)
				{

					lineStr = (lineBuf);
					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					if (i == 0)
						iss >> getVal;
					//delay table info
					for (int j = 0; j < delayTableColumn; j++)  //format using sstream
					{
						iss >> getVal;
						val = stof(getVal);

						v1.insert(v1.begin() + j, val);
					}
					//rise_delay[currentpin].insert(rise_delay[currentpin].begin() + i, v1);

					rise_delay[currentpin][i] = v1;
					v1.clear();
					ifs.getline(lineBuf, 1023);
				}
				flag = 3;
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			//extract rise_transition
			if (firstWord.find("rise_transition") != string::npos && flag == 3)
			{


				iss >> firstWord;
				if (firstWord == "(scalar)")
					continue;
				//loop till the first word of line is "value"
				while (firstWord != "values")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;

					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss >> firstWord;
				}

				for (int i = 0; i < delayTableRow; i++)
				{
					lineStr = (lineBuf);
					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					if (i == 0)
						iss >> getVal;
					//delay table info
					for (int j = 0; j < delayTableColumn; j++)  //format using sstream
					{
						iss >> getVal;
						val = stof(getVal);
						v1.insert(v1.begin() + j, val);
					}
					//rise_slew[currentpin].insert(rise_slew[currentpin].begin() + i, v1);
					rise_slew[currentpin][i] = v1;
					v1.clear();
					ifs.getline(lineBuf, 1023);
				}

				flag = 4;
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			//extract cell_fall
			if (firstWord.find("cell_fall") != string::npos && flag == 4)
			{

				//loop till the first word of line is "value"
				while (firstWord != "values")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;

					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss >> firstWord;
				}

				for (int i = 0; i < delayTableRow; i++)
				{
					lineStr = (lineBuf);
					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					if (i == 0)
						iss >> getVal;
					//delay table info
					for (int j = 0; j < delayTableColumn; j++)  //format using sstream
					{
						iss >> getVal;
						val = stof(getVal);
						v1.insert(v1.begin() + j, val);
					}
					//fall_delay[currentpin].insert(fall_delay[currentpin].begin() + i, v1);
					fall_delay[currentpin][i] = v1;
					v1.clear();
					ifs.getline(lineBuf, 1023);
				}

				flag = 5;
			}

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			//extract fall_transition
			if (firstWord.find("fall_transition") != string::npos && flag == 5)
			{

				iss >> firstWord;
				if (firstWord == "(scalar)")
					continue;
				//loop till the first word of line is "value"
				while (firstWord != "values")
				{
					ifs.getline(lineBuf, 1023);
					lineStr = (lineBuf);
					if (lineStr.empty())	// is empty line?
						continue;

					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					iss >> firstWord;
				}

				for (int i = 0; i < delayTableRow; i++)
				{
					lineStr = (lineBuf);
					istringstream iss(lineStr);
					iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space));
					if (i == 0)
						iss >> getVal;
					//delay table info
					for (int j = 0; j < delayTableColumn; j++)  //format using sstream
					{
						iss >> getVal;
						val = stof(getVal);
						v1.insert(v1.begin() + j, val);
					}
					//fall_slew[currentpin].insert(fall_slew[currentpin].begin() + i, v1);
					fall_slew[currentpin][i] = v1;
					v1.clear();
					ifs.getline(lineBuf, 1023);
				}
				flag = 2;
				if (currentpin == noofinputpins - 1) {
					node.delay_rise = rise_delay;
					node.slew_rise = rise_slew;
					node.delay_fall = fall_delay;
					node.slew_fall = fall_slew;
					//insert into map cellnamne and (delay table,slew table) as a vector  
					NldmMap.insert(std::make_pair(cellName, node));
					flag = 0;
				}
			}

		}
	}

	//Parsing lib file over
	//*****************************************************************************************************************************************//
}

int parseCktFile(int argc, char* argv[], map<string, circuit > &   ckt_vector)
{

	char lineBuf[1024];
	string lineStr;
		//Parse circuit file
	int pos;
	map<string, circuit>::iterator iT2;
	circuit ckt3;
	vector<string> fanin_vec;
	string linestr;
	ifstream ifsc("ParsedVerilogCktFile.txt");
	if (ifsc.is_open() == 0) {
		cout << "Error opening file " << endl;
		return -1;
	}

	while (ifsc.good())
	{
		ifsc.getline(lineBuf, 1023);	// read one line
		lineStr = (lineBuf); // convert to C++ string
		if (lineStr.empty())	// is empty line?
			continue;

		istringstream iss(lineStr); //declare string stream
		pos = iss.peek();
		if (pos == '#') {
			continue; //ignore all comments which starts with #
		}
		iss.imbue(locale(cin.getloc(), new ParenCommaEq_is_space)); //remove delimeters
		string firstWord, secondWord;
		string word;
		fanin_vec.clear();
		iss >> firstWord;
		linestr = firstWord;
		double cap_out = 0;

		if (linestr == "INPUT")
		{
			ckt = {};
			ckt.gatename = firstWord;
			ckt.output_arrival_rise = 0;
			ckt.output_slew_rise = stod(argv[9]);
			ckt.output_arrival_fall = 0;
			ckt.output_slew_fall = stod(argv[9]);
			iss >> firstWord;
			ckt_vector.insert(std::make_pair(firstWord, ckt));
		}
		else if (linestr == "OUTPUT")
		{
			ckt = {};
			ckt.gatename = firstWord;
			iss >> firstWord;
			ckt_vector.insert(std::make_pair(firstWord, ckt));
		}

		else
		{
			iT2 = ckt_vector.find(firstWord);
			if (iT2 == ckt_vector.end())
				ckt = {};
			else
			{
				ckt = iT2->second;// found
				ckt_vector.erase(firstWord);
			}

			iss >> secondWord;
			ckt.gatename = secondWord;
			while (iss >> word)
			{

				fanin_vec.push_back(word);
				iT2=ckt_vector.find(word);
				if (iT2 == ckt_vector.end())
					ckt3 = {};
				else
				{
					ckt3 = iT2->second;// found
					ckt_vector.erase(word);
				}

				ckt3.fanout.push_back(firstWord);
				ckt_vector.insert(std::make_pair(word, ckt3));
				ckt3 = {};
			}
			ckt.fanin = fanin_vec;
			ckt_vector.insert(std::make_pair(firstWord, ckt));
		}
		fanin_vec.clear();
	}
	return 0;
}

void topoSortUtil(string v, queue<string>& Queue, map<string, circuit > &   ckt_vector)
{
	VisitedMap[v] = true;
	vector<string>::iterator it;
	for (it = ckt_vector[v].fanin.begin(); it != ckt_vector[v].fanin.end(); ++it)
		if (!VisitedMap[*it])
			topoSortUtil(*it, Queue, ckt_vector);
	Queue.push(v);
}

void topoSort(char* argv[], map<string, circuit >  &  ckt_vector, string & riseNode, string & fallNode, stack<string> & Stack1)
{
	double max_arrival_rise = 0, max_arrival_fall = 0, min_slack = 200000, min_slack1 = 200000;
	queue<string> Queue;
	stack<string> Stack;
	string NodeName;
	circuit node_traverse;
	vector<string> fanout;
	vector<double> comp1;
	vector<double> comp2;
	map<string, circuit>::iterator it3;


	for (it3 = ckt_vector.begin(); it3 != ckt_vector.end(); ++it3)
	{
		VisitedMap.insert(std::make_pair(it3->first, false));
	}
	for (it3 = ckt_vector.begin(); it3 != ckt_vector.end(); ++it3)
	{
		if (ckt_vector[it3->first].gatename == "")
			continue;
		if (VisitedMap[it3->first] == false)
			topoSortUtil(it3->first, Queue, ckt_vector);
	}

	// Print contents of Queue 
	while (Queue.empty() == false)
	{
		double Capval = 0;
		NodeName = Queue.front();

		Stack.push(NodeName);
		Capval = sumCapacitance(ckt_vector[NodeName].fanout, ckt_vector);

		ckt_vector[NodeName].cload += Capval;
		calculateOutputArrivalTime(NodeName, ckt_vector);  //based on topological order i calculate arrival time for rise and fall
		//cout << pos << " "<< ckt_vector[pos].output_arrival<<endl;
		if (ckt_vector[NodeName].output_arrival_rise > max_arrival_rise)
			max_arrival_rise = ckt_vector[NodeName].output_arrival_rise;
		if (ckt_vector[NodeName].output_arrival_fall > max_arrival_fall)
			max_arrival_fall = ckt_vector[NodeName].output_arrival_fall;

		Queue.pop();
	}
	Stack1 = Stack;

	//Reverse top order
	while (Stack.empty() == false)
	{
		comp1.clear();
		comp2.clear();
		NodeName = Stack.top();
		fanout = ckt_vector[NodeName].fanout;
		if (fanout.size() == 0)
		{
			ckt_vector[NodeName].req_arr_time_rise = CLock;// at output both rise and fall are expected to be at time clock
			ckt_vector[NodeName].req_arr_time_fall = CLock;//
			ckt_vector[NodeName].slack_rise = CLock - ckt_vector[NodeName].output_arrival_rise;
			ckt_vector[NodeName].slack_fall = CLock - ckt_vector[NodeName].output_arrival_fall;
			Stack.pop();
			if (ckt_vector[NodeName].slack_rise < min_slack)
			{
				min_slack = ckt_vector[NodeName].slack_rise;
				riseNode = NodeName; //node which has minimum slack
			}
			if (ckt_vector[NodeName].slack_fall < min_slack1)
			{
				min_slack1 = ckt_vector[NodeName].slack_fall;
				fallNode = NodeName;
			}
			continue;
		}
		for (int i = 0; i < fanout.size(); i++)
		{
			comp1.push_back(ckt_vector[fanout[i]].req_arr_time_rise - ckt_vector[fanout[i]].gate_delay_rise);
			comp2.push_back(ckt_vector[fanout[i]].req_arr_time_fall - ckt_vector[fanout[i]].gate_delay_fall);

		}
		ckt_vector[NodeName].req_arr_time_rise = findmin(comp1);
		ckt_vector[NodeName].req_arr_time_fall = findmin(comp2);

		ckt_vector[NodeName].slack_rise = ckt_vector[NodeName].req_arr_time_rise - ckt_vector[NodeName].output_arrival_rise;
		ckt_vector[NodeName].slack_fall = ckt_vector[NodeName].req_arr_time_fall - ckt_vector[NodeName].output_arrival_fall;
		Stack.pop();
	}

}
vector<string> gateName;
vector<string> num;
void printRiseCriticalPath(string value, map<string, circuit > &   ckt_vector)
{

	vector<int>::iterator it;
	int a;
	if (ckt_vector[value].fanin.size() == 0)
	{
		a = gateName.size();
		myfile << ckt_vector[value].gatename  <<" " <<value;
		for (int m = 0; m < a; m++)
		{
			myfile << ",  " << gateName[a - m - 1]  <<" "<< num[a - m - 1];   //printing from first ie from input to output
		}
		gateName.clear(); num.clear();
		return;
	}
	gateName.push_back(ckt_vector[value].gatename);
	num.push_back(value);
	double min = ckt_vector[ckt_vector[value].fanin[0]].slack_rise;
	string idx = ckt_vector[value].fanin[0];
	int n = ckt_vector[value].fanin.size();
	for (int i = 0; i < n; i++)
	{
		if (ckt_vector[ckt_vector[value].fanin[i]].slack_rise < min)
		{
			idx = ckt_vector[value].fanin[i];
			min = ckt_vector[ckt_vector[value].fanin[i]].slack_rise;
		}
	}
	printRiseCriticalPath(idx, ckt_vector);

}
void printFallCriticalPath(string value, map<string, circuit >  &  ckt_vector)
{

	vector<int>::iterator it;
	int a;
	if (ckt_vector[value].fanin.size() == 0)
	{
		a = gateName.size();
		myfile << ckt_vector[value].gatename << " " << value;
		for (int m = 0; m < a; m++)
		{
			myfile << ",  " << gateName[a - m - 1] << " " << num[a - m - 1];   //printing from first ie from input to output
		}
		gateName.clear(); num.clear();
		return;
	}
	gateName.push_back(ckt_vector[value].gatename);
	num.push_back(value);
	double min = ckt_vector[ckt_vector[value].fanin[0]].slack_fall;
	string idx = ckt_vector[value].fanin[0];
	int n = ckt_vector[value].fanin.size();
	for (int i = 0; i < n; i++)
	{
		if (ckt_vector[ckt_vector[value].fanin[i]].slack_fall < min)
		{
			idx = ckt_vector[value].fanin[i];
			min = ckt_vector[ckt_vector[value].fanin[i]].slack_fall;
		}
	}
	printFallCriticalPath(idx, ckt_vector);

}

double findmin(vector<double>comp)
{

	vector<double>::iterator it;
	double min = comp[0];
	for (it = comp.begin(); it != comp.end(); ++it)
		if (*it < min)
			min = *it;
	return min;

}

double sumCapacitance(vector<string> fanout, map<string, circuit >  &  ckt_vector)
{
	double val = 0;
	map<string, cell>::iterator it2;
	string str;
	vector<string>::iterator it;
	for (it = fanout.begin(); it != fanout.end(); ++it)
	{
		str = ckt_vector[*it].gatename;
		it2 = NldmMap.find(str);
		if (it2 == NldmMap.end())
		{
			node = cell();
			node.capacitance = 0;
		}
		else
			node = it2->second;
		val += node.capacitance;

	}
	return val;

}
double Delay_rise = 0,Delay_fall = 0;
void calculateOutputArrivalTime(string NodeName, map<string, circuit >  &  ckt_vector)
{
	vector<int>::iterator it;
	int size;
	map<string, cell>::iterator iT4;
	string str = ckt_vector[NodeName].gatename;
	map<string, cell>::iterator it2;
	vector<string> vec;
	vector<double> arrival_time_rise;
	vector<double> arrival_time_fall;
	double time_curr_rise, time_curr_fall, slew_in_rise, slew_in_fall;
	double aoutR,aoutF;
	vector<double>::iterator it3;
	 
	if (ckt_vector[NodeName].fanin.size() == 0) //if fanin is zero then its input so for that delay is zero
	{
		return;
	}
	if (ckt_vector[NodeName].fanout.size() == 0) //if fanin is zero then its input so for that delay is zero
	{
		ckt_vector[NodeName].output_arrival_rise = ckt_vector[ckt_vector[NodeName].fanin[0]].output_arrival_rise;
		ckt_vector[NodeName].output_arrival_fall = ckt_vector[ckt_vector[NodeName].fanin[0]].output_arrival_fall;
		ckt_vector[NodeName].output_slew_rise = ckt_vector[ckt_vector[NodeName].fanin[0]].output_slew_rise;
		ckt_vector[NodeName].output_slew_fall = ckt_vector[ckt_vector[NodeName].fanin[0]].output_slew_fall;
		return;
	}
	vec = ckt_vector[NodeName].fanin; //temperary vector for fanin
	size = ckt_vector[NodeName].fanin.size();       //fanin size
	arrival_time_rise.clear();
	arrival_time_fall.clear();

	//get current time and delay
	for (int input_i = 0; input_i < size; input_i++)
	{
		time_curr_rise = ckt_vector[vec[input_i]].output_arrival_rise;       //for each input see when fanin inputs are arriving 
		time_curr_fall = ckt_vector[vec[input_i]].output_arrival_fall;       

		slew_in_rise = ckt_vector[vec[input_i]].output_slew_rise;           //for each input see what is inputs slew
		slew_in_fall = ckt_vector[vec[input_i]].output_slew_fall;
		delay_i(NodeName, slew_in_rise, slew_in_fall, ckt_vector[NodeName].cload, input_i, ckt_vector[NodeName].gatename);    //based on load, pin number, input slew delay of gate calculated


		iT4= NldmMap.find(ckt_vector[NodeName].gatename);
		//based on gate if its inverting or non inverting I will decide transition
		//if its inverting then fall will become rise and vice vesa so its taken care by the gate type
		if (iT4->second.inverting == 0)
		{
			arrival_time_rise.push_back(time_curr_rise + Delay_rise); //store each arrival times
			arrival_time_fall.push_back(time_curr_fall + Delay_fall);
		}
		else
		{
			arrival_time_rise.push_back(time_curr_fall + Delay_rise); //store each arrival times
			arrival_time_fall.push_back(time_curr_rise + Delay_fall);
		}

	}
	//cout << endl;
	aoutR = maxArrivalTime(arrival_time_rise);
	aoutF = maxArrivalTime(arrival_time_fall);

	it3 = find(arrival_time_rise.begin(), arrival_time_rise.end(), aoutR);
	int num = std::distance(arrival_time_rise.begin(), it3);
	ckt_vector[NodeName].gate_delay_rise = aoutR - ckt_vector[ckt_vector[NodeName].fanin[num]].output_arrival_rise;  //updtae  gate delay as the max delay through gate (used for back prpogation)

	it3 = find(arrival_time_fall.begin(), arrival_time_fall.end(), aoutF);
	int num1 = std::distance(arrival_time_fall.begin(), it3);
	ckt_vector[NodeName].gate_delay_fall = aoutF - ckt_vector[ckt_vector[NodeName].fanin[num1]].output_arrival_fall;

	if (iT4->second.inverting == 0)
	{
		ckt_vector[NodeName].output_slew_rise = slew_vec_rise[num]; //since its path based STA output slew is the slew corresponding to worst case delay
		ckt_vector[NodeName].output_slew_fall = slew_vec_fall[num1];
	}
	else
	{
		ckt_vector[NodeName].output_slew_rise = slew_vec_fall[num1]; //since its path based STA output slew is the slew corresponding to worst case delay
		ckt_vector[NodeName].output_slew_fall = slew_vec_rise[num];
	}
	slew_vec_rise.clear();
	slew_vec_fall.clear();

	ckt_vector[NodeName].output_arrival_rise = aoutR;
	ckt_vector[NodeName].output_arrival_fall = aoutF;

	//cout << pos << " "<< aout << endl;
	arrival_time_rise.clear();
	arrival_time_fall.clear();

}
double maxArrivalTime(vector<double> arrival_time)
{
	vector<double>::iterator it;
	double val = 0;
	for (it = arrival_time.begin(); it != arrival_time.end(); ++it)
		if (*it > val)
			val = *it;
	return val;
}


void delay_i(string NodeName, double slew_in_rise,double slew_in_fall, double cl, int Num, string str)
{
	bool found1 = false, found2 = false, found3=false;
	double  slew_rise,slew_fall;
	//string str = ckt_vector[NodeName].gatename;
	map<string, cell>::iterator it2;
	it2 = NldmMap.find(str);
	if (it2 == NldmMap.end())
	{
		//no name matching gate name
		return ;
	}
	else
		node = it2->second;
	int x2R = 0;// node.index_1.size() - 1;
	int x2F = 0; // node.index_1.size() - 1;
	int y2 = 0;// node.index_2.size() - 1;
	int	x1R, y1, x1F;
	for (int i = 0; i < node.index_1.size(); i++)
	{
		if (slew_in_rise == node.index_1[i])
		{
			x2R = i;
			found1 = true;
			break;
		}
		if (slew_in_rise < node.index_1[i])
		{
			x2R = i;
			break;
		}
	}
	for (int i = 0; i < node.index_1.size(); i++)
	{
		if (slew_in_fall == node.index_1[i])
		{
			x2F = i;
			found3 = true;
			break;
		}
		if (slew_in_fall < node.index_1[i])
		{
			x2F = i;
			break;
		}
	}
	for (int i = 0; i < node.index_2.size(); i++)
	{
		if (cl == node.index_2[i])
		{
			y2 = i;
			found2 = true;
			break;
		}
		if (cl < node.index_2[i])
		{
			y2 = i;
			break;
		}
	}
	if (cl > node.index_2[node.index_2.size()-1])
	{
		Delay_rise = (node.delay_rise[Num][0][1]-node.delay_rise[Num][0][0])*cl*0.5;
		Delay_fall = (node.delay_fall[Num][0][1] - node.delay_fall[Num][0][0])*cl*0.5;
		slew_vec_rise.push_back(node.slew_rise[Num][node.index_2.size() - 1][node.index_2.size() - 1] );
		slew_vec_fall.push_back(node.slew_fall[Num][node.index_2.size() - 1][node.index_2.size() - 1] );
		return;
	}
	if(x2R>0)
	x1R = x2R - 1;
	else
	{x1R = 0;x2R = 1;}
	if(y2>0)
	y1 = y2 - 1;
	else
	{y1 = 0; y2 = 1;}
	if(x2F>0)
	x1F = x2F - 1;
	else
	{x1F = 0; x2F = 1;}

	double t1, t2, t3, t4, t5;
	if (found1 == true && found2 == true)
	{
		Delay_rise = node.delay_rise[Num][x2R][y2];
		slew_rise = node.slew_rise[Num][x2R][y2];
	}
	else if (found3 == true && found2 == true)
	{
		Delay_fall = node.delay_fall[Num][x2F][y2];
		slew_fall = node.slew_fall[Num][x2F][y2];
	}
	else
	{
		double temp1 = node.delay_rise[Num][x1R][y1];
		double temp2 = node.delay_rise[Num][x2R][y2];
		t1 = (node.delay_rise[Num][x1R][y1]) * (node.index_2[y2] - cl) * (node.index_1[x2R] - slew_in_rise);
		t2 = (node.delay_rise[Num][x1R][y2]) * (cl - node.index_2[y1]) * (node.index_1[x2R] - slew_in_rise);
		t3 = (node.delay_rise[Num][x2R][y1]) * (node.index_2[y2] - cl) * (slew_in_rise - node.index_1[x1R]);
		t4 = (node.delay_rise[Num][x2R][y2]) * (cl - node.index_2[y1]) * (slew_in_rise - node.index_1[x1R]);
		t5 = (node.index_2[y2] - node.index_2[y1]) * (node.index_1[x2R] - node.index_1[x1R]);
		Delay_rise = (t1 + t2 + t3 + t4) / t5;

		t1 = (node.slew_rise[Num][x1R][y1]) * (node.index_2[y2] - cl) * (node.index_1[x2R] - slew_in_rise);
		t2 = (node.slew_rise[Num][x1R][y2]) * (cl - node.index_2[y1]) * (node.index_1[x2R] - slew_in_rise);
		t3 = (node.slew_rise[Num][x2R][y1]) * (node.index_2[y2] - cl) * (slew_in_rise - node.index_1[x1R]);
		t4 = (node.slew_rise[Num][x2R][y2]) * (cl - node.index_2[y1]) * (slew_in_rise - node.index_1[x1R]);
		t5 = (node.index_2[y2] - node.index_2[y1]) * (node.index_1[x2R] - node.index_1[x1R]);
		slew_rise = (t1 + t2 + t3 + t4) / t5;


		t1 = (node.delay_fall[Num][x1F][y1]) * (node.index_2[y2] - cl) * (node.index_1[x2F] - slew_in_fall);
		t2 = (node.delay_fall[Num][x1F][y2]) * (cl - node.index_2[y1]) * (node.index_1[x2F] - slew_in_fall);
		t3 = (node.delay_fall[Num][x2F][y1]) * (node.index_2[y2] - cl) * (slew_in_fall - node.index_1[x1F]);
		t4 = (node.delay_fall[Num][x2F][y2]) * (cl - node.index_2[y1]) * (slew_in_fall - node.index_1[x1F]);
		t5 = (node.index_2[y2] - node.index_2[y1]) * (node.index_1[x2F] - node.index_1[x1F]);
		Delay_fall = (t1 + t2 + t3 + t4) / t5;

		t1 = (node.slew_fall[Num][x1F][y1]) * (node.index_2[y2] - cl) * (node.index_1[x2F] - slew_in_fall);
		t2 = (node.slew_fall[Num][x1F][y2]) * (cl - node.index_2[y1]) * (node.index_1[x2F] - slew_in_fall);
		t3 = (node.slew_fall[Num][x2F][y1]) * (node.index_2[y2] - cl) * (slew_in_fall - node.index_1[x1F]);
		t4 = (node.slew_fall[Num][x2F][y2]) * (cl - node.index_2[y1]) * (slew_in_fall - node.index_1[x1F]);
		t5 = (node.index_2[y2] - node.index_2[y1]) * (node.index_1[x2F] - node.index_1[x1F]);
		slew_fall = (t1 + t2 + t3 + t4) / t5;
	}
	slew_vec_rise.push_back(slew_rise);
	slew_vec_fall.push_back(slew_fall);

	return ;
}
