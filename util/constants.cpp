#include "constants.h"
#include "networktables/NetworkTable.h"
#include "Timer.h"
#include <set>
#include <string>
#include <fstream>
#include <sstream>

typedef std::set<IntConstant*> IntSet;
typedef std::set<DoubleConstant*> DoubleSet;
typedef IntSet::iterator IntIter;
typedef DoubleSet::iterator DoubleIter;

static IntSet* intbank = 0;

IntConstant::IntConstant(int val, const char* n) :
	value(val), name(n) {
	if (intbank == 0) {
		intbank = new IntSet();
	}
	intbank->insert(this);
}
IntConstant::~IntConstant() {
	if (intbank == 0) {
		return;
	}
	intbank->erase(this);
}
IntConstant::operator int() const {
	return value;
}
void IntConstant::update(int val) {
	value = val;
}
const char* IntConstant::getName() {
	return name;
}

static DoubleSet* doublebank = 0;

DoubleConstant::DoubleConstant(double val, const char* n) :
	value(val), name(n) {
	if (doublebank == 0) {
		doublebank = new DoubleSet();
	}
	doublebank->insert(this);
}
DoubleConstant::~DoubleConstant() {
	if (doublebank == 0) {
		return;
	}
	doublebank->erase(this);
}
DoubleConstant::operator double() const {
	return value;
}
void DoubleConstant::update(double val) {
	value = val;
}
const char* DoubleConstant::getName() {
	return name;
}

// PORTION TWO: load/reload/write
// The WPILib Preferences class is inappropriate
// because asynchronous: it starts a thread that updates
// the table of values used much later than it should be.

const char* PATH = "/c/constants.txt";
const char* TABLE = "Constants";
const char* SAVE_STRING = "S A V E";

void write() {
	std::ofstream f(PATH, std::ios::out);
	if (!f.good()) {
		printf("Constants file could not be written");
		return;
	}

	DoubleIter end = doublebank->end();
	printf("How many doubles? %d\n", doublebank->size());
	for (DoubleIter i = doublebank->begin(); i != end; i++) {
		DoubleConstant* x = *i;
		f << "D " << double(*x) << " " << x->getName() << std::endl;
		printf("%3.6f <<- %s\n", double(*x), x->getName());
	}
	printf("Writing constants done\n");
}

NetworkTable* table = NULL;
bool read() {
	if (table == NULL) {
		table = NetworkTable::GetTable(TABLE);
	}

	bool perfect = true;
	std::map<std::string, double> kvp;

	std::ifstream in(PATH, std::ios::in);
	if (!in.good()) {
		return false;
	}
	std::string line;
	while (std::getline(in, line)) {
		std::istringstream i(line);
		std::string name;
		std::string key;
		double yy;
		i >> key >> yy >> name;
		if (i.eof() && !i.fail() && !i.bad()) {
			if (key == "D") {
				kvp[name] = yy;
			} else {
				printf("Unidentified key: |%s|\n", key.c_str());
				perfect = false;
			}
		} else {
			printf("Malformatted line: |%s|\n", line.c_str());
			perfect = false;
		}
	}

	// update values, put into /Constants/
	DoubleIter end = doublebank->end();
	for (DoubleIter i = doublebank->begin(); i != end; i++) {
		DoubleConstant* x = *i;
		std::string name(x->getName());
		if (kvp.count(name) > 0) {
			double newval = kvp[name];
			if (newval != double(*x)) {
				printf("Constant %s: file is %f, code is %f\n", x->getName(),
						newval, double(*x));
			}
			x->update(kvp[name]);
		} else {
			perfect = false;
			printf("%s not yet present\n", x->getName());
		}
		try {
			table->PutNumber(name, double(*x));
		} catch (std::exception) {
			printf("Nettables failed on pushing %s\n", x->getName());
		}
	}
	return perfect;
}

bool check() {
	if (table == NULL) {
		table = NetworkTable::GetTable(TABLE);
	}
	// read values from SmartDashboard/Constants/
	// based on the keys
	bool change = false;

	DoubleIter end = doublebank->end();
	for (DoubleIter i = doublebank->begin(); i != end; i++) {
		DoubleConstant *x = *i;
		double v = double(*x);
		std::string name(x->getName());
		try {
			v = table->GetNumber(name);
		} catch (std::exception) {
			printf("Nettables errored on %s\n", x->getName());
		}
		if (v != double(*x)) {
			x->update(v);
			change = true;
			printf("New value for %s: %f\n", x->getName(), double(*x));
		}
	}

	return change;
}

class Saver: public ITableListener {
	virtual void ValueChanged(ITable* source, const std::string& key, EntryValue value, bool isNew) {
		if (table->GetBoolean(SAVE_STRING, true)) {
			table->PutBoolean(SAVE_STRING, false);
			printf("\n\nLIVE RELOAD\n\n\n");
			Constants::reload();
		}
	}
} saver;

void Constants::load() {
	double time = GetTime();
	if (table == NULL) {
		table = NetworkTable::GetTable(TABLE);
	}
	table->PutBoolean(SAVE_STRING, false);
	table->AddTableListener(SAVE_STRING, &saver, true);

	if (!read()) {
		printf("Constants file not good; writing\n");
		write();
		if (read()) {
			printf("Second read succeeded!\n");
		} else {
			printf("Second read failed. :-(\n");
		}
	}
	printf("Constants loaded (%.4f s)\n", GetTime() - time);
}

void Constants::reload() {
	double time = GetTime();
	if (check()) {
		printf("Change occured; writing constants to file\n");
		write();
	}
	printf("Constants reloaded (%.4f s)\n", GetTime() - time);
}
