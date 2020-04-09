#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>

using namespace std;

using DH_Table = vector<vector<double>>;

string read_DH(DH_Table table) {
	string path = "config/params.yaml";
	fstream f;
	f.open(path, fstream::out);

	f << "dh:" << endl;
	for(const auto &x : table) {
		f << "- [ ";
		for(int i = 0; i < 4; ++i) {
			f << x[i] << " ";
		}
		f << "]" << endl;
	}
	f.close();
	return path;
}

int main() {
	DH_Table test = {{2,1,3,7}, {1,4,8,8}};
	cout << read_DH(test);
}
