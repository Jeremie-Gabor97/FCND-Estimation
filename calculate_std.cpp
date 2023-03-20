#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

using namespace std;

int main() {
    string fname = "config/log/Graph2.txt";
    // vector<vector<string>> content;
	// vector<string> row;
	string line, word;
    vector<float> values;

    fstream file (fname, ios::in);

    float sum = 0.f;
    bool is_header = true;

	if(file.is_open())
	{
		while(getline(file, line))
		{
			// row.clear();
 
			stringstream str(line);
            if (is_header) {
                is_header = false;
                continue;
            }
            // Skip the timestamp.
            getline(str, word, ',');
            // Get the measurement.
            getline(str, word, ',');
            float value = stof(word);
            sum += value;
            values.push_back(value);
		}
	}
	else {
		cout<<"Could not open the file\n";
    }

    int n = values.size();
    float var = 0;
    for (float value : values) {
        float temp = value - (sum / n);
        var += (temp * temp) / (n - 1);
    }
    cout << "Standard deviation: " << sqrt(var) << endl;
 
	// for(int i=0;i<content.size();i++)
	// {
	// 	for(int j=0;j<content[i].size();j++)
	// 	{
	// 		cout<<content[i][j]<<" ";
	// 	}
	// 	cout<<"\n";
	// }
    return 0;
}