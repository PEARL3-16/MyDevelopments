#include <iostream>
#include <string>

using namespace std;

string heights;
string result;

void solution(){
    result += '[';
    int j = 0;
    string temp = "0";
    for (int i = 0; i<heights.length(); i++){

        if (i == 0)
            result += '0';

        else{
            result += ',';
            for (j = 0; j < i; j++){
                if (heights.at(i) < heights.at(j))
                    temp = to_string(j+1);   
            
                else{
                    if (temp != "0" && heights.at(i) > heights.at(j))
                        continue;
                    else
                        temp = to_string(0);
                }
            }
            result += temp;
            temp = "0";
        }
    }

    result += ']';
}

int main(){
    
    cin >> heights;

    heights.erase(0, 1);
    heights.erase(heights.length()-1, 1);

    for (int i = 0; i<heights.length(); i++){
        if (heights.at(i) == ','){
            heights.erase(i, 1);
        }
    }
    
    solution();

    cout << result << endl;
}