#include <iostream>
#include <cstring>
#include <vector>
#include <cstdlib>

using namespace std;

vector<int> heights;
vector<int> result;
string h;

vector<int> solution(vector<int> heights) {
    vector<int> answer;
    int temp = 0;

    for (int i = 0; i < heights.size(); i++){
        if (i == 0){
            answer.push_back(0);
        }
        else{
            for (int j = 0; j < i; j++){
                cout << "비교:: " << heights[i] << ", " << heights[j] << endl;
                if (heights[i] < heights[j]){
                    temp = j+1;
                }
                else{
                    if (temp != 0 && heights[i] > heights[j]){
                        continue;
                    }
                    else{
                        temp = 0;
                    }
                }
            }
            answer.push_back(temp);
            temp = 0;
        }
    }

    return answer;
}

int main(void){
    cin >> h;

    h.erase(0, 1);
    h.erase(h.length()-1, 1);

    for (int i = 0; i < h.length(); i++){
        if (h.at(i) == ','){
            h.erase(i, 1);
        }
    }

    for (int j = 0 ; j < h.length(); j++){
        int n = h[j] - 48;
        heights.push_back(n);
    }

    result = solution(heights);

    cout << "[";
    for (int k = 0 ; k < result.size(); k++){
        cout << result[k];
        if (k != result.size()-1){
            cout << ",";
        }
    }
    cout << "]" << endl;
}