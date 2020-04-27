#include <iostream>
#include <string>
#include <vector>

using namespace std;

int solution(string arrangement) {
    int answer = 0;
    vector<char> arrange;

    for(int i = 0; i<arrangement.length(); i++){
        if (arrangement[i] == '('){
            arrange.push_back(arrangement[i]);
        }
        else if (arrangement[i] == ')'){
            if (arrangement[i-1] == '('){
                answer += arrange.size() - 1;
            }
            else{
                answer += 1;
            }
            arrange.pop_back();
        }
    }

    return answer;
}

int main(){
    string arrangement;

    cin >> arrangement;

    cout << solution(arrangement) << endl;
}