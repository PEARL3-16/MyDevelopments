#include <iostream>
#include <cstdio>
#include <string>
#include <vector>

using namespace std;

string capitalLetter[] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"};
string smallLetter[] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"};

string solution(string s, int n) {
    string answer = "";

    for(int i = 0; i< s.size(); i++){
        if (isupper(s[i])){
            for (int j = 0; j<26; j++){
                if (s[i].compare(capitalLetter[j]) == 0){
                    answer += capitalLetter[j+n];
                }
            }
        }
        else{
            for (int k = 0; k<26; k++){
                if (s[i].compare(smallLetter[k]) == 0){
                    answer += smallLetter[k+n];
                }
            }
        }
    }

    return answer;
}

int main(void){
    string s, result;
    int n;

    cin >> s >> n;

    result = solution(s, n);

    cout << result << endl;
}