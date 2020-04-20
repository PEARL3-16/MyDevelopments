#include <iostream>
#include <cstdio>
#include <string>
#include <vector>

using namespace std;

string answer, str;

string solution(string s) {
    string answer = "";
    int len = s.length();
   // printf("len:: %d\n", len);

    int r = len / 2;
    //printf("r:: %d\n", r);

    
    if (len % 2 == 0){
        printf("even\n");
        answer = s.at(r+1);
    }
    else{
        printf("odd\n");

        answer += s.at(r+1);
        answer += s.at(r+2);
    }

    return answer;
}

int main(int argc, char* argv[]){

    cin >> str;
  //  cout << str << endl;

    answer = solution(str);

    cout << "answer:: " << answer << endl;

    return 0;
}
